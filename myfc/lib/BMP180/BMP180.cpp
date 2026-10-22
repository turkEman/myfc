#include "BMP180.h"
#include <Wire.h>
#include <math.h>

// ── Calibration coefficients (loaded once in bmp180_init) ────────────────
static int16_t _AC1, _AC2, _AC3;
static uint16_t _AC4, _AC5, _AC6;
static int16_t _B1, _B2, _MB, _MC, _MD;

// Shared between bmp180_compensate_temp() and bmp180_compensate_pressure().
// Written by compensate_temp, read by compensate_pressure — both always
// called from the same task (baro_task), so no concurrency issue.
static int32_t _B5 = 0;

// ── Internal helpers ──────────────────────────────────────────────────────

static bool bmp180_write_reg(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(BMP180_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

// Read one byte from reg.
static bool bmp180_read8(uint8_t reg, uint8_t *out)
{
    Wire.beginTransmission(BMP180_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)
        return false;
    if (Wire.requestFrom((uint8_t)BMP180_ADDR, (uint8_t)1) != 1)
        return false;
    *out = (uint8_t)Wire.read();
    return true;
}

// Read two bytes from reg (MSB first) → big-endian uint16.
// Used only during calibration load in bmp180_init().
static uint16_t bmp180_read16_cal(uint8_t reg)
{
    Wire.beginTransmission(BMP180_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)BMP180_ADDR, (uint8_t)2);
    uint16_t val = ((uint16_t)Wire.read() << 8) | (uint8_t)Wire.read();
    return val;
}

// ── Public API ────────────────────────────────────────────────────────────

// One-time startup — reads all 11 calibration words from EEPROM.
// Wire must already be initialised by the caller (setup()).
// Returns false if the chip-ID register does not read 0x55.
bool bmp180_init(void)
{
    // Verify sensor is present and responding
    uint8_t id = 0;
    if (!bmp180_read8(BMP180_REG_CHIPID, &id) || id != BMP180_CHIP_ID)
        return false;

    // ── Load calibration coefficients from 0xAA–0xBF ─────────────────────
    // Order matches datasheet Table 5.
    _AC1 = (int16_t)bmp180_read16_cal(0xAA);
    _AC2 = (int16_t)bmp180_read16_cal(0xAC);
    _AC3 = (int16_t)bmp180_read16_cal(0xAE);
    _AC4 = bmp180_read16_cal(0xB0);
    _AC5 = bmp180_read16_cal(0xB2);
    _AC6 = bmp180_read16_cal(0xB4);
    _B1 = (int16_t)bmp180_read16_cal(0xB6);
    _B2 = (int16_t)bmp180_read16_cal(0xB8);
    _MB = (int16_t)bmp180_read16_cal(0xBA);
    _MC = (int16_t)bmp180_read16_cal(0xBC);
    _MD = (int16_t)bmp180_read16_cal(0xBE);

    return true;
}

// Issue temperature conversion command.
// Caller waits BMP180_TEMP_WAIT_MS (5 ms) before reading.
void bmp180_start_temp(void)
{
    bmp180_write_reg(BMP180_REG_CTRL, BMP180_CMD_TEMP);
}

// Read 16-bit raw temperature from 0xF6/0xF7.
// Also runs compensation to update _B5 for the next pressure call.
int32_t bmp180_read_temp_raw(void)
{
    Wire.beginTransmission(BMP180_ADDR);
    Wire.write(BMP180_REG_OUT_MSB);

    if (Wire.endTransmission(false) != 0)
        return INT32_MIN;
    if (Wire.requestFrom((uint8_t)BMP180_ADDR,
                         (uint8_t)2) != 2)
        return INT32_MIN;

    int32_t UT = ((int32_t)(uint8_t)Wire.read() << 8) | (int32_t)(uint8_t)Wire.read();

    // Update _B5 immediately so it is ready for the pressure call that follows.
    bmp180_compensate_temp(UT);

    return UT;
}

// Issue pressure conversion command with configured OSS.
// Caller waits BMP180_PRESS_WAIT_MS (8 ms for OSS 1) before reading.
void bmp180_start_pressure(void)
{
    bmp180_write_reg(BMP180_REG_CTRL,
                     BMP180_CMD_PRESS | ((uint8_t)BMP180_OSS << 6));
}

// Burst-read 3 bytes from 0xF6–0xF8, assemble 19-bit raw pressure (UP).
int32_t bmp180_read_pressure_raw(void)
{
    Wire.beginTransmission(BMP180_ADDR);
    Wire.write(BMP180_REG_OUT_MSB);

    if (Wire.endTransmission(false) != 0)
        return INT32_MIN;

    if (Wire.requestFrom((uint8_t)BMP180_ADDR,
                         (uint8_t)3) != 3)
        return INT32_MIN;

    int32_t UP = ((int32_t)(uint8_t)Wire.read() << 16) | ((int32_t)(uint8_t)Wire.read() << 8) | (int32_t)(uint8_t)Wire.read();

    // Align to OSS: shift right by (8 - OSS) as per datasheet section 4.2.
    UP >>= (8 - BMP180_OSS);
    return UP;
}

// ── Bosch datasheet compensation algorithm (section 4.2.3) ───────────────

// Temperature compensation.
// Updates _B5 as a side-effect — must be called before compensate_pressure().
// Returns temperature in °C.
float bmp180_compensate_temp(int32_t UT)
{
    int32_t X1 = ((UT - (int32_t)_AC6) * (int32_t)_AC5) >> 15;
    int32_t X2 = ((int32_t)_MC << 11) / (X1 + (int32_t)_MD);
    _B5 = X1 + X2;
    return (float)((_B5 + 8) >> 4) / 10.0f; // °C
}

// Pressure compensation.
// Requires _B5 to be current (i.e. compensate_temp called this cycle or recently).
// Returns pressure in Pa.
float bmp180_compensate_pressure(int32_t UP)
{
    int32_t B6 = _B5 - 4000;

    int32_t X1 = ((int32_t)_B2 * ((B6 * B6) >> 12)) >> 11;
    int32_t X2 = ((int32_t)_AC2 * B6) >> 11;
    int32_t X3 = X1 + X2;
    int32_t B3 = ((((int32_t)_AC1 * 4 + X3) << BMP180_OSS) + 2) >> 2;

    X1 = ((int32_t)_AC3 * B6) >> 13;
    X2 = ((int32_t)_B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;

    uint32_t B4 = ((uint32_t)_AC4 * (uint32_t)(X3 + 32768)) >> 15;
    uint32_t B7 = ((uint32_t)UP - (uint32_t)B3) * (50000UL >> BMP180_OSS);

    int32_t p;
    if (B7 < 0x80000000UL)
        p = (int32_t)((B7 << 1) / B4);
    else
        p = (int32_t)((B7 / B4) << 1);

    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    p += (X1 + X2 + 3791) >> 4;

    return (float)p; // Pa
}

// // ISA barometric altitude.
// // Using the standard formula: h = 44330 * (1 - (P/P0)^(1/5.255))
// float bmp180_pressure_to_altitude(float pressurePa, float seaLevelPa)
// {
//     return 44330.0f * (1.0f - powf(pressurePa / seaLevelPa, 1.0f / 5.255f));
// }

float bmp180_pressure_to_altitude(float pressurePa, float seaLevelPa)
{
    float ratio = pressurePa / seaLevelPa;
    if (!isfinite(ratio) || ratio <= 0.0f)
        return NAN; // propagate error explicitly
    return 44330.0f * (1.0f - powf(ratio, 1.0f / 5.255f));
}


static constexpr float BARO_MAX_RATE_MS = 25.0f;                              // m/s — above airframe Vz max
static constexpr float BARO_NOMINAL_DT = 1.0f / 25.0f;                        // s   — OSS=2 actual ~30 Hz
static constexpr float BARO_MAX_DELTA_M = BARO_MAX_RATE_MS * BARO_NOMINAL_DT; // ≈ 0.83 m

static float baro_last_good = NAN;
static bool baro_seeded = false;

#include <Arduino.h>
float baroGate(float rawAlt)
{
    if (!isfinite(rawAlt)) {
        Serial.println("Baro gate NAN1");
        return NAN;
    }

    if (!baro_seeded) {
        baro_last_good = rawAlt;
        baro_seeded = true;
        Serial.println("Baro gate boot seed");
        return rawAlt;
    }

    float delta = fabsf(rawAlt - baro_last_good);
    if (delta > BARO_MAX_DELTA_M) {
        Serial.print("Baro gate NAN2. Rejected Value: ");
        Serial.println(rawAlt);
        return NAN;
    }

    baro_last_good = rawAlt;
    Serial.println("Baro gate ret good");
    return rawAlt;
}