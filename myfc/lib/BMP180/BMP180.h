#pragma once
#include <stdint.h>
#include <stdbool.h>

// ═══════════════════════════════════════════════════════════════════════
//
//  Contract:
//    • Wire.begin() and Wire.setClock(400000) must be called by the
//      caller (setup()) before bmp180_init() is invoked.
//    • bmp180_init() must complete before any task is spawned that
//      calls bmp180_write_reg() / bmp180_read16() / bmp180_read_pressure3().
//    • All public functions assume the caller holds the I2C mutex.
//      They do NOT take or release any semaphore themselves.
// ═══════════════════════════════════════════════════════════════════════

// ── I2C address ──────────────────────────────────────────────────────────
#define BMP180_ADDR         0x77

// ── Chip-ID register and expected value ──────────────────────────────────
#define BMP180_REG_CHIPID   0xD0
#define BMP180_CHIP_ID      0x55

// ── Register map ─────────────────────────────────────────────────────────
#define BMP180_REG_CTRL     0xF4    // measurement control
#define BMP180_REG_OUT_MSB  0xF6    // result MSB (also temp MSB)
#define BMP180_REG_OUT_LSB  0xF7    // result LSB
#define BMP180_REG_OUT_XLSB 0xF8    // result XLSB (pressure only)

// Control register commands
#define BMP180_CMD_TEMP     0x2E    // start temperature conversion
// Pressure command base — OR in (OSS << 6) for oversampling
#define BMP180_CMD_PRESS    0x34

// ── Calibration EEPROM base (0xAA–0xBF, 11 × int16/uint16) ──────────────
#define BMP180_CAL_BASE     0xAA

// ── Oversampling setting ──────────────────────────────────────────────────
//   OSS 0 = ultra-low power  4.5 ms  ~0.5 m noise
//   OSS 1 = standard         7.5 ms  ~0.4 m noise
//   OSS 2 = high-res        13.5 ms  ~0.33 m noise
//   OSS 3 = ultra-high-res  25.5 ms  ~0.25 m noise  ← chosen
#define BMP180_OSS          3

// ── Conversion wait times (datasheet Table 8, rounded up 0.5 ms) ─────────
//   Temperature: always 4.5 ms → 5 ms
//   Pressure:    4.5 + 3*OSS ms → OSS 1 → 7.5 ms → 8 ms
#define BMP180_TEMP_WAIT_MS     5
#define BMP180_PRESS_WAIT_MS    26    // OSS 0→5 ms, 1→8 ms, 2→14 ms, 3→26 ms (25.5 ms + margin)
                                      // formula (5 + 3*OSS) is ONLY correct for OSS 0 and 1

// ── Sea-level reference for altitude conversion ───────────────────────────
// Standard ISA sea-level pressure in Pa.
// Override with your ground-level pressure at startup for AGL altitude.
#define BMP180_SEA_LEVEL_PA     101325.0f

// ── Output sample passed through the FreeRTOS queue ──────────────────────
struct BaroSample
{
    float    pressurePa;    // compensated pressure, Pa
    float    altitudeM;     // altitude above sea level, metres (ISA model)
    float    tempC;         // temperature, °C  (updated at ~1 Hz)
};


// ── Public API ────────────────────────────────────────────────────────────

// One-time startup: read and store the 11 calibration coefficients.
// Wire must already be initialised. Returns false if chip-ID is wrong
// (sensor absent or wiring fault).
// Do NOT call from inside a task — call from setup() before spawning tasks.
bool bmp180_init(void);

// Issue the temperature conversion command (write 0x2E to 0xF4).
// Caller must wait BMP180_TEMP_WAIT_MS before calling bmp180_read_temp_raw().
void bmp180_start_temp(void);

// Read the 16-bit raw temperature result from 0xF6/0xF7.
// Call only after BMP180_TEMP_WAIT_MS has elapsed since bmp180_start_temp().
// Also updates the internal _B5 coefficient used by bmp180_compensate_pressure().
// Returns the raw ADC value (UT).
int32_t bmp180_read_temp_raw(void);

// Issue the pressure conversion command (0x34 | OSS<<6 to 0xF4).
// Caller must wait BMP180_PRESS_WAIT_MS before calling bmp180_read_pressure_raw().
void bmp180_start_pressure(void);

// Burst-read 3 bytes from 0xF6/0xF7/0xF8 and return the raw pressure ADC (UP).
// Call only after BMP180_PRESS_WAIT_MS has elapsed since bmp180_start_pressure().
int32_t bmp180_read_pressure_raw(void);

// Bosch datasheet compensation — converts raw temp UT → °C.
// Side-effect: updates the shared _B5 used by bmp180_compensate_pressure().
// You normally call bmp180_read_temp_raw() which calls this internally;
// exposed here so baro_task can re-run it without a re-read if needed.
float bmp180_compensate_temp(int32_t UT);

// Bosch datasheet compensation — converts raw pressure UP → Pa.
// Requires _B5 to be valid (i.e. bmp180_compensate_temp() called first).
float bmp180_compensate_pressure(int32_t UP);

// ISA barometric altitude formula.
// seaLevelPa defaults to BMP180_SEA_LEVEL_PA; pass your ground-pressure
// reading at boot for above-ground-level output.
float bmp180_pressure_to_altitude(float pressurePa,
                                  float seaLevelPa = BMP180_SEA_LEVEL_PA);



                                  
// #define BARO_IIR_ALPHA      0.30f   // confirmed 25 Hz — do not change unless rate changes

// // CRITICAL: gate is relative to homeBaroAlt, not absolute.
// // An absolute ceiling (e.g. 2000 m) silently rejects every sample above it.
// // homeBaroAlt must be written by calibration before baroFilter() is first called.
// #define BARO_SANE_WINDOW    300.0f  // m above/below home — covers normal flight envelope

// float baroFilter(float rawAlt);

float baroGate(float rawAlt);