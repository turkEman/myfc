// #include "QMC5338.h"
// #include <Wire.h>
// #include <math.h>
// #include <Arduino.h>

// // ── Internal helpers ───────────────────────────────────────────────────────────

// static bool qmc_write_reg(uint8_t reg, uint8_t val)
// {
//     Wire.beginTransmission(QMC_ADDR);
//     Wire.write(reg);
//     Wire.write(val);

//     uint8_t err = Wire.endTransmission();

//     if (err != 0)
//     {
//         Serial.print("QMC I2C write FAILED: reg=0x");
//         Serial.print(reg, HEX);
//         Serial.print(" val=0x");
//         Serial.print(val, HEX);
//         Serial.print(" err=");
//         Serial.println(err);
//         return false;
//     }

//     return true;
// }

// bool qmc_read_reg(uint8_t reg, uint8_t *out)
// {
//     Wire.beginTransmission(QMC_ADDR);
//     Wire.write(reg);
//     if (Wire.endTransmission(false) != 0)
//         return false;
//     if (Wire.requestFrom((uint8_t)QMC_ADDR, (uint8_t)1) != 1)
//         return false;
//     *out = (uint8_t)Wire.read();
//     return true;
// }

// // Wrap heading into (-180, +180].
// static float wrap180(float a)
// {
//     while (a > 180.0f)
//         a -= 360.0f;
//     while (a <= -180.0f)
//         a += 360.0f;
//     return a;
// }

// // ── Public API ─────────────────────────────────────────────────────────────────

// bool qmc_init(void)
// {
//     if (!qmc_write_reg(REG_SRESET, 0x01))
//         return false;

//     if (!qmc_write_reg(REG_CTRL2, 0x00))
//         return false;

//     if (!qmc_write_reg(REG_CTRL1, QMC_CTRL1_VAL))
//         return false;

//     return true;
// }

// // Returns true if the sensor has a new sample waiting (DRDY bit in STATUS).
// // Use this in a timer-only path where you have no DRDY pin wired.
// // If you ARE reading the DRDY pin in the caller, skip this and call
// // qmc_read_data() directly — checking pin AND register is redundant.
// bool qmc_data_ready(void)
// {
//     uint8_t status = 0;
//     if (!qmc_read_reg(REG_STATUS, &status))
//         return false;
//     return (status & 0x01) != 0;
// }

// float hard_bias[3] = {
//     402.8780f,   // sensor X
//     -2935.5738f, // sensor Y
//     880.3468f    // sensor Z
// };

// float softiron[3][3] = {
//     {0.955515f, 0.024876f, 0.023080f},  // X row
//     {0.024876f, 0.960275f, -0.032333f}, // Y row
//     {0.023080f, -0.032333f, 1.091898f}  // Z row
// };

// bool qmc_read_data(Vector3f *mag_data)
// {
//     // ── 1. Burst-read 6 bytes (XL XH YL YH ZL ZH) ────────────────────────────
//     Wire.beginTransmission(QMC_ADDR);
//     Wire.write(REG_DATA);
//     if (Wire.endTransmission(false) != 0)
//         return false;
//     if (Wire.requestFrom((uint8_t)QMC_ADDR, (uint8_t)6) != 6)
//         return false;

//     uint8_t xl = (uint8_t)Wire.read();
//     uint8_t xh = (uint8_t)Wire.read();
//     uint8_t yl = (uint8_t)Wire.read();
//     uint8_t yh = (uint8_t)Wire.read();
//     uint8_t zl = (uint8_t)Wire.read();
//     uint8_t zh = (uint8_t)Wire.read();

//     int16_t raw_x = (int16_t)((uint16_t)xh << 8 | xl);
//     int16_t raw_y = (int16_t)((uint16_t)yh << 8 | yl);
//     int16_t raw_z = (int16_t)((uint16_t)zh << 8 | zl);

//     // ── 2. Hard/soft-iron in sensor frame (raw counts, no remap yet) ──────────
//     // Calibration was captured via qmc_read_raw which does NOT remap axes,
//     // so hard_bias[] and softiron[][] are in sensor frame (raw_x, raw_y, raw_z).
//     float fx = (float)raw_x - hard_bias[0];
//     float fy = (float)raw_y - hard_bias[1];
//     float fz = (float)raw_z - hard_bias[2];

//     float cx = softiron[0][0] * fx + softiron[0][1] * fy + softiron[0][2] * fz;
//     float cy = softiron[1][0] * fx + softiron[1][1] * fy + softiron[1][2] * fz;
//     float cz = softiron[2][0] * fx + softiron[2][1] * fy + softiron[2][2] * fz;

//     // ── 3. Axis remap to body frame + scale ───────────────────────────────────
//     float bx = cy * MAG_SCALE;
//     float by = -cx * MAG_SCALE;
//     float bz = cz * MAG_SCALE;

//     // ── 4. Mounting misalignment correction ───────────────────────────────────
//     static const float R[3][3] = {
//         { 0.998763f, -0.045630f,  0.019762f },
//         { 0.045430f,  0.998913f,  0.010427f },
//         {-0.020216f, -0.009516f,  0.999750f }
//     };
//     mag_data->x = R[0][0] * bx + R[0][1] * by + R[0][2] * bz;
//     mag_data->y = R[1][0] * bx + R[1][1] * by + R[1][2] * bz;
//     mag_data->z = R[2][0] * bx + R[2][1] * by + R[2][2] * bz;

//     return true;

//     // // ── 3. Axis remap to body frame + scale ───────────────────────────────────
//     // // body X = sensor Y,  body Y = -sensor X,  body Z = sensor Z
//     // mag_data->x = cy * MAG_SCALE;
//     // mag_data->y = -cx * MAG_SCALE;
//     // mag_data->z = cz * MAG_SCALE;

//     // return true;
// }

// bool qmc_read_raw(int &mx, int &my, int &mz)
// {
//     Wire.beginTransmission(QMC_ADDR);
//     Wire.write(REG_DATA);
//     if (Wire.endTransmission(false) != 0)
//         return false;
//     if (Wire.requestFrom((uint8_t)QMC_ADDR, (uint8_t)6) != 6)
//         return false;

//     uint8_t xl = Wire.read();
//     uint8_t xh = Wire.read();
//     uint8_t yl = Wire.read();
//     uint8_t yh = Wire.read();
//     uint8_t zl = Wire.read();
//     uint8_t zh = Wire.read();

//     // your axis remap, kept in int
//     mx = (int)(int16_t)((uint16_t)xh << 8 | xl);
//     my = (int)(int16_t)((uint16_t)yh << 8 | yl);
//     mz = (int)(int16_t)((uint16_t)zh << 8 | zl);

//     return true;
// }

// bool qmc_compute_heading(const Vector3f *mag_data,
//                          const Quaternionf *q, // ← replaces attitude angles
//                          Vector3f *angle_imu)
// {
//     float mx = mag_data->x;
//     float my = mag_data->y;
//     float mz = mag_data->z;

//     // ── Norm gate (unchanged) ──────────────────────────────────────────────────
//     static constexpr float MAG_REF_NORM = 0.488f;
//     static constexpr float MAG_NORM_GATE = 0.25f;
//     float b_norm = sqrtf(mx * mx + my * my + mz * mz);
//     if (fabsf(b_norm - MAG_REF_NORM) > MAG_REF_NORM * MAG_NORM_GATE)
//         return false;

//     // ── Bottom row of rotation matrix R_bn (body→NED), yaw-independent ────────
//     // These encode roll and pitch without any atan2 or sin/cos.
//     float a = 2.0f * (q->w * q->y - q->x * q->z);        //  sinP
//     float b = 2.0f * (q->y * q->z + q->w * q->x);        //  sinR·cosP
//     float c = 1.0f - 2.0f * (q->x * q->x + q->y * q->y); //  cosR·cosP

//     // ── Tilt-compensated horizontal components (cosP factor cancels in atan2) ──
//     float Xh = mx * (b * b + c * c) + a * (my * b + mz * c);
//     float Yh = my * c - mz * b;

//     // ── Horizontal magnitude guard (same semantics as before) ──────────────────
//     float h_norm_sq = Xh * Xh + Yh * Yh;
//     if (h_norm_sq < (MAG_H_MIN * MAG_H_MIN) ||
//         h_norm_sq < (b_norm * MAG_H_REL_MIN) * (b_norm * MAG_H_REL_MIN))
//         return false;

//     float heading = atan2f(-Yh, Xh) * DEG_PER_RAD + MAG_DECLINATION;
//     angle_imu->z = wrap180(heading);
//     return true;
// }



#include "QMC5338.h"
#include <Wire.h>
#include <Arduino.h>

// ── Low-level I/O ─────────────────────────────────────────────────────────

static bool qmc_write_reg(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(QMC_ADDR);
    Wire.write(reg);
    Wire.write(val);
    uint8_t err = Wire.endTransmission();
    if (err) {
        Serial.printf("[QMC] write ERR  reg=0x%02X val=0x%02X err=%u\n",
                      reg, val, err);
        return false;
    }
    return true;
}

bool qmc_read_reg(uint8_t reg, uint8_t *out)
{
    Wire.beginTransmission(QMC_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)QMC_ADDR, (uint8_t)1) != 1) return false;
    *out = (uint8_t)Wire.read();
    return true;
}

static float wrap180(float a)
{
    while (a >  180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

// ── Initialisation ────────────────────────────────────────────────────────
//
// Sequence per QMC5883L datasheet §6.1 and application note:
//   1. Soft-reset to guarantee clean state on warm reboot.
//   2. SET/RESET period register (always 0x01).
//   3. CTRL2: enable DRDY/INT output pin.  THIS IS THE CRITICAL STEP —
//      INT_ENB = 0 tri-states the pin and the ISR never fires.
//   4. CTRL1: OSR=512 | RNG=±2G | ODR=100Hz | Continuous.

bool qmc_init(void)
{
    // 1. Soft-reset
    if (!qmc_write_reg(REG_CTRL2, CTRL2_SOFT_RST))
        return false;
    delay(5);                               // datasheet: ≥5 ms after soft-reset

    // 2. SET/RESET period (mandatory per §6.1, value always 0x01)
    if (!qmc_write_reg(REG_SRESET, 0x01))
        return false;

    // 3. Enable DRDY output (INT_ENB = 1)
    if (!qmc_write_reg(REG_CTRL2, QMC_CTRL2_VAL))
        return false;

    // 4. Start continuous measurement
    if (!qmc_write_reg(REG_CTRL1, QMC_CTRL1_VAL))
        return false;

    return true;
}

// ── Chip presence check ───────────────────────────────────────────────────
// Reads register 0x0D (= REG_CHIP_ID; same numeric value as QMC_ADDR but
// different namespace).  All QMC5883L silicon returns 0xFF here.

bool qmc_check(void)
{
    uint8_t id = 0;
    if (!qmc_read_reg(REG_CHIP_ID, &id)) {
        Serial.println("[QMC] check: I2C NAK — sensor absent or SDA/SCL fault");
        return false;
    }
    if (id != QMC_CHIP_ID_VAL) {
        Serial.printf("[QMC] check: chip-ID 0x%02X, expected 0x%02X\n",
                      id, QMC_CHIP_ID_VAL);
        return false;
    }
    return true;
}

// ── DRDY software poll (use when no interrupt pin is wired) ───────────────

bool qmc_data_ready(void)
{
    uint8_t status = 0;
    if (!qmc_read_reg(REG_STATUS, &status)) return false;
    return (status & 0x01) != 0;           // DRDY bit
}

// ── Calibration constants — sensor frame, raw counts ─────────────────────
// Captured via qmc_read_raw() (no axis remap), so they live in sensor frame.

static const float hard_bias[3] = {
     402.8780f,    // sensor X
    -2935.5738f,   // sensor Y
     880.3468f     // sensor Z
};

static const float softiron[3][3] = {
    {  0.955515f,  0.024876f,  0.023080f },
    {  0.024876f,  0.960275f, -0.032333f },
    {  0.023080f, -0.032333f,  1.091898f }
};

// Small physical misalignment of the mag module on the PCB.
// Determined by cross-calibration with the IMU gyro.
static const float R_mount[3][3] = {
    {  0.998763f, -0.045630f,  0.019762f },
    {  0.045430f,  0.998913f,  0.010427f },
    { -0.020216f, -0.009516f,  0.999750f }
};

// ── Calibrated read ───────────────────────────────────────────────────────
//
// Pipeline:
//   raw counts
//   → hard-iron subtraction       (sensor frame)
//   → soft-iron multiply          (sensor frame)
//   → axis remap + scale          (body frame, Gauss)
//   → mounting misalignment R     (body frame, Gauss)

bool qmc_read_data(Vector3f *mag_data)
{
    // Burst-read 6 bytes: XL XH YL YH ZL ZH
    Wire.beginTransmission(QMC_ADDR);
    Wire.write(REG_DATA);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)QMC_ADDR, (uint8_t)6) != 6) return false;

    const int16_t raw_x = (int16_t)((uint16_t)(Wire.read()) | (uint16_t)(Wire.read()) << 8);
    const int16_t raw_y = (int16_t)((uint16_t)(Wire.read()) | (uint16_t)(Wire.read()) << 8);
    const int16_t raw_z = (int16_t)((uint16_t)(Wire.read()) | (uint16_t)(Wire.read()) << 8);

    // Hard-iron (sensor frame)
    float fx = (float)raw_x - hard_bias[0];
    float fy = (float)raw_y - hard_bias[1];
    float fz = (float)raw_z - hard_bias[2];

    // Soft-iron (sensor frame)
    float cx = softiron[0][0]*fx + softiron[0][1]*fy + softiron[0][2]*fz;
    float cy = softiron[1][0]*fx + softiron[1][1]*fy + softiron[1][2]*fz;
    float cz = softiron[2][0]*fx + softiron[2][1]*fy + softiron[2][2]*fz;

    // Axis remap to body frame + scale to Gauss
    // body X = +sensor Y,  body Y = −sensor X,  body Z = +sensor Z
    float bx =  cy * MAG_SCALE;
    float by = -cx * MAG_SCALE;
    float bz =  cz * MAG_SCALE;

    // Mounting misalignment rotation
    mag_data->x = R_mount[0][0]*bx + R_mount[0][1]*by + R_mount[0][2]*bz;
    mag_data->y = R_mount[1][0]*bx + R_mount[1][1]*by + R_mount[1][2]*bz;
    mag_data->z = R_mount[2][0]*bx + R_mount[2][1]*by + R_mount[2][2]*bz;

    return true;
}

// ── Raw read (used during calibration capture only) ───────────────────────

bool qmc_read_raw(int &mx, int &my, int &mz)
{
    Wire.beginTransmission(QMC_ADDR);
    Wire.write(REG_DATA);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)QMC_ADDR, (uint8_t)6) != 6) return false;

    mx = (int)(int16_t)((uint16_t)(Wire.read()) | (uint16_t)(Wire.read()) << 8);
    my = (int)(int16_t)((uint16_t)(Wire.read()) | (uint16_t)(Wire.read()) << 8);
    mz = (int)(int16_t)((uint16_t)(Wire.read()) | (uint16_t)(Wire.read()) << 8);
    return true;
}

// ── Tilt-compensated heading ──────────────────────────────────────────────
//
// Uses the bottom row of R_bn (body→NED) derived from the EKF quaternion.
// No Euler-angle extraction is needed.
//
// Bottom row of R_bn (yaw-independent):
//   a = sinP       = 2(q.w·q.y − q.x·q.z)
//   b = sinR·cosP  = 2(q.y·q.z + q.w·q.x)
//   c = cosR·cosP  = 1 − 2(q.x² + q.y²)
//
// Tilt-compensated horizontal projections (Freescale AN4248 equivalent):
//   Xh = mx·(b²+c²) + a·(my·b + mz·c)  =  cosP·[mx·cosP + my·sinR·sinP + mz·cosR·sinP]
//   Yh = my·c − mz·b                    =  cosP·[my·cosR − mz·sinR]
//
// The shared cosP factor cancels in atan2, so the formula is exact for any
// pitch ≠ ±90° (handled by MAG_H_REL_MIN guard).

bool qmc_compute_heading(const Vector3f   *mag_data,
                         const Quaternionf *q,
                         Vector3f         *angle_imu)
{
    const float mx = mag_data->x;
    const float my = mag_data->y;
    const float mz = mag_data->z;

    // ── Total-field norm gate ─────────────────────────────────────────────
    // Reject if |B| deviates >25% from the reference norm (0.488 G).
    // Catches saturated readings and strong nearby interference.
    static constexpr float MAG_REF_NORM  = 0.488f;
    static constexpr float MAG_NORM_GATE = 0.25f;

    const float b_norm = sqrtf(mx*mx + my*my + mz*mz);
    if (fabsf(b_norm - MAG_REF_NORM) > MAG_REF_NORM * MAG_NORM_GATE)
        return false;

    // ── Bottom row of R_bn (roll/pitch only; yaw drops out) ──────────────
    const float a = 2.0f*(q->w*q->y - q->x*q->z);         // sinP
    const float b = 2.0f*(q->y*q->z + q->w*q->x);         // sinR·cosP
    const float c = 1.0f - 2.0f*(q->x*q->x + q->y*q->y); // cosR·cosP

    // ── Horizontal projections ────────────────────────────────────────────
    const float Xh = mx*(b*b + c*c) + a*(my*b + mz*c);
    const float Yh = my*c - mz*b;

    // ── Horizontal magnitude guard ────────────────────────────────────────
    // Rejects near-vertical attitudes where heading is geometrically undefined.
    const float h2 = Xh*Xh + Yh*Yh;
    if (h2 < (MAG_H_MIN * MAG_H_MIN) ||
        h2 < (b_norm * MAG_H_REL_MIN) * (b_norm * MAG_H_REL_MIN))
        return false;

    // angle_imu.z written ONLY on valid sample; last good value kept otherwise
    angle_imu->z = wrap180(atan2f(-Yh, Xh) * DEG_PER_RAD + MAG_DECLINATION);
    return true;
}

// Call this in setup() right after qmc_init().
// Prints every control register and flags mismatches.
void qmc_verify_regs(void)
{
    struct { uint8_t reg; uint8_t want; const char *name; } cfg[] = {
        { REG_SRESET, 0x01,          "SRESET" },
        { REG_CTRL2,  QMC_CTRL2_VAL, "CTRL2 (INT_ENB)" },
        { REG_CTRL1,  QMC_CTRL1_VAL, "CTRL1 (OSR/RNG/ODR/MODE)" },
        { REG_STATUS, 0xFF,          "STATUS (any)" },   // just read, don't check
    };

    Serial.println("[QMC] ── Register readback ──────────────────────────");
    for (auto &r : cfg)
    {
        uint8_t val = 0xAA;
        bool ok = qmc_read_reg(r.reg, &val);
        if (!ok) {
            Serial.printf("[QMC]   0x%02X %-24s → I2C NAK (sensor absent?)\n",
                          r.reg, r.name);
            continue;
        }
        if (r.want == 0xFF)     // STATUS: just display
        {
            Serial.printf("[QMC]   0x%02X %-24s = 0x%02X  DRDY=%d OVL=%d\n",
                          r.reg, r.name, val, val & 0x01, (val >> 1) & 0x01);
        }
        else
        {
            bool match = (val == r.want);
            Serial.printf("[QMC]   0x%02X %-24s = 0x%02X  (want 0x%02X) %s\n",
                          r.reg, r.name, val, r.want, match ? "OK" : "<-- MISMATCH");
        }
    }
    Serial.println("[QMC] ───────────────────────────────────────────────");
}