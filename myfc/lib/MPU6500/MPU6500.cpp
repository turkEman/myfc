#include "mpu6500.h"
#include <driver/gpio.h>
#include <math.h>

// ── SPI instance ──────────────────────────────────────────────────────────────
SPIClass mySPI(VSPI);

static const SPISettings mpuSPISettings(12000000, MSBFIRST, SPI_MODE0);

// ── IIR filter state ──────────────────────────────────────────────────────────
static float iir_ax = 0.0f, iir_ay = 0.0f, iir_az = 0.0f;
static bool iir_seeded = false;

// R_sb is still needed for gyro body-frame rotation.
// It is no longer used in mpu_read_accel — B[][] subsumes it there.
float R_sb[3][3];

// Gyro in deg/s; accel in g.
float offset_gx, offset_gy, offset_gz;
float offset_ax, offset_ay, offset_az;

// Raw register values exposed for diagnostics / logging
int16_t raw_accel_x, raw_accel_y, raw_accel_z;
int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;

// ── 6-position calibration results (hardcoded — redo if sensor is remounted) ──
//
// b[]: zero-g bias in raw counts (negated-sensor frame), from 6-position mean.
//   True zero-g offsets — NOT the flat-position offsets, which conflate bias
//   with gravity and badly underestimate b_z (was −8.9, truth is −84.6).
//
// B[][]: combined scale + cross-axis + mount-rotation correction matrix.
//   Replaces the old (raw − offset)/ACCEL_SCALE + R_sb chain in one shot.
//   body_accel_g = B * (raw_negated − b)  →  result in g, body frame.
//
// To recalibrate: run 6-position routine, recompute b[] and B[][] offline,
// update the constants below and reflash. No runtime code changes needed.

//  accel raw: [   +77.1,  +4125.7,   -104.0]   |raw|=4128 (expect ~4096)
//   gyro  raw: [   +56.2,   +189.2,    +23.3]

// --- Sanity check (should be ±1 g or 0 g on each axis) ---
//   pos 1: got [+0.0008, +0.0003, +0.9960]  expect [+0, +0, +1]  err=0.0040 g  OK
//   pos 2: got [+0.0008, +0.0003, -1.0040]  expect [+0, +0, -1]  err=0.0040 g  OK
//   pos 3: got [+0.9990, +0.0012, +0.0018]  expect [+1, +0, +0]  err=0.0024 g  OK
//   pos 4: got [-1.0010, +0.0012, +0.0018]  expect [-1, +0, +0]  err=0.0024 g  OK
//   pos 5: got [+0.0002, +0.9985, +0.0021]  expect [+0, +1, +0]  err=0.0026 g  OK
//   pos 6: got [+0.0002, -1.0015, +0.0021]  expect [+0, -1, +0]  err=0.0026 g  OK
//     All positions OK.


// === COPY THE BLOCK BELOW INTO mpu6500.cpp ===

static const float accel_b[3] = {-34.45f, 22.58f, -93.54f};

static const float accel_B[3][3] = {
    {-0.999086f, +0.027313f, -0.041630f},
    {-0.027078f, -0.998969f, +0.005309f},
    {-0.035956f, +0.005625f, +0.987999f}};

static const float gyro_b[3] = {56.51f, 188.91f, 23.36f};


// ═════════════════════════════════════════════════════════════════════════════
//  Low-level SPI helpers
// ═════════════════════════════════════════════════════════════════════════════

static inline void csLow() { gpio_set_level((gpio_num_t)PIN_SPI_CS, 0); }
static inline void csHigh() { gpio_set_level((gpio_num_t)PIN_SPI_CS, 1); }

static void readRegistersBurst(uint8_t startReg, uint8_t *buf, uint8_t len)
{
    mySPI.beginTransaction(mpuSPISettings);
    csLow();
    mySPI.transfer(0x80 | startReg);
    for (uint8_t i = 0; i < len; i++)
        buf[i] = mySPI.transfer(0x00);
    csHigh();
    mySPI.endTransaction();
}

static void writeRegister(uint8_t reg, uint8_t value)
{
    mySPI.beginTransaction(mpuSPISettings);
    csLow();
    mySPI.transfer(reg & 0x7F);
    mySPI.transfer(value);
    csHigh();
    mySPI.endTransaction();
}

uint8_t readRegister(uint8_t reg)
{
    mySPI.beginTransaction(mpuSPISettings);
    csLow();
    mySPI.transfer(0x80 | reg);
    uint8_t val = mySPI.transfer(0x00);
    csHigh();
    mySPI.endTransaction();
    return val;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Init
// ═════════════════════════════════════════════════════════════════════════════

void mpu_init(void)
{
    offset_gx = 0.0f;
    offset_gy = 0.0f;
    offset_gz = 0.0f;
    offset_ax = 0.0f;
    offset_ay = 0.0f;
    offset_az = 0.0f;
    raw_gyro_x = 0;
    raw_gyro_y = 0;
    raw_gyro_z = 0;
    raw_accel_x = 0;
    raw_accel_y = 0;
    raw_accel_z = 0;

    gpio_set_direction((gpio_num_t)PIN_SPI_CS, GPIO_MODE_OUTPUT);
    csHigh();
    mySPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);

    writeRegister(REG_PWR_MGMT_1, 0x80); // ADD: full device reset
    delay(100);                          // wait for reset to complete

    writeRegister(REG_PWR_MGMT_1, 0x01); // then clock select
    delay(50);                           // give PLL time to lock (was 100, 50 is enough here)

    writeRegister(REG_SMPLRT_DIV, 0x00);
    writeRegister(REG_CONFIG, 0x03); // DLFP on

    writeRegister(REG_GYRO_CONFIG, 0x08);
    writeRegister(REG_ACCEL_CONFIG, 0x10);
    writeRegister(REG_ACCEL_CONFIG2, 0x03); // dlfp for accel is on
}

// ═════════════════════════════════════════════════════════════════════════════
//  apply_calibration_offsets()
//
//  Replaces compute_mount_rotation(). Call once at startup (after mpu_init).
//
//  Responsibilities:
//    1. Build R_sb from accel_b[] so gyro rotation into body frame is correct.
//       (Uses the same flat-gravity direction the old compute_mount_rotation did,
//        but derived from the more accurate 6-position b[] rather than a live read.)
//    2. Seed the IIR filter with the expected static state (az = +1 g).
//    3. Print diagnostics so you can confirm the matrix looks sane.
//
//  accel_b and accel_B are the only things you edit when recalibrating.
//  No runtime sampling needed — everything is baked in.
// ═════════════════════════════════════════════════════════════════════════════

void mpu_apply_calibration_offsets(void)
{
    // ── Build R_sb from the zero-g bias direction ────────────────────────────
    // accel_b[] is the sensor reading when no gravity is present, in the
    // negated-sensor frame.  When the craft is flat, the actual raw reading
    // is approximately accel_b[] + [0, 0, g_mag].  The mount tilt is encoded
    // in accel_b[0] and accel_b[1] — they should be near zero for a level mount.
    //
    // For the gyro rotation we still need R_sb.  We recover the gravity
    // direction that was used during calibration from accel_B's third column
    // (the column that maps sensor-z to body-z), which captures the tilt.
    // Simpler: just re-derive from accel_b the same way the old function did.

    // Gravity direction in negated-sensor frame ≈ -accel_b normalised.
    // (accel_b is the zero-g reading; gravity pulls the reading *toward* g_mag
    //  on the dominant axis, so the gravity unit vector is in the opposite
    //  direction of the bias for the non-dominant axes, and +z for the dominant.)
    // The cleanest approach: use the off-diagonal terms of B directly.
    // B[2][0] = sin(pitch_error),  B[2][1] ≈ -sin(roll_error).
    // But rebuilding R_sb from the same quaternion math is more transparent:

    float gx = -accel_b[0]; // x-component of gravity in sensor frame
    float gy = -accel_b[1]; // y-component
    float gz = 4096.0f;     // dominant z — use nominal g_mag; tilt terms are small

    // NOTE: if your b_z is large (it's −84.6 here, which is ~2% of 4096) this
    // approximation has <0.1° error on pitch/roll — perfectly acceptable for R_sb.

    float mag = sqrtf(gx * gx + gy * gy + gz * gz);
    gx /= mag;
    gy /= mag;
    gz /= mag;

    // Quaternion rotating g_hat → [0,0,+1]  (same math as old compute_mount_rotation)
    float q_w = 1.0f + gz;
    float q_x = gy;
    float q_y = -gx;
    float q_z = 0.0f;
    float n = sqrtf(q_w * q_w + q_x * q_x + q_y * q_y + q_z * q_z);
    q_w /= n;
    q_x /= n;
    q_y /= n;

    R_sb[0][0] = 1 - 2 * (q_y * q_y + q_z * q_z);
    R_sb[0][1] = 2 * (q_x * q_y - q_w * q_z);
    R_sb[0][2] = 2 * (q_x * q_z + q_w * q_y);
    R_sb[1][0] = 2 * (q_x * q_y + q_w * q_z);
    R_sb[1][1] = 1 - 2 * (q_x * q_x + q_z * q_z);
    R_sb[1][2] = 2 * (q_y * q_z - q_w * q_x);
    R_sb[2][0] = 2 * (q_x * q_z - q_w * q_y);
    R_sb[2][1] = 2 * (q_y * q_z + q_w * q_x);
    R_sb[2][2] = 1 - 2 * (q_x * q_x + q_y * q_y);

    // ── Seed IIR to calibrated static state ──────────────────────────────────
    iir_ax = 0.0f;
    iir_ay = 0.0f;
    iir_az = 1.0f; // +1 g in body z, after B is applied
    iir_seeded = true;

    // ── Diagnostics ──────────────────────────────────────────────────────────
    Serial.printf("apply_calibration_offsets:\n");
    Serial.printf("  accel bias (b): bx=%.2f  by=%.2f  bz=%.2f  counts\n",
                  accel_b[0], accel_b[1], accel_b[2]);
    Serial.printf("  gyro  bias:     gx=%.2f  gy=%.2f  gz=%.2f  counts\n",
                  gyro_b[0], gyro_b[1], gyro_b[2]);
    Serial.printf("  mount tilt — pitch: %.3f°  roll: %.3f°\n",
                  asinf(-gx) * DEG_PER_RAD,
                  atan2f(gy, gz) * DEG_PER_RAD);
    Serial.printf("  B[0]: %+.4f %+.4f %+.4f\n", accel_B[0][0], accel_B[0][1], accel_B[0][2]);
    Serial.printf("  B[1]: %+.4f %+.4f %+.4f\n", accel_B[1][0], accel_B[1][1], accel_B[1][2]);
    Serial.printf("  B[2]: %+.4f %+.4f %+.4f\n", accel_B[2][0], accel_B[2][1], accel_B[2][2]);
}

// ═════════════════════════════════════════════════════════════════════════════
//  mpu_calibrate() — kept for future re-calibration sessions
//
//  You don't run this every boot. Run it when:
//    - Sensor is physically remounted / angle changes
//    - After significant temperature change / time drift
//    - As a sanity check against the hardcoded constants
//
//  After running it, update accel_b[], accel_B[][], gyro_b[] above and reflash.
// ═════════════════════════════════════════════════════════════════════════════

float offset_raw_gx = 0.0, offset_raw_gy = 0.0, offset_raw_gz = 0.0;
float offset_raw_ax = 0.0, offset_raw_ay = 0.0, offset_raw_az = 0.0;
float accel_mean_raw_ax = 0.0f;
float accel_mean_raw_ay = 0.0f;
float accel_mean_raw_az = 0.0f;

static inline int16_t decode16(uint8_t hi, uint8_t lo)
{
    return (int16_t)((uint16_t(hi) << 8) | lo);
}

float mpu_calibrate(int count)
{
    double sum_gx = 0, sum_gy = 0, sum_gz = 0;
    double sum_ax = 0, sum_ay = 0, sum_az = 0;
    uint8_t buf[6];

    for (int i = 0; i < count; i++)
    {
        readRegistersBurst(REG_GYRO_START, buf, 6);
        sum_gx += decode16(buf[0], buf[1]);
        sum_gy += decode16(buf[2], buf[3]);
        sum_gz += decode16(buf[4], buf[5]);

        readRegistersBurst(REG_ACCEL_START, buf, 6);
        sum_ax += -(int32_t)decode16(buf[0], buf[1]);
        sum_ay += -(int32_t)decode16(buf[2], buf[3]);
        sum_az += -(int32_t)decode16(buf[4], buf[5]);

        delayMicroseconds(1000);
    }

    offset_raw_gx = float(sum_gx / count);
    offset_raw_gy = float(sum_gy / count);
    offset_raw_gz = float(sum_gz / count);

    accel_mean_raw_ax = float(sum_ax / count);
    accel_mean_raw_ay = float(sum_ay / count);
    accel_mean_raw_az = float(sum_az / count);

    const float g_mag = sqrtf(accel_mean_raw_ax * accel_mean_raw_ax +
                              accel_mean_raw_ay * accel_mean_raw_ay +
                              accel_mean_raw_az * accel_mean_raw_az);

    offset_raw_ax = accel_mean_raw_ax;
    offset_raw_ay = accel_mean_raw_ay;
    offset_raw_az = accel_mean_raw_az - g_mag;

    Serial.printf("mpu_calibrate (flat position only — use for cross-check):\n");
    Serial.printf("  gyro  raw gx: %+.3f  gy: %+.3f  gz: %+.3f  counts\n",
                  offset_raw_gx, offset_raw_gy, offset_raw_gz);
    Serial.printf("  accel raw ax: %+.3f  ay: %+.3f  az: %+.3f  counts\n",
                  offset_raw_ax, offset_raw_ay, offset_raw_az);
    Serial.printf("  g_mag: %.3f counts  (%.4f g)\n", g_mag, g_mag / ACCEL_SCALE);

    return g_mag;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Gyro read
// ═════════════════════════════════════════════════════════════════════════════

void mpu_read_gyro(Vector3f *gyro)
{
    uint8_t buf[6];
    readRegistersBurst(REG_GYRO_START, buf, 6);

    raw_gyro_x = decode16(buf[0], buf[1]);
    raw_gyro_y = decode16(buf[2], buf[3]);
    raw_gyro_z = decode16(buf[4], buf[5]);

    // Use hardcoded 6-position averaged gyro bias (better than single flat-cal)
    const float sx = (float(raw_gyro_x) - gyro_b[0]) * GYRO_SCALE;
    const float sy = (float(raw_gyro_y) - gyro_b[1]) * GYRO_SCALE;
    const float sz = (float(raw_gyro_z) - gyro_b[2]) * GYRO_SCALE;

    // Rotate into body frame
    gyro->x = R_sb[0][0] * sx + R_sb[0][1] * sy + R_sb[0][2] * sz;
    gyro->y = R_sb[1][0] * sx + R_sb[1][1] * sy + R_sb[1][2] * sz;
    gyro->z = R_sb[2][0] * sx + R_sb[2][1] * sy + R_sb[2][2] * sz;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Accel read
// ═════════════════════════════════════════════════════════════════════════════

void mpu_read_accel(Vector3f *accel_data, Vector3f *angle_imu)
{
    uint8_t buf[6];
    readRegistersBurst(REG_ACCEL_START, buf, 6);

    raw_accel_x = -1 * decode16(buf[0], buf[1]);
    raw_accel_y = -1 * decode16(buf[2], buf[3]);
    raw_accel_z = -1 * decode16(buf[4], buf[5]);

    // Subtract zero-g bias, then apply B (scale + cross-axis + mount rotation)
    // Result is in g, body frame — no separate R_sb step needed here.
    const float dx = float(raw_accel_x) - accel_b[0];
    const float dy = float(raw_accel_y) - accel_b[1];
    const float dz = float(raw_accel_z) - accel_b[2];

    const float ax_raw = (accel_B[0][0] * dx + accel_B[0][1] * dy + accel_B[0][2] * dz) / ACCEL_SCALE;
    const float ay_raw = (accel_B[1][0] * dx + accel_B[1][1] * dy + accel_B[1][2] * dz) / ACCEL_SCALE;
    const float az_raw = (accel_B[2][0] * dx + accel_B[2][1] * dy + accel_B[2][2] * dz) / ACCEL_SCALE;

    // ── Sanity gate ───────────────────────────────────────────────────────────
    if (!isfinite(ax_raw) || !isfinite(ay_raw) || !isfinite(az_raw))
        return;
    if (fabsf(ax_raw) > ACCEL_SANITY_MAX ||
        fabsf(ay_raw) > ACCEL_SANITY_MAX ||
        fabsf(az_raw) > ACCEL_SANITY_MAX)
        return;

    // ── IIR seed fallback ─────────────────────────────────────────────────────
    if (!iir_seeded)
    {
        iir_ax = ax_raw;
        iir_ay = ay_raw;
        iir_az = az_raw;
        iir_seeded = true;
    }

    // ── Software IIR — alpha = 0.10 → ~8 Hz cutoff at 500 Hz ─────────────────
    iir_ax = ACCEL_IIR_ALPHA * ax_raw + (1.0f - ACCEL_IIR_ALPHA) * iir_ax;
    iir_ay = ACCEL_IIR_ALPHA * ay_raw + (1.0f - ACCEL_IIR_ALPHA) * iir_ay;
    iir_az = ACCEL_IIR_ALPHA * az_raw + (1.0f - ACCEL_IIR_ALPHA) * iir_az;

    // accel_data->x = iir_ax;
    // accel_data->y = iir_ay;
    // accel_data->z = -iir_az;

    // Compute the final values once, consistently
    const float ax = iir_ax;
    const float ay = iir_ay;
    const float az = -iir_az; // NED: flat+stationary = -1g on Z

    accel_data->x = ax;
    accel_data->y = ay;
    accel_data->z = az;

    // // ── Angle computation ─────────────────────────────────────────────────────

    // Angle computation — now uses the same az that goes to the EKF
    // Flat: ax≈0, ay≈0, az≈-1
    // Roll:  tilt around X → ay changes, az changes
    // Pitch: tilt around Y → ax changes, az changes
    const float denom_x = sqrtf(ay * ay + az * az);
    const float denom_y = sqrtf(ax * ax + az * az);

    if (denom_x > ACCEL_DENOM_MIN)
        angle_imu->x = atan2f(-ay, -az) * DEG_PER_RAD; // roll
    if (denom_y > ACCEL_DENOM_MIN)
        angle_imu->y = atan2f(ax, -az) * DEG_PER_RAD; // pitch
}

// ════════════════════════════════════════════════════════════════════════════
//  mpu_run_6pos_calibration()
//
//  One-time 6-position accel + gyro-bias calibration.
//  Call from setup() AFTER mpu_init(), INSTEAD of mpu_apply_calibration_offsets().
//  Copy the Serial output into the three constant blocks at the top of this file,
//  remove this call, reflash, and resume normal boot.
//
//  WHAT IT COMPUTES
//  ─────────────────
//  accel_b[3]     Zero-g bias in negated-sensor raw counts.
//                 = average of all 6 means (gravity cancels across paired axes).
//
//  accel_B[3][3]  Scale + cross-axis + mount-rotation matrix.
//                 Half-differences of each paired axis form the columns of D.
//                 B = ACCEL_SCALE · D⁻¹   →   B·(raw_neg − b)/ACCEL_SCALE = iir_val
//                 (iir_val is the pre-negation value; accel_data = −iir_val)
//
//  gyro_b[3]      Zero-rate bias averaged over all 6 stationary phases
//                 (~6000 samples total — better than the old single flat-cal).
//
//  GYRO TILT COMPENSATION
//  ───────────────────────
//  R_sb is rebuilt automatically from the new accel_b[] by
//  mpu_apply_calibration_offsets() on every subsequent boot. Nothing extra to do.
//
//  BODY FRAME (must match flight controller):  +X forward   +Y right   +Z up
//  When flat:  iir_az → +1,  accel_data->z → −1 g  (NED-style negated output)
//
//  POSITION ORDER — follow exactly
//  ────────────────────────────────
//  1  FLAT         body +Z up    top of drone toward ceiling, nose forward  (iir_az → +1)
//  2  UPSIDE DOWN  body −Z up    belly toward ceiling, nose forward          (iir_az → −1)
//  3  NOSE UP      body +X up    pitch BACK  90°, nose toward ceiling        (iir_ax → +1)
//  4  NOSE DOWN    body −X up    pitch FWD   90°, nose toward floor          (iir_ax → −1)
//  5  RIGHT SIDE   body +Y up    roll LEFT   90°, right arm toward ceiling   (iir_ay → +1)
//  6  LEFT SIDE    body −Y up    roll RIGHT  90°, left arm toward ceiling    (iir_ay → −1)
// ════════════════════════════════════════════════════════════════════════════

#define CAL6_N_SAMPLES 5000 // samples per position  (~1 s at 1 ms/sample)
#define CAL6_INIT_S 10      // seconds to reach position 1 from boot
#define CAL6_NEXT_S 8       // seconds to reach each subsequent position

// ── 3×3 matrix inverse (Cramer's rule) ──────────────────────────────────────
static bool _cal6_inv3x3(const float M[3][3], float out[3][3])
{
    float det = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
    if (fabsf(det) < 1e-6f)
        return false;
    float r = 1.0f / det;
    out[0][0] = (M[1][1] * M[2][2] - M[1][2] * M[2][1]) * r;
    out[0][1] = -(M[0][1] * M[2][2] - M[0][2] * M[2][1]) * r;
    out[0][2] = (M[0][1] * M[1][2] - M[0][2] * M[1][1]) * r;
    out[1][0] = -(M[1][0] * M[2][2] - M[1][2] * M[2][0]) * r;
    out[1][1] = (M[0][0] * M[2][2] - M[0][2] * M[2][0]) * r;
    out[1][2] = -(M[0][0] * M[1][2] - M[0][2] * M[1][0]) * r;
    out[2][0] = (M[1][0] * M[2][1] - M[1][1] * M[2][0]) * r;
    out[2][1] = -(M[0][0] * M[2][1] - M[0][1] * M[2][0]) * r;
    out[2][2] = (M[0][0] * M[1][1] - M[0][1] * M[1][0]) * r;
    return true;
}

void mpu_run_6pos_calibration(void)
{
    static const char *LABEL[6] = {
        "FLAT        (body +Z up)  top toward ceiling, nose forward",
        "UPSIDE DOWN (body -Z up)  belly toward ceiling, nose forward",
        "NOSE UP     (body +X up)  pitch BACK  90 deg  —  nose toward ceiling",
        "NOSE DOWN   (body -X up)  pitch FWD   90 deg  —  nose toward floor",
        "RIGHT SIDE  (body +Y up)  roll LEFT   90 deg  —  right arm toward ceiling",
        "LEFT SIDE   (body -Y up)  roll RIGHT  90 deg  —  left arm toward ceiling",
    };

    // Expected iir value (B*(mean-b)/ACCEL_SCALE) per position.
    // Positions 0–1 → Z axis pair, 2–3 → X axis pair, 4–5 → Y axis pair.
    static const float EXPECT[6][3] = {
        {0, 0, +1}, // flat
        {0, 0, -1}, // upside down
        {+1, 0, 0}, // nose up
        {-1, 0, 0}, // nose down
        {0, +1, 0}, // right side
        {0, -1, 0}, // left side
    };

    float mean_a[6][3]; // mean negated-sensor raw accel per position
    float mean_g[6][3]; // mean raw gyro per position
    uint8_t buf[6];

    // ──────────────────────────────────────────────────────────────────────────
    //  STEP 1 — Collect 1000 samples at each of 6 orientations
    // ──────────────────────────────────────────────────────────────────────────
    Serial.println("\n\n=== MPU6500  6-POSITION CALIBRATION ===");
    Serial.println("Body frame:  +X forward   +Y right   +Z up");
    Serial.println("Keep the drone PERFECTLY STILL during 'COLLECTING'.");
    Serial.println("Motors off. No vibration.\n");

    for (int p = 0; p < 6; p++)
    {
        int ds = (p == 0) ? CAL6_INIT_S : CAL6_NEXT_S;

        Serial.printf("\n-- Position %d/6 --\n%s\n", p + 1, LABEL[p]);
        Serial.printf("Time to reposition: ");
        for (int s = ds; s > 0; s--)
        {
            Serial.printf("%d ", s);
            delay(1500);
        }

        Serial.printf("\nCOLLECTING");

        double sa[3] = {0, 0, 0};
        double sg[3] = {0, 0, 0};

        for (int i = 0; i < CAL6_N_SAMPLES; i++)
        {
            // Accel — negate on the way in, same as mpu_read_accel()
            readRegistersBurst(REG_ACCEL_START, buf, 6);
            sa[0] += -(int32_t)decode16(buf[0], buf[1]);
            sa[1] += -(int32_t)decode16(buf[2], buf[3]);
            sa[2] += -(int32_t)decode16(buf[4], buf[5]);

            // Gyro — raw counts, not negated
            readRegistersBurst(REG_GYRO_START, buf, 6);
            sg[0] += (int32_t)decode16(buf[0], buf[1]);
            sg[1] += (int32_t)decode16(buf[2], buf[3]);
            sg[2] += (int32_t)decode16(buf[4], buf[5]);

            if ((i & 0xFF) == 0xFF)
                Serial.print('.'); // dot every 256 samples
            delayMicroseconds(1500);
        }
        Serial.println(" done.");

        for (int k = 0; k < 3; k++)
        {
            mean_a[p][k] = float(sa[k] / CAL6_N_SAMPLES);
            mean_g[p][k] = float(sg[k] / CAL6_N_SAMPLES);
        }

        float gmag = sqrtf(mean_a[p][0] * mean_a[p][0] +
                           mean_a[p][1] * mean_a[p][1] +
                           mean_a[p][2] * mean_a[p][2]);
        Serial.printf("  accel raw: [%+8.1f, %+8.1f, %+8.1f]   |raw|=%.0f (expect ~%.0f)\n",
                      mean_a[p][0], mean_a[p][1], mean_a[p][2], gmag, (float)ACCEL_SCALE);
        Serial.printf("  gyro  raw: [%+8.1f, %+8.1f, %+8.1f]\n",
                      mean_g[p][0], mean_g[p][1], mean_g[p][2]);
    }

    // ──────────────────────────────────────────────────────────────────────────
    //  STEP 2 — accel_b[]:  zero-g bias
    //
    //  Average of all 6 means. For each paired axis, the +g and −g contributions
    //  cancel exactly, leaving only the electronics zero-g offset.
    //  Equivalent to averaging the three pair midpoints.
    // ──────────────────────────────────────────────────────────────────────────
    float b[3] = {0, 0, 0};
    for (int p = 0; p < 6; p++)
        for (int k = 0; k < 3; k++)
            b[k] += mean_a[p][k];
    for (int k = 0; k < 3; k++)
        b[k] /= 6.0f;

    //  STEP 3 — accel_B[][]:  scale + cross-axis + mount-rotation matrix
    //
    //  Half-difference for each body axis:
    //    d_x = (mean[nose-up] − mean[nose-dn]) / 2   ← raw-space response to body +X
    //    d_y = (mean[right]   − mean[left])    / 2   ← raw-space response to body +Y
    //    d_z = (mean[flat]    − mean[inverted]) / 2  ← raw-space response to body +Z
    //
    //  These three vectors become the COLUMNS of D (raw × 3):
    //    D * e_body = d_axis   →   raw = D * body_in_g
    //
    //  We want the inverse transform:  body_in_g = D⁻¹ * (raw − b)
    //  But mpu_read_accel divides by ACCEL_SCALE, so we bake that in:
    //    body_in_g = (ACCEL_SCALE * D⁻¹) * (raw − b) / ACCEL_SCALE
    //             = B * (raw − b) / ACCEL_SCALE
    //  where  B = ACCEL_SCALE * D⁻¹
    //
    //  Pair mapping (indices into mean_a[]):
    //    p0=flat, p1=upside-down → body Z axis
    //    p2=nose-up, p3=nose-dn  → body X axis
    //    p4=right,   p5=left     → body Y axis
    // ──────────────────────────────────────────────────────────────────────────

    // d columns: [d_z | d_x | d_y]  arranged as D[row][col]
    float D[3][3];
    // col 0 = body Z  (flat vs upside-down,  positions 0 & 1)
    for (int k = 0; k < 3; k++)
        D[k][2] = (mean_a[0][k] - mean_a[1][k]) * 0.5f;
    // col 1 = body X  (nose-up vs nose-dn,   positions 2 & 3)
    for (int k = 0; k < 3; k++)
        D[k][0] = (mean_a[2][k] - mean_a[3][k]) * 0.5f;
    // col 2 = body Y  (right vs left,         positions 4 & 5)
    for (int k = 0; k < 3; k++)
        D[k][1] = (mean_a[4][k] - mean_a[5][k]) * 0.5f;

    float Dinv[3][3];
    if (!_cal6_inv3x3(D, Dinv))
    {
        Serial.println("\n!!! D matrix is singular — calibration FAILED.");
        Serial.println("    Check positions: each axis must be exercised distinctly.");
        return;
    }

    float B[3][3];
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            B[r][c] = Dinv[r][c] * (float)ACCEL_SCALE;

    // ──────────────────────────────────────────────────────────────────────────
    //  STEP 4 — gyro_b[]:  zero-rate bias
    //
    //  Average raw gyro counts over all 6 stationary phases.
    //  ~6000 samples → much lower variance than the single flat-cal.
    //  Gyro doesn't depend on orientation (rotation rate = 0 in all positions),
    //  so we can simply average without any axis pairing.
    // ──────────────────────────────────────────────────────────────────────────
    float gb[3] = {0, 0, 0};
    for (int p = 0; p < 6; p++)
        for (int k = 0; k < 3; k++)
            gb[k] += mean_g[p][k];
    for (int k = 0; k < 3; k++)
        gb[k] /= 6.0f;

    // ──────────────────────────────────────────────────────────────────────────
    //  STEP 5 — Sanity check: apply B to each position, compare with EXPECT[]
    //
    //  This catches obvious blunders (wrong orientation, drone moved during
    //  collection) before you bake the constants into flash.
    // ──────────────────────────────────────────────────────────────────────────
    Serial.println("\n--- Sanity check (should be ±1 g or 0 g on each axis) ---");
    bool sane = true;
    for (int p = 0; p < 6; p++)
    {
        float dx = mean_a[p][0] - b[0];
        float dy = mean_a[p][1] - b[1];
        float dz = mean_a[p][2] - b[2];

        float cx = (B[0][0] * dx + B[0][1] * dy + B[0][2] * dz) / (float)ACCEL_SCALE;
        float cy = (B[1][0] * dx + B[1][1] * dy + B[1][2] * dz) / (float)ACCEL_SCALE;
        float cz = (B[2][0] * dx + B[2][1] * dy + B[2][2] * dz) / (float)ACCEL_SCALE;

        float ex = cx - EXPECT[p][0];
        float ey = cy - EXPECT[p][1];
        float ez = cz - EXPECT[p][2];
        float err = sqrtf(ex * ex + ey * ey + ez * ez);

        bool ok = (err < 0.05f); // < 50 mg residual = good
        if (!ok)
            sane = false;

        Serial.printf("  pos %d: got [%+.4f, %+.4f, %+.4f]  "
                      "expect [%+.0f, %+.0f, %+.0f]  err=%.4f g  %s\n",
                      p + 1, cx, cy, cz,
                      EXPECT[p][0], EXPECT[p][1], EXPECT[p][2],
                      err, ok ? "OK" : "!!! FAIL");
    }
    if (!sane)
    {
        Serial.println("\n!!! One or more positions failed the sanity check.");
        Serial.println("    Likely causes: wrong orientation, drone moved, vibration.");
        Serial.println("    DO NOT hardcode these values. Re-run calibration.");
        return;
    }
    Serial.println("    All positions OK.");

    // ──────────────────────────────────────────────────────────────────────────
    //  STEP 6 — Print hardcode block
    //
    //  Copy-paste this entire block into the constant section at the top of
    //  mpu6500.cpp, replacing the existing accel_b[], accel_B[][], gyro_b[].
    //  Then remove the mpu_run_6pos_calibration() call from setup() and reflash.
    // ──────────────────────────────────────────────────────────────────────────
    Serial.println("\n\n=== COPY THE BLOCK BELOW INTO mpu6500.cpp ===\n");

    Serial.printf("static const float accel_b[3] = {%.2ff, %.2ff, %.2ff};\n\n",
                  b[0], b[1], b[2]);

    Serial.printf("static const float accel_B[3][3] = {\n");
    Serial.printf("    {%+.6ff, %+.6ff, %+.6ff},\n", B[0][0], B[0][1], B[0][2]);
    Serial.printf("    {%+.6ff, %+.6ff, %+.6ff},\n", B[1][0], B[1][1], B[1][2]);
    Serial.printf("    {%+.6ff, %+.6ff, %+.6ff}};\n\n", B[2][0], B[2][1], B[2][2]);

    Serial.printf("static const float gyro_b[3] = {%.2ff, %.2ff, %.2ff};\n",
                  gb[0], gb[1], gb[2]);

    Serial.println("\n=== END OF BLOCK ===");
    Serial.println("\nSteps:");
    Serial.println("  1. Replace the three constant blocks in mpu6500.cpp.");
    Serial.println("  2. Remove mpu_run_6pos_calibration() from setup().");
    Serial.println("  3. Reflash.  mpu_apply_calibration_offsets() will rebuild R_sb automatically.");
}