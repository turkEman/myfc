// ================================================================
// SECTION 1 — INCLUDES
// ================================================================

#include <Arduino.h>
#include <MPU6500.h>
#include <fc_config.h>
#include <AMEKF.h>
#include <fc_pins.h>
#include <QMC5338.h>
#include <Wire.h>
#include <BMP180.h>
#include <PosKF.h>
#include <wifi_bridge.h>
#include <PWM.h>
#include <PID.h>
#include <BatteryMonitor.h>
#include "gyrofilter.h"

#define WEB_EN 1

// ================================================================
// SECTION 2 — EXTERNAL DECLARATIONS
// ================================================================

// SPI instance — defined in MPU6500_SPI.cpp
extern SPIClass mySPI;

// Sensor task queues and task launchers — defined in mag/baro TUs
extern QueueHandle_t magDataQueue, baroDataQueue;
extern void magTask_start(SemaphoreHandle_t i2cMutex);
extern void baroTask_start(SemaphoreHandle_t i2cMutex);

// ════════════════════════════════════════════════════════════════════════════
// Attitude history ring buffer
//
// Written by att_history_push() on every IMU tick (Core 0, 1 kHz).
// Read by lookupQuaternion() inside fc_update_attitude_mag() (Core 0, 100 Hz).
// Both callers are on Core 0 in the same task context → no mutex needed.
// ════════════════════════════════════════════════════════════════════════════

static constexpr int ATT_BUF_LEN = 64; // 64 ms at 1 kHz; ample for 5 ms lookup

struct AttEntry
{
    int64_t ts_us;
    Quaternionf q;
};

static AttEntry s_att_buf[ATT_BUF_LEN];
static int s_att_head = 0;  // next write slot
static int s_att_count = 0; // saturates at ATT_BUF_LEN

// Call this once per IMU tick, immediately after amekf updates the quaternion.
static void att_history_push(int64_t ts_us, const Quaternionf &q)
{
    s_att_buf[s_att_head] = {ts_us, q};
    s_att_head = (s_att_head + 1) % ATT_BUF_LEN;
    if (s_att_count < ATT_BUF_LEN)
        ++s_att_count;
}

// Return the buffered quaternion whose timestamp is nearest to target_us.
// Returns identity { w=1, x=0, y=0, z=0 } until the first push has been made.
static Quaternionf lookupQuaternion(int64_t target_us)
{
    if (s_att_count == 0)
        return Quaternionf{1.0f, 0.0f, 0.0f, 0.0f};

    // Walk only the valid portion of the ring (oldest … newest).
    const int start = (s_att_head - s_att_count + ATT_BUF_LEN) % ATT_BUF_LEN;
    int best = start;
    int64_t best_dt = llabs(s_att_buf[start].ts_us - target_us);

    for (int i = 1; i < s_att_count; ++i)
    {
        const int idx = (start + i) % ATT_BUF_LEN;
        const int64_t dt = llabs(s_att_buf[idx].ts_us - target_us);
        if (dt < best_dt)
        {
            best_dt = dt;
            best = idx;
        }
    }
    return s_att_buf[best].q;
}

// ================================================================
// SECTION 3 — GLOBAL FLIGHT STATE
// ================================================================

static volatile bool motorsArmed = false;
static volatile bool rxSignalPresent = false; // written by 100Hz RC task; consumed by 490Hz motor task
static bool rxWasLost = false;                // latches on signal loss; cleared only by explicit disarm gesture
                                              // (not volatile — never leaves the 100Hz block)
static uint32_t prevBatteryMicros = 0;
float BatteryVoltage = 0;
// ================================================================
// SECTION 4 — RC INPUTS
// ================================================================

// PWM capture objects (default pulse = mid/min as appropriate)
static PWM_in rx_aileron(FC_PIN_RX_AILERON, FC_RC_MID_US);
static PWM_in rx_throttle(FC_PIN_RX_THROTTLE, FC_RC_MIN_US);
static PWM_in rx_arm(FC_PIN_RX_ARM, FC_RC_MIN_US);
static PWM_in rx_elevator(FC_PIN_RX_ELEVATOR, FC_RC_MID_US);
static PWM_in rx_aux(FC_PIN_RX_AUX, FC_RC_MID_US);
static PWM_in rx_rudder(FC_PIN_RX_RUDDER, FC_RC_MID_US);

// Normalised channel values — mapped to 1000–2000 µs symmetric range
static uint16_t aileron = FC_RC_MID_US;
static uint16_t elevator = FC_RC_MID_US;
static uint16_t throttle = FC_RC_MIN_US;
static uint16_t rudder = FC_RC_MID_US;
static uint16_t armPulse = FC_RC_MIN_US;
static uint16_t auxPulse = FC_RC_MID_US; // ⚠ UNUSED — received but never consumed in control

// Raw receiver pulse widths (pre-mapping)
uint16_t rawAil = 0;
uint16_t rawEle = 0;
uint16_t rawThr = 0;
uint16_t rawRud = 0;
uint16_t rawArm = 0;
uint16_t rawAux = 0; // ⚠ UNUSED — stored into auxPulse which is never consumed

// ================================================================
// SECTION 5 — ESC OUTPUTS
// ================================================================

// Motor positions (X-frame, viewed from above):
//   esc_pwm1 = front right  (CCW)
//   esc_pwm2 = front left   (CW)
//   esc_pwm3 = back left    (CCW)
//   esc_pwm4 = back right   (CW)
static PWM_out esc_pwm1;
static PWM_out esc_pwm2;
static PWM_out esc_pwm3;
static PWM_out esc_pwm4;

// Computed pulse widths (µs as float, for mixer arithmetic)
static float pulse_esc1 = 0.0f;
static float pulse_esc2 = 0.0f;
static float pulse_esc3 = 0.0f;
static float pulse_esc4 = 0.0f;

// ================================================================
// SECTION 6 — PID CONTROLLERS
// ================================================================

// ── Outer loop: angle → rate setpoint (500 Hz) ───────────────────
// static PIDController angleRollPid(2.0f, 0.0f, 0.0f, 200.0f, 50.0f);
// static PIDController anglePitchPid(2.0f, 0.0f, 0.0f, 200.0f, 50.0f);

// // ── Inner loop: rate → torque demand (1000 Hz) ───────────────────
// // static PIDController rateRollPid(1.80f, 0.00f, 0.03f, 300, FC_PID_I_LIMIT);
// // static PIDController ratePitchPid(1.80f, 0.00f, 0.03f, 300, FC_PID_I_LIMIT);
// // static PIDController rateYawPid(3.3f, 0.00, 0.0f, 250, FC_PID_I_LIMIT);
// static PIDController rateRollPid (1.35f, 0.005f, 0.03f, 300.0f, FC_PID_I_LIMIT);
// static PIDController ratePitchPid(1.40f, 0.010f, 0.03f, 300.0f, FC_PID_I_LIMIT);
// static PIDController rateYawPid  (2.00f, 0.000f, 0.00f, 250.0f, FC_PID_I_LIMIT);

// static PIDController rateYawPid(0.0f, 0.00, 0.0f, 250, FC_PID_I_LIMIT);
// static PIDController rateRollPid(0.0f, 0.00f, 0.00f, 300, FC_PID_I_LIMIT);

// ── Outer loop ───────────────────────────────────────────────────────────
// Pitch fine, roll has 3.9 Hz oscillation → reduce roll angle Kp slightly
// Only change if you see visible roll wobble; leave both at 2.0 if it flies clean

static PIDController angleRollPid (1.8f, 0.0f, 0.0f, 200.0f, 50.0f);  // was 2.0
static PIDController anglePitchPid(2.0f, 0.0f, 0.0f, 200.0f, 50.0f);  // unchanged

// ── Inner loop ───────────────────────────────────────────────────────────
// Roll Kp: 1.35 → 1.45  (slow rise, post-peak ringing, D/P ratio)
// Pitch Kp: keep 1.40   (already good at 9% overshoot, 52ms rise)
// Kd: keep 0.03         (small in absolute terms, not the problem)
// Ki: keep as-is        (1% of limit used — healthy, not winding up)

static PIDController rateRollPid (1.45f, 0.005f, 0.03f, 300.0f, FC_PID_I_LIMIT);
static PIDController ratePitchPid(1.40f, 0.010f, 0.03f, 300.0f, FC_PID_I_LIMIT);
static PIDController rateYawPid  (2.00f, 0.000f, 0.00f, 250.0f, FC_PID_I_LIMIT);

// ── PID setpoints ─────────────────────────────────────────────────
static float DesiredAngleRoll = 0.0f;
static float DesiredAnglePitch = 0.0f;
static float DesiredRateRoll = 0.0f;
static float DesiredRatePitch = 0.0f;
static float DesiredRateYaw = 0.0f;

// ── Inner loop torque outputs (fed into mixer) ────────────────────
static float R = 0.0f; // roll torque
static float P = 0.0f; // pitch torque
static float Y = 0.0f; // yaw torque

// ── WiFi-bridge gain mirrors (written by GCS, applied on control receive) ─
float roll_rate_pid[3] = {0, 0, 0}; // [Kp, Ki, Kd]
float pitch_rate_pid[3] = {0, 0, 0};
float yaw_rate_pid[3] = {0, 0, 0};
float angle_p[2] = {0, 0}; // [roll Kp, pitch Kp]

// ── WiFi-bridge EKF noise mirrors ────────────────────────────────
float wb_gyroNoise = 0;
float wb_gyroBias = 0;
float wb_accelNoise = 0;

// ── Arming debounce state ─────────────────────────────────────────
static uint32_t armInRangeSinceMs = 0;
static uint32_t armOutRangeSinceMs = 0;

// ================================================================
// SECTION 7 — SENSOR FUSION (AMEKF)
// ================================================================

AMEKF amekf;
AMEKF::Config amekf_cfg;

// ================================================================
// SECTION 8 — SENSOR DATA
// ================================================================

Vector3f gyro_data;        // raw gyro output (deg/s or rad/s per amekf_cfg.gyroInDegrees)
Vector3f accel_data;       // raw accel output (g)
Vector3f angle_imu;        // roll/pitch from accel, yaw from tilt-compensated mag (display)
Vector3f angle_ekf;        // roll/pitch/yaw from AMEKF (used in control)
Vector3f mag_data;         // calibrated magnetometer vector
Vector3f accel_world;      // linear accel in world NED frame (m/s²), gravity removed
Vector3f gyro_biasRemoved; // ⚠ UNUSED — declared and zero-init'd, never written during flight / not really

// ── Gyro filter signals ───────────────────────────────────────────────────
Vector3f gyro_filtered; // P measurement  — PT1@50Hz  (fast, low lag)
Vector3f gyro_dterm;    // D measurement  — notch@80Hz → PT1@20Hz

// ── P-path filters (one PT1 per axis) ────────────────────────────────────
PT1Filter gfRoll, gfPitch, gfYaw;

// ── D-path filters: notch then PT1 ───────────────────────────────────────
BiquadNotch notchRoll, notchPitch, notchYaw;
PT1Filter gfDRoll, gfDPitch, gfDYaw;

// ================================================================
// SECTION 9 — ALTITUDE / POSITION (Baro + IMU Kalman filter)
// ================================================================

PosKF altKF;
PosKF::Config altCfg;

SemaphoreHandle_t i2cMutex = nullptr; // shared I2C bus mutex (owned by Core 1 tasks)

float homeBaroAlt = 0.0f; // ground-level baro altitude captured at boot (m)

// Static accelerometer norm measured after calibration (≈ 1.0 g)
// Used for gravity cancellation in getLinearAccelWorldNED().
float accelReferenceNorm = 1.0;

// Latest baro reading (populated inside fc_update_position_baro)
float baro_altitude = 0,
      baro_pressure = 0,
      baro_temp = 0;

BaroSample baroSample;
float position_z_measurement = 0.0f; // homeBaroAlt − baro_altitude (NED depth, m)

// ================================================================
// SECTION 10 — LOOP TIMING
// ================================================================

// ── Main loop dt guard ────────────────────────────────────────────
static uint32_t prevMicros = 0;
static float dt = 0.0f; // ⚠ UNUSED — computed every loop, never consumed

// ── Gyro / AMEKF predict (1000 Hz) ───────────────────────────────
static uint32_t prevGyroMicros = 0;
static float dtGyro = 1.0f / (float)FC_GYRO_HZ;

// ── Accelerometer / AMEKF correction (500 Hz) ────────────────────
static uint32_t prevAccelMicros = 0;
static float dtAccel = 1.0f / (float)FC_ACCEL_HZ;

// ── Outer attitude PID (250 Hz) ───────────────────────────────────
// ⚠ UNUSED — dtOuter is computed but fc_update_outer_loop() is disabled
static uint32_t prevOuterMicros = 0;
static float dtOuter = 1.0f / (float)FC_OUTER_LOOP_HZ;

// ── Inner rate PID (1000 Hz) ──────────────────────────────────────
static uint32_t prevInnerMicros = 0;
static float dtInner = 1.0f / (float)FC_INNER_LOOP_HZ;

// ── Magnetometer update (100 Hz) ─────────────────────────────────
static uint32_t prevMagMicros = 0;

// ── RC input (100 Hz) ────────────────────────────────────────────
static uint32_t prevRcMicros = 0;

// ── Motor output (490 Hz) ────────────────────────────────────────
static uint32_t prevMotorMicros = 0;

// ── Barometric update (50 Hz) ────────────────────────────────────
static uint32_t prevBaroMicros = 0;

// ── Altitude control (reserved — timer declared, never used in loop) ─
static uint32_t prevAltiMicros = 0; // ⚠ UNUSED

// ── Telemetry snapshot (200 Hz) ──────────────────────────────────
static uint32_t last_telem_us = 0;
static constexpr uint32_t TELEM_INTERVAL_US = 5000; // 5 000 µs = 200 Hz

// ── Serial debug print (200 Hz) ──────────────────────────────────
uint32_t last_print = 0;

// ── Miscellaneous ─────────────────────────────────────────────────
static volatile uint32_t loopOverrunCount = 0; // ⚠ UNUSED — never incremented

// ⚠ UNUSED — declared globally, never assigned or read
uint32_t last_attitudeUpdate = 0;

// ================================================================
// SECTION 11 — FORWARD DECLARATIONS
// ================================================================

// ── Arming / disarming ────────────────────────────────────────────
static void fc_reset_pids(void);
static void fc_write_motors_min(void);
static void fc_disarm(uint8_t reason);
static bool fc_arm_switch_active(uint16_t pulse);
static void fc_update_arm_state(uint16_t switchPulse, uint16_t throttlePulse);
static void fc_reset_gyro_filters(void);

// ── RC input ─────────────────────────────────────────────────────
static inline uint16_t fc_map_u16(uint16_t x,
                                  uint16_t in_min, uint16_t in_max,
                                  uint16_t out_min, uint16_t out_max);
static void fc_read_rx(void);

// ── Attitude estimation ───────────────────────────────────────────
static void fc_update_attitude_gyro(void);
static void fc_update_attitude_accel(void);
static void fc_update_attitude_mag(void);

// ── Altitude estimation ───────────────────────────────────────────
static void fc_update_position_baro(void);

// ── Control loops ─────────────────────────────────────────────────
static inline float fc_constrain_f(float v, float lo, float hi);
static void fc_update_outer_loop(void);
static void fc_update_inner_loop(void);

// ── Motor mixing and output ───────────────────────────────────────
static void fc_mix_quad_x(void);
static void fc_write_motors(void);

// ── Hardware checks ───────────────────────────────────────────────
bool fc_check_mpu(void);
// bool qmc_check(void);

// ── Calibration helpers ───────────────────────────────────────────
static float calcBaroHomeAlt(int N);
static void calcMagNoise(int N);

// ── Math / physics helpers ────────────────────────────────────────
constexpr float G = 9.80665f;
Vector3f getLinearAccelWorldNED(const Quaternionf &q,
                                const Vector3f &accelBody,
                                float accelReferenceNorm);
float mpu_measure_static_norm(int count = 500);

// ================================================================
// setup()
// ================================================================

// ── WiFi telemetry float cache ───────────────────────────────────────────
// getPterm/Iterm/Dterm() return by value, so we can't register them directly.
// We write these each log tick, then wifiBridge_sendTelemetry() reads the ptrs.
#if WEB_EN
float telem_roll_P, telem_roll_I, telem_roll_D;
float telem_pitch_P, telem_pitch_I, telem_pitch_D;
float telem_yaw_P;
float telem_m1, telem_m2, telem_m3, telem_m4;
#endif

void setup()
{
    gyro_data = {0.00, 0.00, 0.00};
    accel_data = {0.00, 0.00, 0.00};
    angle_imu = {0.00, 0.00, 0.00};
    angle_ekf = {0.00, 0.00, 0.00};
    mag_data = {0.00, 0.00, 0.00};
    accel_world = {0.00, 0.00, 0.00};
    gyro_biasRemoved = {0.00, 0.00, 0.00};

    Serial.begin(115200);
    delay(500);

    // // ── Cache initial PID gains into WiFi-bridge mirror arrays ───
    roll_rate_pid[0] = rateRollPid.getKp();
    roll_rate_pid[1] = rateRollPid.getKi();
    roll_rate_pid[2] = rateRollPid.getKd();

    pitch_rate_pid[0] = ratePitchPid.getKp();
    pitch_rate_pid[1] = ratePitchPid.getKi();
    pitch_rate_pid[2] = ratePitchPid.getKd();

    yaw_rate_pid[0] = rateYawPid.getKp();
    yaw_rate_pid[1] = rateYawPid.getKi();
    yaw_rate_pid[2] = rateYawPid.getKd();

    angle_p[0] = angleRollPid.getKp();
    angle_p[1] = anglePitchPid.getKp();

    // ── ESC arm safety — must be set before any motor writes ─────
    PWM_out::setGlobalArmFlag(&motorsArmed);
    PWM_out::setDisarmedPulse(FC_ESC_MIN_US);

    // ── IMU (MPU6500 over SPI) ────────────────────────────────────
    pinMode(PIN_SPI_CS, OUTPUT);
    digitalWrite(PIN_SPI_CS, HIGH);

    mpu_init(); // handles mySPI.begin() + all registers internally
    if (!fc_check_mpu())
    {
        fc_disarm(5);
        while (1)
            delay(1000);
    }

    // mpu_run_6pos_calibration();
    // delay(10000);
    mpu_apply_calibration_offsets();
    accelReferenceNorm = mpu_measure_static_norm(500);

    // ── I2C peripherals (magnetometer + barometer) ────────────────
    Wire.begin(FC_PIN_I2C_SDA, FC_PIN_I2C_SCL);
    Wire.setClock(400000);

    // pinMode(PIN_QMC_DRDY, INPUT);
    bool qmc_init_success = qmc_init();
    delay(500);
    if (qmc_init_success)

        Serial.println("============= QMC init Success =============");

    else
    {
        Serial.println("fail");
        while (1)
            ;
    }

    if (!bmp180_init())
    {
        fc_disarm(5);
        Serial.println("BMP180 not found — check wiring!");
        while (1)
            delay(1000);
    }

    if (!qmc_check())
    {
        fc_disarm(5);
        Serial.println("QMC not found!");
        while (1)
            delay(1000);
    }
    // ── FreeRTOS I2C mutex + sensor tasks ─────────────────────────
    // Highest-priority task spawned last so the RTOS scheduler does not
    // immediately context-switch on the first spawn.
    i2cMutex = xSemaphoreCreateMutex();
    configASSERT(i2cMutex);

    magTask_start(i2cMutex);  // sensor lives on Core 1
    baroTask_start(i2cMutex); // priority 6, Core 1

    // for (;;)
    // {
    //     int mx, my, mz;
    //     if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)))
    //     {
    //         qmc_read_raw(mx, my, mz); // ← sensor frame, no remap, raw counts
    //         xSemaphoreGive(i2cMutex);
    //         Serial.printf("%d,%d,%d\n", mx, my, mz);
    //     }
    //     vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz
    // }

    // delay(10000);

    // ── AMEKF configuration ───────────────────────────────────────
    amekf.begin();
    amekf_cfg = amekf.getConfig();
    amekf_cfg.gyroInDegrees = true; // MPU gyro outputs deg/s
    amekf_cfg.accelReferenceNorm = accelReferenceNorm;

    amekf_cfg.accelNoise = 0.10f;         // R measure — trust in accel gravity correction
    amekf_cfg.gyroNoiseRad = 0.041f;      // Q angle   — uncertainty added per gyro step
    amekf_cfg.gyroBiasNoiseRad = 0.0005f; // Q bias    — how fast bias is allowed to drift
    amekf_cfg.initialAngleStdRad = 10.0f * DEG2RAD_F;
    amekf_cfg.initialBiasStdRad = 1.0f * DEG2RAD_F;
    amekf_cfg.accelGravitySign = -1.0f; // flat stationary Z reads -1 g

    // Hard-reject only extreme accel (crash / motor vibration)
    amekf_cfg.accelHardRejectMin = 0.50f;
    amekf_cfg.accelHardRejectMax = 1.60f;
    amekf_cfg.accelInflationGain = 5.0f;

    // calcMagNoise(1000); // un-comment to measure live mag noise
    amekf_cfg.magNoise = 0.20f;
    amekf_cfg.magReferenceNorm = 0.466330f; // calibrated magnitude
    amekf_cfg.magGateMin = 0.70f;           // reject below 70% of reference
    amekf_cfg.magGateMax = 1.30f;           // reject above 130% of reference
    amekf.setConfig(amekf_cfg);

    // ── Average mag readings before init ─────────────────────────────────────
    const int MAG_INIT_SAMPLES = 32;
    Vector3f mag_accum = {0.0f, 0.0f, 0.0f};

    for (int i = 0; i < MAG_INIT_SAMPLES; i++)
    {
        Vector3f m;
        qmc_read_data(&m);
        mag_accum.x += m.x;
        mag_accum.y += m.y;
        mag_accum.z += m.z;
        delay(20); // pace it to your ODR (20 ms → 50 Hz)
    }

    Vector3f mag_data = {
        mag_accum.x / MAG_INIT_SAMPLES,
        mag_accum.y / MAG_INIT_SAMPLES,
        mag_accum.z / MAG_INIT_SAMPLES};

    // ── Prime IIR filter before EKF initialisation ───────────────
    for (int i = 0; i < 20; i++)
    {
        mpu_read_accel(&accel_data, &angle_imu);
        delay(5); // 20 × 5 ms = 100 ms — enough for 8 Hz IIR to settle
    }

    // ── Prime IIR filter before EKF initialisation ───────────────
    for (int i = 0; i < 20; i++)
    {
        qmc_read_data(&mag_data); // fresh mag reading too
        // qmc_compute_heading(&mag_data);
        delay(50); // 20 × 50 ms = 1s
    }

    int initResult = amekf.initFromIMU(accel_data, mag_data);
    if (initResult != 0)
    {
        Serial.print("WARN: initFromIMU failed, code: ");
        Serial.println(initResult);
    }

    // ── Barometric home altitude ──────────────────────────────────
    homeBaroAlt = calcBaroHomeAlt(5);
    Serial.print("Home baro alt set: ");
    Serial.println(homeBaroAlt);
    delay(1000);

    // ── Altitude Kalman filter configuration ──────────────────────
    altCfg.R = 0.034f;
    altCfg.sigmaA = 0.05f;
    altCfg.sigmaB = 1.0e-3f;
    altCfg.mahaGate = 9.0f; // 3σ Mahalanobis gate
    altCfg.biasClamp = 1.0f;
    // altCfg.sigmaBaro = 0.25f;

    altKF.setConfig(altCfg);
    altKF.begin(0); // ground = 0 m AGL

    // Mean: 308.223389, Variance: 0.097412, StdDev: 0.312108
    // Home baro alt set: 308.22
    prevMicros = micros();

    // ── WiFi-bridge EKF noise mirrors ────────────────────────────
    wb_gyroNoise = amekf_cfg.gyroNoiseRad;
    wb_gyroBias = amekf_cfg.gyroBiasNoiseRad;
    wb_accelNoise = amekf_cfg.accelNoise;

    // ── RC inputs ─────────────────────────────────────────────────
    rx_aileron.begin();
    rx_throttle.begin();
    rx_arm.begin();
    rx_elevator.begin();
    rx_aux.begin();
    rx_rudder.begin();

    // ── ESC outputs — attach while disarmed; PWM_out forces FC_ESC_MIN_US ─
    esc_pwm1.attach(FC_PIN_ESC_FRONT_RIGHT, FC_ESC_MIN_US, FC_ESC_MAX_US);
    esc_pwm2.attach(FC_PIN_ESC_FRONT_LEFT, FC_ESC_MIN_US, FC_ESC_MAX_US);
    esc_pwm3.attach(FC_PIN_ESC_BACK_LEFT, FC_ESC_MIN_US, FC_ESC_MAX_US);
    esc_pwm4.attach(FC_PIN_ESC_BACK_RIGHT, FC_ESC_MIN_US, FC_ESC_MAX_US);

    // ── P-path: light PT1 at 50Hz ─────────────────────────────────────────────
    // Hardware DLPF (41Hz) already handles broadband noise.
    // This PT1 smooths aliasing artefacts above that without adding much lag.
    gfRoll.init(50.0f, (float)FC_INNER_LOOP_HZ);
    gfPitch.init(50.0f, (float)FC_INNER_LOOP_HZ);
    gfYaw.init(50.0f, (float)FC_INNER_LOOP_HZ);

    gfRoll.reset(gyro_data.x);
    gfPitch.reset(gyro_data.y);
    gfYaw.reset(gyro_data.z);

    // ── D-path: biquad notch at 80Hz Q=1.5 ────────────────────────────────────
    // Q=1.5 → -3dB bandwidth = 53Hz → covers 53–107Hz.
    // Catches prop-frequency resonance across the full throttle range.
    notchRoll.init(80.0f, 1.5f, (float)FC_INNER_LOOP_HZ);
    notchPitch.init(80.0f, 1.5f, (float)FC_INNER_LOOP_HZ);
    notchYaw.init(80.0f, 1.5f, (float)FC_INNER_LOOP_HZ);

    notchRoll.reset(gyro_data.x);
    notchPitch.reset(gyro_data.y);
    notchYaw.reset(gyro_data.z);

    // ── D-path: aggressive PT1 at 20Hz after notch ────────────────────────────
    // Kills broadband noise above the notch. D never needs bandwidth above 20Hz.
    gfDRoll.init(20.0f, (float)FC_INNER_LOOP_HZ);
    gfDPitch.init(20.0f, (float)FC_INNER_LOOP_HZ);
    gfDYaw.init(20.0f, (float)FC_INNER_LOOP_HZ);

    gfDRoll.reset(gyro_data.x);
    gfDPitch.reset(gyro_data.y);
    gfDYaw.reset(gyro_data.z);

    // ── PID internal D filter: transparent (external chain handles it) ─────────
    rateRollPid.setDerivativeFilterTau(0.003f); // ~53Hz — nearly open
    ratePitchPid.setDerivativeFilterTau(0.003f);
    rateYawPid.setDerivativeFilterTau(0.003f);

    rateRollPid.setBackCalculation(0.1f); // enable when Ki > 0
#if WEB_EN
    // ── Telemetry: ESP32 → laptop (must match TELEM_NAMES order in bridge.py) ──
    wifiBridge_addTelemetry("gx", &gyro_filtered.x);
    wifiBridge_addTelemetry("gy", &gyro_filtered.y);
    wifiBridge_addTelemetry("gz", &gyro_filtered.z);
    wifiBridge_addTelemetry("roll", &angle_ekf.x);
    wifiBridge_addTelemetry("pitch", &angle_ekf.y);
    wifiBridge_addTelemetry("d_angle_roll", &DesiredAngleRoll);
    wifiBridge_addTelemetry("d_angle_pitch", &DesiredAnglePitch);
    wifiBridge_addTelemetry("d_rate_roll", &DesiredRateRoll);
    wifiBridge_addTelemetry("d_rate_pitch", &DesiredRatePitch);
    wifiBridge_addTelemetry("d_rate_yaw", &DesiredRateYaw);
    wifiBridge_addTelemetry("roll_P", &telem_roll_P); // cache — updated each tick
    wifiBridge_addTelemetry("roll_I", &telem_roll_I);
    wifiBridge_addTelemetry("roll_D", &telem_roll_D);
    wifiBridge_addTelemetry("pitch_P", &telem_pitch_P);
    wifiBridge_addTelemetry("pitch_I", &telem_pitch_I);
    wifiBridge_addTelemetry("pitch_D", &telem_pitch_D);
    wifiBridge_addTelemetry("yaw_P", &telem_yaw_P);
    wifiBridge_addTelemetry("m1", &telem_m1);
    wifiBridge_addTelemetry("m2", &telem_m2);
    wifiBridge_addTelemetry("m3", &telem_m3);
    wifiBridge_addTelemetry("m4", &telem_m4);

    // ── Controls: laptop → ESP32 (names must match JS keys in controls-config.js) ──
    wifiBridge_addControl("roll_rate_p", &roll_rate_pid[0]);
    wifiBridge_addControl("roll_rate_i", &roll_rate_pid[1]);
    wifiBridge_addControl("roll_rate_d", &roll_rate_pid[2]);
    wifiBridge_addControl("pitch_rate_p", &pitch_rate_pid[0]);
    wifiBridge_addControl("pitch_rate_i", &pitch_rate_pid[1]);
    wifiBridge_addControl("pitch_rate_d", &pitch_rate_pid[2]);
    wifiBridge_addControl("yaw_rate_p", &yaw_rate_pid[0]);
    wifiBridge_addControl("yaw_rate_i", &yaw_rate_pid[1]);
    wifiBridge_addControl("yaw_rate_d", &yaw_rate_pid[2]);

    wifiBridge_init(); // always last — spawns Core 1 task
#endif
    Battery_init();
}
uint32_t lastLogUs = 0;

// ================================================================
// loop()
// ================================================================

void loop()
{
    // in loop(), top of every iteration
    static bool prevArmed = false;
    if (prevArmed && !motorsArmed)
    {
        Serial.printf("[DISARMED-SILENT] motorsArmed went false outside fc_disarm! "
                      "throttle=%u armPulse=%u now=%lu\n",
                      throttle, armPulse, (unsigned long)millis());
    }
    prevArmed = motorsArmed;

    uint32_t loopStartUs = micros();

    // ── Main loop dt guard ────────────────────────────────────────
    // ⚠ NOTE: dt is computed here but never passed to any function.
    dt = (loopStartUs - prevMicros) / 1000000.0f;
    prevMicros = loopStartUs;
    if (dt <= 0.0f || dt > FC_MAX_ALLOWED_DT_S)
        dt = 1.0f / (float)FC_MAIN_LOOP_HZ;

    // ── 1000 Hz: gyro read + AMEKF predict ───────────────────────
    if (loopStartUs - prevGyroMicros >= FC_GYRO_US)
    {
        dtGyro = (loopStartUs - prevGyroMicros) / 1000000.0f;
        if (dtGyro > 0.005f)
            dtGyro = 0.005f; // floor at 200 Hz if preempted badly

        prevGyroMicros = loopStartUs;
        fc_update_attitude_gyro();

        // ── 500 Hz: accel read + AMEKF correct ───────────────────
        if (loopStartUs - prevAccelMicros >= FC_ACCEL_US)
        {
            dtAccel = (loopStartUs - prevAccelMicros) / 1000000.0f;
            if (dtAccel > FC_MAX_ALLOWED_DT_S)
                dtAccel = FC_MAX_ALLOWED_DT_S;

            prevAccelMicros = loopStartUs;
            fc_update_attitude_accel();

            accel_world = getLinearAccelWorldNED(amekf.getQuaternion(), accel_data, accelReferenceNorm);
            altKF.predict(accel_world.z, dtAccel);
        }

        angle_ekf = amekf.getEulerDeg();

        // ── 500 Hz: outer attitude PID ────────────────────────────
        // ⚠ BUG: fc_update_outer_loop() is DISABLED.
        //   DesiredRateRoll, DesiredRatePitch, and DesiredRateYaw
        //   remain 0.0f — pilot has no roll, pitch, or yaw control.
        //   See BUG LOG at bottom of file.
        if (loopStartUs - prevOuterMicros >= FC_OUTER_LOOP_US)
        {
            dtOuter = (loopStartUs - prevOuterMicros) / 1000000.0f;
            if (dtOuter > FC_MAX_ALLOWED_DT_S)
                dtOuter = FC_MAX_ALLOWED_DT_S;

            prevOuterMicros = loopStartUs;
            fc_update_outer_loop();
        }

        // ── 1000 Hz: inner rate PID + motor mix ──────────────────
        uint32_t nowPostGyro = micros();
        dtInner = (nowPostGyro - prevInnerMicros) / 1000000.0f;
        if (dtInner <= 0.0f || dtInner > FC_MAX_ALLOWED_DT_S)
            dtInner = 1.0f / (float)FC_INNER_LOOP_HZ;

        prevInnerMicros = nowPostGyro;
        fc_update_inner_loop();
        fc_mix_quad_x();
    }

    // ── 100 Hz: magnetometer + AMEKF mag update ──────────────────
    // Separate block — 264 µs I2C read must not charge the 1 kHz gyro budget.
    // DRDY check inside fc_update_attitude_mag() provides exact gating;
    // the 10 ms timer prevents calling faster than the sensor delivers.
    if (loopStartUs - prevMagMicros >= 10000)
    {
        prevMagMicros = loopStartUs;
        // Serial.println("trying updt_atti_mag");
        fc_update_attitude_mag();
    }

    // ── 100 Hz: RC input + signal-loss + arm state machine ───────
    if (loopStartUs - prevRcMicros >= FC_RC_US) // FC_RC_US = 10 000 µs → 100 Hz
    {
        prevRcMicros = loopStartUs;
        fc_read_rx();
    }

    // ── 490 Hz: motor PWM output ──────────────────────────────────
    if (loopStartUs - prevMotorMicros >= FC_MOTOR_US)
    {
        prevMotorMicros = loopStartUs;
        fc_write_motors();
    }

    // ── 50 Hz: barometric altitude update ────────────────────────
    // xQueueReceive timeout = 0 — never blocks the FC loop.
    // // baro_task state machine on Core 1 handles all I2C and conversion timing.
    if (loopStartUs - prevBaroMicros >= FC_BARO_US)
    {
        prevBaroMicros = loopStartUs;
        // fc_update_position_baro();
    }

    // ── 10 Hz: battery voltage IIR update ────────────────────────
    if (loopStartUs - prevBatteryMicros >= 100000) // 100 ms
    {
        prevBatteryMicros = loopStartUs;
        Battery_update();
        BatteryVoltage = Battery_getVoltage();
    }
#if WEB_EN
    // ── 250 Hz telemetry send ────────────────────────────────────────────
    if (loopStartUs - lastLogUs >= 4000)
    {
        lastLogUs = loopStartUs;

        // Refresh caches — getPterm() returns float by value, not float&
        telem_roll_P  = rateRollPid.getPterm();
        telem_roll_I  = rateRollPid.getIterm();
        telem_roll_D  = rateRollPid.getDterm();
        telem_pitch_P = ratePitchPid.getPterm();
        telem_pitch_I = ratePitchPid.getIterm();
        telem_pitch_D = ratePitchPid.getDterm();
        telem_yaw_P   = rateYawPid.getPterm();
        telem_m1      = (float)pulse_esc1;
        telem_m2      = (float)pulse_esc2;
        telem_m3      = (float)pulse_esc3;
        telem_m4      = (float)pulse_esc4;

        // Non-blocking seqlock write + notify Core 1 — costs < 2 µs
        wifiBridge_sendTelemetry();
    }

    // ── Control receive — check every loop iteration, ~0 cost if nothing new ──
    if (wifiBridge_receiveControl())
    {
        // Bridge has already written roll_rate_pid[], pitch_rate_pid[], etc.
        // Just push the new values into the PID objects.
        rateRollPid.setGains( roll_rate_pid[0],  roll_rate_pid[1],  roll_rate_pid[2]);
        ratePitchPid.setGains(pitch_rate_pid[0], pitch_rate_pid[1], pitch_rate_pid[2]);
        rateYawPid.setGains(  yaw_rate_pid[0],   yaw_rate_pid[1],   yaw_rate_pid[2]);
    }
#endif

    // amekf_cfg.gyroNoiseRad = wb_gyroNoise;
    // amekf_cfg.gyroBiasNoiseRad = wb_gyroBias;
    // amekf_cfg.accelNoise = wb_accelNoise;
    // amekf.setConfig(amekf_cfg);
    // }

    // // commented out in flight
    // // ── 10 Hz: serial debug print ────────────────────────────────
    // if (loopStartUs - last_print > 10000)
    // {
    //     last_print = micros();
    // }
    static uint32_t lastLogUs = 0;
    if (loopStartUs - lastLogUs >= 4000) // 250 Hz
    {
        lastLogUs = loopStartUs;
        // Serial.printf(
        //     "%lu,"                   // timestamp µs
        //     "%.2f,%.2f,%.2f,"        // gyro xyz (actual rates)
        //     "%.3f,%.3f,"             // ekf roll, pitch (actual angles)
        //     "%.2f,%.2f,"             // desired angle roll, pitch
        //     "%.2f,%.2f,%.2f,"        // desired rate roll, pitch, yaw
        //     "%.2f,%.2f,%.2f,"        // roll PID: P I D terms
        //     "%.2f,%.2f,%.2f,"        // pitch PID: P I D terms
        //     "%.2f,"                  // yaw PID: P term only
        //     "%.0f,%.0f,%.0f,%.0f\n", // motor pulses
        //     micros(),
        //     gyro_filtered.x, gyro_filtered.y, gyro_filtered.z,
        //     angle_ekf.x, angle_ekf.y,
        //     DesiredAngleRoll, DesiredAnglePitch,
        //     DesiredRateRoll, DesiredRatePitch, DesiredRateYaw,
        //     rateRollPid.getPterm(), rateRollPid.getIterm(), rateRollPid.getDterm(),
        //     ratePitchPid.getPterm(), ratePitchPid.getIterm(), ratePitchPid.getDterm(),
        //     rateYawPid.getPterm(),
        //     pulse_esc1, pulse_esc2, pulse_esc3, pulse_esc4);

        Serial.print(aileron);
        Serial.print('\t');
        Serial.print(elevator);
        Serial.print('\t');
        Serial.print(throttle);
        Serial.print('\t');
        Serial.print(rudder);
        Serial.print('\t');
        Serial.print(armPulse);
        Serial.print('\t');
        Serial.print(motorsArmed);
        Serial.print('\n');

        // Serial.print(mag_data.x);
        // Serial.print('\t');
        // Serial.print(mag_data.y);
        // Serial.print('\t');
        // Serial.print(mag_data.z);
        // Serial.print('\t');
        // Serial.println(angle_imu.z);
    }
}

// ================================================================
// SECTION 12 — ARMING / DISARMING
// ================================================================

static void fc_reset_pids(void)
{
    rateRollPid.reset();
    ratePitchPid.reset();
    rateYawPid.reset();

    angleRollPid.reset();
    anglePitchPid.reset();
}

static void fc_write_motors_min(void)
{
    esc_pwm1.writeMicroseconds(FC_ESC_MIN_US);
    esc_pwm2.writeMicroseconds(FC_ESC_MIN_US);
    esc_pwm3.writeMicroseconds(FC_ESC_MIN_US);
    esc_pwm4.writeMicroseconds(FC_ESC_MIN_US);
}

static void fc_disarm(uint8_t reason)
{
    if (motorsArmed)
    {
        motorsArmed = false;
        fc_write_motors_min();
        fc_reset_pids();

        fc_reset_gyro_filters();

        pulse_esc1 = FC_ESC_MIN_US;
        pulse_esc2 = FC_ESC_MIN_US;
        pulse_esc3 = FC_ESC_MIN_US;
        pulse_esc4 = FC_ESC_MIN_US;

        Serial.printf("[fc_disarm] CODE: %u\n", reason);
    }
}

// Call this wherever you handle the disarmed → armed transition:
static void fc_reset_gyro_filters(void)
{
    // Reset P-path
    gfRoll.reset(gyro_data.x);
    gfPitch.reset(gyro_data.y);
    gfYaw.reset(gyro_data.z);

    // Reset D-path — notch first, then PT1
    notchRoll.reset(gyro_data.x);
    notchPitch.reset(gyro_data.y);
    notchYaw.reset(gyro_data.z);

    gfDRoll.reset(gyro_data.x);
    gfDPitch.reset(gyro_data.y);
    gfDYaw.reset(gyro_data.z);
}

static bool fc_arm_switch_active(uint16_t p)
{
    return (p >= FC_ARM_SWITCH_LO_US && p <= FC_ARM_SWITCH_HI_US);
}

static void fc_update_arm_state(uint16_t switchPulse, uint16_t throttlePulse)
{
    uint32_t now = millis();
    bool switchOn = fc_arm_switch_active(switchPulse);
    bool throttleLow = (throttlePulse < FC_ESC_IDLE_US);

    if (!switchOn)
    {
        armInRangeSinceMs = 0;

        if (armOutRangeSinceMs == 0)
            armOutRangeSinceMs = now;

        if (motorsArmed && (now - armOutRangeSinceMs >= FC_DISARM_DEBOUNCE_MS))
            fc_disarm(2);

        return;
    }

    // switch is ON
    armOutRangeSinceMs = 0;

    if (!throttleLow)
    {
        armInRangeSinceMs = 0;
        return;
    }

    if (armInRangeSinceMs == 0)
        armInRangeSinceMs = now;

    if (!motorsArmed && (now - armInRangeSinceMs >= FC_ARM_DEBOUNCE_MS))
    {
        fc_reset_pids();
        motorsArmed = true;
        Serial.printf("[ARMED] now=%lu switchPulse=%u throttlePulse=%u\n",
                      (unsigned long)now, switchPulse, throttlePulse);
    }
}

// ================================================================
// SECTION 13 — RC INPUT
// ================================================================

// Integer linear map with input clamping.
static inline uint16_t fc_map_u16(uint16_t x,
                                  uint16_t in_min, uint16_t in_max,
                                  uint16_t out_min, uint16_t out_max)
{
    if (x < in_min)
        x = in_min;
    if (x > in_max)
        x = in_max;

    uint32_t num = (uint32_t)(x - in_min) * (uint32_t)(out_max - out_min);
    uint32_t den = (uint32_t)(in_max - in_min);

    if (den == 0)
        return out_min;

    return (uint16_t)(out_min + (num / den));
}

static void fc_read_rx(void)
{
    // ── DRDY gate ─────────────────────────────────────────────────
    // Bitwise | (not ||): all flags consumed in one pass, no short-circuit.
    bool anyNew = rx_aileron.hasNewData() |
                  rx_throttle.hasNewData() |
                  rx_elevator.hasNewData() |
                  rx_rudder.hasNewData() |
                  rx_arm.hasNewData() |
                  rx_aux.hasNewData();

    if (!anyNew)
        return;

    // ── Raw reads ─────────────────────────────────────────────────
    rawAil = rx_aileron.getPulse();
    rawThr = rx_throttle.getPulse();
    rawEle = rx_elevator.getPulse();
    rawRud = rx_rudder.getPulse();
    rawArm = rx_arm.getPulse();
    rawAux = rx_aux.getPulse();

    // ── Normalise ─────────────────────────────────────────────────
    aileron = (rawAil <= FC_RC_AIL_MID_US)
                  ? fc_map_u16(rawAil, FC_RC_AIL_MIN_US, FC_RC_AIL_MID_US, FC_RC_MIN_US, FC_RC_MID_US)
                  : fc_map_u16(rawAil, FC_RC_AIL_MID_US, FC_RC_AIL_MAX_US, FC_RC_MID_US, FC_RC_MAX_US);

    elevator = (rawEle <= FC_RC_ELE_MID_US)
                   ? fc_map_u16(rawEle, FC_RC_ELE_MIN_US, FC_RC_ELE_MID_US, FC_RC_MIN_US, FC_RC_MID_US)
                   : fc_map_u16(rawEle, FC_RC_ELE_MID_US, FC_RC_ELE_MAX_US, FC_RC_MID_US, FC_RC_MAX_US);

    throttle = (rawThr <= FC_RC_THR_MID_US)
                   ? fc_map_u16(rawThr, FC_RC_THR_MIN_US, FC_RC_THR_MID_US, FC_RC_MIN_US, FC_RC_MID_US)
                   : fc_map_u16(rawThr, FC_RC_THR_MID_US, FC_RC_THR_MAX_US, FC_RC_MID_US, FC_RC_MAX_US);

    rudder = (rawRud <= FC_RC_RUD_MID_US)
                 ? fc_map_u16(rawRud, FC_RC_RUD_MIN_US, FC_RC_RUD_MID_US, FC_RC_MIN_US, FC_RC_MID_US)
                 : fc_map_u16(rawRud, FC_RC_RUD_MID_US, FC_RC_RUD_MAX_US, FC_RC_MID_US, FC_RC_MAX_US);

    armPulse = rawArm;
    auxPulse = rawAux;

    // ── Signal-loss + arm state machine ──────────────────────────
    // isAlive() is independent of hasNewData() — checks lastValidPulseUs,
    // not the consumed flag — so calling it here is safe and correct.
    bool rxAlive = rx_throttle.isAlive() &&
                   rx_aileron.isAlive() &&
                   rx_elevator.isAlive() &&
                   rx_rudder.isAlive() &&
                   rx_arm.isAlive();

    if (!rxAlive)
    {
        // rxSignalPresent = false;
        // rxWasLost = true; // latch: reconnect alone won't re-arm
        // fc_disarm(1);
        return;
    }

    rxSignalPresent = true;

    // Latch clears only when arm switch returns to disarmed position.
    if (rxWasLost && armPulse < FC_RC_MID_US)
        rxWasLost = false;

    if (!rxWasLost)
        ;
    fc_update_arm_state(armPulse, throttle);
}

// ================================================================
// SECTION 14 — ATTITUDE ESTIMATION
// ================================================================

static void fc_update_attitude_gyro(void)
{
    mpu_read_gyro(&gyro_data);

    // EKF predict always receives raw gyro — it subtracts its own bias
    amekf.predict(gyro_data, dtGyro);
    att_history_push(esp_timer_get_time(), amekf.getQuaternion());

    // ── P-path: hardware DLPF(41Hz) → PT1@50Hz ───────────────────────────
    gyro_filtered.x = gfRoll.update(gyro_data.x);
    gyro_filtered.y = gfPitch.update(gyro_data.y);
    gyro_filtered.z = gfYaw.update(gyro_data.z);

    // ── D-path: P-signal → notch@80Hz → PT1@20Hz ─────────────────────────
    // Feed notch from gyro_filtered (already hardware+50Hz filtered),
    // so the notch only sees the residual 80Hz that slipped through,
    // not raw broadband noise from ADC.
    gyro_dterm.x = gfDRoll.update(notchRoll.update(gyro_filtered.x));
    gyro_dterm.y = gfDPitch.update(notchPitch.update(gyro_filtered.y));
    gyro_dterm.z = gfDYaw.update(notchYaw.update(gyro_filtered.z));
}

// Called at 500 Hz — accel read feeds the AMEKF correction step.
static void fc_update_attitude_accel(void)
{
    mpu_read_accel(&accel_data, &angle_imu);
    amekf.updateAccel(accel_data);

    // Serial.print((int)(throttle-1000)/10);
    // Serial.print('\t');
    // Serial.print(micros());
    // Serial.print('\n');

    // Serial.print((int)motorsArmed);
    // Serial.print('\t');
    // Serial.println((int)rxSignalPresent);
}
// ════════════════════════════════════════════════════════════════════════════
// 100 Hz heading update — called from loop() on Core 0
//
// Timing model (ODR=100Hz, OSR=512):
//   ┌──────── 10 ms ODR window ────────┐
//   │  averaging                       │ DRDY↑
//   └──────────────────────────────────┘
//         ↑ effective sample instant
//       (window centre, 5 ms before DRDY)
//
// We recover the quaternion from that instant via the ring buffer.
// ════════════════════════════════════════════════════════════════════════════

extern QueueHandle_t magDataQueue; // owned by mag_task.cpp

static void fc_update_attitude_mag(void)
{
    MagPacket pkt;
    if (!xQueueReceive(magDataQueue, &pkt, 0))
        return; // nothing new this tick

    // Update the EKF mag observation with the freshly calibrated vector.
    mag_data = pkt.mag;
    amekf.updateMag(mag_data);

    // Recover the attitude that was current when the field was sampled.
    // At ODR=100Hz the window centre is 5 ms before DRDY.
    static constexpr int32_t ODR_CENTRE_US = 5000;
    const Quaternionf q_at_sense =
        lookupQuaternion(pkt.timestamp_us - ODR_CENTRE_US);

    // Optional latency monitor — enable to verify pipeline timing.
#if 0
    const int32_t age_us = (int32_t)(esp_timer_get_time() - pkt.timestamp_us);
    Serial.printf("[Data] %f\t %f't %f\n", mag_data.x, mag_data.y, mag_data.z);

    Serial.printf("[MAG] age=%d us  q_dt=%lld us\n",
                  age_us,
                  llabs(pkt.timestamp_us - ODR_CENTRE_US - s_att_buf[(s_att_head - 1 + ATT_BUF_LEN) % ATT_BUF_LEN].ts_us));
#endif

    // Tilt-compensated heading → angle_imu.z (degrees, NED, −180 … +180].
    // Returns false on norm gate or near-vertical attitude; last value kept.
    qmc_compute_heading(&mag_data, &q_at_sense, &angle_imu);
}

// ================================================================
// SECTION 15 — ALTITUDE ESTIMATION
// ================================================================

// Called at 50 Hz — non-blocking queue receive (timeout = 0).
// If no new baro sample is available, the KF coasts on the IMU prediction
// step (run at 500 Hz in the accel block) until the next measurement arrives.
static void fc_update_position_baro(void)
{
    if (xQueueReceive(baroDataQueue, &baroSample, 0))
    {
        baro_altitude = baroSample.altitudeM;
        baro_pressure = baroSample.pressurePa;
        baro_temp = baroSample.tempC;

        if (!isfinite(baro_altitude))
            return;

        position_z_measurement = homeBaroAlt - baro_altitude;
        altKF.update(position_z_measurement);

        // Serial.print(position_z_measurement, 6); // raw baro depth (m)
        // Serial.print('\t');
        // Serial.println(altKF.getDepth(), 6); // x[0]: est. NED depth (m)
    }
    // No new sample: KF coasts on IMU prediction — no action needed.
}

// ================================================================
// SECTION 16 — CONTROL LOOPS
// ================================================================

static inline float fc_constrain_f(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

// Outer loop (500 Hz): pilot stick → desired angle → desired rate setpoint.
static void fc_update_outer_loop(void)
{
    DesiredAngleRoll = FC_ROLL_PITCH_ANGLE_SCALE * ((float)aileron - 1500.0f);
    DesiredAnglePitch = -FC_ROLL_PITCH_ANGLE_SCALE * ((float)elevator - 1500.0f);
    DesiredRateYaw = FC_YAW_RATE_SCALE * ((float)rudder - 1500.0f);

    DesiredAngleRoll = fc_constrain_f(DesiredAngleRoll, -FC_MAX_ANGLE_DEG, FC_MAX_ANGLE_DEG);
    DesiredAnglePitch = fc_constrain_f(DesiredAnglePitch, -FC_MAX_ANGLE_DEG, FC_MAX_ANGLE_DEG);

    bool allowIntegrator = motorsArmed && (throttle > FC_ESC_IDLE_US);

    DesiredRateRoll = angleRollPid.compute(DesiredAngleRoll, angle_ekf.x, dtOuter, allowIntegrator);
    DesiredRatePitch = anglePitchPid.compute(DesiredAnglePitch, angle_ekf.y, dtOuter, allowIntegrator);
}

// Inner loop (1000 Hz): desired rate → torque demands R, P, Y.
static void fc_update_inner_loop(void)
{
    bool allowIntegrator = motorsArmed && (throttle > FC_ESC_IDLE_US);

    // 3-arg compute(): measurement = gyro_filtered (P uses this)
    //                  d_measurement = gyro_dterm   (D uses this)
    R = rateRollPid.compute(DesiredRateRoll, gyro_filtered.x, gyro_dterm.x, dtInner, allowIntegrator);
    P = ratePitchPid.compute(DesiredRatePitch, gyro_filtered.y, gyro_dterm.y, dtInner, allowIntegrator);
    Y = rateYawPid.compute(DesiredRateYaw, gyro_filtered.z, gyro_dterm.z, dtInner, allowIntegrator);
}

// ================================================================
// SECTION 17 — MOTOR MIXING AND OUTPUT
// ================================================================

// X-frame mixing (viewed from above, NED body frame, standard signs):
//   m1 (FR, CCW) = T − P − R + Y
//   m2 (FL, CW)  = T − P + R − Y
//   m3 (BL, CCW) = T + P + R + Y
//   m4 (BR, CW)  = T + P − R − Y
//
static void fc_mix_quad_x(void)
{
    if (!motorsArmed || throttle <= FC_ESC_IDLE_US)
    {
        pulse_esc1 = pulse_esc2 = pulse_esc3 = pulse_esc4 = FC_ESC_MIN_US;
        return;
    }

    // 1. Compute raw mix (PID contribution only, no throttle yet)
    float m1 = +P - R + Y;
    float m2 = +P + R - Y;
    float m3 = -P + R + Y;
    float m4 = -P - R - Y;

    // 2. Find mix bounds
    float mixMax = max(max(m1, m2), max(m3, m4));
    float mixMin = min(min(m1, m2), min(m3, m4));

    // 3. Constrain throttle to fit mix without clipping
    //    (normalize to idle..max range)
    float thr = (float)throttle;
    float thrMin = (float)FC_ESC_IDLE_US;
    float thrMax = (float)FC_ESC_MAX_US;

    thr = fc_constrain_f(thr, thrMin - mixMin, thrMax - mixMax);

    // 4. Add throttle — no floor shift, no scale needed
    pulse_esc1 = fc_constrain_f(thr + m1, thrMin, thrMax);
    pulse_esc2 = fc_constrain_f(thr + m2, thrMin, thrMax);
    pulse_esc3 = fc_constrain_f(thr + m3, thrMin, thrMax);
    pulse_esc4 = fc_constrain_f(thr + m4, thrMin, thrMax);

    // pulse_esc1 = fc_constrain_f(throttle, FC_ESC_MIN_US, FC_ESC_SAFE_MAX_US);
    // pulse_esc2 = fc_constrain_f(throttle, FC_ESC_MIN_US, FC_ESC_SAFE_MAX_US);
    // pulse_esc3 = fc_constrain_f(throttle, FC_ESC_MIN_US, FC_ESC_SAFE_MAX_US);
    // pulse_esc4 = fc_constrain_f(throttle, FC_ESC_MIN_US, FC_ESC_SAFE_MAX_US);
}

static void fc_write_motors(void)
{
    if (!motorsArmed)
    {
        fc_write_motors_min();
        return;
    }

    esc_pwm1.writeMicroseconds((uint16_t)pulse_esc1);
    esc_pwm2.writeMicroseconds((uint16_t)pulse_esc2);
    esc_pwm3.writeMicroseconds((uint16_t)pulse_esc3);
    esc_pwm4.writeMicroseconds((uint16_t)pulse_esc4);
}

// ================================================================
// SECTION 18 — HARDWARE CHECKS
// ================================================================

bool fc_check_mpu(void)
{
    uint32_t startMs = millis();

    while ((millis() - startMs) < 2000)
    {
        uint8_t wai = readRegister(0x75);
        Serial.print("WHO_AM_I: 0x");
        Serial.println(wai, HEX);

        if (wai == 0x70)
        {
            Serial.println("MPU6500 OK");
            return true;
        }

        delay(50);
    }

    Serial.println("MPU6500 FAIL");
    return false;
}

// bool qmc_check(void)
// {
//     // Step 1: bus probe — does anything ACK at this address?
//     Wire.beginTransmission(QMC_ADDR);
//     if (Wire.endTransmission() != 0)
//         return false;

//     // Step 2: chip-ID register — QMC5883L always returns 0xFF.
//     uint8_t id = 0;
//     bool t = qmc_read_reg(REG_CHIP_ID, &id);
//     Serial.print("========== QMC ID: ");
//     Serial.println(id);

//     if (!t)
//         return false;
//     return (id == QMC_CHIP_ID_VAL);
// }

// ================================================================
// SECTION 19 — CALIBRATION HELPERS
// ================================================================

// Collects N baro samples (blocking) and returns the mean altitude (m).
// Also prints variance and std-dev — useful for sizing altCfg.R.
static float calcBaroHomeAlt(int N)
{
    Serial.println("========= Calib Baro ==========");

    int count = 0;
    float wf_mean = 0.0f;
    float wf_M2 = 0.0f;
    BaroSample sample;

    while (count < N)
    {
        if (xQueueReceive(baroDataQueue, &sample, pdMS_TO_TICKS(60)))
        {
            if (!isfinite(sample.altitudeM))
            {
                continue; // skip invalid raw samples
            }

            float a = sample.altitudeM;

            count++;
            float delta = a - wf_mean;
            wf_mean += delta / count;
            float delta2 = a - wf_mean;
            wf_M2 += delta * delta2; // Welford online algorithm

            Serial.print(count);
            Serial.print('\t');
            Serial.print(a, 6);
            Serial.print('\t');
            Serial.println(wf_mean, 6);
        }
    }

    if (count < 2)
    {
        Serial.println("ERROR: not enough samples");
        return NAN;
    }

    float variance = wf_M2 / (count - 1); // unbiased estimate
    float stddev = sqrtf(variance);

    Serial.print("Mean: ");
    Serial.print(wf_mean, 6);
    Serial.print(", Variance: ");
    Serial.print(variance, 6);
    Serial.print(", StdDev: ");
    Serial.println(stddev, 6);

    return wf_mean;
}

// Collects N mag samples and prints per-axis noise statistics.
// Un-comment the call in setup() to recalibrate amekf_cfg.magNoise.
static void calcMagNoise(int N)
{
    Serial.println("========= Calib Mag ==========");

    int count = 0;

    float meanX = 0.0f, meanY = 0.0f, meanZ = 0.0f;
    float M2X = 0.0f, M2Y = 0.0f, M2Z = 0.0f;

    Vector3f sample;

    while (count < N)
    {
        if (xQueueReceive(magDataQueue, &sample, pdMS_TO_TICKS(60)))
        {
            if (!isfinite(sample.x) ||
                !isfinite(sample.y) ||
                !isfinite(sample.z))
            {
                continue;
            }

            count++;

            // X
            float deltaX = sample.x - meanX;
            meanX += deltaX / count;
            float delta2X = sample.x - meanX;
            M2X += deltaX * delta2X;

            // Y
            float deltaY = sample.y - meanY;
            meanY += deltaY / count;
            float delta2Y = sample.y - meanY;
            M2Y += deltaY * delta2Y;

            // Z
            float deltaZ = sample.z - meanZ;
            meanZ += deltaZ / count;
            float delta2Z = sample.z - meanZ;
            M2Z += deltaZ * delta2Z;
        }
    }

    if (count < 2)
    {
        Serial.println("ERROR: not enough samples");
        return;
    }

    float varianceX = M2X / (count - 1);
    float varianceY = M2Y / (count - 1);
    float varianceZ = M2Z / (count - 1);

    float stddevX = sqrtf(varianceX);
    float stddevY = sqrtf(varianceY);
    float stddevZ = sqrtf(varianceZ);

    Serial.println("----- Magnetometer Noise -----");
    Serial.print("Mean X: ");
    Serial.println(meanX, 6);
    Serial.print("Mean Y: ");
    Serial.println(meanY, 6);
    Serial.print("Mean Z: ");
    Serial.println(meanZ, 6);
    Serial.print("Variance X: ");
    Serial.println(varianceX, 6);
    Serial.print("Variance Y: ");
    Serial.println(varianceY, 6);
    Serial.print("Variance Z: ");
    Serial.println(varianceZ, 6);
    Serial.print("StdDev X (sigma): ");
    Serial.println(stddevX, 6);
    Serial.print("StdDev Y (sigma): ");
    Serial.println(stddevY, 6);
    Serial.print("StdDev Z (sigma): ");
    Serial.println(stddevZ, 6);
    Serial.println("------------------------------");
}

// ================================================================
// SECTION 20 — MATH / PHYSICS HELPERS
// ================================================================

// Rotates the body-frame accelerometer vector into world NED frame via
// the current quaternion attitude, then subtracts gravity to yield
// linear acceleration in m/s².
//
// accelBody is in g (raw MPU output); return value is in m/s².
//
// ⚠ NOTE: amekf_cfg.accelGravitySign is NOT applied here — gravity
//   cancellation sign is hardcoded as +accelReferenceNorm.  If the
//   sensor mount changes, this function must be updated to match.
Vector3f getLinearAccelWorldNED(const Quaternionf &q,
                                const Vector3f &accelBody,
                                float accelReferenceNorm)
{
    const float qw = q.w, qx = q.x, qy = q.y, qz = q.z;

    const float ww = qw * qw, xx = qx * qx, yy = qy * qy, zz = qz * qz;
    const float wx = qw * qx, wy = qw * qy, wz = qw * qz;
    const float xy = qx * qy, xz = qx * qz, yz = qy * qz;

    // Rotate body (FRD) → world (NED), still in g
    Vector3f f;
    f.x = (ww + xx - yy - zz) * accelBody.x + 2.0f * (xy - wz) * accelBody.y + 2.0f * (xz + wy) * accelBody.z;
    f.y = 2.0f * (xy + wz) * accelBody.x + (ww - xx + yy - zz) * accelBody.y + 2.0f * (yz - wx) * accelBody.z;
    f.z = 2.0f * (xz - wy) * accelBody.x + 2.0f * (yz + wx) * accelBody.y + (ww - xx - yy + zz) * accelBody.z;

    // Remove gravity: sensor reads -1 g at rest, so +accelReferenceNorm cancels it.
    // Scale to m/s².
    Vector3f out;
    out.x = f.x * G;
    out.y = f.y * G;
    out.z = (f.z + accelReferenceNorm) * G;

    return out;
}

// Measures the static accelerometer vector magnitude after calibration
// offsets are applied. Sets accelReferenceNorm for gravity cancellation.
// Call after mpu_apply_calibration_offsets(), before arming.
float mpu_measure_static_norm(int count)
{
    double sum = 0;
    Vector3f a;
    Vector3f dummy;

    for (int i = 0; i < count; i++)
    {
        mpu_read_accel(&a, &dummy);
        sum += sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
        delayMicroseconds(2000);
    }

    float norm = float(sum / count);
    Serial.printf("Static accel norm: %.4f g\n", norm);
    return norm;
}

// ================================================================
// BUG LOG
// ================================================================
//
// ── CRITICAL BUG 1 — Outer loop disabled, no pilot control ───────
//
//   In loop(), the call to fc_update_outer_loop() is commented out:
//
//       // fc_update_outer_loop();
//
//   fc_update_outer_loop() is the ONLY place that writes:
//       DesiredRateRoll   (from aileron  stick via angle PID)
//       DesiredRatePitch  (from elevator stick via angle PID)
//       DesiredRateYaw    (from rudder   stick, direct rate scale)
//
//   With the call disabled, all three stay 0.0f permanently.
//   The inner rate PIDs receive a setpoint of 0 on every axis —
//   the craft will fight to hold zero rates (level) but the pilot
//   has ZERO roll, pitch, and yaw authority from the sticks.
//   Only throttle has any effect.
//
//   Fix: uncomment  fc_update_outer_loop();  in loop().
//
// ── CRITICAL BUG 2 — accelGravitySign ignored in gravity removal ──
//
//   amekf_cfg.accelGravitySign = -1.0f  is set in setup(), but
//   getLinearAccelWorldNED() does not read it.  The gravity removal
//   sign is hardcoded:
//
//       out.z = (f.z + accelReferenceNorm) * G;
//
//   For the current mount (flat, Z reads -1 g) this is correct.
//   If the sensor is remounted, this will silently produce the wrong
//   linear acceleration fed into the altitude Kalman filter.
//
// ── POTENTIAL BUG 3 — aliased pointer in qmc_compute_heading ─────
//
//   fc_update_attitude_mag() calls:
//
//       qmc_compute_heading(&mag_data, &angle_imu, &angle_imu);
//
//   The second argument (roll/pitch input) and the third (heading
//   output) are the same object.  This is safe only if the
//   implementation reads angle_imu.x and .y before writing
//   angle_imu.z.  If the write comes first, heading will be computed
//   from a corrupted roll/pitch, giving a wrong tilt compensation.
//   Verify the implementation of qmc_compute_heading.
//
// ── UNUSED VARIABLES ─────────────────────────────────────────────
//
//   float dt                       — computed every loop, never consumed.
//   float dtOuter                  — computed when outer timer fires,
//                                    but fc_update_outer_loop() is disabled.
//   uint32_t prevAltiMicros        — declared, never referenced in loop().
//   uint32_t last_attitudeUpdate   — declared, never assigned or read.
//   uint32_t loopOverrunCount      — declared, never incremented.
//   Vector3f gyro_Kfiltered        — zero-init'd in setup(), never written
//                                    to or read from during flight.
//   uint16_t auxPulse / rawAux     — received from RX, never used in
//                                    any control or telemetry path.
//
// ================================================================

// 1150    1       -11.42
// 1150    1       -10.81
// 1150    1       -11.91
// 1150    1       -6.40
// 1150    1       -8.13
// 1150    1       -11.88
// 1150    1       -11.04
// 1150    1       -13.09
// 1150    1       -16.59
// 1150    1       -12.65
// 1150    1       -10.05
// 1150    1       -8.12
// 1150    1       -3.98
// 1150    1       -1.55
// [fc_disarm] CODE: 1
// [DISARMED-SILENT] motorsArmed went false outside fc_disarm! throttle=1150 armPulse=1204 now=70807
// 1150    0       -1.20
// 1149    0       -0.46
// 1149    0       -2.36
// 1152    0       -3.51
// 1152    0       -1.89
// 1152    0       -1.53
// 1152    0       -2.98
// 1152    0       0.37
// 1152    0       1.92
// 1152    0       -0.58
// 1152    0       2.55
// 1152    0       3.80
// 1152    0       1.38
// 1152    0       1.88
// 1152    0       3.75
// 1152    0       -0.43