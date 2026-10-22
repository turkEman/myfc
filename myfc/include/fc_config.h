#ifndef FC_CONFIG_H
#define FC_CONFIG_H

#include <stdint.h>

// ================================================================
// Feature switches
// ================================================================
#define FC_ENABLE_WEB 1
#define FC_ENABLE_PRINT 1

// ================================================================
// Main flight loop timing
// ================================================================
#define FC_MAIN_LOOP_HZ 1000UL
#define FC_MAIN_LOOP_US (1000000UL / FC_MAIN_LOOP_HZ)

// Inner (rate) control loop / gyro / MEKF prediction
#define FC_INNER_LOOP_HZ 1000UL
#define FC_INNER_LOOP_US (1000000UL / FC_INNER_LOOP_HZ)

// Outer (attitude) control loop
#define FC_OUTER_LOOP_HZ 250UL
#define FC_OUTER_LOOP_US (1000000UL / FC_OUTER_LOOP_HZ)

// Gyro polling / MEKF prediction
#define FC_GYRO_HZ 1000UL
#define FC_GYRO_US (1000000UL / FC_GYRO_HZ)

// Accelerometer polling / MEKF correction
#define FC_ACCEL_HZ 500UL
#define FC_ACCEL_US (1000000UL / FC_ACCEL_HZ)

#define FC_MAG_HZ 100UL
#define FC_MAG_US (1000000UL / FC_MAG_HZ)

// RC input polling
#define FC_RC_HZ 100UL
#define FC_RC_US (1000000UL / FC_RC_HZ)

#define FC_BARO_HZ 50UL
#define FC_BARO_US (1000000UL / FC_BARO_HZ)

// Motor / ESC output
#define FC_MOTOR_HZ 490UL
#define FC_MOTOR_US (1000000UL / FC_MOTOR_HZ)

#define FC_WEB_SERVICE_HZ 50UL
#define FC_WEB_SERVICE_US (1000000UL / FC_WEB_SERVICE_HZ)

#define FC_PRINT_HZ 100UL
#define FC_PRINT_PERIOD_MS (1000UL / FC_PRINT_HZ)

#define FC_MAX_ALLOWED_DT_S 0.010f

// ================================================================
// ESC / motor output
// ================================================================
// 490 Hz gives ~2040 us period, so 1000-2000 us ESC pulses fit.
// 621 Hz only gives ~1610 us period, so 2000 us cannot physically fit.
#define FC_ESC_PWM_HZ 490UL
#define FC_ESC_PWM_RES_BITS 16
#define FC_ESC_PERIOD_GUARD_US 10UL

#define FC_ESC_MIN_US 1000U
#define FC_ESC_IDLE_US 1080U
#define FC_ESC_MID_US 1500U
#define FC_ESC_MAX_US 2000U
#define FC_ESC_SAFE_MAX_US 2000U

#define FC_PID_START_US 1100U // just above idle
#define FC_PID_FULL_US 1150U  // full authority quickly

// // ================================================================
// // RC input calibration
// // Preserves your current measured endpoints.
// // ================================================================
// #define FC_RC_MIN_US           1000U
// #define FC_RC_MID_US           1500U
// #define FC_RC_MAX_US           2000U

// #define FC_RC_AIL_MIN_US       1110U
// #define FC_RC_AIL_MID_US       1520U
// #define FC_RC_AIL_MAX_US       1920U

// #define FC_RC_ELE_MIN_US       1107U
// #define FC_RC_ELE_MID_US       1505U
// #define FC_RC_ELE_MAX_US       1919U

// #define FC_RC_THR_MIN_US       1105U
// #define FC_RC_THR_MID_US       1531U
// #define FC_RC_THR_MAX_US       1931U

// #define FC_RC_RUD_MIN_US       1110U
// #define FC_RC_RUD_MID_US       1520U
// #define FC_RC_RUD_MAX_US       1933U

// ================================================================
// RC input calibration
// Values derived from logged data means at each stick position.
// MIN/MAX = lowest/highest mean across all files where that
// channel was at its extreme. MID = mean from centered recording.
// ================================================================
#define FC_RC_MIN_US 1000U
#define FC_RC_MID_US 1500U
#define FC_RC_MAX_US 2000U

#define FC_RC_AIL_MIN_US 1106U // LU mean 1106.49
#define FC_RC_AIL_MID_US 1524U // mid mean 1523.96
#define FC_RC_AIL_MAX_US 1929U // RB mean 1929.23

#define FC_RC_ELE_MIN_US 1106U // RB mean 1106.13
#define FC_RC_ELE_MID_US 1504U // mid mean 1504.11
#define FC_RC_ELE_MAX_US 1926U // LU mean 1926.17

#define FC_RC_THR_MIN_US 1106U // RB mean 1106.02
#define FC_RC_THR_MID_US 1531U // mid mean 1531.40
#define FC_RC_THR_MAX_US 1933U // RU mean 1932.72

#define FC_RC_RUD_MIN_US 1107U // LB mean 1107.29
#define FC_RC_RUD_MID_US 1522U // mid mean 1522.32
#define FC_RC_RUD_MAX_US 1931U // RB mean 1931.04

// ================================================================
// Arming / disarming
// UP (~1200 µs) = armed, DOWN (~1800 µs) = disarmed, no-signal = 1000 µs = disarmed
#define FC_ARM_SWITCH_LO_US    1170U
#define FC_ARM_SWITCH_HI_US    1210U
#define FC_ARM_DEBOUNCE_MS      70UL   // ~5 frames @ 70 Hz
#define FC_DISARM_DEBOUNCE_MS   70UL

// ================================================================
// Control limits
// ================================================================
#define FC_PID_OUTPUT_LIMIT 300.0f
#define FC_PID_I_LIMIT 150.0f

#define FC_MAX_ANGLE_DEG 45.0f
#define FC_ROLL_PITCH_ANGLE_SCALE 0.1f
#define FC_YAW_RATE_SCALE 0.15f

// ================================================================
// Status LED
// ================================================================
#define FC_ENABLE_STATUS_LED 1
#define FC_LED_ON_MS 80UL
#define FC_LED_DOUBLE_GAP_MS 170UL
#define FC_LED_BLINK_PERIOD_MS 1000UL

// ================================================================
// Sensor / bus config
// ================================================================
#define FC_MPU_SPI_HZ 20000000UL
#define FC_I2C_HZ 1000000UL

// ================================================================
// FreeRTOS task layout
// Core 1: Arduino loop / flight loop
// Core 0: IO, WiFi, logging, slow tasks
// ================================================================
#define FC_CORE_IO 0
#define FC_CORE_FLIGHT 1

#define FC_TASK_PRIO_PRINT 1
#define FC_TASK_PRIO_QMC 2

#define FC_TASK_STACK_PRINT 4096
#define FC_TASK_STACK_QMC 3072

#endif // FC_CONFIG_H
