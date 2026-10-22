#ifndef MPU6500_H
#define MPU6500_H

#include <fc_core.h>
#include <Arduino.h>
#include <SPI.h>

// ── SPI pins ─────────────────────────────────────────────────────────────────
#define PIN_SPI_SCK       18
#define PIN_SPI_MISO      19
#define PIN_SPI_MOSI      23
#define PIN_SPI_CS        5

// ── Register map ─────────────────────────────────────────────────────────────
#define REG_PWR_MGMT_1    0x6B
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_CONFIG2 0x1D
#define REG_ACCEL_START   0x3B   // ACCEL_XOUT_H
#define REG_GYRO_START    0x43   // GYRO_XOUT_H
#define REG_WHO_AM_I      0x75

// ── Conversion factors ───────────────────────────────────────────────────────
// ±500 dps  → datasheet sensitivity 65.5 LSB/dps
#define GYRO_SCALE        (1.0f / 65.5f)
// ±8g       → 4096 LSB/g
#define ACCEL_SCALE       4096.0f

// ── Accel filter / sanity ────────────────────────────────────────────────────
// Software IIR alpha at 500 Hz polling → ~8 Hz effective cutoff.
// Combined with hardware 41 Hz DLPF this gives a two-stage filter that
// kills the sub-41 Hz motor vibration that survives the hardware filter.
#define ACCEL_IIR_ALPHA   0.10f     // ~8 Hz effective cutoff at 500 Hz
// #define ACCEL_IIR_ALPHA   1.0f

// Denominator guard for atan2 — below this the tilt angle is undefined.
// Raised to 0.30 (vs old 0.15) because the IIR gives us cleaner vectors.
#define ACCEL_DENOM_MIN   0.30f

// Values above this in any axis are physically impossible in flight and
// indicate an SPI byte-misalignment event — discard the whole sample.
#define ACCEL_SANITY_MAX  20.0f

// ── Externals ────────────────────────────────────────────────────────────────
// SPI instance (needed by fc_check_mpu and any other peripheral on the bus)
extern SPIClass mySPI;

// ── Low-level SPI (public for fc_check_mpu WHO_AM_I read) ───────────────────
uint8_t readRegister(uint8_t);

// ── Public API ───────────────────────────────────────────────────────────────

// Call once in setup(). Handles mySPI.begin(), CS pin, and all register init.
void mpu_init(void);


bool fc_check_mpu(void);

// Calibrate gyro + accel by averaging `count` samples at 1 kHz pace.
// Hold craft flat and still during this call.
// Seeds the accel IIR state so the filter is warm before the first flight read.
// returns g_mag
float mpu_calibrate(int);

// Read gyro only — call every 1 kHz loop iteration.
// Populates gyro->x/y/z in deg/s (calibrated, no filtering needed for gyro).
void mpu_read_gyro(Vector3f *);

// Read accel only — call every other iteration (500 Hz).
// Applies sanity gate, two-stage IIR, and atan2 angle calculation.
// Populates accel_data->x/y/z (filtered, calibrated, in g).
// Populates angle_imu->x (roll) and angle_imu->y (pitch) in degrees.
// angle_imu->z is intentionally left untouched — magnetometer driver owns it.
void mpu_read_accel(Vector3f *, Vector3f *);


void mpu_run_6pos_calibration(void);

void mpu_apply_calibration_offsets(void);


#endif // MPU6500_H