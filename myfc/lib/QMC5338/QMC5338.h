#pragma once
#include <stdint.h>
#include "fc_core.h"

// ── I2C address ────────────────────────────────────────────────────────────
#define QMC_ADDR        0x0D    // 7-bit; hard-wired, no alternatives

// ── Register map  (QMC5883L datasheet Rev 1.0) ────────────────────────────
#define REG_DATA        0x00    // Burst origin: XL XH YL YH ZL ZH (0x00–0x05)
#define REG_STATUS      0x06    // bit[1]=OVL   bit[0]=DRDY
#define REG_TOUT_L      0x07    // Temperature LSB (optional)
#define REG_TOUT_H      0x08    // Temperature MSB (optional)
#define REG_CTRL1       0x09    // [7:6]=OSR  [5:4]=RNG  [3:2]=ODR  [1:0]=MODE
#define REG_CTRL2       0x0A    // [7]=SOFT_RST  [6]=ROL_PNT  [0]=INT_ENB
#define REG_SRESET      0x0B    // SET/RESET period — always write 0x01 on init
#define REG_CHIP_ID     0x0D    // Read-only 0xFF.
                                // Register address == I2C address by coincidence;
                                // they live in different namespaces (reg vs bus).

// ── CTRL1: verified against Table 6 of datasheet ─────────────────────────
//   OSR [7:6]  over-sample ratio  00→512  01→256  10→128  11→64
//   RNG [5:4]  full-scale range   00→±2G  01→±8G
//   ODR [3:2]  output data rate   00→10Hz 01→50Hz 10→100Hz 11→200Hz
//   MODE[1:0]  power mode         00→Standby  01→Continuous
//
//   0x09 = 0b_00_00_10_01
//            OSR=512  RNG=±2G  ODR=100Hz  Continuous
//
//   OSR=512 + ODR=100Hz is valid: the modulator runs at 512×100Hz = 51.2kHz.
#define QMC_CTRL1_VAL   0x09

// ── CTRL2 bit masks ───────────────────────────────────────────────────────
#define CTRL2_SOFT_RST  0x80    // bit 7: soft-reset (self-clearing, ~5 ms)
#define CTRL2_ROL_PNT   0x40    // bit 6: register-pointer roll-over
#define CTRL2_INT_ENB   0x01    // bit 0: 1 = DRDY/INT pin active
                                //        0 = pin disabled (tri-stated)
                                // *** Writing 0x00 here is the COMMON BUG that
                                //     prevents the hardware DRDY pin from firing. ***

// Normal operating value: enable DRDY output, no roll-over needed (burst reads)
#define QMC_CTRL2_VAL   CTRL2_INT_ENB   // 0x01

// ── Chip ID ───────────────────────────────────────────────────────────────
#define QMC_CHIP_ID_VAL 0xFF

// ── Scaling ───────────────────────────────────────────────────────────────
// ±2 G range → 12000 LSB/G  (datasheet §5.1 Table 3)
#define MAG_SCALE       (1.0f / 12000.0f)

// ── Heading constants ─────────────────────────────────────────────────────
#define MAG_DECLINATION 2.20f   // degrees, east positive; update for your site
#define MAG_H_MIN       0.02f   // minimum credible horizontal field (Gauss)
#define MAG_H_REL_MIN   0.10f   // minimum |Bh| / |B| ratio

// ── Timestamped packet transported via magDataQueue ───────────────────────
struct MagPacket {
    Vector3f mag;           // calibrated body-frame vector (Gauss)
    int64_t  timestamp_us;  // esp_timer_get_time() latched in DRDY ISR
};

// ── Public API ────────────────────────────────────────────────────────────
// Wire.begin() must be called in setup() before any of these.
bool qmc_read_reg   (uint8_t reg, uint8_t *out);
bool qmc_init       (void);   // soft-reset then configure; call once
bool qmc_check      (void);   // verify chip responds and ID == 0xFF
bool qmc_data_ready (void);   // poll STATUS.DRDY (no interrupt needed)
bool qmc_read_data  (Vector3f *);           // calibrated body-frame sample
bool qmc_read_raw   (int &mx, int &my, int &mz);
bool qmc_compute_heading(const Vector3f *, const Quaternionf *, Vector3f *);
void qmc_verify_regs(void);