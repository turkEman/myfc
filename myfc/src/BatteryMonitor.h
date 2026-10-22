#pragma once
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
//  BatteryMonitor.h  —  3S LiPo voltage reader for ESP32
//
//  setup()  →  Battery_init()
//  10 Hz timer (every 100 ms)  →  Battery_update()
//  anywhere  →  Battery_getVoltage()
// ─────────────────────────────────────────────────────────────────────────────

// ── Hardware ─────────────────────────────────────────────────────────────────
static constexpr int   BAT_PIN     = 35;
static constexpr float BAT_R1      = 10000.0f;              // top resistor  (Ω)
static constexpr float BAT_R2      =  3300.0f;              // bottom resistor (Ω)
static constexpr float BAT_DIVIDER = (BAT_R1 + BAT_R2) / BAT_R2;  // 4.030

// ── IIR filter ───────────────────────────────────────────────────────────────
// α = 0.1 at 10 Hz  →  time constant ≈ 0.9 s, smooth but still tracks real sag
// raise α (e.g. 0.3) if you want faster response to motor load spikes
static constexpr float BAT_ALPHA = 0.1f;

// ── 3S LiPo thresholds ───────────────────────────────────────────────────────
static constexpr float BAT_FULL_V = 12.60f;   // 4.20 V × 3
static constexpr float BAT_WARN_V = 10.50f;   // 3.50 V × 3  → warn / RTH
static constexpr float BAT_CRIT_V =  9.60f;   // 3.20 V × 3  → land now

// ── Private state ─────────────────────────────────────────────────────────────
static float _bat_filtered = 0.0f;
static float _bat_cal      = 1.0f;   // multiplicative calibration scale factor

// ── API ───────────────────────────────────────────────────────────────────────

inline void Battery_init()
{
    analogSetPinAttenuation(BAT_PIN, ADC_11db);

    // Seed filter with a real reading so it starts accurate, not at 0 V
    long sum = 0;
    for (int i = 0; i < 64; i++) sum += analogRead(BAT_PIN);
    float adc_v = (sum / 64.0f) * (3.3f / 4095.0f);
    _bat_filtered = adc_v * BAT_DIVIDER * _bat_cal;
}

// Call from a 10 Hz timer — every 100 ms
inline void Battery_update()
{
    // 8-sample hardware average first, then IIR
    long sum = 0;
    for (int i = 0; i < 8; i++) sum += analogRead(BAT_PIN);
    float adc_v = (sum / 8.0f) * (3.3f / 4095.0f);
    float raw_v = adc_v * BAT_DIVIDER * _bat_cal;

    // y[n] = α·x[n] + (1−α)·y[n−1]
    _bat_filtered += BAT_ALPHA * (raw_v - _bat_filtered);
}

inline float Battery_getVoltage() { return _bat_filtered; }

// Linear 0–100 % estimate (LiPo curve is non-linear; good enough for HUD)
inline int Battery_getPercent()
{
    float pct = (_bat_filtered - BAT_CRIT_V) / (BAT_FULL_V - BAT_CRIT_V) * 100.0f;
    return (int)constrain(pct, 0.0f, 100.0f);
}

inline bool Battery_isLow()  { return _bat_filtered <= BAT_WARN_V; }
inline bool Battery_isCrit() { return _bat_filtered <= BAT_CRIT_V; }

// ── Calibration ───────────────────────────────────────────────────────────────
// 1. Connect battery, no motors running
// 2. Read battery voltage with your DMM directly at the battery connector
// 3. Call Battery_calibrate(dmm_reading);
// Corrects for resistor tolerance. Lives in RAM — write _bat_cal to
// NVS/Preferences if you want it to survive reboots.
inline void Battery_calibrate(float dmm_volts)
{
    long sum = 0;
    for (int i = 0; i < 64; i++) sum += analogRead(BAT_PIN);
    float adc_v  = (sum / 64.0f) * (3.3f / 4095.0f);
    float meas_v = adc_v * BAT_DIVIDER;   // raw, before cal

    if (meas_v > 1.0f)                    // sanity — battery must be connected
    {
        _bat_cal = dmm_volts / meas_v;
        // also update the live filter immediately
        _bat_filtered *= _bat_cal;
    }
}