// #ifndef PWM_H
// #define PWM_H

// #include <Arduino.h>
// #include <stdint.h>
// #include <driver/gpio.h>
// #include <soc/gpio_struct.h>
// #include <fc_config.h>

// #ifndef MIN_ESC_PWM
// #define MIN_ESC_PWM FC_ESC_MIN_US
// #endif
// #ifndef MAX_ESC_PWM
// #define MAX_ESC_PWM FC_ESC_MAX_US
// #endif

// // ESP32 original has 16 LEDC channels (8 high-speed, 8 low-speed).
// // S3/C3/S2 have 8. We only use high-speed bank (0–7) to be safe on all targets.
// #define PWM_OUT_MAX_CHANNELS 8

// // ================================================================
// // PWM_out — safe ESC pulse writer using ESP32 LEDC peripheral.
// //
// // Design notes:
// //   - Constants (period, duty range) are cached at attach() time.
// //     writeRawMicroseconds() does only a clamp + one multiply + ledcWrite().
// //   - A shared arm flag provides a driver-level safety gate independent
// //     of application-level arming logic.
// //   - Compatible with Arduino-ESP32 v2.x (ledcSetup/ledcAttachPin)
// //     and v3.x (ledcAttachChannel) via preprocessor selection.
// // ================================================================

// class PWM_out
// {
// public:
//     PWM_out();

//     void attach(uint8_t pinNum,
//                 uint16_t minUs = FC_ESC_MIN_US,
//                 uint16_t maxUs = FC_ESC_MAX_US);

//     // Write a pulse width in microseconds.
//     // Forces disarmed output if arm flag is not set.
//     void writeMicroseconds(uint16_t us);

//     // Pass the address of your global motorsArmed variable.
//     // If null or *flag == false, all writes output disarmed pulse.
//     static void setGlobalArmFlag(volatile bool *armedFlag);

//     // Pulse written while disarmed. Default = FC_ESC_MIN_US.
//     static void setDisarmedPulse(uint16_t us);

// private:
//     void writeRawMicroseconds(uint16_t us);

//     uint8_t  pin;
//     uint8_t  channel;
//     uint16_t minUs;
//     uint16_t maxUs;
//     bool     attached;

//     // Cached at attach() — never recomputed per write call.
//     uint32_t period_us;      // 1000000 / FC_ESC_PWM_HZ
//     uint32_t maxDuty;        // (1 << FC_ESC_PWM_RES_BITS) - 1
//     uint32_t effectiveMaxUs; // clamped by SAFE_MAX and PERIOD_GUARD

//     static uint8_t        nextChannel;
//     static volatile bool *s_armedFlag;
//     static uint16_t       s_disarmedUs;
// };


// // ================================================================
// // PWM_in — interrupt-driven RC PWM pulse reader.
// //
// // Design notes:
// //   - No mutex. All payload fields are volatile scalars (≤32-bit,
// //     aligned). On Xtensa LX6 these are single-instruction reads:
// //     inherently atomic. ISR and main loop both run on Core 0, so
// //     there is no cross-core coherency concern.
// //   - hasNewData() lets the caller avoid processing stale values.
// //     It returns true exactly once per new valid edge and clears
// //     the flag. In the worst case (ISR fires between the read and
// //     the clear) the flag is re-set by the ISR: zero data loss,
// //     just a harmless extra true on the next call.
// // ================================================================

// class PWM_in
// {
// public:
//     PWM_in(uint8_t pin, uint16_t init_val = FC_RC_MID_US);

//     void begin();

//     // Returns last validated pulse (MIN_VALID_PULSE_US–MAX_VALID_PULSE_US).
//     uint16_t getPulse() const;

//     // Returns last raw pulse even if out of the valid range.
//     uint16_t getRawPulse() const;

//     // Returns true once per new valid pulse. Clears the flag.
//     // Use bitwise | (not ||) when checking multiple channels so all
//     // flags are evaluated and cleared in one pass.
//     bool hasNewData();

// private:
//     static void IRAM_ATTR isrRouter(void *arg);
//     void        IRAM_ATTR handleInterrupt();

//     static inline uint8_t IRAM_ATTR fastReadPin(uint8_t p)
//     {
//         if (p < 32) return (GPIO.in      >> p)        & 0x1;
//         else        return (GPIO.in1.val >> (p - 32)) & 0x1;
//     }

//     uint8_t pin;

//     volatile uint16_t pulseWidth;    // last validated pulse
//     volatile uint16_t pulseWidthRaw; // last raw pulse
//     volatile uint32_t lastRise;      // micros() at last rising edge
//     volatile bool     validRise;     // rising edge was captured before this falling edge
//     volatile bool     _newData;      // set by ISR, cleared by hasNewData()
// };

// #endif // PWM_H

#ifndef PWM_H
#define PWM_H

#include <Arduino.h>
#include <stdint.h>
#include <driver/gpio.h>
#include <soc/gpio_struct.h>
#include <fc_config.h>

#ifndef MIN_ESC_PWM
#define MIN_ESC_PWM FC_ESC_MIN_US
#endif
#ifndef MAX_ESC_PWM
#define MAX_ESC_PWM FC_ESC_MAX_US
#endif

// ESP32 original has 16 LEDC channels (8 high-speed, 8 low-speed).
// S3/C3/S2 have 8. We only use high-speed bank (0–7) to be safe on all targets.
#define PWM_OUT_MAX_CHANNELS 8

// Default RX signal-loss timeout used by PWM_in::isAlive() when the caller
// doesn't pass an explicit value. 150ms ~= 10-15 missed frames at a
// 70-100Hz RC update rate: long enough to ride out burst RF noise,
// short enough to keep failsafe reaction time acceptable. Override per-call
// if a specific channel needs a different threshold.
#ifndef PWM_IN_DEFAULT_TIMEOUT_US
#define PWM_IN_DEFAULT_TIMEOUT_US 150000UL
#endif

// ================================================================
// PWM_out — safe ESC pulse writer using ESP32 LEDC peripheral.
//
// Design notes:
//   - Constants (period, duty range) are cached at attach() time.
//     writeRawMicroseconds() does only a clamp + one multiply + ledcWrite().
//   - A shared arm flag provides a driver-level safety gate independent
//     of application-level arming logic.
//   - Compatible with Arduino-ESP32 v2.x (ledcSetup/ledcAttachPin)
//     and v3.x (ledcAttachChannel) via preprocessor selection.
//     IMPORTANT: v3.x's LEDC API is keyed by GPIO pin, not by LEDC
//     channel index, for every call *after* attach — including
//     ledcWrite(). This class dispatches the write call by version too
//     (see writeRawMicroseconds() in the .cpp), not just the attach call.
// ================================================================

class PWM_out
{
public:
    PWM_out();

    void attach(uint8_t pinNum,
                uint16_t minUs = FC_ESC_MIN_US,
                uint16_t maxUs = FC_ESC_MAX_US);

    // Write a pulse width in microseconds.
    // Forces disarmed output if arm flag is not set.
    void writeMicroseconds(uint16_t us);

    // Pass the address of your global motorsArmed variable.
    // If null or *flag == false, all writes output disarmed pulse.
    static void setGlobalArmFlag(volatile bool *armedFlag);

    // Pulse written while disarmed. Default = FC_ESC_MIN_US.
    static void setDisarmedPulse(uint16_t us);

private:
    void writeRawMicroseconds(uint16_t us);

    uint8_t  pin;
    uint8_t  channel;
    uint16_t minUs;
    uint16_t maxUs;
    bool     attached;

    // Cached at attach() — never recomputed per write call.
    uint32_t period_us;      // 1000000 / FC_ESC_PWM_HZ
    uint32_t maxDuty;        // (1 << FC_ESC_PWM_RES_BITS) - 1
    uint32_t effectiveMaxUs; // clamped by SAFE_MAX and PERIOD_GUARD

    static uint8_t        nextChannel;
    static volatile bool *s_armedFlag;
    static uint16_t       s_disarmedUs;
};


// ================================================================
// PWM_in — interrupt-driven RC PWM pulse reader.
//
// Design notes:
//   - No mutex. All payload fields are volatile scalars (≤32-bit,
//     aligned). On Xtensa LX6 these are single-instruction reads:
//     inherently atomic.
//   - ISR and main loop both run on Core 1 (the default Arduino
//     setup()/loop() core). attachInterruptArg() called from setup()
//     installs the GPIO ISR on whichever core called it, which is
//     Core 1 by default — NOT Core 0. If you ever add a FreeRTOS task
//     pinned to Core 0 (telemetry, logging, WiFi bridge, etc.) that
//     reads these same volatiles, that access is NOT covered by the
//     single-core atomicity argument above and needs its own
//     synchronization (e.g. a critical section or copying the value
//     inside a Core-1-only accessor).
//   - hasNewData() lets the caller avoid processing stale values.
//     It returns true exactly once per new valid edge and clears
//     the flag. In the rare case where the ISR fires between the
//     guard check and the clear, the *value* returned by getPulse()
//     is already the newer one — so no data is lost — but that
//     newer arrival doesn't get its own separate "true" on the next
//     call (it was already consumed). Harmless at 70-100Hz RC vs
//     100Hz polling.
// ================================================================

class PWM_in
{
public:
    PWM_in(uint8_t pin, uint16_t init_val = FC_RC_MID_US);

    void begin();

    // Returns last validated pulse (MIN_VALID_PULSE_US–MAX_VALID_PULSE_US).
    uint16_t getPulse() const;

    // Returns last raw pulse even if out of the valid range.
    uint16_t getRawPulse() const;

    // Returns true once per new valid pulse. Clears the flag.
    // Use bitwise | (not ||) when checking multiple channels so all
    // flags are evaluated and cleared in one pass.
    bool hasNewData();

    // ── RX signal-loss detection ────────────────────────────────
    // Returns true if a validated pulse was captured within the last
    // `timeoutUs` microseconds. Before the first valid pulse is ever
    // received, lastValidPulseUs == 0, so this correctly reports
    // "not alive" from boot until the first good frame arrives —
    // the FC should never consider itself RX-connected at power-on
    // just because it hasn't timed out yet.
    bool isAlive(uint32_t timeoutUs = PWM_IN_DEFAULT_TIMEOUT_US) const;

    // Microseconds elapsed since the last validated pulse. Useful for
    // diagnostics/telemetry rather than the pass/fail isAlive() gate.
    uint32_t microsSinceLastValid() const;

private:
    static void IRAM_ATTR isrRouter(void *arg);
    void        IRAM_ATTR handleInterrupt();

    static inline uint8_t IRAM_ATTR fastReadPin(uint8_t p)
    {
        if (p < 32) return (GPIO.in      >> p)        & 0x1;
        else        return (GPIO.in1.val >> (p - 32)) & 0x1;
    }

    uint8_t pin;

    volatile uint16_t pulseWidth;      // last validated pulse
    volatile uint16_t pulseWidthRaw;   // last raw pulse
    volatile uint32_t lastRise;        // micros() at last rising edge
    volatile bool     validRise;       // rising edge was captured before this falling edge
    volatile bool     _newData;        // set by ISR, cleared by hasNewData()

    // Timestamp (micros()) of the last *validated* pulse. Updated only in
    // the ISR's validation branch, independent of hasNewData()'s
    // consume-once semantics. Sole source of truth for isAlive().
    volatile uint32_t lastValidPulseUs;
};

#endif // PWM_H