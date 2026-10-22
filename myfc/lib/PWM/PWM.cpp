#include "PWM.h"

// Catches a misconfigured / accidental zero FC_ESC_PWM_HZ at compile time
// instead of a division-by-zero surfacing as a boot-time crash.
static_assert(FC_ESC_PWM_HZ > 0, "FC_ESC_PWM_HZ must be nonzero");

// ================================================================
// PWM_out
// ================================================================

uint8_t        PWM_out::nextChannel  = 0;
volatile bool *PWM_out::s_armedFlag  = nullptr;
uint16_t       PWM_out::s_disarmedUs = FC_ESC_MIN_US;

PWM_out::PWM_out()
    : pin(0),
      channel(nextChannel++),
      minUs(FC_ESC_MIN_US),
      maxUs(FC_ESC_MAX_US),
      attached(false),
      period_us(0),
      maxDuty(0),
      effectiveMaxUs(0)
{
    // PWM_out objects are typically declared as globals and constructed
    // before setup(). The channel counter increments at construction time
    // (not at attach()), so declaring a 5th PWM_out anywhere reserves a
    // 5th channel immediately, before setup() runs. If more than
    // PWM_OUT_MAX_CHANNELS are created the LEDC calls will silently
    // operate on an invalid channel. Assert here so the problem surfaces
    // immediately at boot rather than as strange motor behavior.
    assert(channel < PWM_OUT_MAX_CHANNELS &&
           "Too many PWM_out instances — exceeded LEDC channel limit (max 8)");
}

void PWM_out::setGlobalArmFlag(volatile bool *armedFlag)
{
    s_armedFlag = armedFlag;
}

void PWM_out::setDisarmedPulse(uint16_t us)
{
    if (us < FC_ESC_MIN_US) us = FC_ESC_MIN_US;
    if (us > FC_ESC_MAX_US) us = FC_ESC_MAX_US;
    s_disarmedUs = us;
}

void PWM_out::attach(uint8_t pinNum, uint16_t minPulseUs, uint16_t maxPulseUs)
{
    pin   = pinNum;
    minUs = minPulseUs;
    maxUs = maxPulseUs;

    // Arduino-ESP32 v3.x deprecated ledcSetup + ledcAttachPin in favour of
    // a single ledcAttachChannel call. Select at compile time.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttachChannel(pin, FC_ESC_PWM_HZ, FC_ESC_PWM_RES_BITS, channel);
#else
    ledcSetup(channel, FC_ESC_PWM_HZ, FC_ESC_PWM_RES_BITS);
    ledcAttachPin(pin, channel);
#endif

    // ── Cache constants ─────────────────────────────────────────────
    // These never change after attach(). Recomputing them inside
    // writeRawMicroseconds() on every 490 Hz × 4 motor call was waste.

    period_us = 1000000UL / FC_ESC_PWM_HZ;
    maxDuty   = (1UL << FC_ESC_PWM_RES_BITS) - 1UL;

    // Effective maximum: start from user maxUs, then apply guards.
    effectiveMaxUs = (uint32_t)maxUs;

    // Guard 1: never exceed the hardware-safe maximum pulse.
    if (effectiveMaxUs > (uint32_t)FC_ESC_SAFE_MAX_US)
        effectiveMaxUs = (uint32_t)FC_ESC_SAFE_MAX_US;

    // Guard 2: never let the pulse consume the full PWM period.
    // At 490 Hz, period ≈ 2040 µs. With FC_ESC_PERIOD_GUARD_US = 40,
    // effectiveMaxUs is capped at 2000 µs.
    if (period_us > (uint32_t)FC_ESC_PERIOD_GUARD_US)
    {
        uint32_t periodLim = period_us - (uint32_t)FC_ESC_PERIOD_GUARD_US;
        if (effectiveMaxUs > periodLim)
            effectiveMaxUs = periodLim;
    }

    // Guard 3: effectiveMaxUs must be at least minUs or the output range
    // collapses. This only happens if FC_ESC_PERIOD_GUARD_US is misconfigured.
    if (effectiveMaxUs < (uint32_t)minUs)
        effectiveMaxUs = (uint32_t)minUs;
    // ────────────────────────────────────────────────────────────────

    attached = true;

    // Immediately drive a safe output so the ESC sees a valid low pulse
    // the moment the pin is configured, before the first motor write.
    writeRawMicroseconds(s_disarmedUs);
}

void PWM_out::writeMicroseconds(uint16_t us)
{
    // Driver-level arm gate — independent of application arming logic.
    // If the flag pointer is null or the flag is false, force disarmed output.
    if (s_armedFlag == nullptr || !(*s_armedFlag))
    {
        writeRawMicroseconds(s_disarmedUs);
        return;
    }
    writeRawMicroseconds(us);
}

void PWM_out::writeRawMicroseconds(uint16_t us)
{
    if (!attached) return;

    // Clamp to [minUs, effectiveMaxUs].
    if (us < minUs)                      us = minUs;
    if ((uint32_t)us > effectiveMaxUs)   us = (uint16_t)effectiveMaxUs;

    // Map microseconds → LEDC duty count.
    // Using uint64_t intermediate: us(max≈2000) * maxDuty(max≈65535) = ~131M,
    // fits in uint32_t but use 64-bit anyway to be safe at higher resolutions.
    uint32_t duty = (uint64_t)us * maxDuty / period_us;
    if (duty > maxDuty) duty = maxDuty;  // clamp overflow (should never trigger)

    // FIX: Arduino-ESP32 v3.x's LEDC API is keyed by GPIO *pin*, not by
    // the LEDC *channel* index, for every call after attach — including
    // ledcWrite(). ledcAttachChannel() above still lets you request a
    // specific channel, but ledcWrite(channel, duty) on v3.x would pass
    // the raw channel number (0-3) as if it were a GPIO pin, silently
    // targeting the wrong (or unattached) pin instead of the ESC output.
    // Dispatch by version so this matches whichever attach path ran above.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pin, duty);
#else
    ledcWrite(channel, duty);
#endif
}


// ================================================================
// PWM_in
// ================================================================

static constexpr uint16_t MIN_VALID_PULSE_US = 300;
static constexpr uint16_t MAX_VALID_PULSE_US = 3000;

PWM_in::PWM_in(uint8_t pinNum, uint16_t init_val)
    : pin(pinNum),
      pulseWidth(init_val),
      pulseWidthRaw(init_val),
      lastRise(0),
      validRise(false),
      _newData(false),
      lastValidPulseUs(0)   // 0 == "never received a valid pulse" -> isAlive() is false at boot
{}

void PWM_in::begin()
{
    pinMode(pin, INPUT);
    attachInterruptArg(digitalPinToInterrupt(pin), isrRouter, this, CHANGE);
}

// No mutex needed.
// pulseWidth is volatile uint16_t. On Xtensa LX6 (ESP32) aligned 16/32-bit
// reads are single instructions — atomic by hardware.
// ISR and this caller are both on Core 1 (see class-level note in PWM.h),
// so there is no cross-core cache-coherency concern for THIS access pattern.
uint16_t PWM_in::getPulse() const
{
    return pulseWidth;
}

uint16_t PWM_in::getRawPulse() const
{
    return pulseWidthRaw;
}

bool PWM_in::hasNewData()
{
    if (!_newData) return false;
    _newData = false;   // clear first: if ISR fires here it just re-sets the flag,
    return true;        // which will be caught on the next hasNewData() call — no loss.
}

bool PWM_in::isAlive(uint32_t timeoutUs) const
{
    // lastValidPulseUs == 0 means "no valid pulse ever received" — micros()
    // minus 0 is a huge number on any real system uptime, so this correctly
    // reads as "not alive" rather than accidentally passing at boot.
    return (micros() - lastValidPulseUs) < timeoutUs;
}

uint32_t PWM_in::microsSinceLastValid() const
{
    return micros() - lastValidPulseUs;
}

void IRAM_ATTR PWM_in::isrRouter(void *arg)
{
    reinterpret_cast<PWM_in *>(arg)->handleInterrupt();
}

void IRAM_ATTR PWM_in::handleInterrupt()
{
    const uint8_t  state = fastReadPin(pin);
    const uint32_t now   = micros();

    if (state)
    {
        // Rising edge — record timestamp and mark it valid.
        lastRise  = now;
        validRise = true;
    }
    else
    {
        // Falling edge — measure width if we have a valid rising edge.
        if (validRise)
        {
            uint32_t width = now - lastRise;

            // Saturate to uint16_t max on extreme glitches.
            if (width > 0xFFFFu) width = 0xFFFFu;

            pulseWidthRaw = (uint16_t)width;

            if (width >= MIN_VALID_PULSE_US && width <= MAX_VALID_PULSE_US)
            {
                pulseWidth       = (uint16_t)width;
                lastValidPulseUs = now;  // NEW: stamp for isAlive() / signal-loss detection
                _newData         = true; // signal fresh valid data to main loop
            }

            validRise = false;
        }
    }
}