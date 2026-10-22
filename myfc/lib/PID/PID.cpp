// // PID.cpp
// #include "PID.h"
// #include <Arduino.h>

// #define MAX(a,b) ((a) > (b) ? (a) : (b))

// PIDController::PIDController(float kp, float ki, float kd, float outputLimit, float integralLimit)
//     : kp_(kp), ki_(ki), kd_(kd), prevError_(0.0f), prevMeasurement_(0.0f),
//       integral_(0.0f), dFiltered_(0.0f),
//       outputLimit_(outputLimit), integralLimit_(integralLimit),
//       tau_d_(0.02f), antiWindupGain_(0.0f) // default tau ~20ms
// {
// }

// float PIDController::compute(float setpoint, float measurement, float dt, bool integrate)
// {
//     // Guard dt
//     if (!(dt > 0.0f))
//         return 0.0f;
//     // clamp dt to reasonable bounds to avoid huge steps after pauses
//     if (dt > 0.1f)
//     { // too large, treat as reset/skip or clamp
//         dt = 0.1f;
//     }

//     // error = setpoint - measurement
//     float error = setpoint - measurement;

//     // Proportional (optionally add setpoint weighting here)
//     float pTerm = kp_ * error;

//     // ---------- Integral (store raw integral) ----------
//     // trapezoidal on raw integral (e = error)
//     if (integrate)
//     {
//         integral_ += 0.5f * (error + prevError_) * dt;
//     }
//     // FIX: clamp the raw integral value here
//     if (ki_ != 0.0f)
//     {
//         integral_ = clamp(integral_, -integralLimit_ / ki_, integralLimit_ / ki_);
//     }
//     else
//     {
//         integral_ = 0.0f; // avoid divide-by-zero if Ki = 0
//     }
//     // clamp integral after scaling by Ki to preserve your integralLimit semantics:
//     float iTerm = ki_ * integral_;
//     iTerm = clamp(iTerm, -integralLimit_, integralLimit_);

//     // ---------- Derivative on measurement (reduces setpoint-kick) ----------
//     // raw derivative: - (meas - prevMeas) / dt  (negative because d(error) = -d(meas) if setpoint constant)
//     float rawD = 0.0f;
//     rawD = -(measurement - prevMeasurement_) / dt;

//     // 1st order low-pass on D: alpha = dt / (tau + dt)
//     float alpha = dt / (tau_d_ + dt);
//     dFiltered_ += alpha * (rawD - dFiltered_);
//     float dTerm = kd_ * dFiltered_;

//     // ---------- Combine ----------
//     float outputUnclamped = pTerm + iTerm + dTerm;

//     // clamp final output
//     float output = clamp(outputUnclamped, -outputLimit_, outputLimit_);

//     // ---------- Anti-windup back-calculation ----------
//     if (antiWindupGain_ > 0.0f)
//     {
//         // difference between unclamped and applied output drives integrator correction
//         float aw = (output - outputUnclamped); // negative if we saturated positive
//         // apply back-calculation scaled by KBW and dt — integrate into raw integral
//         integral_ += antiWindupGain_ * aw * dt;
//         // re-evaluate iTerm after back-calc (and clamp)
//         iTerm = ki_ * integral_;
//         iTerm = clamp(iTerm, -integralLimit_, integralLimit_);
//         // recompute output (optional; here we keep 'output' as final)
//         output = clamp(pTerm + iTerm + dTerm, -outputLimit_, outputLimit_);
//     }

//     // store for next iteration
//     prevError_ = error;
//     prevMeasurement_ = measurement;

//     lastPterm_ = pTerm;
//     lastIterm_ = iTerm;
//     lastDterm_ = dTerm;
//     return output;
// }

// float PIDController::clamp(float v, float lo, float hi) {
//     if (v < lo) return lo;
//     if (v > hi) return hi;
//     return v;
// }


// void PIDController::reset()
// {
//     prevError_ = 0.0f;
//     prevMeasurement_ = 0.0f;
//     integral_ = 0.0f;
//     dFiltered_ = 0.0f;
// }

// // simple setters/getters
// void PIDController::setGains(float kp, float ki, float kd)
// {
//     kp_ = kp;
//     ki_ = ki;
//     kd_ = kd;
// }

// void PIDController::setKp(float kp)
// {
//     kp_ = kp;
// }
// void PIDController::setLimits(float outputLimit, float integralLimit)
// {
//     outputLimit_ = outputLimit;
//     integralLimit_ = integralLimit;
// }
// void PIDController::setDerivativeFilterTau(float tau) { tau_d_ = MAX(tau, 1e-6f); }
// void PIDController::setBackCalculation(float antiWindupGain) { antiWindupGain_ = antiWindupGain; }
// float PIDController::getKp() const { return kp_; }
// float PIDController::getKi() const { return ki_; }
// float PIDController::getKd() const { return kd_; }


#include "PID.h"
#include <Arduino.h>
#define MAX(a,b) ((a) > (b) ? (a) : (b))

PIDController::PIDController(float kp, float ki, float kd,
                             float outputLimit, float integralLimit)
    : kp_(kp), ki_(ki), kd_(kd),
      prevError_(0.0f), prevMeasurement_(0.0f),
      prevDMeasurement_(0.0f),               // NEW — init to zero
      integral_(0.0f), dFiltered_(0.0f),
      outputLimit_(outputLimit), integralLimit_(integralLimit),
      tau_d_(0.02f), antiWindupGain_(0.0f)
{
}

// ── Original overload — unchanged behaviour ─────────────────────────────────
float PIDController::compute(float setpoint, float measurement,
                             float dt, bool integrate)
{
    // measurement used for both P and D — same as before
    return computeInternal(setpoint, measurement, measurement, dt, integrate);
}

// ── New overload — separate D measurement ───────────────────────────────────
float PIDController::compute(float setpoint, float measurement,
                             float d_measurement,
                             float dt, bool integrate)
{
    return computeInternal(setpoint, measurement, d_measurement, dt, integrate);
}

// ── Shared implementation ────────────────────────────────────────────────────
float PIDController::computeInternal(float setpoint, float measurement,
                                     float d_measurement,
                                     float dt, bool integrate)
{
    if (!(dt > 0.0f)) return 0.0f;
    if (dt > 0.1f)    dt = 0.1f;

    float error = setpoint - measurement;       // P: error from fast signal
    float pTerm = kp_ * error;

    // Integral — trapezoidal
    if (integrate) {
        integral_ += 0.5f * (error + prevError_) * dt;
    }
    if (ki_ != 0.0f) {
        integral_ = clamp(integral_, -integralLimit_ / ki_, integralLimit_ / ki_);
    } else {
        integral_ = 0.0f;
    }
    float iTerm = clamp(ki_ * integral_, -integralLimit_, integralLimit_);

    // D: derivative on d_measurement (the more filtered signal)
    float rawD    = -(d_measurement - prevDMeasurement_) / dt;
    float alpha   = dt / (tau_d_ + dt);
    dFiltered_   += alpha * (rawD - dFiltered_);
    float dTerm   = kd_ * dFiltered_;

    float outputUnclamped = pTerm + iTerm + dTerm;
    float output          = clamp(outputUnclamped, -outputLimit_, outputLimit_);

    // Anti-windup back-calculation
    if (antiWindupGain_ > 0.0f) {
        float aw = output - outputUnclamped;
        integral_ += antiWindupGain_ * aw * dt;
        iTerm  = clamp(ki_ * integral_, -integralLimit_, integralLimit_);
        output = clamp(pTerm + iTerm + dTerm, -outputLimit_, outputLimit_);
    }

    // Store state
    prevError_        = error;
    prevMeasurement_  = measurement;
    prevDMeasurement_ = d_measurement;           // NEW

    lastPterm_ = pTerm;
    lastIterm_ = iTerm;
    lastDterm_ = dTerm;
    return output;
}

void PIDController::reset()
{
    prevError_        = 0.0f;
    prevMeasurement_  = 0.0f;
    prevDMeasurement_ = 0.0f;        // NEW
    integral_         = 0.0f;
    dFiltered_        = 0.0f;
    lastPterm_        = 0.0f;
    lastIterm_        = 0.0f;
    lastDterm_        = 0.0f;
}

// rest of setters/getters unchanged
float PIDController::clamp(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
void PIDController::setGains(float kp, float ki, float kd) { kp_=kp; ki_=ki; kd_=kd; }
void PIDController::setKp(float kp) { kp_ = kp; }
void PIDController::setLimits(float o, float i) { outputLimit_=o; integralLimit_=i; }
void PIDController::setDerivativeFilterTau(float tau) { tau_d_ = MAX(tau, 1e-6f); }
void PIDController::setBackCalculation(float g) { antiWindupGain_ = g; }
float PIDController::getKp() const { return kp_; }
float PIDController::getKi() const { return ki_; }
float PIDController::getKd() const { return kd_; }