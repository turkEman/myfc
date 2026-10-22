// #ifndef PID_H
// #define PID_H

// class PIDController
// {
// public:
//     PIDController(float kp = 0.0f,
//                   float ki = 0.0f,
//                   float kd = 0.0f,
//                   float outputLimit = 400.0f,
//                   float integralLimit = 400.0f);

//     // compute: add optional integrate flag (false when disarmed or when you want integrator frozen)
//     float compute(float setpoint, float measurement, float dt, bool integrate = true);

//     void reset();

//     // setters
//     void setGains(float kp, float ki, float kd);
//     void setLimits(float outputLimit, float integralLimit);
//     void setDerivativeFilterTau(float tau);         // seconds
//     void setBackCalculation(float antiWindupGain_); // KBW, 0.0 = disabled
//     void setKp(float kp);

//     float getKp() const;
//     float getKi() const;
//     float getKd() const;

//     // Getters — pure reads, zero cost
//     float getPterm() const { return lastPterm_; }
//     float getIterm() const { return lastIterm_; }
//     float getDterm() const { return lastDterm_; }

// private:
//     float kp_, ki_, kd_;
//     float prevError_;
//     float prevMeasurement_;
//     float integral_;  // raw integral: sum(error * dt)
//     float dFiltered_; // filtered derivative (on measurement)
//     float outputLimit_;
//     float integralLimit_;

//     // derivative filter (1-pole): alpha = dt / (tau + dt)
//     float tau_d_; // filter time constant (seconds)

//     // anti-windup via back-calculation:
//     float antiWindupGain_; // KBW, typical small (e.g. 0.1)

// private:
//     float lastPterm_ = 0.0f;
//     float lastIterm_ = 0.0f;
//     float lastDterm_ = 0.0f;

//     static float clamp(float v, float lo, float hi);
// };

// #endif // PID_H

#ifndef PID_H
#define PID_H

class PIDController
{
public:
    PIDController(float kp = 0.0f,
                  float ki = 0.0f,
                  float kd = 0.0f,
                  float outputLimit = 400.0f,
                  float integralLimit = 400.0f);

    // Original signature — unchanged, backward compatible
    float compute(float setpoint, float measurement,
                  float dt, bool integrate = true);

    // New overload — separate D measurement (more filtered signal)
    // P uses measurement, D uses d_measurement
    float compute(float setpoint, float measurement, float d_measurement,
                  float dt, bool integrate = true);

    void reset();

    void setGains(float kp, float ki, float kd);
    void setLimits(float outputLimit, float integralLimit);
    void setDerivativeFilterTau(float tau);
    void setBackCalculation(float antiWindupGain);
    void setKp(float kp);

    float getKp() const;
    float getKi() const;
    float getKd() const;

    float getPterm() const { return lastPterm_; }
    float getIterm() const { return lastIterm_; }
    float getDterm() const { return lastDterm_; }

private:
    float kp_, ki_, kd_;
    float prevError_;
    float prevMeasurement_;     // kept for original compute() compat
    float prevDMeasurement_;    // NEW: separate D history for new overload
    float integral_;
    float dFiltered_;
    float outputLimit_;
    float integralLimit_;
    float tau_d_;
    float antiWindupGain_;

    float lastPterm_ = 0.0f;
    float lastIterm_ = 0.0f;
    float lastDterm_ = 0.0f;

    // Internal shared implementation
    float computeInternal(float setpoint, float measurement,
                          float d_measurement, float dt, bool integrate);

    static float clamp(float v, float lo, float hi);
};

#endif