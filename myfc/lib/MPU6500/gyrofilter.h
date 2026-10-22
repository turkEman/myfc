#pragma once

#ifndef M_PI
#define M_PI		3.14159265358979323846
#endif

// first-order IIR low-pass (single-pole RC equivalent)
class PT1Filter {
public:
    PT1Filter() : _alpha(0.0f), _state(0.0f) {}

    // Call once in setup(), or if cutoff/rate ever changes at runtime
    void init(float cutoffHz, float sampleRateHz)
    {
        // Bilinear (Tustin) discretisation — standard for embedded rate loops
        // alpha = (2π·fc·dt) / (1 + 2π·fc·dt)
        float omega = 2.0f * M_PI * cutoffHz;
        float dt    = 1.0f / sampleRateHz;
        _alpha = (omega * dt) / (1.0f + omega * dt);
        _state = 0.0f;
    }

    // Call once per sample — returns filtered value
    inline float update(float input)
    {
        // Incremental form: one multiply + one add, identical to
        // _state = _alpha*input + (1-_alpha)*_state
        _state += _alpha * (input - _state);
        return _state;
    }

    void  reset(float v = 0.0f) { _state = v; }
    float getState()      const { return _state; }
    float getAlpha()      const { return _alpha; }

private:
    float _alpha;
    float _state;
};

#include <math.h>

class BiquadNotch {
public:
    BiquadNotch() : b0(1),b1(0),b2(0),a1(0),a2(0),
                    x1(0),x2(0),y1(0),y2(0) {}

    // centerHz=80, Q=1.5, sampleRateHz=1000
    void init(float centerHz, float Q, float sampleRateHz) {
        float w0    = 2.0f * (float)M_PI * centerHz / sampleRateHz;
        float alpha = sinf(w0) / (2.0f * Q);
        float cw    = cosf(w0);
        float a0inv = 1.0f / (1.0f + alpha);
        b0 =  a0inv;
        b1 = -2.0f * cw * a0inv;
        b2 =  a0inv;
        a1 =  b1;                        // b1 == a1 for notch
        a2 = (1.0f - alpha) * a0inv;
    }

    inline float update(float x) {
        float y = b0*x + b1*x1 + b2*x2
                        - a1*y1 - a2*y2;
        x2=x1; x1=x;
        y2=y1; y1=y;
        return y;
    }

    void reset(float v = 0.0f) {
        x1=x2=y1=y2=v;
    }

private:
    float b0,b1,b2,a1,a2;
    float x1,x2,y1,y2;
};