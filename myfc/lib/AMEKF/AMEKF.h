#ifndef AMEKF_H
#define AMEKF_H

#include <fc_core.h>

#define PI_F 3.14159265358979323846f
#define DEG2RAD_F (PI_F / 180.0f)
#define RAD2DEG_F (180.0f / PI_F)

class AMEKF
{
public:
    struct Config
    {
        bool gyroInDegrees;       // true if gyro input is deg/s
        float accelReferenceNorm; // 1.0 if accel is in g, 9.80665 if m/s^2
        float accelGateMin;       // update allowed if norm/ref >= min
        float accelGateMax;       // update allowed if norm/ref <= max

        float accelNoise;       // normalized accel vector noise, typical 0.05-0.20
        float gyroNoiseRad;     // gyro noise, rad/s
        float gyroBiasNoiseRad; // gyro bias random walk, rad/s/sqrt(s)

        float initialAngleStdRad; // initial attitude uncertainty
        float initialBiasStdRad;  // initial gyro-bias uncertainty

        float accelGravitySign; // +1 if level accel reads +Z, -1 if level accel reads -Z

        // --- Magnetometer ---
        // Measurement noise std dev (normalized units). Start around 0.1–0.3.
        // Higher = trust mag less, gyro/accel dominate. Lower = mag pulls harder.
        float magNoise;

        // Raw magnitude of your magnetometer at your location (in whatever units
        // the QMC returns — typically uT or raw LSB). Set 0.0f to disable the gate
        // entirely and auto-set reference from first valid reading.
        float magReferenceNorm;

        // Reject update if (mag_norm / magReferenceNorm) falls outside this range.
        // Wider than accel gate because hard-iron distortion varies more.
        float magGateMin;
        float magGateMax;

        // Soft trust scaling for accel update.
        // Outside [accelHardRejectMin, accelHardRejectMax] -> reject entirely (crash/garbage).
        // Inside that range, trust degrades smoothly the further ratio is from 1.0.
        float accelHardRejectMin; // e.g. 0.50f
        float accelHardRejectMax; // e.g. 1.60f
        float accelInflationGain; // how aggressively rAccel grows with deviation, start ~5.0f
    };

    AMEKF();

    void begin();
    void setConfig(const Config &cfg);
    Config getConfig() const;

    void reset();

    // AMEKF.h — add to public section
    int initFromIMU(const Vector3f &accel, const Vector3f &mag);
    bool initFromAccel(const Vector3f &accel);

    // Fast 1 kHz prediction from gyro.
    void predict(const Vector3f &gyro, float dt);

    // 500 Hz correction from accelerometer gravity direction.
    bool updateAccel(const Vector3f &accel);

    // 100 Hz correction from magnetometer heading direction
    // Call with raw QMC magnetometer data (body frame, any consistent unit).
    // Returns true if the update was applied, false if gated out.
    bool updateMag(const Vector3f &mag);

    // Manually set the reference field direction in the world/NED frame.
    // Call this if you know your local magnetic field vector (e.g. from WMM model).
    // Input does NOT need to be normalized — the function normalizes it.
    void setMagReference(const Vector3f &magWorld);

    // Force the reference to be re-captured on the next valid updateMag() call.
    void resetMagReference();

    bool isMagReferenceSet() const;
    float getLastMagNorm() const;
    bool wasLastMagUpdateAccepted() const;

    Quaternionf getQuaternion() const;
    Vector3f getEulerRad() const;
    Vector3f getEulerDeg() const;

    Vector3f getGyroBiasRad() const;
    Vector3f getGyroBiasDeg() const;

    void setQuaternion(const Quaternionf &qNew);
    void setGyroBiasRad(const Vector3f &biasRad);

    float getLastAccelNorm() const;
    bool wasLastAccelUpdateAccepted() const;

private:
    Quaternionf q;     // body -> world attitude quaternion
    Vector3f gyroBias; // rad/s

    // Error-state covariance:
    // [dtheta_x, dtheta_y, dtheta_z, dbias_x, dbias_y, dbias_z]
    float P[6][6];

    Config config;

    float lastAccelNorm;
    bool lastAccelAccepted;

    // Magnetic field reference direction stored in world (NED) frame, normalized.
    // Auto-set from first valid updateMag() call, or set manually via setMagReference().
    Vector3f magReference;
    bool magReferenceSet;
    float lastMagNorm;
    bool lastMagAccepted;

    void resetCovariance();
    void applyErrorState(const float dx[6]);
    void symmetrizeP();
};

#endif