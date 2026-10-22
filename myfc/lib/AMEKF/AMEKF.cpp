#include "AMEKF.h"
#include <math.h>
#include <string.h>

static float sqr(float x)
{
    return x * x;
}

static Vector3f v3(float x, float y, float z)
{
    Vector3f v = {x, y, z};
    return v;
}

static Vector3f sub(const Vector3f &a, const Vector3f &b)
{
    return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}

static Vector3f scale(const Vector3f &a, float s)
{
    return v3(a.x * s, a.y * s, a.z * s);
}

static float dot(const Vector3f &a, const Vector3f &b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static float norm(const Vector3f &a)
{
    return sqrtf(dot(a, a));
}

static bool normalize(Vector3f &a)
{
    float n = norm(a);

    if (n < 1.0e-9f || !isfinite(n))
    {
        return false;
    }

    float inv = 1.0f / n;
    a.x *= inv;
    a.y *= inv;
    a.z *= inv;

    return true;
}

static Quaternionf quat(float w, float x, float y, float z)
{
    Quaternionf q = {w, x, y, z};
    return q;
}

static Quaternionf quatMul(const Quaternionf &a, const Quaternionf &b)
{
    return quat(
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w);
}

static Quaternionf quatConj(const Quaternionf &q)
{
    return quat(q.w, -q.x, -q.y, -q.z);
}

static void quatNormalize(Quaternionf &q)
{
    float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);

    if (n < 1.0e-9f || !isfinite(n))
    {
        q = quat(1.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    float inv = 1.0f / n;

    q.w *= inv;
    q.x *= inv;
    q.y *= inv;
    q.z *= inv;

    // q and -q are the same attitude. Keep one consistent hemisphere.
    if (q.w < 0.0f)
    {
        q.w = -q.w;
        q.x = -q.x;
        q.y = -q.y;
        q.z = -q.z;
    }
}

static Quaternionf deltaQuatFromRotationVector(const Vector3f &rv)
{
    float theta = norm(rv);

    if (theta < 1.0e-6f)
    {
        // dq ~= [1, 0.5 * rv]
        Quaternionf dq = quat(
            1.0f,
            0.5f * rv.x,
            0.5f * rv.y,
            0.5f * rv.z);

        quatNormalize(dq);
        return dq;
    }

    float half = 0.5f * theta;
    float s = sinf(half) / theta;

    return quat(
        cosf(half),
        rv.x * s,
        rv.y * s,
        rv.z * s);
}

static Vector3f rotateWorldToBody(const Quaternionf &qBodyToWorld, const Vector3f &vWorld)
{
    // vBody = q^-1 * vWorld * q
    Quaternionf vq = quat(0.0f, vWorld.x, vWorld.y, vWorld.z);
    Quaternionf r = quatMul(quatMul(quatConj(qBodyToWorld), vq), qBodyToWorld);

    return v3(r.x, r.y, r.z);
}

// Rotate a vector from body frame to world frame.
// vWorld = q * vBody * q^-1
static Vector3f rotateBodyToWorld(const Quaternionf &qBodyToWorld, const Vector3f &vBody)
{
    Quaternionf vq = quat(0.0f, vBody.x, vBody.y, vBody.z);
    Quaternionf r = quatMul(quatMul(qBodyToWorld, vq), quatConj(qBodyToWorld));
    return v3(r.x, r.y, r.z);
}

static Quaternionf quatFromEulerRad(float roll, float pitch, float yaw)
{
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);

    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    Quaternionf q;

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    quatNormalize(q);

    return q;
}

static void skew(const Vector3f &v, float S[3][3])
{
    S[0][0] = 0.0f;
    S[0][1] = -v.z;
    S[0][2] = v.y;

    S[1][0] = v.z;
    S[1][1] = 0.0f;
    S[1][2] = -v.x;

    S[2][0] = -v.y;
    S[2][1] = v.x;
    S[2][2] = 0.0f;
}

static bool invert3x3(const float A[3][3], float invA[3][3])
{
    float a00 = A[0][0];
    float a01 = A[0][1];
    float a02 = A[0][2];

    float a10 = A[1][0];
    float a11 = A[1][1];
    float a12 = A[1][2];

    float a20 = A[2][0];
    float a21 = A[2][1];
    float a22 = A[2][2];

    float c00 = a11 * a22 - a12 * a21;
    float c01 = -(a10 * a22 - a12 * a20);
    float c02 = a10 * a21 - a11 * a20;

    float c10 = -(a01 * a22 - a02 * a21);
    float c11 = a00 * a22 - a02 * a20;
    float c12 = -(a00 * a21 - a01 * a20);

    float c20 = a01 * a12 - a02 * a11;
    float c21 = -(a00 * a12 - a02 * a10);
    float c22 = a00 * a11 - a01 * a10;

    float det = a00 * c00 + a01 * c01 + a02 * c02;

    if (fabsf(det) < 1.0e-12f || !isfinite(det))
    {
        return false;
    }

    float invDet = 1.0f / det;

    invA[0][0] = c00 * invDet;
    invA[0][1] = c10 * invDet;
    invA[0][2] = c20 * invDet;

    invA[1][0] = c01 * invDet;
    invA[1][1] = c11 * invDet;
    invA[1][2] = c21 * invDet;

    invA[2][0] = c02 * invDet;
    invA[2][1] = c12 * invDet;
    invA[2][2] = c22 * invDet;

    return true;
}

AMEKF::AMEKF()
{
    config.gyroInDegrees = true;

    // Use 1.0 if your accel_data is in g.
    // Use 9.80665 if your accel_data is in m/s^2.
    // config.accelReferenceNorm = 1.0f;
    config.accelReferenceNorm = 0.9756f; // will b overwritten in setup() after real measurement

    // During crashes, vibration, or strong movement, accel norm can be far from 1g.
    // Then accel is not a clean gravity measurement, so skip update.
    config.accelGateMin = 0.70f;
    config.accelGateMax = 1.30f;
    config.accelHardRejectMin = 0.70f;
    config.accelHardRejectMax = 1.30f;
    config.accelInflationGain = 10.0f;

    // Start tuning from here.
    config.accelNoise = 0.10f;         // r_measure
    config.gyroNoiseRad = 0.02f;       // q_angle
    config.gyroBiasNoiseRad = 0.0005f; // q_bias

    config.initialAngleStdRad = 10.0f * DEG2RAD_F;
    config.initialBiasStdRad = 0.5f * DEG2RAD_F;

    // Most MPU setups read +Z when flat and stationary.
    // If yours reads -Z when flat, set this to -1.
    config.accelGravitySign = 1.0f;

    config.magNoise = 0.20f;        // start here; tighten if QMC is stable
    config.magReferenceNorm = 0.0f; // 0 = gate disabled, auto-capture reference
    config.magGateMin = 0.50f;
    config.magGateMax = 2.00f;

    reset();
}

void AMEKF::begin()
{
    reset();
}

void AMEKF::setConfig(const Config &cfg)
{
    config = cfg;
}

AMEKF::Config AMEKF::getConfig() const
{
    return config;
}

void AMEKF::reset()
{
    q = quat(1.0f, 0.0f, 0.0f, 0.0f);
    gyroBias = v3(0.0f, 0.0f, 0.0f);

    lastAccelNorm = 0.0f;
    lastAccelAccepted = false;

    magReferenceSet = false;
    magReference = v3(0.0f, 0.0f, 0.0f);
    lastMagNorm = 0.0f;
    lastMagAccepted = false;

    resetCovariance();
}

void AMEKF::resetCovariance()
{
    memset(P, 0, sizeof(P));

    float a = sqr(config.initialAngleStdRad);
    float b = sqr(config.initialBiasStdRad);

    P[0][0] = a;
    P[1][1] = a;
    P[2][2] = a;

    P[3][3] = b;
    P[4][4] = b;
    P[5][5] = b;
}

bool AMEKF::initFromAccel(const Vector3f &accel)
{
    Vector3f a = accel;

    lastAccelNorm = norm(a);

    if (!normalize(a))
    {
        return false;
    }

    if (config.accelGravitySign < 0.0f)
    {
        a.x = -a.x;
        a.y = -a.y;
        a.z = -a.z;
    }

    float roll = atan2f(a.y, a.z);
    float pitch = atan2f(-a.x, sqrtf(a.y * a.y + a.z * a.z));
    float yaw = 0.0f;

    q = quatFromEulerRad(roll, pitch, yaw);

    gyroBias = v3(0.0f, 0.0f, 0.0f);

    resetCovariance();

    return true;
}

int AMEKF::initFromIMU(const Vector3f &accel, const Vector3f &mag)
{
    // ── Step 1: roll and pitch from accelerometer ─────────────────────────
    Vector3f a = accel;
    lastAccelNorm = norm(a);

    if (!normalize(a))
        return 1;

    if (config.accelGravitySign < 0.0f)
    {
        a.x = -a.x;
        a.y = -a.y;
        a.z = -a.z;
    }

    float roll = atan2f(a.y, a.z);
    float pitch = atan2f(-a.x, sqrtf(a.y * a.y + a.z * a.z));

    // ── Step 2: tilt-compensated yaw from magnetometer ────────────────────
    Vector3f m = mag;

    if (!normalize(m))
        return 2;

    float cr = cosf(roll);
    float sr = sinf(roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);

    float mx = m.x * cp + m.y * sr * sp + m.z * cr * sp;
    float my = m.y * cr - m.z * sr;

    float yaw = atan2f(-my, mx);

    // ── Step 3: build initial quaternion ──────────────────────────────────
    q = quatFromEulerRad(roll, pitch, yaw);
    quatNormalize(q);

    gyroBias = v3(0.0f, 0.0f, 0.0f);
    resetCovariance();

    // ── Step 4: set mag reference in world frame ───────────────────────────
    // q is now set, rotate the normalized body-frame mag into world frame.
    // This pre-arms the reference so the first updateMag() call doesn't
    // auto-capture from a potentially drifted post-init state.
    magReference = rotateBodyToWorld(q, m);

    if (!normalize(magReference))
    {
        magReferenceSet = false;
        return 3;
    }

    magReferenceSet = true;

    return 0;
}
// ── paste at the top of AMEKF.cpp ────────────────────────────
// static uint32_t _diagCounter = 0;
// #include <Arduino.h>
void AMEKF::predict(const Vector3f &gyroInput, float dt)
{
    if (dt <= 0.0f || dt > 0.1f || !isfinite(dt))
    {
        return;
    }

    Vector3f gyro = gyroInput;

    if (config.gyroInDegrees)
    {
        gyro = scale(gyro, DEG2RAD_F);
    }

    Vector3f omega = sub(gyro, gyroBias);

    // Nominal quaternion propagation:
    // q_body_to_world = q * exp(omega_body * dt)
    Vector3f rv = scale(omega, dt);
    Quaternionf dq = deltaQuatFromRotationVector(rv);

    q = quatMul(q, dq);
    quatNormalize(q);

    // Covariance propagation.
    // Error state:
    // [dtheta_x, dtheta_y, dtheta_z, dbias_x, dbias_y, dbias_z]
    float W[3][3];
    skew(omega, W);

    float Phi[6][6];
    memset(Phi, 0, sizeof(Phi));

    for (int i = 0; i < 6; i++)
    {
        Phi[i][i] = 1.0f;
    }

    // dtheta_dot ~= -skew(omega) * dtheta - dbias
    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 3; c++)
        {
            Phi[r][c] += -W[r][c] * dt;
        }

        Phi[r][r + 3] = -dt;
    }

    float FP[6][6];
    float Pnew[6][6];

    memset(FP, 0, sizeof(FP));
    memset(Pnew, 0, sizeof(Pnew));

    for (int r = 0; r < 6; r++)
    {
        for (int c = 0; c < 6; c++)
        {
            for (int k = 0; k < 6; k++)
            {
                FP[r][c] += Phi[r][k] * P[k][c];
            }
        }
    }

    for (int r = 0; r < 6; r++)
    {
        for (int c = 0; c < 6; c++)
        {
            for (int k = 0; k < 6; k++)
            {
                Pnew[r][c] += FP[r][k] * Phi[c][k];
            }
        }
    }

    float qAngle = sqr(config.gyroNoiseRad) * dt;
    float qBias = sqr(config.gyroBiasNoiseRad) * dt;

    Pnew[0][0] += qAngle;
    Pnew[1][1] += qAngle;
    Pnew[2][2] += qAngle;

    Pnew[3][3] += qBias;
    Pnew[4][4] += qBias;
    Pnew[5][5] += qBias;

    memcpy(P, Pnew, sizeof(P));

    symmetrizeP();

    // // ── DIAG ─────────────────────────────────────────────────────────────────
    // // prints once per second at 1000 Hz
    // if ((_diagCounter % 1000) == 0)
    // {
    //     Serial.print(F("[PRED] t="));
    //     Serial.print(micros());
    //     Serial.print(F("  om="));
    //     Serial.print(omega.x, 4);
    //     Serial.print(',');
    //     Serial.print(omega.y, 4);
    //     Serial.print(',');
    //     Serial.print(omega.z, 4);
    //     // bias in deg/s — is it converging and staying put?
    //     Serial.print(F("  bias_deg="));
    //     Serial.print(gyroBias.x * RAD2DEG_F, 4);
    //     Serial.print(',');
    //     Serial.print(gyroBias.y * RAD2DEG_F, 4);
    //     Serial.print(',');
    //     Serial.print(gyroBias.z * RAD2DEG_F, 4);
    //     // P diag: attitude std-devs (deg) and bias std-devs (deg/s)
    //     // sigma_att should settle < 1 deg, sigma_bias should shrink slowly
    //     Serial.print(F("  sAtt_deg="));
    //     Serial.print(sqrtf(P[0][0]) * RAD2DEG_F, 3);
    //     Serial.print(',');
    //     Serial.print(sqrtf(P[1][1]) * RAD2DEG_F, 3);
    //     Serial.print(',');
    //     Serial.print(sqrtf(P[2][2]) * RAD2DEG_F, 3);
    //     Serial.print(F("  sBias_deg="));
    //     Serial.print(sqrtf(P[3][3]) * RAD2DEG_F, 4);
    //     Serial.print(',');
    //     Serial.print(sqrtf(P[4][4]) * RAD2DEG_F, 4);
    //     Serial.print(',');
    //     Serial.print(sqrtf(P[5][5]) * RAD2DEG_F, 4);
    //     Serial.println();
    // }
    // _diagCounter++;
    // // ─────────────────────────────────────────────────────────────────────────
}

bool AMEKF::updateAccel(const Vector3f &accelInput)
{
    Vector3f z = accelInput;

    lastAccelNorm = norm(z);
    lastAccelAccepted = false;

    float accelTrustInflation = 1.0f; // 1.0 = full trust, grows as accel deviates from 1g

    if (config.accelReferenceNorm > 1.0e-6f)
    {
        float ratio = lastAccelNorm / config.accelReferenceNorm;

        // Hard reject outside the permitted range
        if (ratio < config.accelHardRejectMin ||
            ratio > config.accelHardRejectMax)
        {
            return false;
        }

        // Quadratic R inflation:
        // Minimum at ratio = 1.0
        // Maximum at either hard-rejection boundary
        float deviation = ratio - 1.0f;

        float normalizedDeviation;
        if (deviation < 0.0f)
        {
            normalizedDeviation = deviation / (config.accelHardRejectMin - 1.0f);
        }
        else
        {
            normalizedDeviation = deviation / (config.accelHardRejectMax - 1.0f);
        }

        float accelTrustInflation = 1.0f + (config.accelInflationGain - 1.0f) * sqr(normalizedDeviation);
    }

    if (!normalize(z))
    {
        return false;
    }

    Vector3f gWorld = v3(
        0.0f,
        0.0f,
        config.accelGravitySign >= 0.0f ? 1.0f : -1.0f);

    Vector3f h = rotateWorldToBody(q, gWorld);

    if (!normalize(h))
    {
        return false;
    }

    // Innovation: measured gravity direction - predicted gravity direction.
    float y[3] = {
        z.x - h.x,
        z.y - h.y,
        z.z - h.z};

    // Measurement Jacobian:
    // H = [skew(h), 0]
    float H[3][6];
    memset(H, 0, sizeof(H));

    float Sh[3][3];
    skew(h, Sh);

    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 3; c++)
        {
            H[r][c] = Sh[r][c];
        }
    }

    // S = H P H' + R
    float HP[3][6];
    memset(HP, 0, sizeof(HP));

    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 6; c++)
        {
            for (int k = 0; k < 6; k++)
            {
                HP[r][c] += H[r][k] * P[k][c];
            }
        }
    }

    float S[3][3];
    memset(S, 0, sizeof(S));

    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 3; c++)
        {
            for (int k = 0; k < 6; k++)
            {
                S[r][c] += HP[r][k] * H[c][k];
            }
        }
    }

    // float rAccel = sqr(config.accelNoise);
    float rAccel = sqr(config.accelNoise) * accelTrustInflation;

    S[0][0] += rAccel;
    S[1][1] += rAccel;
    S[2][2] += rAccel;

    float Sinv[3][3];

    if (!invert3x3(S, Sinv))
    {
        return false;
    }

    // K = P H' S^-1
    float PHt[6][3];
    memset(PHt, 0, sizeof(PHt));

    for (int r = 0; r < 6; r++)
    {
        for (int c = 0; c < 3; c++)
        {
            for (int k = 0; k < 6; k++)
            {
                PHt[r][c] += P[r][k] * H[c][k];
            }
        }
    }

    float K[6][3];
    memset(K, 0, sizeof(K));

    for (int r = 0; r < 6; r++)
    {
        for (int c = 0; c < 3; c++)
        {
            for (int k = 0; k < 3; k++)
            {
                K[r][c] += PHt[r][k] * Sinv[k][c];
            }
        }
    }

    float dx[6] = {0, 0, 0, 0, 0, 0};

    for (int r = 0; r < 6; r++)
    {
        for (int k = 0; k < 3; k++)
        {
            dx[r] += K[r][k] * y[k];
        }
    }

    // // ── DIAG ─────────────────────────────────────────────────────────────────
    // if ((_diagCounter % 500) == 0) // 500 Hz rate → once per second
    // {
    //     Serial.print(F("[ACCEL] t="));
    //     Serial.print(micros());
    //     // innovation: how far measured gravity direction is from predicted
    //     // should be small (<0.1) when settled, not zero (then accel is rejected)
    //     Serial.print(F("  innov="));
    //     Serial.print(y[0], 4);
    //     Serial.print(',');
    //     Serial.print(y[1], 4);
    //     Serial.print(',');
    //     Serial.print(y[2], 4);
    //     // trust inflation: 1.0 = full trust, high = accel is suspect
    //     Serial.print(F("  trust="));
    //     Serial.print(accelTrustInflation, 3);
    //     // K rows for bias states: THIS is what drives bias estimation
    //     // near zero = filter is NOT correcting bias from accel, bad sign
    //     Serial.print(F("  K_bias="));
    //     Serial.print(K[3][0], 5);
    //     Serial.print(',');
    //     Serial.print(K[4][1], 5);
    //     Serial.print(',');
    //     Serial.print(K[5][2], 5);
    //     // bias correction applied this step
    //     Serial.print(F("  dx_bias="));
    //     Serial.print(dx[3] * RAD2DEG_F, 5);
    //     Serial.print(',');
    //     Serial.print(dx[4] * RAD2DEG_F, 5);
    //     Serial.print(',');
    //     Serial.print(dx[5] * RAD2DEG_F, 5);
    //     Serial.println();
    // }
    // // ─────────────────────────────────────────────────────────────────────────

    applyErrorState(dx);

    // Joseph covariance update:
    // P = (I - K H) P (I - K H)' + K R K'
    float KH[6][6];
    memset(KH, 0, sizeof(KH));

    for (int r = 0; r < 6; r++)
    {
        for (int c = 0; c < 6; c++)
        {
            for (int k = 0; k < 3; k++)
            {
                KH[r][c] += K[r][k] * H[k][c];
            }
        }
    }

    float A[6][6];
    memset(A, 0, sizeof(A));

    for (int r = 0; r < 6; r++)
    {
        for (int c = 0; c < 6; c++)
        {
            A[r][c] = -KH[r][c];
        }

        A[r][r] += 1.0f;
    }

    float AP[6][6];
    float Pnew[6][6];

    memset(AP, 0, sizeof(AP));
    memset(Pnew, 0, sizeof(Pnew));

    for (int r = 0; r < 6; r++)
    {
        for (int c = 0; c < 6; c++)
        {
            for (int k = 0; k < 6; k++)
            {
                AP[r][c] += A[r][k] * P[k][c];
            }
        }
    }

    for (int r = 0; r < 6; r++)
    {
        for (int c = 0; c < 6; c++)
        {
            for (int k = 0; k < 6; k++)
            {
                Pnew[r][c] += AP[r][k] * A[c][k];
            }
        }
    }

    for (int r = 0; r < 6; r++)
    {
        for (int c = 0; c < 6; c++)
        {
            float KRKt = 0.0f;

            for (int k = 0; k < 3; k++)
            {
                KRKt += K[r][k] * rAccel * K[c][k];
            }

            Pnew[r][c] += KRKt;
        }
    }

    memcpy(P, Pnew, sizeof(P));

    symmetrizeP();

    lastAccelAccepted = true;

    return true;
}

void AMEKF::applyErrorState(const float dx[6])
{
    Vector3f dtheta = v3(dx[0], dx[1], dx[2]);

    Quaternionf dq = deltaQuatFromRotationVector(dtheta);

    // Right-multiplicative correction for body-frame small attitude error.
    q = quatMul(q, dq);
    quatNormalize(q);

    gyroBias.x += dx[3];
    gyroBias.y += dx[4];
    gyroBias.z += dx[5];
}

bool AMEKF::updateMag(const Vector3f &magInput)
{
    Vector3f z = magInput;
    lastMagNorm = norm(z);
    lastMagAccepted = false;

    // ── Magnitude gate ──────────────────────────────────────────────────────
    if (config.magReferenceNorm > 1.0e-6f)
    {
        float ratio = lastMagNorm / config.magReferenceNorm;

        if (ratio < config.magGateMin || ratio > config.magGateMax)
        {
            return false;
        }
    }

    if (!normalize(z))
    {
        return false;
    }

    // ── Auto-capture reference on first valid reading ────────────────────────
    if (!magReferenceSet)
    {
        magReference = rotateBodyToWorld(q, z);

        if (!normalize(magReference))
        {
            return false;
        }

        magReferenceSet = true;
    }

    // ── Predicted measurement ────────────────────────────────────────────────
    Vector3f h = rotateWorldToBody(q, magReference);

    if (!normalize(h))
    {
        return false;
    }

    // ── Innovation ───────────────────────────────────────────────────────────
    float y[3] = {
        z.x - h.x,
        z.y - h.y,
        z.z - h.z};

    // ── Measurement Jacobian  H = [ skew(h) | 0_{3x3} ] ─────────────────────
    float H[3][6];
    memset(H, 0, sizeof(H));

    float Sh[3][3];
    skew(h, Sh);

    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 3; c++)
        {
            H[r][c] = Sh[r][c];
        }
    }

    // ── Innovation covariance  S = H P H' + R ───────────────────────────────
    float HP[3][6];
    memset(HP, 0, sizeof(HP));

    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 6; c++)
        {
            for (int k = 0; k < 6; k++)
            {
                HP[r][c] += H[r][k] * P[k][c];
            }
        }
    }

    float S[3][3];
    memset(S, 0, sizeof(S));

    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 3; c++)
        {
            for (int k = 0; k < 6; k++)
            {
                S[r][c] += HP[r][k] * H[c][k];
            }
        }
    }

    float rMag = sqr(config.magNoise);

    S[0][0] += rMag;
    S[1][1] += rMag;
    S[2][2] += rMag;

    float Sinv[3][3];

    if (!invert3x3(S, Sinv))
    {
        return false;
    }

    // ── Kalman gain  K = P H' S^-1 ──────────────────────────────────────────
    float PHt[6][3];
    memset(PHt, 0, sizeof(PHt));

    for (int r = 0; r < 6; r++)
    {
        for (int c = 0; c < 3; c++)
        {
            for (int k = 0; k < 6; k++)
            {
                PHt[r][c] += P[r][k] * H[c][k];
            }
        }
    }

    float K[6][3];
    memset(K, 0, sizeof(K));

    for (int r = 0; r < 6; r++)
    {
        for (int c = 0; c < 3; c++)
        {
            for (int k = 0; k < 3; k++)
            {
                K[r][c] += PHt[r][k] * Sinv[k][c];
            }
        }
    }

    // ── Yaw-only correction ──────────────────────────────────────────────────
    // Mag is noisy; roll (δθx) and pitch (δθy) are left entirely to the
    // accelerometer.  Zeroing rows 0 and 1 of K before applying dx and the
    // covariance update achieves this cleanly: the Joseph form P = (I-KH)P(I-KH)'+KRK'
    // is valid for any K, so P[0][0] and P[1][1] are simply not shrunk by this
    // measurement — consistent with mag not observing those states.
    for (int c = 0; c < 3; c++)
    {
        K[0][c] = 0.0f; // δθx (roll)  — corrected by accel only
        K[1][c] = 0.0f; // δθy (pitch) — corrected by accel only
    }

    // ── Error state correction  dx = K * y ──────────────────────────────────
    float dx[6] = {0, 0, 0, 0, 0, 0};

    for (int r = 0; r < 6; r++)
    {
        for (int k = 0; k < 3; k++)
        {
            dx[r] += K[r][k] * y[k];
        }
    }

    // // ── DIAG ─────────────────────────────────────────────────────────────────
    //     if ((_diagCounter % 100) == 0)   // 100 Hz rate → once per second
    // {
    //     Serial.print(F("[MAG] t="));
    //     Serial.print(micros());
    //     Serial.print(F("  innov="));
    //     Serial.print(y[0], 4);
    //     Serial.print(',');
    //     Serial.print(y[1], 4);
    //     Serial.print(',');
    //     Serial.print(y[2], 4);
    //     // K[2] is the only non-zeroed attitude row (yaw correction)
    //     // K[0] and K[1] were zeroed intentionally — confirm here
    //     Serial.print(F("  K_yaw="));
    //     Serial.print(K[2][0], 5);
    //     Serial.print(F("  dx_yaw="));
    //     Serial.print(dx[2] * RAD2DEG_F, 4);
    //     Serial.println();
    // }
    // // ─────────────────────────────────────────────────────────────────────────

    applyErrorState(dx);

    // ── Joseph-form covariance update  P = (I-KH) P (I-KH)' + K R K' ───────
    float KH[6][6];
    memset(KH, 0, sizeof(KH));

    for (int r = 0; r < 6; r++)
    {
        for (int c = 0; c < 6; c++)
        {
            for (int k = 0; k < 3; k++)
            {
                KH[r][c] += K[r][k] * H[k][c];
            }
        }
    }

    float A[6][6];
    memset(A, 0, sizeof(A));

    for (int r = 0; r < 6; r++)
    {
        for (int c = 0; c < 6; c++)
        {
            A[r][c] = -KH[r][c];
        }

        A[r][r] += 1.0f;
    }

    float AP[6][6];
    float Pnew[6][6];

    memset(AP, 0, sizeof(AP));
    memset(Pnew, 0, sizeof(Pnew));

    for (int r = 0; r < 6; r++)
    {
        for (int c = 0; c < 6; c++)
        {
            for (int k = 0; k < 6; k++)
            {
                AP[r][c] += A[r][k] * P[k][c];
            }
        }
    }

    for (int r = 0; r < 6; r++)
    {
        for (int c = 0; c < 6; c++)
        {
            for (int k = 0; k < 6; k++)
            {
                Pnew[r][c] += AP[r][k] * A[c][k];
            }
        }
    }

    for (int r = 0; r < 6; r++)
    {
        for (int c = 0; c < 6; c++)
        {
            float KRKt = 0.0f;

            for (int k = 0; k < 3; k++)
            {
                KRKt += K[r][k] * rMag * K[c][k];
            }

            Pnew[r][c] += KRKt;
        }
    }

    memcpy(P, Pnew, sizeof(P));

    symmetrizeP();

    lastMagAccepted = true;
    return true;
}

void AMEKF::setMagReference(const Vector3f &magWorld)
{
    magReference = magWorld;

    if (!normalize(magReference))
    {
        magReferenceSet = false;
        return;
    }

    magReferenceSet = true;
}

void AMEKF::resetMagReference()
{
    magReferenceSet = false;
    magReference = v3(0.0f, 0.0f, 0.0f);
}

bool AMEKF::isMagReferenceSet() const
{
    return magReferenceSet;
}

float AMEKF::getLastMagNorm() const
{
    return lastMagNorm;
}

bool AMEKF::wasLastMagUpdateAccepted() const
{
    return lastMagAccepted;
}

void AMEKF::symmetrizeP()
{
    for (int r = 0; r < 6; r++)
    {
        for (int c = r + 1; c < 6; c++)
        {
            float v = 0.5f * (P[r][c] + P[c][r]);

            P[r][c] = v;
            P[c][r] = v;
        }
    }

    for (int i = 0; i < 6; i++)
    {
        if (P[i][i] < 1.0e-12f)
        {
            P[i][i] = 1.0e-12f;
        }
    }
}

Quaternionf AMEKF::getQuaternion() const
{
    return q;
}

Vector3f AMEKF::getEulerRad() const
{
    // ZYX Euler angles:
    // roll around X, pitch around Y, yaw around Z.
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);

    float roll = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (q.w * q.y - q.z * q.x);

    float pitch;

    if (fabsf(sinp) >= 1.0f)
    {
        pitch = copysignf(PI_F / 2.0f, sinp);
    }
    else
    {
        pitch = asinf(sinp);
    }

    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);

    float yaw = atan2f(siny_cosp, cosy_cosp);

    return v3(roll, pitch, yaw);
}

Vector3f AMEKF::getEulerDeg() const
{
    Vector3f e = getEulerRad();

    return scale(e, RAD2DEG_F);
}

Vector3f AMEKF::getGyroBiasRad() const
{
    return gyroBias;
}

Vector3f AMEKF::getGyroBiasDeg() const
{
    return scale(gyroBias, RAD2DEG_F);
}

void AMEKF::setQuaternion(const Quaternionf &qNew)
{
    q = qNew;
    quatNormalize(q);

    resetCovariance();
}

void AMEKF::setGyroBiasRad(const Vector3f &biasRad)
{
    gyroBias = biasRad;
}

float AMEKF::getLastAccelNorm() const
{
    return lastAccelNorm;
}

bool AMEKF::wasLastAccelUpdateAccepted() const
{
    return lastAccelAccepted;
}