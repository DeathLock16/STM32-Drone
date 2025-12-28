#include "Madgwick.h"

static inline float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

void Madgwick_Init(Madgwick_t* m, float beta)
{
    m->q0 = 1.0f;
    m->q1 = 0.0f;
    m->q2 = 0.0f;
    m->q3 = 0.0f;
    m->beta = beta;
}

void Madgwick_UpdateIMU(
    Madgwick_t* m,
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float dt)
{
    float q0 = m->q0;
    float q1 = m->q1;
    float q2 = m->q2;
    float q3 = m->q3;

    float recipNorm;
    float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;
    float qDot1, qDot2, qDot3, qDot4;

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        s0 = 4.0f*q0*q2*q2 + 2.0f*q2*ax + 4.0f*q0*q1*q1 - 2.0f*q1*ay;
        s1 = 4.0f*q1*q3*q3 - 2.0f*q3*ax + 4.0f*q0*q0*q1 - 2.0f*q0*ay;
        s2 = 4.0f*q0*q0*q2 + 2.0f*q0*ax + 4.0f*q2*q3*q3 - 2.0f*q3*ay;
        s3 = 4.0f*q1*q1*q3 - 2.0f*q1*ax + 4.0f*q2*q2*q3 - 2.0f*q2*ay;

        recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
    }

    qDot1 = 0.5f * (-q1*gx - q2*gy - q3*gz) - m->beta * s0;
    qDot2 = 0.5f * ( q0*gx + q2*gz - q3*gy) - m->beta * s1;
    qDot3 = 0.5f * ( q0*gy - q1*gz + q3*gx) - m->beta * s2;
    qDot4 = 0.5f * ( q0*gz + q1*gy - q2*gx) - m->beta * s3;

    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    m->q0 = q0 * recipNorm;
    m->q1 = q1 * recipNorm;
    m->q2 = q2 * recipNorm;
    m->q3 = q3 * recipNorm;
}
