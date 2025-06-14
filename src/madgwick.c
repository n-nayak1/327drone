/**
 * MadgwickAHRS Filter Implementation in C
 * Written by Milan on 6/2/2025 (thank you Milan!!!)
 * Adapted from: https://github.com/arduino-libraries/MadgwickAHRS
 * Added a shittyIMUFactor to counter gyro dominance at tilts
 */

#include "madgwick.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void MadgwickAHRS_init(MadgwickAHRS* self, float beta, float sampleFreq, float shittyIMUFactor) {
    self->shittyIMUFactor = shittyIMUFactor;
    self->beta = beta;
    self->sampleFreq = sampleFreq;
    self->q0 = 1.0f;
    self->q1 = 0.0f;
    self->q2 = 0.0f;
    self->q3 = 0.0f;
}

void MadgwickAHRS_update(MadgwickAHRS* self, float gx, float gy, float gz,
                         float ax, float ay, float az) {
    float betaIncFactor;
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot0, qDot1, qDot2, qDot3;
    float q0 = self->q0, q1 = self->q1, q2 = self->q2, q3 = self->q3;

    // Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // Normalize accelerometer
    recipNorm = sqrtf(ax * ax + ay * ay + az * az);
    // When the accel has a norm > 1, we are flat, so we want drift correction (higher beta)
    // When the accel has a norm < 1, we are tilted, so we don't want drift correction (lower beta)
    betaIncFactor = recipNorm * self->shittyIMUFactor;
    if (recipNorm == 0.0f) return;
    recipNorm = 1.0f / recipNorm;
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Rate of change of quaternion
    qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Gradient descent corrective step
    float f1 = 2.0f * (q1 * q3 - q0 * q2) - ax;
    float f2 = 2.0f * (q0 * q1 + q2 * q3) - ay;
    float f3 = 2.0f * (0.5f - q1 * q1 - q2 * q2) - az;
    s0 = -2.0f * q2 * f1 + 2.0f * q1 * f2;
    s1 =  2.0f * q3 * f1 + 2.0f * q0 * f2 - 4.0f * q1 * f3;
    s2 = -2.0f * q0 * f1 + 2.0f * q3 * f2 - 4.0f * q2 * f3;
    s3 =  2.0f * q1 * f1 + 2.0f * q2 * f2;

    recipNorm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (recipNorm == 0.0f) return;
    recipNorm = 1.0f / recipNorm;
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot0 -= self->beta * betaIncFactor * s0;
    qDot1 -= self->beta * betaIncFactor * s1;
    qDot2 -= self->beta * betaIncFactor * s2;
    qDot3 -= self->beta * betaIncFactor * s3;

    // Integrate to yield quaternion
    q0 += qDot0 / self->sampleFreq;
    q1 += qDot1 / self->sampleFreq;
    q2 += qDot2 / self->sampleFreq;
    q3 += qDot3 / self->sampleFreq;

    // Normalize quaternion
    recipNorm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    recipNorm = 1.0f / recipNorm;
    self->q0 = q0 * recipNorm;
    self->q1 = q1 * recipNorm;
    self->q2 = q2 * recipNorm;
    self->q3 = q3 * recipNorm;
}

void MadgwickAHRS_getQuaternion(const MadgwickAHRS* self, float* qw, float* qx, float* qy, float* qz) {
    *qw = self->q0;
    *qx = self->q1;
    *qy = self->q2;
    *qz = self->q3;
}

float MadgwickAHRS_getRoll(const MadgwickAHRS* self) {
    float sinr = 2.0f * (self->q0 * self->q1 + self->q2 * self->q3);
    float cosr = 1.0f - 2.0f * (self->q1 * self->q1 + self->q2 * self->q2);
    return atan2f(sinr, cosr) * 180.0f / M_PI;
}

float MadgwickAHRS_getPitch(const MadgwickAHRS* self) {
    float sinp = 2.0f * (self->q0 * self->q2 - self->q3 * self->q1);
    if (fabsf(sinp) >= 1.0f)
        return (sinp > 0.0f ? 1.0f : -1.0f) * (M_PI / 2.0f);
    else
        return asinf(sinp) * 180.0f / M_PI;
}

float MadgwickAHRS_getYaw(const MadgwickAHRS* self) {
    float siny = 2.0f * (self->q0 * self->q3 + self->q1 * self->q2);
    float cosy = 1.0f - 2.0f * (self->q2 * self->q2 + self->q3 * self->q3);
    return atan2f(siny, cosy) * 180.0f / M_PI;
}
