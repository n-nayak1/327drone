/**
 * MadgwickAHRS Filter Implementation in C
 * Written by Milan on 6/2/2025
 * Adapted from: https://github.com/arduino-libraries/MadgwickAHRS/tree/master
 * Added a shittyIMUFactor to counter gyro dominance at tilts
 */
#ifndef MADGWICK_H
#define MADGWICK_H

typedef struct {
    float beta;
    float shittyIMUFactor;
    float sampleFreq;
    float q0, q1, q2, q3;
} MadgwickAHRS;

void MadgwickAHRS_init(MadgwickAHRS* self, float beta, float sampleFreq, float shittyIMUFactor);
void MadgwickAHRS_update(MadgwickAHRS* self, float gx, float gy, float gz, float ax, float ay, float az);
void MadgwickAHRS_getQuaternion(const MadgwickAHRS* self, float* qw, float* qx, float* qy, float* qz);
float MadgwickAHRS_getRoll(const MadgwickAHRS* self);
float MadgwickAHRS_getPitch(const MadgwickAHRS* self);
float MadgwickAHRS_getYaw(const MadgwickAHRS* self);

#endif
