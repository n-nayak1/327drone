#include "filter.h"
#include "Fusion.h"
#include "mpu6050.h"
#include "esp_timer.h"
#include <stdio.h>

#define DEG_TO_RAD(x) ((x) * 0.0174533f)
#define CALIBRATION_SAMPLES 3000

static FusionAhrs ahrs;
static FusionOffset offset;
static FusionEuler cached_euler;

static float pitch_offset = 0.0f;
static float roll_offset = 0.0f;
static float yaw_drift_rate = 0.0f;

static float pitch_accum = 0.0f;
static float roll_accum = 0.0f;
static float yaw_start = 0.0f;
static float yaw_end = 0.0f;

static int calib_samples = 0;
static int64_t calib_start_us = 0;
static int64_t last_update_us = 0;
static bool calibration_done = false;

void filter_init() {
    const int assumed_rate = 100;
    FusionOffsetInitialise(&offset, assumed_rate);
    FusionAhrsInitialise(&ahrs);

    FusionAhrsSettings settings = {
        .convention = FusionConventionNwu,
        .gain = 0.5f,
        .gyroscopeRange = 2000.0f,
        .accelerationRejection = 10.0f,
        .magneticRejection = 10.0f,
        .recoveryTriggerPeriod = 5 * assumed_rate,
    };
    FusionAhrsSetSettings(&ahrs, &settings);

    calib_start_us = esp_timer_get_time();
    last_update_us = calib_start_us;
    pitch_accum = roll_accum = 0.0f;
    pitch_offset = roll_offset = yaw_drift_rate = 0.0f;
    calib_samples = 0;
    calibration_done = false;
}

void filter_update(const imu_data_t* imu) {
    int64_t now = esp_timer_get_time();
    float dt = (now - last_update_us) / 1e6f;
    last_update_us = now;

    FusionVector gyro = {
        .axis.x = imu->gx,
        .axis.y = imu->gy,
        .axis.z = imu->gz
    };
    FusionVector accel = {
        .axis.x = imu->ax,
        .axis.y = imu->ay,
        .axis.z = imu->az
    };

    gyro = FusionOffsetUpdate(&offset, gyro);
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyro, accel, dt);

    cached_euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

    if (!calibration_done) {
        pitch_accum += cached_euler.angle.pitch;
        roll_accum  += cached_euler.angle.roll;

        if (calib_samples == 0) {
            yaw_start = cached_euler.angle.yaw;
        }

        if (calib_samples == CALIBRATION_SAMPLES - 1) {
            yaw_end = cached_euler.angle.yaw;
            float elapsed_time = (now - calib_start_us) / 1e6f;
            yaw_drift_rate = (yaw_end - yaw_start) / elapsed_time;
        }

        calib_samples++;

        if (calib_samples >= CALIBRATION_SAMPLES) {
            pitch_offset = pitch_accum / calib_samples;
            roll_offset  = roll_accum / calib_samples;
            calibration_done = true;

            printf("[Filter] Calibration complete: pitch=%.2f, roll=%.2f, yaw drift=%.4f deg/s\n",
                   pitch_offset, roll_offset, yaw_drift_rate);
        }
    }
}

euler_angles_t filter_get_euler() {
    float elapsed_time = (esp_timer_get_time() - calib_start_us) / 1e6f;
    float corrected_yaw = cached_euler.angle.yaw - yaw_drift_rate * elapsed_time;

    return (euler_angles_t) {
        .roll = cached_euler.angle.roll - roll_offset,
        .pitch = cached_euler.angle.pitch - pitch_offset,
        .yaw = corrected_yaw
    };
}
