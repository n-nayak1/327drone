/**
 * Modified this file to work with my madgwick filter while preserving structure - Milan
 */

#include "filter.h"
#define CALIBRATION_SAMPLES 3000

// Static filter instance
static MadgwickAHRS filter;

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
    // This is what you want to tune
    // First value is beta, or filter gain
    // Second value is the sampling rate
    // Third value is the shittyIMU factor, which adjusts the beta depending on how tilted the IMU is. Use it to work against sensor drift
    MadgwickAHRS_init(&filter, 0.05f, 10.0f, 1.0f);

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

    // Run Madgwick filter update
    MadgwickAHRS_update(&filter, imu->gx, imu->gy, imu->gz, imu->ax, imu->ay, imu->az);

    float roll  = MadgwickAHRS_getRoll(&filter);
    float pitch = MadgwickAHRS_getPitch(&filter);
    float yaw   = MadgwickAHRS_getYaw(&filter);

    if (!calibration_done) {
        pitch_accum += pitch;
        roll_accum  += roll;

        if (calib_samples == 0) {
            yaw_start = yaw;
        }

        if (calib_samples == CALIBRATION_SAMPLES - 1) {
            yaw_end = yaw;
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
    float corrected_yaw = MadgwickAHRS_getYaw(&filter) - yaw_drift_rate * elapsed_time;

    return (euler_angles_t) {
        .roll = MadgwickAHRS_getRoll(&filter) - roll_offset,
        .pitch = MadgwickAHRS_getPitch(&filter) - pitch_offset,
        .yaw = corrected_yaw
    };
}