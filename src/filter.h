#ifndef FILTER_H
#define FILTER_H

#include "mpu6050.h"
#include "madgwick.h"
#include "esp_timer.h"
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

// Struct to hold filtered angles (in radians)
typedef struct {
    float roll;
    float pitch;
    float yaw;
} euler_angles_t;

/**
 * Initializes the sensor fusion filter.
 * Starts internal timer and clears calibration state.
 */
void filter_init(void);

/**
 * Updates the filter using a new IMU data sample.
 * Call this at a regular interval (e.g., in a loop or ISR).
 *
 * @param imu Pointer to IMU data structure (gyro and accel).
 */
void filter_update(const imu_data_t* imu);

/**
 * Retrieves the current estimated roll, pitch, and yaw angles.
 * Angles are corrected for drift and offsets if calibration is complete.
 *
 * @return Struct of roll, pitch, yaw in radians.
 */
euler_angles_t filter_get_euler(void);

#ifdef __cplusplus
}
#endif

#endif  // FILTER_H