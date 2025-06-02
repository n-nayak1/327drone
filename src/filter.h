#ifndef FILTER_H
#define FILTER_H

#include "mpu6050.h"

// Struct to hold filtered angles
typedef struct {
    float roll;
    float pitch;
    float yaw;
} euler_angles_t;

/**
 * Initialize the sensor fusion filter.
 * 
 * @param sample_rate_hz Expected update rate (used for initial setup, not dt).
 */
void filter_init();

/**
 * Update the filter using new IMU data.
 * 
 * @param imu Pointer to IMU data struct (accel + gyro).
 */
void filter_update(const imu_data_t* imu);

/**
 * Get the current roll, pitch, and yaw angles (in degrees).
 * These are automatically zeroed after calibration period.
 */
euler_angles_t filter_get_euler(void);

#endif  // FILTER_H
