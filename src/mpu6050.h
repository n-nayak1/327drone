#ifndef MPU6050_H
#define MPU6050_H

#include "driver/i2c.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void mpu6050_init();
void mpu6050_read_raw(int16_t *ax, int16_t *ay, int16_t *az,
                      int16_t *gx, int16_t *gy, int16_t *gz);

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
} imu_data_t;

void mpu6050_update();                  // Reads and converts new values
const imu_data_t* mpu6050_get_data();   // Returns latest converted values

void imu_log_task(void *arg);


#endif
