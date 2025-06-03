#include <stdio.h>
#include "motor_control.h"
#include "mpu6050.h"
#include "feedback.h"
#include "filter.h"
#include "circular_buffer.h"


void app_main(void) {
    motor_init(MOTOR1_PIN, MOTOR1_CHANNEL);
    motor_init(MOTOR2_PIN, MOTOR2_CHANNEL);
    motor_init(MOTOR3_PIN, MOTOR3_CHANNEL);
    motor_init(MOTOR4_PIN, MOTOR4_CHANNEL);
    mpu6050_init();
    filter_init();

    CircularBuffer cb;
    cb_init(&cb);

    int count = 0;

    while (1) {
        mpu6050_update();
        const imu_data_t* imu = mpu6050_get_data();
        imu_data_t imu_avg;

        // use averaged IMU data
        // also I didn't implement it, but you want to integrate gyroscope data
        // so imu_avg has averaged imu data and integrated gyroscopic data. That way, we make best use of all our samples
        if (count < BUFFER_SIZE) {
            cb_push(&cb, imu->ax, imu->ay, imu->az);
            count++;
        } else {
            float ax_avg, ay_avg, az_avg;
            cb_average(&cb, &ax_avg, &ay_avg, &az_avg);


            imu_avg.gx = imu->gx;
            imu_avg.gy = imu->gy;
            imu_avg.gz = imu->gz;
            imu_avg.ax = ax_avg;
            imu_avg.ay = ay_avg;
            imu_avg.az = az_avg;

            filter_update(&imu_avg);

        }

        euler_angles_t angles = filter_get_euler();
        // control_loop(angles.roll, angles.pitch, angles.yaw);
        motor_set_speed(MOTOR1_CHANNEL, 100);
        motor_set_speed(MOTOR2_CHANNEL, 100);
        motor_set_speed(MOTOR3_CHANNEL, 100);
        motor_set_speed(MOTOR4_CHANNEL, 100);

        printf("[Euler] Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n", angles.roll, angles.pitch, angles.yaw);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

