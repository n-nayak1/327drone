#include <stdio.h>
#include "motor_control.h"
#include "mpu6050.h"
#include "feedback.h"
#include "filter.h"


void app_main(void) {
    motor_init(MOTOR1_PIN, MOTOR1_CHANNEL);
    motor_init(MOTOR2_PIN, MOTOR2_CHANNEL);
    motor_init(MOTOR3_PIN, MOTOR3_CHANNEL);
    motor_init(MOTOR4_PIN, MOTOR4_CHANNEL);
    mpu6050_init();
    filter_init();  // or whatever loop rate in Hz


    while (1) {
        mpu6050_update();
        const imu_data_t* imu = mpu6050_get_data();

        filter_update(imu);  // Pass raw sensor data
        euler_angles_t angles = filter_get_euler();  // Get roll, pitch, yaw

        // control_loop(angles.roll, angles.pitch, angles.yaw);  // Pass into PID

        printf("[Euler] Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n", angles.roll, angles.pitch, angles.yaw);

        vTaskDelay(pdMS_TO_TICKS(100));
    }

}
