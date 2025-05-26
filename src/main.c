#include <stdio.h>
#include "motor_control.h"
#include "mpu6050.h"
#include "feedback.h"


void app_main(void) {
    motor_init(MOTOR1_PIN, MOTOR1_CHANNEL);
    motor_init(MOTOR2_PIN, MOTOR2_CHANNEL);
    motor_init(MOTOR3_PIN, MOTOR3_CHANNEL);
    motor_init(MOTOR4_PIN, MOTOR4_CHANNEL);
    mpu6050_init();

    while (1) {
        mpu6050_update();      // Read new sensor values
        all_motor_feedback();      // Run feedback control based on ax
        vTaskDelay(pdMS_TO_TICKS(100));  // Loop ~10 Hz
        
        // const imu_data_t* imu = mpu6050_get_data();
        // printf("Accel (g): X=%.2f Y=%.2f Z=%.2f | Gyro (°/s): X=%.2f Y=%.2f Z=%.2f\n",
        // imu->ax, imu->ay, imu->az,
        // imu->gx, imu->gy, imu->gz);

        // motor_test(MOTOR1_CHANNEL);
    }
}
