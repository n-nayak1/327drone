#include <stdio.h>
#include "motor_control.h"
#include "mpu6050.h"
#include "feedback.h"
#include "filter.h"
#include "rc_control.h"

float DesiredAngleRoll = 0;
float DesiredAnglePitch = 0;
int InputThrottle = 0;
float DesiredRateYaw = 0;

void app_main(void) {
    motor_init(MOTOR1_PIN, MOTOR1_CHANNEL);
    motor_init(MOTOR2_PIN, MOTOR2_CHANNEL);
    motor_init(MOTOR3_PIN, MOTOR3_CHANNEL);
    motor_init(MOTOR4_PIN, MOTOR4_CHANNEL);
    mpu6050_init();
    filter_init();  // or whatever loop rate in Hz
    setup_gpio_interrupts();

    float DesiredAngleRoll = 0;
    float DesiredAnglePitch = 0;
    int InputThrottle = 0;
    float DesiredRateYaw = 0;

    while (1) {
        DesiredAngleRoll  = 0.1f * (ReceiverValue[0] - 1500);
        DesiredAnglePitch = 0.1f * (ReceiverValue[1] - 1500);
        InputThrottle     = ReceiverValue[2];
        DesiredRateYaw    = 0.15f * (ReceiverValue[3] - 1500);

        printf("Roll: %.2f  Pitch: %.2f  Throttle: %d  YawRate: %.2f\n",
               DesiredAngleRoll, DesiredAnglePitch, InputThrottle, DesiredRateYaw);

        for (int i = 0; i < NUM_CHANNELS; i++) {
            if (isr_fired[i]) {
                printf("Channel %d ISR fired\n", i);
                isr_fired[i] = false;
            }
        }

        for (int i = 0; i < NUM_CHANNELS; i++) {
            printf("CH[%d] = %d µs\n", i, ReceiverValue[i]);
        }
        

        mpu6050_update();
        const imu_data_t* imu = mpu6050_get_data();

        filter_update(imu);  // Pass raw sensor data
        euler_angles_t angles = filter_get_euler();  // Get roll, pitch, yaw

        // control_loop(angles.roll, angles.pitch, angles.yaw);  // Pass into PID

        printf("[Euler] Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n", angles.roll, angles.pitch, angles.yaw);
        motor_set_speed(MOTOR1_CHANNEL, 50);
        motor_set_speed(MOTOR2_CHANNEL, 50);
        motor_set_speed(MOTOR3_CHANNEL, 50);
        motor_set_speed(MOTOR4_CHANNEL, 50);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

}
