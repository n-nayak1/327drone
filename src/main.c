#include <stdio.h>
#include "motor_control.h"
#include "mpu6050.h"
#include "feedback.h"
#include "filter.h"
#include "rc_control.h"

// float DesiredAngleRoll = 0;
// float DesiredAnglePitch = 0;
// int InputThrottle = 0;
// float DesiredRateYaw = 0;

float DesiredAngleRoll = 0;
float DesiredAnglePitch = 0;
int InputThrottle = 0;

void app_main(void) {
    motor_init(MOTOR1_PIN, MOTOR1_CHANNEL);
    motor_init(MOTOR4_PIN, MOTOR2_CHANNEL);
    motor_init(MOTOR3_PIN, MOTOR3_CHANNEL);
    motor_init(MOTOR2_PIN, MOTOR4_CHANNEL);
    mpu6050_init();
    filter_init();  // or whatever loop rate in Hz
    setup_gpio_interrupts();

    // make sure the controller is connected to the receiver before looping
    while (ReceiverValue[4] <= 900) {}

    while (1) {
        // DesiredAngleRoll  = 0.1f * (ReceiverValue[0] - 1500);
        // DesiredAnglePitch = 0.1f * (ReceiverValue[1] - 1500);
        // InputThrottle     = (ReceiverValue[2] - 1000.f)/10.f;
        // DesiredRateYaw    = 0.15f * (ReceiverValue[3] - 1500);

        InputThrottle     = (ReceiverValue[2] - 1000.f)/10.f;  // [0, 100] range
        if (InputThrottle > 100) {
            InputThrottle = 100;
        }
        if (InputThrottle < 0) {
            InputThrottle = 0;
        }

        // if kill switch on turn the throttle off
        if (ReceiverValue[4] > 1500) {
            InputThrottle = 0;
        }

        printf("Input Throttle: %.2d\n", InputThrottle);
        // feedback_set_throttle(InputThrottle);

        // printf("Roll: %.2f  Pitch: %.2f  Throttle: %f  YawRate: %.2f\n",
        //        DesiredAngleRoll, DesiredAnglePitch, InputThrottle, DesiredRateYaw);

        // for (int i = 0; i < NUM_CHANNELS; i++) {
        //     if (isr_fired[i]) {
        //         printf("Channel %d ISR fired\n", i);
        //         isr_fired[i] = false;
        //     }
        // }

        // for (int i = 0; i < NUM_CHANNELS; i++) {
        //     printf("CH[%d] = %d µs\n", i, ReceiverValue[i]);
        // }
        

        mpu6050_update();
        const imu_data_t* imu = mpu6050_get_data();
        
        filter_update(imu);
        euler_angles_t angles = filter_get_euler();  // Get roll, pitch, yaw

        if (ReceiverValue[4] < 1500 && ReceiverValue[4] > 500){
            if (InputThrottle <= 100) {
                motor_set_speed(MOTOR1_CHANNEL, InputThrottle);
                motor_set_speed(MOTOR4_CHANNEL, InputThrottle);
                motor_set_speed(MOTOR3_CHANNEL, InputThrottle);
                motor_set_speed(MOTOR2_CHANNEL, InputThrottle);
                printf("direct control");
            }
            // else {
            //     control_loop(angles.roll, angles.pitch, DesiredAngleRoll, DesiredAnglePitch);
            //     printf("control loop");
            // }
        }
        else {
            motor_set_speed(MOTOR1_CHANNEL, 0.f);
            motor_set_speed(MOTOR4_CHANNEL, 0.f);
            motor_set_speed(MOTOR3_CHANNEL, 0.f);
            motor_set_speed(MOTOR2_CHANNEL, 0.f);
        }
        

        printf("[Euler] Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n", angles.roll, angles.pitch, angles.yaw);
        // motor_set_speed(MOTOR1_CHANNEL, InputThrottle);
        // motor_set_speed(MOTOR2_CHANNEL, InputThrottle);
        // motor_set_speed(MOTOR3_CHANNEL, InputThrottle);
        // motor_set_speed(MOTOR4_CHANNEL, InputThrottle);
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }

}
