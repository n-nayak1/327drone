#include <stdio.h>
#include "motor_control.h"
#include "mpu6050.h"
#include "feedback.h"
#include "filter.h"
#include "rc_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CONTROL_LOOP_PERIOD_MS 10
#define KILL_SWITCH_TASK_STACK 2048
#define MAIN_TASK_STACK        4096

// float DesiredAngleRoll = 0;
// float DesiredAnglePitch = 0;
// int InputThrottle = 0;
// float DesiredRateYaw = 0;

float DesiredAngleRoll = 0;
float DesiredAnglePitch = 0;
int InputThrottle = 0;

void kill_switch_task(void* arg) {
    while (1) {
        if (ReceiverValue[4] > 1500) {
            InputThrottle = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(1));  // check as frequently as possible
    }
}

void main_control_task(void* arg) {
    // Wait for controller connection
    while (ReceiverValue[4] <= 900) {}

    while (1) {
        // Parse RC values
        DesiredAngleRoll  = 0.1f * (ReceiverValue[0] - 1500);
        DesiredAnglePitch = 0.1f * (ReceiverValue[1] - 1500);
        InputThrottle     = (ReceiverValue[2] - 1000.f) / 10.f;
        if (InputThrottle > 100) InputThrottle = 100;
        if (InputThrottle < 0)   InputThrottle = 0;

        // Update sensors and filtering
        mpu6050_update();
        const imu_data_t* imu = mpu6050_get_data();
        filter_update(imu);
        euler_angles_t angles = filter_get_euler();

        // Control logic
        if (ReceiverValue[4] < 1500 && ReceiverValue[4] > 500) {
            feedback_set_throttle(InputThrottle);
            control_loop(angles.roll, angles.pitch, DesiredAngleRoll, DesiredAnglePitch);
        } else {
            motor_set_speed(MOTOR1_CHANNEL, 0.f);
            motor_set_speed(MOTOR4_CHANNEL, 0.f);
            motor_set_speed(MOTOR3_CHANNEL, 0.f);
            motor_set_speed(MOTOR2_CHANNEL, 0.f);
        }
        // else {
        //     control_loop(angles.roll, angles.pitch, DesiredAngleRoll, DesiredAnglePitch);
        // }

        printf("[Euler] Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n", angles.roll, angles.pitch, angles.yaw);
        vTaskDelay(pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS));
    }
}


void app_main(void) {
    // Init hardware
    motor_init(MOTOR1_PIN, MOTOR1_CHANNEL);
    motor_init(MOTOR4_PIN, MOTOR2_CHANNEL);
    motor_init(MOTOR3_PIN, MOTOR3_CHANNEL);
    motor_init(MOTOR2_PIN, MOTOR4_CHANNEL);
    mpu6050_init();
    filter_init();
    setup_gpio_interrupts();

    // Create tasks
    xTaskCreatePinnedToCore(kill_switch_task, "KillSwitchTask", KILL_SWITCH_TASK_STACK, NULL, configMAX_PRIORITIES - 1, NULL, 0);
    xTaskCreatePinnedToCore(main_control_task, "MainControlTask", MAIN_TASK_STACK, NULL, configMAX_PRIORITIES - 2, NULL, 1);
}
