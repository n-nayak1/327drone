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
    setup_gpio_interrupts();

    while (1) {
        DesiredAngleRoll  = 0.1f * (ReceiverValue[0] - 1500);
        DesiredAnglePitch = 0.1f * (ReceiverValue[1] - 1500);
        InputThrottle     = ReceiverValue[2];
        DesiredRateYaw    = 0.15f * (ReceiverValue[3] - 1500);

        printf("Roll: %.2f  Pitch: %.2f  Throttle: %d  YawRate: %.2f\n",
               DesiredAngleRoll, DesiredAnglePitch, InputThrottle, DesiredRateYaw);

        mpu6050_update();
        const imu_data_t* imu = mpu6050_get_data();

        filter_update(imu);  // Pass raw sensor data
        euler_angles_t angles = filter_get_euler();  // Get roll, pitch, yaw

        // control_loop(angles.roll, angles.pitch, angles.yaw);  // Pass into PID

        printf("[Euler] Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n", angles.roll, angles.pitch, angles.yaw);

        vTaskDelay(pdMS_TO_TICKS(100));
    }

}
