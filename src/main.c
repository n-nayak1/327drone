#include <stdio.h>
#include "motor_control.h"
#include "mpu6050.h"
#include "feedback.h"
#include "filter.h"
#include "rc_control.h"


void app_main(void) {
    setup_gpio_interrupts();
    
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
