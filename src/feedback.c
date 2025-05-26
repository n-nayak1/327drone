#include "feedback.h"
#include "motor_control.h"
#include "mpu6050.h"
#include <stdio.h>

// PID constants (P-only for now)
#define Kp 20.0f
#define BASE_THROTTLE 50.0f
#define MAX_THROTTLE 80.0f
#define MIN_THROTTLE 20.0f

void basic_feedback() {
    const imu_data_t* imu = mpu6050_get_data();

    float error = 0.0f - imu->ax;           // Desired is 0 (level)
    float correction = Kp * error;
    float throttle = BASE_THROTTLE + correction;

    if (throttle > MAX_THROTTLE) throttle = MAX_THROTTLE;
    if (throttle < MIN_THROTTLE) throttle = MIN_THROTTLE;

    motor_set_speed(MOTOR1_CHANNEL, throttle);

    printf("[Feedback] ax: %.3f | error: %.3f | throttle: %.2f%%\n",
           imu->ax, error, throttle);
}
