#include "feedback.h"
#include "motor_control.h"
#include "mpu6050.h"
#include <stdio.h>
#include <math.h>

// PID constants (P-only for now)
#define Kp 50.0f
#define BASE_THROTTLE 25.0f
#define MAX_THROTTLE 100.0f
#define MIN_THROTTLE 0.0f

void basic_feedback(ledc_channel_t channel) {
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

void all_motor_feedback() {
    const imu_data_t* imu = mpu6050_get_data();

    float pitch_error = 0.0f - imu->ax;  // Tilt forward/back
    float roll_error  = 0.0f - imu->ay;  // Tilt left/right

    float pitch_corr = Kp * pitch_error;
    float roll_corr  = Kp * roll_error;

    // Each motor's correction = weighted sum of pitch + roll
    float m1_throttle = BASE_THROTTLE + pitch_corr + roll_corr; // Front Left
    float m2_throttle = BASE_THROTTLE - pitch_corr + roll_corr; // Front Right
    float m3_throttle = BASE_THROTTLE - pitch_corr - roll_corr; // Back Left
    float m4_throttle = BASE_THROTTLE + pitch_corr - roll_corr; // Back Right

    // Clamp throttle to safe range
    m1_throttle = fminf(fmaxf(m1_throttle, MIN_THROTTLE), MAX_THROTTLE);
    m2_throttle = fminf(fmaxf(m2_throttle, MIN_THROTTLE), MAX_THROTTLE);
    m3_throttle = fminf(fmaxf(m3_throttle, MIN_THROTTLE), MAX_THROTTLE);
    m4_throttle = fminf(fmaxf(m4_throttle, MIN_THROTTLE), MAX_THROTTLE);

    // Set PWM output
    motor_set_speed(MOTOR1_CHANNEL, m1_throttle);
    motor_set_speed(MOTOR2_CHANNEL, m2_throttle);
    motor_set_speed(MOTOR3_CHANNEL, m3_throttle);
    motor_set_speed(MOTOR4_CHANNEL, m4_throttle);

    // Debug print
    printf("[Feedback] ax: %.2f | ay: %.2f | az: %.2f\n", imu->ax, imu->ay, imu->az);
    printf("M1: %.1f | M2: %.1f | M3: %.1f | M4: %.1f\n",
           m1_throttle, m2_throttle, m3_throttle, m4_throttle);
}
