#include "feedback.h"
#include "motor_control.h"
#include "mpu6050.h"
#include <stdio.h>
#include <math.h>

// PID constants (P-only for now)
#define BASE_THROTTLE 50.0f
#define MAX_THROTTLE 100.0f
#define MIN_THROTTLE 0.0f

// PID constants for each axis
static const float Kp_pitch = 1.0f, Ki_pitch = 0.0f, Kd_pitch = 0.1f;
static const float Kp_roll  = 1.0f, Ki_roll  = 0.0f, Kd_roll  = 0.1f;
static const float Kp_yaw   = 1.0f, Ki_yaw   = 0.0f, Kd_yaw   = 0.1f;

// Error tracking variables
static float pitch_integral = 0, pitch_last = 0;
static float roll_integral  = 0, roll_last  = 0;
static float yaw_integral   = 0, yaw_last   = 0;

void apply_pid_feedback(control_axis_t axis, float measured, float setpoint,
                        float Kp, float Ki, float Kd,
                        float* integral_error, float* last_error) {
    float error = setpoint - measured;
    *integral_error += error;
    float derivative = error - *last_error;
    *last_error = error;

    float correction = Kp * error + Ki * (*integral_error) + Kd * derivative;

    // Start with base throttle
    float m1 = BASE_THROTTLE;
    float m2 = BASE_THROTTLE;
    float m3 = BASE_THROTTLE;
    float m4 = BASE_THROTTLE;

    // Apply correction to motors depending on the axis
    switch (axis) {
        case AXIS_PITCH:
            // Forward/backward tilt
            m1 += correction;  // Front Left
            m2 += correction;  // Front Right
            m3 -= correction;  // Back Left
            m4 -= correction;  // Back Right
            break;
        case AXIS_ROLL:
            // Left/right tilt
            m1 += correction;  // Front Left
            m2 -= correction;  // Front Right
            m3 += correction;  // Back Left
            m4 -= correction;  // Back Right
            break;
        case AXIS_YAW:
            // Opposing motor pairs spin faster/slower
            m1 += correction;
            m2 -= correction;
            m3 += correction;
            m4 -= correction;
            break;
    }

    // Clamp each motor
    m1 = fminf(fmaxf(m1, MIN_THROTTLE), MAX_THROTTLE);
    m2 = fminf(fmaxf(m2, MIN_THROTTLE), MAX_THROTTLE);
    m3 = fminf(fmaxf(m3, MIN_THROTTLE), MAX_THROTTLE);
    m4 = fminf(fmaxf(m4, MIN_THROTTLE), MAX_THROTTLE);

    // Set motor speeds
    motor_set_speed(MOTOR1_CHANNEL, m1);
    motor_set_speed(MOTOR2_CHANNEL, m2);
    motor_set_speed(MOTOR3_CHANNEL, m3);
    motor_set_speed(MOTOR4_CHANNEL, m4);

    // Optional debug
    printf("[PID] axis: %d | error: %.2f | correction: %.2f\n", axis, error, correction);
}


void control_loop(float roll, float pitch, float yaw) {
    apply_pid_feedback(AXIS_PITCH, pitch, 0.0f,
                       Kp_pitch, Ki_pitch, Kd_pitch,
                       &pitch_integral, &pitch_last);
    apply_pid_feedback(AXIS_ROLL, roll, 0.0f,
                       Kp_roll, Ki_roll, Kd_roll,
                       &roll_integral, &roll_last);
    apply_pid_feedback(AXIS_YAW, yaw, 0.0f,
                       Kp_yaw, Ki_yaw, Kd_yaw,
                       &yaw_integral, &yaw_last);
}
