#include "feedback.h"
#include "motor_control.h"
#include "mpu6050.h"
#include <stdio.h>
#include <math.h>

// Limits
#define MAX_THROTTLE 100.0f
#define MIN_THROTTLE 0.0f

// PID constants
static const float Kp_pitch = 2.0f, Ki_pitch = 0.5f, Kd_pitch = 0.1f;
static const float Kp_roll  = 2.0f, Ki_roll  = 0.5f, Kd_roll  = 0.1f;

// State tracking
static float pitch_integral = 0, pitch_last = 0;
static float roll_integral  = 0, roll_last  = 0;

// External throttle input from RC
static float base_throttle = 0;

void feedback_set_throttle(float t) {
    base_throttle = fminf(fmaxf(t, MIN_THROTTLE), MAX_THROTTLE);
}

void control_loop(float measured_roll, float measured_pitch,
                  float desired_roll, float desired_pitch) {

    float error_pitch = desired_pitch - measured_pitch;
    float error_roll  = desired_roll - measured_roll;

    // --- Anti-windup clamp range ---
    const float I_CLAMP_MIN = -200.0f;
    const float I_CLAMP_MAX =  200.0f;

    // --- Pitch PID ---
    pitch_integral += error_pitch;
    if (pitch_integral > I_CLAMP_MAX) pitch_integral = I_CLAMP_MAX;
    if (pitch_integral < I_CLAMP_MIN) pitch_integral = I_CLAMP_MIN;

    float derivative_pitch = error_pitch - pitch_last;
    pitch_last = error_pitch;

    float pitch_output = Kp_pitch * error_pitch +
                         Ki_pitch * pitch_integral +
                         Kd_pitch * derivative_pitch;

    // --- Roll PID ---
    roll_integral += error_roll;
    if (roll_integral > I_CLAMP_MAX) roll_integral = I_CLAMP_MAX;
    if (roll_integral < I_CLAMP_MIN) roll_integral = I_CLAMP_MIN;

    float derivative_roll = error_roll - roll_last;
    roll_last = error_roll;

    float roll_output = Kp_roll * error_roll +
                        Ki_roll * roll_integral +
                        Kd_roll * derivative_roll;

    // --- Motor Mixing ---
    float m1 = base_throttle + pitch_output + roll_output;  // Front Left
    float m2 = base_throttle + pitch_output - roll_output;  // Front Right
    float m3 = base_throttle - pitch_output + roll_output;  // Back Left
    float m4 = base_throttle - pitch_output - roll_output;  // Back Right

    // --- Clamp final outputs ---
    m1 = fminf(fmaxf(m1, MIN_THROTTLE), MAX_THROTTLE);
    m2 = fminf(fmaxf(m2, MIN_THROTTLE), MAX_THROTTLE);
    m3 = fminf(fmaxf(m3, MIN_THROTTLE), MAX_THROTTLE);
    m4 = fminf(fmaxf(m4, MIN_THROTTLE), MAX_THROTTLE);

    // --- Output to motors ---
    motor_set_speed(MOTOR1_CHANNEL, m1);
    motor_set_speed(MOTOR2_CHANNEL, m2);
    motor_set_speed(MOTOR3_CHANNEL, m3);
    motor_set_speed(MOTOR4_CHANNEL, m4);
}
