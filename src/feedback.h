#ifndef FEEDBACK_H
#define FEEDBACK_H

#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef enum {
    AXIS_PITCH,
    AXIS_ROLL,
    AXIS_YAW
} control_axis_t;

/**
 * @brief Sets the base throttle for all motors [0–100%].
 */
void feedback_set_throttle(float base_throttle);

/**
 * @brief Computes and applies PID corrections to roll and pitch.
 * 
 * @param measured_roll   Current roll angle (°)
 * @param measured_pitch  Current pitch angle (°)
 * @param desired_roll    Target roll angle from RC (°)
 * @param desired_pitch   Target pitch angle from RC (°)
 */
void control_loop(float measured_roll, float measured_pitch,
                  float desired_roll, float desired_pitch);

#endif // FEEDBACK_H
