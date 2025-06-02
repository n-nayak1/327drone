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

void control_loop(float roll, float pitch, float yaw);


#endif
