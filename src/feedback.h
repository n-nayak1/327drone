#ifndef FEEDBACK_H
#define FEEDBACK_H

#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void basic_feedback(ledc_channel_t channel);
void all_motor_feedback();

#endif
