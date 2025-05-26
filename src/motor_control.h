#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MOTOR1_PIN GPIO_NUM_18
#define MOTOR1_CHANNEL LEDC_CHANNEL_0

void motor_init(gpio_num_t pwm_pin, ledc_channel_t channel);
void motor_set_speed(ledc_channel_t channel, float percent);

#endif
