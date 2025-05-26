#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MOTOR1_PIN GPIO_NUM_18
#define MOTOR2_PIN GPIO_NUM_16
#define MOTOR3_PIN GPIO_NUM_17
#define MOTOR4_PIN GPIO_NUM_19

#define MOTOR1_CHANNEL LEDC_CHANNEL_0
#define MOTOR2_CHANNEL LEDC_CHANNEL_1
#define MOTOR3_CHANNEL LEDC_CHANNEL_2
#define MOTOR4_CHANNEL LEDC_CHANNEL_3

void motor_init(gpio_num_t pwm_pin, ledc_channel_t channel);
void motor_set_speed(ledc_channel_t channel, float percent);
void motor_test(ledc_channel_t channel);

#endif
