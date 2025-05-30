#include "motor_control.h"


#define PWM_TIMER          LEDC_TIMER_0
#define PWM_MODE           LEDC_HIGH_SPEED_MODE

// MOTOR CONSTANTS
#define PWM_FREQ_HZ        50
#define PWM_RESOLUTION     LEDC_TIMER_16_BIT
#define DUTY_MIN           3277  // ~1.0 ms
#define DUTY_MAX           (DUTY_MIN + ((6553 - DUTY_MIN) * 1.0f))  // 100% throttle cap (~1.3 ms)

// LED CONSTANTS
// #define PWM_FREQ_HZ        5000
// #define PWM_RESOLUTION     LEDC_TIMER_10_BIT
// #define DUTY_MIN           0
// #define DUTY_MAX           1023



void motor_init(gpio_num_t pwm_pin, ledc_channel_t channel) {
    static bool timer_initialized = false;

    if (!timer_initialized) {
        ledc_timer_config_t timer_config = {
            .speed_mode       = PWM_MODE,
            .timer_num        = PWM_TIMER,
            .duty_resolution  = PWM_RESOLUTION,
            .freq_hz          = PWM_FREQ_HZ,
            .clk_cfg          = LEDC_AUTO_CLK
        };
        ledc_timer_config(&timer_config);
        timer_initialized = true;
    }

    ledc_channel_config_t channel_config = {
        .gpio_num       = pwm_pin,
        .speed_mode     = PWM_MODE,
        .channel        = channel,
        .timer_sel      = PWM_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&channel_config);

    // Arm ESC (0% throttle)
    ledc_set_duty(PWM_MODE, channel, DUTY_MIN);
    ledc_update_duty(PWM_MODE, channel);
    vTaskDelay(pdMS_TO_TICKS(2000));
}

void motor_set_speed(ledc_channel_t channel, float percent) {
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    uint32_t duty = DUTY_MIN + (uint32_t)((DUTY_MAX - DUTY_MIN) * (percent / 100.0f));
    ledc_set_duty(PWM_MODE, channel, duty);
    ledc_update_duty(PWM_MODE, channel);
}

void motor_test() {
    for (int i = 0; i <= 100; i += 10) {
        motor_set_speed(MOTOR4_CHANNEL, i);
        motor_set_speed(MOTOR3_CHANNEL, i);
        motor_set_speed(MOTOR2_CHANNEL, i);
        motor_set_speed(MOTOR1_CHANNEL, i);
        printf("Throttle: %d%%\n", i);
        vTaskDelay(pdMS_TO_TICKS(700));
    }
    for (int i = 100; i >= 0; i -= 10) {
        motor_set_speed(MOTOR4_CHANNEL, i);
        motor_set_speed(MOTOR3_CHANNEL, i);
        motor_set_speed(MOTOR2_CHANNEL, i);
        motor_set_speed(MOTOR1_CHANNEL, i);
        printf("Throttle: %d%%\n", i);
        vTaskDelay(pdMS_TO_TICKS(700));
    }
}

