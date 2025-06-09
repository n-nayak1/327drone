#include <stdio.h>
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_attr.h"

#define NUM_CHANNELS 6

// ---------------- GPIO pins ----------------
const gpio_num_t channel_pins[NUM_CHANNELS] = {
    GPIO_NUM_12, GPIO_NUM_14, GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_32, GPIO_NUM_33
};

// ---------------- Globals ----------------
volatile int64_t timer[NUM_CHANNELS] = {0};
volatile int last_channel[NUM_CHANNELS] = {0};
volatile int ReceiverValue[NUM_CHANNELS] = {0};

// ---------------- Logging tags ----------------
static const char* TAG_GPIO = "GPIO_SETUP";

// ---------------- ISR declaration ----------------
static void channelInterruptHandler(void* arg);

// ---------------- ISR definition ----------------
static void IRAM_ATTR channelInterruptHandler(void* arg)
{
    int index = (int)(uintptr_t)arg;
    int pin = channel_pins[index];
    int level = gpio_get_level(pin);
    int64_t current_time = esp_timer_get_time();

    if (level) {
        if (!last_channel[index]) {
            last_channel[index] = 1;
            timer[index] = current_time;
        }
    } else if (last_channel[index]) {
        last_channel[index] = 0;
        ReceiverValue[index] = current_time - timer[index];
    }
}

// ---------------- GPIO setup ----------------
void setup_gpio_interrupts()
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pin_bit_mask = 0
    };

    for (int i = 0; i < NUM_CHANNELS; i++) {
        io_conf.pin_bit_mask |= (1ULL << channel_pins[i]);
    }

    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));

    for (int i = 0; i < NUM_CHANNELS; i++) {
        ESP_ERROR_CHECK(gpio_isr_handler_add(channel_pins[i], channelInterruptHandler, (void*)(uintptr_t)i));
    }

    ESP_LOGI(TAG_GPIO, "Configured %d GPIOs with interrupts.", NUM_CHANNELS);
}
