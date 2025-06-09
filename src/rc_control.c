#include "rc_control.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "driver/gpio.h"

#define TAG "RC_CONTROL"
#define CYCLES_PER_US 160  // ← just define this ONCE, no redefinition

// ---------------- GPIO pins ----------------
const gpio_num_t channel_pins[NUM_CHANNELS] = {
    GPIO_NUM_12, GPIO_NUM_14, GPIO_NUM_26, GPIO_NUM_27, GPIO_NUM_32, GPIO_NUM_33
};

// ---------------- Globals ----------------
volatile uint16_t ReceiverValue[NUM_CHANNELS] = {0};
volatile bool isr_fired[NUM_CHANNELS] = {false};
volatile uint32_t start_cycles[NUM_CHANNELS] = {0};

// ---------------- Simple cycle counter ----------------
static inline uint32_t esp_cpu_get_ccount(void)
{
    uint32_t ccount;
    __asm__ __volatile__("rsr.ccount %0" : "=a" (ccount));
    return ccount;
}

// ---------------- ISR ----------------
static void IRAM_ATTR channelInterruptHandler(void* arg)
{
    int index = (int)(uintptr_t)arg;
    int pin = channel_pins[index];
    int level = gpio_get_level(pin);
    uint32_t now = esp_cpu_get_ccount();

    if (level) {
        start_cycles[index] = now;
    } else {
        uint32_t delta = now - start_cycles[index];
        ReceiverValue[index] = delta / CYCLES_PER_US;
        isr_fired[index] = true;
    }
}

// ---------------- GPIO Setup ----------------
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

    ESP_LOGI(TAG, "Configured %d GPIOs for RC PWM input", NUM_CHANNELS);
}
