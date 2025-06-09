#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

#define NUM_CHANNELS 6

extern const gpio_num_t channel_pins[NUM_CHANNELS];
extern volatile uint16_t ReceiverValue[NUM_CHANNELS];  // Pulse widths in µs
extern volatile bool isr_fired[NUM_CHANNELS];          // Optional debug flag

void setup_gpio_interrupts(void);
