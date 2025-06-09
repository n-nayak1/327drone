#pragma once

#include <stdint.h>

#define NUM_CHANNELS 6

extern volatile int ReceiverValue[NUM_CHANNELS];

void setup_gpio_interrupts(void);
