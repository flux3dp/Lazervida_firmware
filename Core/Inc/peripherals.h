#ifndef PERIPHERALS_H

#include <stdbool.h>
#include <stdint.h>

#include "Adafruit_MSA311.h"

void peripherals_init();

void I2C_scanning();

// Connect to MSA311 interrupt signal pin
void enable_interrupt_for_collision();
void disable_interrupt_for_collision();

#endif