#ifndef PERIPHERALS_H

#include <stdbool.h>

void peripherals_init();
void turn_on_fan();
void turn_off_fan();
void toggle_fan();
bool get_fan_state();

#endif