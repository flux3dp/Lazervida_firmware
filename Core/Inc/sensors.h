#ifndef SENSORS_H

#include <stdbool.h>
#include <stdint.h>

void sensors_init();

extern uint32_t adc_Buffer[1];
#define DETECT_3V3_VALUE (adc_Buffer[0])

#endif