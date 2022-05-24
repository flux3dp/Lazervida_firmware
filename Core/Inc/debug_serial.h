#ifndef DEBUG_SERIAL_H

#include <stddef.h>
#include <stdint.h>

#define DEBUG_SERIAL_ON 0

#if DEBUG_SERIAL_ON
void debug_serial_init();

uint8_t debug_serial_read();
void debug_serial_write_data(uint8_t data);
void debug_serial_rx_handler();
#endif

#endif