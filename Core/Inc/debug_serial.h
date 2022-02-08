#ifndef DEBUG_SERIAL_H

#include <stddef.h>
#include <stdint.h>

void debug_serial_init();

uint8_t debug_serial_read();
void debug_serial_write_data(uint8_t data);
void debug_serial_rx_handler();

#endif