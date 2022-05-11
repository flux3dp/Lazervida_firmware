#ifndef FLUX_MACHINE_H

#include <stdint.h>
#include <stdbool.h>

volatile uint8_t host_com_port_open;
float reference_tilt;

typedef union {
    uint32_t value;
    struct {
        uint32_t fast_raster_print :1,
                 unassigned  :31;
    };
} cmd_process_locker_t;

typedef union {
    uint32_t value;
    struct {
        uint32_t fast_raster_print :1,
                 unassigned  :31;
    };
} cmd_process_unlocker_t;

typedef enum {
  kOff,
  kOn,
  kFlash,
  kBreath,
  kManual
} LedMode;

extern volatile cmd_process_locker_t cmd_process_locker;
extern volatile cmd_process_unlocker_t cmd_process_unlocker;

extern volatile bool machine_power_on;

void flux_periodic_handling();

void set_stepper_MS1();
void reset_stepper_MS1();
void set_power_24v();
void reset_power_24v();
void set_led_mode(LedMode mode);
void set_led_power(uint32_t val);

void start_firmware_update();

#endif