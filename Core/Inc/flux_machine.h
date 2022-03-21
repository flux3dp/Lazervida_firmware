#ifndef FLUX_MACHINE_H

#include <stdint.h>

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

extern volatile cmd_process_locker_t cmd_process_locker;
extern volatile cmd_process_unlocker_t cmd_process_unlocker;

void flux_periodic_handling();

void set_stepper_MS1();
void reset_stepper_MS1();
void set_stepper_power();
void reset_stepper_power();

void start_firmware_update();

#endif