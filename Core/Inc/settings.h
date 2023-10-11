/*
  settings.h - eeprom configuration handling
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef settings_h
#define settings_h

#include "grbl.h"


/**
 * To add (change) a new settings,
 *    1. Change the current SETTINGS_VERSION_LATEST into SETTINGS_VERSION_XX
 *    2. Add a new macro SETTINGS_VERSION_LATEST with value incremented
 *    3. Change the declaration of current settings_t into settings_vXX_t
 *    4. Create a new settings_t with new setting attributes
 *    5. Could add a default value macro in defaults.h
 *    6. Modify "const settings_t defaults = ...." (add default value to the new attribute)
 *    7. Modify read_global_settings()
 *    8. Modify settings_store_global_setting()
 *    9. Modify report_grbl_settings()
 */


// Version of the EEPROM data. Will be used to migrate existing data from older versions of Grbl
// when firmware is upgraded. Always stored in byte 0 of eeprom
#define SETTINGS_VERSION_10     10  // NOTE: Check settings_reset() when moving to next version.
#define SETTINGS_VERSION_11     11  // NOTE: Check settings_reset() when moving to next version.
#define SETTINGS_VERSION_LATEST 12  // NOTE: Check settings_reset() when moving to next version.

// Define bit flag masks for the boolean settings in settings.flag.
#define BIT_REPORT_INCHES      0
#define BIT_LASER_MODE         1
#define BIT_INVERT_ST_ENABLE   2
#define BIT_HARD_LIMIT_ENABLE  3
#define BIT_HOMING_ENABLE      4
#define BIT_SOFT_LIMIT_ENABLE  5
#define BIT_INVERT_LIMIT_PINS  6
#define BIT_INVERT_PROBE_PIN   7

#define BITFLAG_REPORT_INCHES      bit(BIT_REPORT_INCHES)
#define BITFLAG_LASER_MODE         bit(BIT_LASER_MODE)
#define BITFLAG_INVERT_ST_ENABLE   bit(BIT_INVERT_ST_ENABLE)
#define BITFLAG_HARD_LIMIT_ENABLE  bit(BIT_HARD_LIMIT_ENABLE)
#define BITFLAG_HOMING_ENABLE      bit(BIT_HOMING_ENABLE)
#define BITFLAG_SOFT_LIMIT_ENABLE  bit(BIT_SOFT_LIMIT_ENABLE)
#define BITFLAG_INVERT_LIMIT_PINS  bit(BIT_INVERT_LIMIT_PINS)
#define BITFLAG_INVERT_PROBE_PIN   bit(BIT_INVERT_PROBE_PIN)

// Define status reporting boolean enable bit flags in settings.status_report_mask
#define BITFLAG_RT_STATUS_POSITION_TYPE     bit(0)
#define BITFLAG_RT_STATUS_BUFFER_STATE      bit(1)

// Define settings restore bitflags.
#define SETTINGS_RESTORE_DEFAULTS bit(0)
#define SETTINGS_RESTORE_PARAMETERS bit(1)
#define SETTINGS_RESTORE_STARTUP_LINES bit(2)
#define SETTINGS_RESTORE_BUILD_INFO bit(3)
#ifndef SETTINGS_RESTORE_ALL
  #define SETTINGS_RESTORE_ALL 0xFF // All bitflags
#endif

// Define EEPROM memory address location values for Grbl settings and parameters
// NOTE: The Atmega328p has 1KB EEPROM. The upper half is reserved for parameters and
// the startup script. The lower half contains the global settings and space for future
// developments.
// The followings are the start byte index of each data in Virtual_EEPROM[]
// The byte 0 is for settings VERSION
#define EEPROM_ADDR_GLOBAL_GRBL_SETTINGS  1U
#define EEPROM_ADDR_PARAMETERS            512U
#define EEPROM_ADDR_STARTUP_BLOCK         768U
#define EEPROM_ADDR_BUILD_INFO            942U

// Define EEPROM address indexing for coordinate parameters
#define N_COORDINATE_SYSTEM 6  // Number of supported work coordinate systems (from index 1)
#define SETTING_INDEX_NCOORD N_COORDINATE_SYSTEM+1 // Total number of system stored (from index 0)
// NOTE: Work coordinate indices are (0=G54, 1=G55, ... , 6=G59)
#define SETTING_INDEX_G28    N_COORDINATE_SYSTEM    // Home position 1
#define SETTING_INDEX_G30    N_COORDINATE_SYSTEM+1  // Home position 2
// #define SETTING_INDEX_G92    N_COORDINATE_SYSTEM+2  // Coordinate offset (G92.2,G92.3 not supported)

// Define Grbl axis settings numbering scheme. Starts at START_VAL, every INCREMENT, over N_SETTINGS.
#define AXIS_N_SETTINGS          4
#define AXIS_SETTINGS_START_VAL  100 // NOTE: Reserving settings values >= 100 for axis settings. Up to 255.
#define AXIS_SETTINGS_INCREMENT  10  // Must be greater than the number of axis settings

// Global persistent settings (Stored from byte EEPROM_ADDR_GLOBAL_GRBL_SETTINGS onwards)
typedef struct {
  // Axis settings
  float steps_per_mm[N_AXIS];
  float max_rate[N_AXIS];
  float acceleration[N_AXIS];
  float max_travel[N_AXIS];

  // Remaining Grbl settings
  uint8_t pulse_microseconds;
  uint8_t step_invert_mask;
  uint8_t dir_invert_mask;
  uint8_t stepper_idle_lock_time; // If max value 255, steppers do not disable.
  uint8_t status_report_mask; // Mask to indicate desired report data.
  float junction_deviation;
  float arc_tolerance;

  float rpm_max;
  float rpm_min;

  uint8_t flags;  // Contains default boolean settings

  uint8_t homing_dir_mask;
  float homing_feed_rate;
  float homing_seek_rate;
  uint16_t homing_debounce_delay;
  float homing_pulloff;
} settings_v10_t;
typedef struct {
  // Axis settings
  float steps_per_mm[N_AXIS]; // $100    ~ 100    + N_AXIS
  float max_rate[N_AXIS];     // $100+10 ~ 100+10 + N_AXIS
  float acceleration[N_AXIS]; // $100+20 ~ 100+20 + N_AXIS
  float max_travel[N_AXIS];   // $100+30 ~ 100+30 + N_AXIS

  // Remaining Grbl settings
  uint8_t pulse_microseconds; // $0
  uint8_t step_invert_mask;   // $2
  uint8_t dir_invert_mask;    // $3
  uint8_t stepper_idle_lock_time; // $1: If max value 255, steppers do not disable.
  uint8_t status_report_mask; // $10: Mask to indicate desired report data.
  float junction_deviation;   // $11
  float arc_tolerance;        // $12

  float rpm_max;              // $30
  float rpm_min;              // $31

  // $4, $5, $6, $13, $20, $21, $22, $32
  uint8_t flags;  // Contains default boolean settings

  uint8_t homing_dir_mask;    // $23
  float homing_feed_rate;     // $24
  float homing_seek_rate;     // $25
  uint16_t homing_debounce_delay; // $26
  float homing_pulloff;       // $27

  // ========= FLUX's dedicated =========
  // Added since V11
  uint8_t disable_tilt_detect; // $33

} settings_v11_t;
typedef struct {
  // Axis settings
  float steps_per_mm[N_AXIS]; // $100    ~ 100    + N_AXIS
  float max_rate[N_AXIS];     // $100+10 ~ 100+10 + N_AXIS
  float acceleration[N_AXIS]; // $100+20 ~ 100+20 + N_AXIS
  float max_travel[N_AXIS];   // $100+30 ~ 100+30 + N_AXIS

  // Remaining Grbl settings
  uint8_t pulse_microseconds; // $0
  uint8_t step_invert_mask;   // $2
  uint8_t dir_invert_mask;    // $3
  uint8_t stepper_idle_lock_time; // $1: If max value 255, steppers do not disable.
  uint8_t status_report_mask; // $10: Mask to indicate desired report data.
  float junction_deviation;   // $11
  float arc_tolerance;        // $12

  float rpm_max;              // $30
  float rpm_min;              // $31

  // $4, $5, $6, $13, $20, $21, $22, $32
  uint8_t flags;  // Contains default boolean settings

  uint8_t homing_dir_mask;    // $23
  float homing_feed_rate;     // $24
  float homing_seek_rate;     // $25
  uint16_t homing_debounce_delay; // $26
  float homing_pulloff;       // $27

  // ========= FLUX's dedicated =========
  // Added since V11
  uint8_t disable_tilt_detect; // $259
  // Added since V12
  float   tilt_detect_threshold; // $260
} settings_t;

extern settings_t settings;

#if N_AXIS <= Z_AXIS 
typedef struct {
  float steps_per_mm; // $102
  float max_rate;     // $112
  float acceleration; // $122
  float max_travel;   // $132
} fake_z_axis_info_t;
extern fake_z_axis_info_t fake_z_axis_info;
#endif

// Initialize the configuration subsystem (load settings from EEPROM)
void settings_init();

// Helper function to clear and restore EEPROM defaults
void settings_restore(uint8_t restore_flag);

// A helper method to set new settings from command line
uint8_t settings_store_global_setting(uint16_t parameter, float value);

// Stores the protocol line variable as a startup line in EEPROM
void settings_store_startup_line(uint8_t n, char *line);

// Reads an EEPROM startup line to the protocol line variable
uint8_t settings_read_startup_line(uint8_t n, char *line);

// Stores build info user-defined string
void settings_store_build_info(char *line);

// Reads build info user-defined string
uint8_t settings_read_build_info(char *line);

// Writes selected coordinate data to EEPROM
void settings_write_coord_data(uint8_t coord_select, float *coord_data);

// Reads selected coordinate data from EEPROM
uint8_t settings_read_coord_data(uint8_t coord_select, float *coord_data);

// Returns the step pin mask according to Grbl's internal axis numbering
uint16_t get_step_pin_mask(uint8_t i);

// Returns the direction pin mask according to Grbl's internal axis numbering
uint16_t get_direction_pin_mask(uint8_t i);

// Returns the limit pin mask according to Grbl's internal axis numbering
uint16_t get_limit_pin_mask(uint8_t i);


#endif
