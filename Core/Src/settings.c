/*
  settings.c - eeprom configuration handling
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

#include "grbl.h"

settings_t settings;

const settings_t defaults = {\
    .pulse_microseconds = DEFAULT_STEP_PULSE_MICROSECONDS,
    .stepper_idle_lock_time = DEFAULT_STEPPER_IDLE_LOCK_TIME,
    .status_report_mask = DEFAULT_STATUS_REPORT_MASK,
    .junction_deviation = DEFAULT_JUNCTION_DEVIATION,
    .arc_tolerance = DEFAULT_ARC_TOLERANCE,
    .flags = (DEFAULT_REPORT_INCHES << BIT_REPORT_INCHES) | \
             (DEFAULT_LASER_MODE << BIT_LASER_MODE) | \
             (DEFAULT_INVERT_ST_ENABLE << BIT_INVERT_ST_ENABLE) | \
             (DEFAULT_HARD_LIMIT_ENABLE << BIT_HARD_LIMIT_ENABLE) | \
             (DEFAULT_HOMING_ENABLE << BIT_HOMING_ENABLE) | \
             (DEFAULT_SOFT_LIMIT_ENABLE << BIT_SOFT_LIMIT_ENABLE) | \
             (DEFAULT_INVERT_LIMIT_PINS << BIT_INVERT_LIMIT_PINS) | \
             (DEFAULT_INVERT_PROBE_PIN << BIT_INVERT_PROBE_PIN),
    // ======= FLUX's dedicated =======
};


// Method to store startup lines into EEPROM
void settings_store_startup_line(uint8_t n, char *line)
{
  #ifdef FORCE_BUFFER_SYNC_DURING_EEPROM_WRITE
    protocol_buffer_synchronize(); // A startup line may contain a motion and be executing. 
  #endif
  uint32_t addr = n*(LINE_BUFFER_SIZE+1)+EEPROM_ADDR_STARTUP_BLOCK;
  memcpy_to_eeprom_with_checksum(addr,(char*)line, LINE_BUFFER_SIZE);
}


// Method to store build info into EEPROM
// NOTE: This function can only be called in IDLE state.
void settings_store_build_info(char *line)
{
  // Build info can only be stored when state is IDLE.
  memcpy_to_eeprom_with_checksum(EEPROM_ADDR_BUILD_INFO,(char*)line, LINE_BUFFER_SIZE);
}


// Method to store coord data parameters into EEPROM
void settings_write_coord_data(uint8_t coord_select, float *coord_data)
{
  return;
}


// Method to store Grbl global settings struct and version number into EEPROM
// NOTE: This function can only be called in IDLE state.
void write_global_settings()
{
  eeprom_put_char(0, SETTINGS_VERSION_LATEST);
  memcpy_to_eeprom_with_checksum(EEPROM_ADDR_GLOBAL_GRBL_SETTINGS, (char*)&settings, sizeof(settings_t));
}


// Method to restore EEPROM-saved Grbl global settings back to defaults.
void settings_restore(uint8_t restore_flag) {
  bool eeprom_is_changed = false;
  if (restore_flag & SETTINGS_RESTORE_DEFAULTS) {    
    settings = defaults;
    write_global_settings();
  }

  if (restore_flag & SETTINGS_RESTORE_STARTUP_LINES) {
    #if N_STARTUP_LINE > 0
      eeprom_put_char_n_bytes(
        EEPROM_ADDR_STARTUP_BLOCK + 0*(LINE_BUFFER_SIZE+1), 
        0, 
        LINE_BUFFER_SIZE+1); // (Line buf) + (1-byte Checksum)
    #endif
    #if N_STARTUP_LINE > 1
      eeprom_put_char_n_bytes(
        EEPROM_ADDR_STARTUP_BLOCK + 1*(LINE_BUFFER_SIZE+1), 
        0, 
        LINE_BUFFER_SIZE+1); // (Line buf) + (1-byte Checksum)
    #endif
    eeprom_is_changed = true;
  }

  if (restore_flag & SETTINGS_RESTORE_BUILD_INFO) {
    eeprom_put_char_n_bytes(
      EEPROM_ADDR_BUILD_INFO, 
      0, 
      LINE_BUFFER_SIZE+1); // (Line buf) + (1-byte Checksum)
    eeprom_is_changed = true;
  }

  if (eeprom_is_changed) {
    write_to_flash();
  }
}


// Reads startup line from EEPROM. Updated pointed line string data.
uint8_t settings_read_startup_line(uint8_t n, char *line)
{
  // NOTE: n should < N_STARTUP_LINE
  uint32_t addr = n*(LINE_BUFFER_SIZE+1) + EEPROM_ADDR_STARTUP_BLOCK;
  if (!(memcpy_from_eeprom_with_checksum((char*)line, addr, LINE_BUFFER_SIZE))) {
    // Reset line with default value
    memset(line, 0, LINE_BUFFER_SIZE); // Empty line
    settings_store_startup_line(n, line);
    return(false);
  }
  return(true);
}


// Reads startup line from EEPROM. Updated pointed line string data.
uint8_t settings_read_build_info(char *line)
{
  if (!(memcpy_from_eeprom_with_checksum((char*)line, EEPROM_ADDR_BUILD_INFO, LINE_BUFFER_SIZE))) {
    // Reset line with default value
    memset(line, 0, LINE_BUFFER_SIZE); // Empty line
    settings_store_build_info(line);
    return(false);
  }
  return(true);
}


// Read selected coordinate data from EEPROM. Updates pointed coord_data value.
uint8_t settings_read_coord_data(uint8_t coord_select, float *coord_data)
{
  return(true);
}


// Reads Grbl global settings struct from EEPROM.
uint8_t read_global_settings() {
  // Check version-byte of eeprom
  uint8_t version = eeprom_get_char(0);
  if (version == SETTINGS_VERSION_LATEST) {
    // Read settings-record and check checksum
    if (!(memcpy_from_eeprom_with_checksum((char*)&settings, EEPROM_ADDR_GLOBAL_GRBL_SETTINGS, sizeof(settings_t)))) {
      return(false);
    }
  } else {
    return(false);
  }
  return(true);
}


// A helper method to set settings from command line
uint8_t settings_store_global_setting(uint16_t parameter, float value) {
  if (value < 0.0) { return(STATUS_NEGATIVE_VALUE); }
  
  if (parameter >= 256) { // $256 ~ ???
    // Store non-axis Grbl settings
    //uint8_t int_value = trunc(value);
    switch(parameter) {
      case 259:
        // settings.disable_tilt_detect = value ? true : false;
        break;
      case 260:
        // valid value range: 0 ~ 3
        // settings.tilt_detect_threshold = value > 3 ? 3 : (value < 0 ? 0 : value);
        break;
      default:
        break;
    }
  } else { // $0 ~ $99
    // Store non-axis Grbl settings
    uint8_t int_value = trunc(value);
    switch(parameter) {
      case 0:
        if (int_value < 3) { return(STATUS_SETTING_STEP_PULSE_MIN); }
        settings.pulse_microseconds = int_value; break;
      case 1: settings.stepper_idle_lock_time = int_value; break;
      case 13:
        if (int_value) { settings.flags |= BITFLAG_REPORT_INCHES; }
        else { settings.flags &= ~BITFLAG_REPORT_INCHES; }
        system_flag_wco_change(); // Make sure WCO is immediately updated.
        break;
      default:
        return(STATUS_INVALID_STATEMENT);
    }
  }
  write_global_settings();
  return(STATUS_OK);
}


// Initialize the config subsystem
void settings_init() {
  if(!read_global_settings()) {
    report_status_message(STATUS_SETTING_READ_FAIL);
    settings_restore(SETTINGS_RESTORE_ALL); // Force restore all EEPROM data.
    report_grbl_settings();
  }
}