/*
  protocol.c - controls Grbl execution protocol and procedures
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

#include <stdint.h>

#include "grbl.h"
#include "debug_serial.h"
#include "flux_machine.h"
#include "sensors.h"

// Define line flags. Includes comment type tracking and line overflow detection.
#define LINE_FLAG_OVERFLOW bit(0)
#define LINE_FLAG_COMMENT_PARENTHESES bit(1)
#define LINE_FLAG_COMMENT_SEMICOLON bit(2)

static char line[LINE_BUFFER_SIZE]; // Line to be executed. Zero-terminated.

static void protocol_exec_rt_suspend();

#if DEBUG_USB_CDC_CONNECT
uint8_t ctrl_line_state_change = 0;
uint16_t ctrl_line_state = 0;
#endif

int outputTag = 0;
/*
  GRBL PRIMARY LOOP:
*/
void protocol_main_loop()
{
  // Check for and report alarm state after a reset, error, or an initial power up.
  // NOTE: Sleep mode disables the stepper drivers and position can't be guaranteed.
  // Re-initialize the sleep state as an ALARM mode to ensure user homes or acknowledges.
  if (sys.state & (STATE_ALARM | STATE_SLEEP)) {
    report_feedback_message(MESSAGE_ALARM_LOCK);
    sys.state = STATE_ALARM; // Ensure alarm state is set.
  } else {
    // Check if the safety door is open.
    sys.state = STATE_IDLE;
    // if (system_check_safety_door_ajar()) {
    //   bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
    //   protocol_execute_realtime(); // Enter safety door mode. Should return as IDLE state.
    // }
    // All systems go!
    system_execute_startup(line); // Execute startup script.
  }

  // ---------------------------------------------------------------------------------
  // Primary loop! Upon a system abort, this exits back to main() to reset the system.
  // This is also where Grbl idles while waiting for something to do.
  // ---------------------------------------------------------------------------------

  uint32_t unlock_bits = 0;
  uint8_t resp_status;
  //cmd_process_unlocker.value = 0;
  //cmd_process_locker.value = 0;
  uint8_t line_flags = 0;
  uint8_t char_counter = 0;
  uint8_t c;
  uint32_t last_modbus_query = 0;
  for (;;) {

    // Process one line of incoming serial data, as the data becomes available. Performs an
    // initial filtering by removing spaces and comments and capitalizing all letters.
    //while (cmd_process_locker.value == 0 && (c = serial_read()) != SERIAL_NO_DATA) {
    while ((c = serial_read()) != SERIAL_NO_DATA) {
      if ((c == '\n') || (c == '\r')) { // End of line reached

        protocol_execute_realtime(); // Runtime command check point.
        if (sys.abort) { return; } // Bail to calling function upon system abort

        line[char_counter] = 0; // Set string termination character.
        #ifdef REPORT_ECHO_LINE_RECEIVED
          report_echo_line_received(line);
        #endif

        // Direct and execute one line of formatted input, and report status of execution.
        if (line_flags & LINE_FLAG_OVERFLOW) {
          // Report line overflow error.
          report_status_message(STATUS_OVERFLOW);
        } else if (line[0] == 0) {
          // Empty or comment line. For syncing purposes.
          report_status_message(STATUS_OK);
        } else if (line[0] == '$') {
          // Grbl '$' system command
          report_status_message(system_execute_line(line));
        } else if (sys.state & (STATE_ALARM | STATE_JOG)) {
          // Everything else is gcode. Block if in alarm or jog mode.
          report_status_message(STATUS_SYSTEM_GC_LOCK);
        } else {
          // Parse and execute g-code block.
          resp_status = gc_execute_line(line);
          if (sys.abort) {
            // Do nothing here
          } else {
            report_status_message(resp_status);
          }
        }

        // Reset tracking data for next line.
        line_flags = 0;
        char_counter = 0;

      } else {

        if (line_flags) {
          // Throw away all (except EOL) comment characters and overflow characters.
          if (c == ')') {
            // End of '()' comment. Resume line allowed.
            if (line_flags & LINE_FLAG_COMMENT_PARENTHESES) { line_flags &= ~(LINE_FLAG_COMMENT_PARENTHESES); }
          }
        } else {
          if (c <= ' ') {
            // Throw away whitepace and control characters
          } else if (c == '/') {
            // Block delete NOT SUPPORTED. Ignore character.
            // NOTE: If supported, would simply need to check the system if block delete is enabled.
          } else if (c == '(') {
            // Enable comments flag and ignore all characters until ')' or EOL.
            // NOTE: This doesn't follow the NIST definition exactly, but is good enough for now.
            // In the future, we could simply remove the items within the comments, but retain the
            // comment control characters, so that the g-code parser can error-check it.
            line_flags |= LINE_FLAG_COMMENT_PARENTHESES;
          } else if (c == ';') {
            // NOTE: ';' comment to EOL is a LinuxCNC definition. Not NIST.
            line_flags |= LINE_FLAG_COMMENT_SEMICOLON;
          // TODO: Install '%' feature
          // } else if (c == '%') {
            // Program start-end percent sign NOT SUPPORTED.
            // NOTE: This maybe installed to tell Grbl when a program is running vs manual input,
            // where, during a program, the system auto-cycle start will continue to execute
            // everything until the next '%' sign. This will help fix resuming issues with certain
            // functions that empty the planner buffer to execute its task on-time.
          } else if (char_counter >= (LINE_BUFFER_SIZE-1)) {
            // Detect line buffer overflow and set flag.
            line_flags |= LINE_FLAG_OVERFLOW;
          } else if (c >= 'a' && c <= 'z') { // Upcase lowercase
            line[char_counter++] = c-'a'+'A';
          } else {
            line[char_counter++] = c;
          }
        }

      }
    }

    // =============== FLUX's dedicated code ===============
    
    if (millis() - last_modbus_query > 100) {
      modbus_query();
      last_modbus_query = millis();
      outputTag += 1;
      int knob_val = adc_Buffer[0] * 1000 / 4096;
      printf("Knob %d, Wind Speed %d\n", knob_val, wind_speed);
      int target_speed = knob_val * 255 / 1000;
      int fanCompensation = target_speed - wind_speed;
      printString("Target ");
      printInteger(target_speed);
      printString(", Measure ");
      printInteger(wind_speed);
      int fanSpeed = max(min(1000, knob_val + fanCompensation),0);
      printString(", Fan Strength ");
      printInteger(fanSpeed);
      printString("\n");
      controlFan(1000 - fanSpeed); // Set fan 0~1000
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, knob_val);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, knob_val);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, knob_val / 3);

      // I2c write example
      // uint8_t resistance = 0x40;
      // HAL_I2C_Mem_Write(&hi2c1, 0x7C, 0, 1, &resistance, 1, 1000);
      // HAL_Delay(1);
      // HAL_I2C_Mem_Write(&hi2c1, 0x7D, 0, 1, &resistance, 1, 1000);
      // HAL_Delay(1);
      // uint8_t wrote_data;
      // HAL_I2C_Mem_Read(&hi2c1, 0x7C, 0, 1, &wrote_data, 1, 1000);
      // printf("Written register %x\n", wrote_data);
    }
    // =====================================================


    // If there are no more characters in the serial read buffer to be processed and executed,
    // this indicates that g-code streaming has either filled the planner buffer or has
    // completed. In either case, auto-cycle start, if enabled, any queued moves.
    protocol_auto_cycle_start();

    protocol_execute_realtime();  // Runtime command check point.
    if (sys.abort) { return; } // Bail to main() program loop to reset system.
  }

  return; /* Never reached */
}


// Block until all buffered steps are executed or in a cycle state. Works with feed hold
// during a synchronize call, if it should happen. Also, waits for clean cycle end.
void protocol_buffer_synchronize()
{
  // If system is queued, ensure cycle resumes if the auto start flag is present.
  // ============= Modified by FLUX ================
  //protocol_auto_cycle_start();
  do {
    // NOTE: Call protocol_auto_cycle_start() repetitively instead of once at first.
    //       This is necessary to handle the case that interrupt happens so frequently 
    //       that st_prep_buffer() isn't called before segment buffer is cleared (and plan buffer still contain blocks)
    protocol_auto_cycle_start();
    // ====================================================
    protocol_execute_realtime();   // Check and execute run-time commands
    if (sys.abort) { return; } // Check for system abort
  } while ((sys.state == STATE_CYCLE));
}


// Auto-cycle start triggers when there is a motion ready to execute and if the main program is not
// actively parsing commands.
// NOTE: This function is called from the main loop, buffer sync, and mc_line() only and executes
// when one of these conditions exist respectively: There are no more blocks sent (i.e. streaming
// is finished, single commands), a command that needs to wait for the motions in the buffer to
// execute calls a buffer sync, or the planner buffer is full and ready to go.
void protocol_auto_cycle_start()
{
  system_set_exec_state_flag(EXEC_CYCLE_START); 
}


// This function is the general interface to Grbl's real-time command execution system. It is called
// from various check points in the main program, primarily where there may be a while loop waiting
// for a buffer to clear space or any point where the execution time from the last check point may
// be more than a fraction of a second. This is a way to execute realtime commands asynchronously
// (aka multitasking) with grbl's g-code parsing and planning functions. This function also serves
// as an interface for the interrupts to set the system realtime flags, where only the main program
// handles them, removing the need to define more computationally-expensive volatile variables. This
// also provides a controlled way to execute certain tasks without having two or more instances of
// the same task, such as the planner recalculating the buffer upon a feedhold or overrides.
// NOTE: The sys_rt_exec_state variable flags are set by any process, step or serial interrupts, pinouts,
// limit switches, or the main program.
void protocol_execute_realtime()
{
  protocol_exec_rt_system();
  if (sys.suspend) { protocol_exec_rt_suspend(); }
}


// Executes run-time commands, when required. This function primarily operates as Grbl's state
// machine and controls the various real-time features Grbl has to offer.
// NOTE: Do not alter this unless you know exactly what you are doing!
void protocol_exec_rt_system()
{
  flux_periodic_handling();

  uint8_t led_alarm_flash_idx = 0;
  uint8_t rt_exec; // Temp variable to avoid calling volatile multiple times.
  rt_exec = sys_rt_exec_alarm; // Copy volatile sys_rt_exec_alarm.
  if (rt_exec) { // Enter only if any bit flag is true
    // System alarm. Everything has shutdown by something that has gone severely wrong. Report
    // the source of the error to the user. If critical, Grbl disables by entering an infinite
    // loop until system reset/abort.
    sys.state = STATE_ALARM; // Set system alarm state
    report_alarm_message(rt_exec);
    // Halt everything upon a critical event flag. Currently hard and soft limits flag this.
    if ((rt_exec == EXEC_ALARM_HARD_LIMIT) || (rt_exec == EXEC_ALARM_SOFT_LIMIT)) {
      report_feedback_message(MESSAGE_CRITICAL_EVENT);
      system_clear_exec_state_flag(EXEC_RESET); // Disable any existing reset
      // =============== FLUX's dedicated handling ==============
      set_led_mode(kManual);
      // =============== End of FLUX's dedicated handling ==============
      do {
        // Block everything, except reset and status reports, until user issues reset or power
        // cycles. Hard limits typically occur while unattended or not paying attention. Gives
        // the user and a GUI time to do what is needed before resetting, like killing the
        // incoming stream. The same could be said about soft limits. While the position is not
        // lost, continued streaming could cause a serious crash if by chance it gets executed.
        
        // =============== FLUX's dedicated handling ==============
        // Show LED alarm indicator (request user to power cycle the machine)
        if (led_alarm_flash_idx == 0) {
          set_led_power(1000);
        } else if (led_alarm_flash_idx == 1) {
          set_led_power(0);
        } else if (led_alarm_flash_idx == 2) {
          set_led_power(1000);
        } else if (led_alarm_flash_idx == 3) {
          set_led_power(0);
        } else if (led_alarm_flash_idx == 4) {
          set_led_power(1000);
        } else if (led_alarm_flash_idx <= 8) {
          set_led_power(0);
        }
        HAL_Delay(500);
        led_alarm_flash_idx++;
        if (led_alarm_flash_idx > 8) {
          led_alarm_flash_idx = 0;
        }
        // =============== End of FLUX's dedicated handling ==============
      } while (bit_isfalse(sys_rt_exec_state,EXEC_RESET));
    }
    system_clear_exec_alarm(); // Clear alarm
  }

  rt_exec = sys_rt_exec_state; // Copy volatile sys_rt_exec_state.
  if (rt_exec) {

    // Execute system abort.
    if (rt_exec & EXEC_RESET) {
      sys.abort = true;  // Only place this is set true.
      return; // Nothing else to do but exit.
    }

    // Execute and serial print status
    if (rt_exec & EXEC_STATUS_REPORT) {
      report_realtime_status();
      system_clear_exec_state_flag(EXEC_STATUS_REPORT);
    }

    // NOTE: Once hold is initiated, the system immediately enters a suspend state to block all
    // main program processes until either reset or resumed. This ensures a hold completes safely.
    if (rt_exec & (EXEC_MOTION_CANCEL | EXEC_FEED_HOLD | EXEC_SAFETY_DOOR | EXEC_SLEEP)) {

      // State check for allowable states for hold methods.
      if (!(sys.state & (STATE_ALARM | STATE_CHECK_MODE))) {
        // If IDLE, Grbl is not in motion. Simply indicate suspend state and hold is complete.
        if (sys.state == STATE_IDLE) { sys.suspend = SUSPEND_HOLD_COMPLETE; }

        // Execute a safety door stop with a feed hold and disable spindle/coolant.
        // NOTE: Safety door differs from feed holds by stopping everything no matter state, disables powered
        // devices (spindle/coolant), and blocks resuming until switch is re-engaged.
      }

      if (rt_exec & EXEC_SLEEP) {
        if (sys.state == STATE_ALARM) { sys.suspend |= (SUSPEND_RETRACT_COMPLETE|SUSPEND_HOLD_COMPLETE); }
        sys.state = STATE_SLEEP; 
      }

      system_clear_exec_state_flag((EXEC_MOTION_CANCEL | EXEC_FEED_HOLD | EXEC_SAFETY_DOOR | EXEC_SLEEP));
    }

    // Execute a cycle start by starting the stepper interrupt to begin executing the blocks in queue.
    if (rt_exec & EXEC_CYCLE_START) {
      // Block if called at same time as the hold commands: feed hold, motion cancel, and safety door.
      // Ensures auto-cycle-start doesn't resume a hold without an explicit user-input.
      if (!(rt_exec & (EXEC_FEED_HOLD | EXEC_MOTION_CANCEL | EXEC_SAFETY_DOOR))) {

        // Cycle start only when IDLE or when a hold is complete and ready to resume.
        if ((sys.state == STATE_IDLE) || ((sys.state & STATE_HOLD) && (sys.suspend & SUSPEND_HOLD_COMPLETE))) {
          if (sys.state == STATE_HOLD && sys.spindle_stop_ovr) {
            sys.spindle_stop_ovr |= SPINDLE_STOP_OVR_RESTORE_CYCLE; // Set to restore in suspend routine and cycle start after.
          } else {
            // Start cycle only if queued motions exist in planner buffer and the motion is not canceled.
            sys.step_control = STEP_CONTROL_NORMAL_OP; // Restore step control to normal operation
            sys.suspend = SUSPEND_DISABLE; // Break suspend state.
            sys.state = STATE_IDLE;
          }
        }
      }
      system_clear_exec_state_flag(EXEC_CYCLE_START);
    }

    if (rt_exec & EXEC_CYCLE_STOP) {
      // Reinitializes the cycle plan and stepper system after a feed hold for a resume. Called by
      // realtime command execution in the main program, ensuring that the planner re-plans safely.
      // NOTE: Bresenham algorithm variables are still maintained through both the planner and stepper
      // cycle reinitializations. The stepper path should continue exactly as if nothing has happened.
      // NOTE: EXEC_CYCLE_STOP is set by the stepper subsystem when a cycle or feed hold completes.
      if ((sys.state & (STATE_HOLD|STATE_SAFETY_DOOR|STATE_SLEEP)) && !(sys.soft_limit) && !(sys.suspend & SUSPEND_JOG_CANCEL)) {
      } else {
        // Motion complete. Includes CYCLE/JOG/HOMING states and jog cancel/motion cancel/soft limit events.
        // NOTE: Motion and jog cancel both immediately return to idle after the hold completes.
      }
      system_clear_exec_state_flag(EXEC_CYCLE_STOP);
    }
  }

  // Execute overrides.
  rt_exec = sys_rt_exec_motion_override; // Copy volatile sys_rt_exec_motion_override
  if (rt_exec) {
    system_clear_exec_motion_overrides(); // Clear all motion override flags.

    uint8_t new_f_override =  sys.f_override;

    uint8_t new_r_override = sys.r_override;
    if (rt_exec & EXEC_RAPID_OVR_RESET) { new_r_override = DEFAULT_RAPID_OVERRIDE; }
    if (rt_exec & EXEC_RAPID_OVR_MEDIUM) { new_r_override = RAPID_OVERRIDE_MEDIUM; }
    if (rt_exec & EXEC_RAPID_OVR_LOW) { new_r_override = RAPID_OVERRIDE_LOW; }

    if ((new_f_override != sys.f_override) || (new_r_override != sys.r_override)) {
      sys.f_override = new_f_override;
      sys.r_override = new_r_override;
      sys.report_ovr_counter = 0; // Set to report change immediately
    }
  }

  rt_exec = sys_rt_exec_accessory_override;
  if (rt_exec) {
    system_clear_exec_accessory_overrides(); // Clear all accessory override flags.

    // NOTE: Unlike motion overrides, spindle overrides do not require a planner reinitialization.
    uint8_t last_s_override =  sys.spindle_speed_ovr;
    if (rt_exec & EXEC_SPINDLE_OVR_RESET) { last_s_override = DEFAULT_SPINDLE_SPEED_OVERRIDE; }
    if (rt_exec & EXEC_SPINDLE_OVR_COARSE_PLUS) { last_s_override += SPINDLE_OVERRIDE_COARSE_INCREMENT; }
    if (rt_exec & EXEC_SPINDLE_OVR_COARSE_MINUS) { last_s_override -= SPINDLE_OVERRIDE_COARSE_INCREMENT; }
    if (rt_exec & EXEC_SPINDLE_OVR_FINE_PLUS) { last_s_override += SPINDLE_OVERRIDE_FINE_INCREMENT; }
    if (rt_exec & EXEC_SPINDLE_OVR_FINE_MINUS) { last_s_override -= SPINDLE_OVERRIDE_FINE_INCREMENT; }
    last_s_override = min(last_s_override,MAX_SPINDLE_SPEED_OVERRIDE);
    last_s_override = max(last_s_override,MIN_SPINDLE_SPEED_OVERRIDE);

    if (last_s_override != sys.spindle_speed_ovr) {
      sys.spindle_speed_ovr = last_s_override;
      // NOTE: Spindle speed overrides during HOLD state are taken care of by suspend function.
      if (sys.state == STATE_IDLE) { 
      }
      sys.report_ovr_counter = 0; // Set to report change immediately
    }

    if (rt_exec & EXEC_SPINDLE_OVR_STOP) {
      // Spindle stop override allowed only while in HOLD state.
      // NOTE: Report counters are set in spindle_set_state() when spindle stop is executed.
      if (sys.state == STATE_HOLD) {
        if (!(sys.spindle_stop_ovr)) { sys.spindle_stop_ovr = SPINDLE_STOP_OVR_INITIATE; }
        else if (sys.spindle_stop_ovr & SPINDLE_STOP_OVR_ENABLED) { sys.spindle_stop_ovr |= SPINDLE_STOP_OVR_RESTORE; }
      }
    }
  }

  #ifdef DEBUG
    if (sys_rt_exec_debug) {
      report_realtime_debug();
      sys_rt_exec_debug = 0;
    }
  #endif
}


// Handles Grbl system suspend procedures, such as feed hold, safety door, and parking motion.
// The system will enter this loop, create local variables for suspend tasks, and return to
// whatever function that invoked the suspend, such that Grbl resumes normal operation.
// This function is written in a way to promote custom parking motions. Simply use this as a
// template
static void protocol_exec_rt_suspend()
{
  uint8_t restore_condition;

  while (sys.suspend) {

    if (sys.abort) { return; }

    // Block until initial hold is complete and the machine has stopped motion.
    if (sys.suspend & SUSPEND_HOLD_COMPLETE) {

      // Parking manager. Handles de/re-energizing, switch state checks, and parking motions for 
      // the safety door and sleep states.
      if (sys.state & (STATE_SAFETY_DOOR | STATE_SLEEP)) {
      
        // Handles retraction motions and de-energizing.
        if (bit_isfalse(sys.suspend,SUSPEND_RETRACT_COMPLETE)) {
          sys.suspend &= ~(SUSPEND_RESTART_RETRACT);
          sys.suspend |= SUSPEND_RETRACT_COMPLETE;

        } else {

          
          if (sys.state == STATE_SLEEP) {
            report_feedback_message(MESSAGE_SLEEP_MODE);
            while (!(sys.abort)) { protocol_exec_rt_system(); } // Do nothing until reset.
            return; // Abort received. Return to re-initialize.
          }    
          
          // Allows resuming from parking/safety door. Actively checks if safety door is closed and ready to resume.
          if (sys.state == STATE_SAFETY_DOOR) {
            //if (!(system_check_safety_door_ajar())) {
              sys.suspend &= ~(SUSPEND_SAFETY_DOOR_AJAR); // Reset door ajar flag to denote ready to resume.
            //}
          }

          // Handles parking restore and safety door resume.
          if (sys.suspend & SUSPEND_INITIATE_RESTORE) {

            #ifdef PARKING_ENABLE
              // Execute fast restore motion to the pull-out position. Parking requires homing enabled.
              // NOTE: State is will remain DOOR, until the de-energizing and retract is complete.
              #ifdef ENABLE_PARKING_OVERRIDE_CONTROL
              if (((settings.flags & (BITFLAG_HOMING_ENABLE|BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) &&
                   (sys.override_ctrl == OVERRIDE_PARKING_MOTION)) {
              #else
              if ((settings.flags & (BITFLAG_HOMING_ENABLE|BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) {
              #endif
                // Check to ensure the motion doesn't move below pull-out position.
                if (parking_target[PARKING_AXIS] <= PARKING_TARGET) {
                  parking_target[PARKING_AXIS] = retract_waypoint;
                  pl_data->feed_rate = PARKING_RATE;
                  mc_parking_motion(parking_target, pl_data);
                }
              }
            #endif

            // Delayed Tasks: Restart spindle and coolant, delay to power-up, then resume cycle.

            #ifdef PARKING_ENABLE
              // Execute slow plunge motion from pull-out position to resume position.
              #ifdef ENABLE_PARKING_OVERRIDE_CONTROL
              if (((settings.flags & (BITFLAG_HOMING_ENABLE|BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) &&
                   (sys.override_ctrl == OVERRIDE_PARKING_MOTION)) {
              #else
              if ((settings.flags & (BITFLAG_HOMING_ENABLE|BITFLAG_LASER_MODE)) == BITFLAG_HOMING_ENABLE) {
              #endif
                // Block if safety door re-opened during prior restore actions.
                if (bit_isfalse(sys.suspend,SUSPEND_RESTART_RETRACT)) {
                  // Regardless if the retract parking motion was a valid/safe motion or not, the
                  // restore parking motion should logically be valid, either by returning to the
                  // original position through valid machine space or by not moving at all.
                  pl_data->feed_rate = PARKING_PULLOUT_RATE;
									pl_data->condition |= (restore_condition & PL_COND_ACCESSORY_MASK); // Restore accessory state
									pl_data->spindle_speed = restore_spindle_speed;
                  mc_parking_motion(restore_target, pl_data);
                }
              }
            #endif

            if (bit_isfalse(sys.suspend,SUSPEND_RESTART_RETRACT)) {
              sys.suspend |= SUSPEND_RESTORE_COMPLETE;
              system_set_exec_state_flag(EXEC_CYCLE_START); // Set to resume program.
            }
          }

        }


      }
    }

    protocol_exec_rt_system();

  }
}
