/*
  limits.c - code pertaining to limit-switches and performing the homing cycle
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
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
#include "main.h"

// Homing axis search distance multiplier. Computed by this value times the cycle travel.
#ifndef HOMING_AXIS_SEARCH_SCALAR
  #define HOMING_AXIS_SEARCH_SCALAR  1.5 // Must be > 1 to ensure limit switch will be engaged.
#endif
#ifndef HOMING_AXIS_LOCATE_SCALAR
  #define HOMING_AXIS_LOCATE_SCALAR  5.0 // Must be > 1 to ensure limit switch is cleared.
#endif

extern uint32_t MSA311_polling_ts;

#ifdef ENABLE_DUAL_AXIS
  // Flags for dual axis async limit trigger check.
  #define DUAL_AXIS_CHECK_DISABLE     0  // Must be zero
  #define DUAL_AXIS_CHECK_ENABLE      bit(0)
  #define DUAL_AXIS_CHECK_TRIGGER_1   bit(1)
  #define DUAL_AXIS_CHECK_TRIGGER_2   bit(2)
#endif

/**
 * @brief Config limit pins as EXTI interrupt pins or simple input pins
 * 
 */
void limits_init()
{
  return;
}


/**
 * @brief Disable hard limit interrupt
 *        limit pins become simple input pins
 * 
 */
void limits_disable()
{
  // Diable limit pins EXTI interrupt
  HAL_GPIO_DeInit(LIMIT_X_GPIO_Port, LIMIT_X_Pin);
  HAL_GPIO_DeInit(LIMIT_Y_GPIO_Port, LIMIT_Y_Pin);

  // Simple input
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LIMIT_Y_Pin|LIMIT_X_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  #ifdef DISABLE_LIMIT_PIN_PULL_UP
    GPIO_InitStruct.Pull = GPIO_NOPULL;
  #else
    GPIO_InitStruct.Pull = GPIO_PULLUP;
  #endif
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}


// Returns limit state as a bit-wise uint8 variable. Each bit indicates an axis limit, where
// triggered is 1 and not triggered is 0. Invert mask is applied. Axes are defined by their
// number in bit position, i.e. Z_AXIS is (1<<2) or bit 2, and Y_AXIS is (1<<1) or bit 1.
uint8_t limits_get_state()
{
  uint8_t limit_state = 0;
  uint8_t pin = (read_limit_port() & LIMIT_MASK);

  #ifdef INVERT_LIMIT_PIN_MASK
    pin ^= INVERT_LIMIT_PIN_MASK;
  #endif
  if (bit_isfalse(settings.flags,BITFLAG_INVERT_LIMIT_PINS)) { pin ^= LIMIT_MASK; }
  if (pin) {
    uint8_t idx;
    for (idx=0; idx<N_AXIS; idx++) {
      if (pin & get_limit_pin_mask(idx)) { limit_state |= (1 << idx); }
    }
    #ifdef ENABLE_DUAL_AXIS
      #error "DUAL_LIMIT_BIT not defined"
      if (pin & (1<<DUAL_LIMIT_BIT)) { limit_state |= (1 << N_AXIS); }
    #endif
  }
  return(limit_state);
}


// This is the Limit Pin Change Interrupt, which handles the hard limit feature. A bouncing
// limit switch can cause a lot of problems, like false readings and multiple interrupt calls.
// If a switch is triggered at all, something bad has happened and treat it as such, regardless
// if a limit switch is being disengaged. It's impossible to reliably tell the state of a
// bouncing pin because the Arduino microcontroller does not retain any state information when
// detecting a pin change. If we poll the pins in the ISR, you can miss the correct reading if the 
// switch is bouncing.
// NOTE: Do not attach an e-stop to the limit pins, because this interrupt is disabled during
// homing cycles and will not respond correctly. Upon user request or need, there may be a
// special pinout for an e-stop, but it is generally recommended to just directly connect
// your e-stop switch to the Arduino reset pin, since it is the most correct way to do this.
#ifndef ENABLE_SOFTWARE_DEBOUNCE
/*
// TODO: Limit pins EXTI interrupt IRQ
  ISR(LIMIT_INT_vect) // DEFAULT: Limit pin change interrupt process.
  {
    // Ignore limit switches if already in an alarm state or in-process of executing an alarm.
    // When in the alarm state, Grbl should have been reset or will force a reset, so any pending
    // moves in the planner and serial buffers are all cleared and newly sent blocks will be
    // locked out until a homing cycle or a kill lock command. Allows the user to disable the hard
    // limit setting if their limits are constantly triggering after a reset and move their axes.
    if (sys.state != STATE_ALARM) {
      if (!(sys_rt_exec_alarm)) {
        #ifdef HARD_LIMIT_FORCE_STATE_CHECK
          // Check limit pin state.
          if (limits_get_state()) {
            mc_reset(); // Initiate system kill.
            system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT); // Indicate hard limit critical event
          }
        #else
          mc_reset(); // Initiate system kill.
          system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT); // Indicate hard limit critical event
        #endif
      }
    }
  }
*/
#else // OPTIONAL: Software debounce limit pin routine.
/* TODO: 
  // Upon limit pin change, enable watchdog timer to create a short delay. 
  ISR(LIMIT_INT_vect) { 
    if (!(WDTCSR & (1<<WDIE))) { 
      WDTCSR |= (1<<WDIE); 
    } 
  }
  ISR(WDT_vect) // Watchdog timer ISR
  {
    WDTCSR &= ~(1<<WDIE); // Disable watchdog timer. 
    if (sys.state != STATE_ALARM) {  // Ignore if already in alarm state. 
      if (!(sys_rt_exec_alarm)) {
        // Check limit pin state. 
        if (limits_get_state()) {
          mc_reset(); // Initiate system kill.
          system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT); // Indicate hard limit critical event
        }
      }  
    }
  }
*/
#endif

// Homes the specified cycle axes, sets the machine position, and performs a pull-off motion after
// completing. Homing is a special motion case, which involves rapid uncontrolled stops to locate
// the trigger point of the limit switches. The rapid stops are handled by a system level axis lock
// mask, which prevents the stepper algorithm from executing step pulses. Homing motions typically
// circumvent the processes for executing motions in normal operation.
// NOTE: Only the abort realtime command can interrupt this process.
// TODO: Move limit pin-specific calls to a general function for portability.
// NOTE: sys_position will be reset/assigned at the end of the function
void limits_go_home(uint8_t cycle_mask)
{
  return;
}


// Performs a soft limit check. Called from mc_line() only. Assumes the machine has been homed,
// the workspace volume is in all negative space, and the system is in normal operation.
// NOTE: Used by jogging to limit travel within soft-limit volume.
void limits_soft_check(float *target)
{
  if (system_check_travel_limits(target)) {
    sys.soft_limit = true;
    // Force feed hold if cycle is active. All buffered blocks are guaranteed to be within
    // workspace volume so just come to a controlled stop so position is not lost. When complete
    // enter alarm mode.
    if (sys.state == STATE_CYCLE) {
      system_set_exec_state_flag(EXEC_FEED_HOLD);
      do {
        protocol_execute_realtime();
        if (sys.abort) { return; }
      } while ( sys.state != STATE_IDLE );
    }
    mc_reset(); // Issue system reset and ensure spindle and coolant are shutdown.
    system_set_exec_alarm(EXEC_ALARM_SOFT_LIMIT); // Indicate soft limit critical event
    protocol_execute_realtime(); // Execute to enter critical event loop and system abort
    return;
  }
}
