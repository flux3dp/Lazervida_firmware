
#include "flux_machine.h"
#include "main.h"
#include "sensors.h"
#include "stepper.h"
#include "system.h"
#include "fast_raster_print.h"
#include "Adafruit_MSA311.h"
#include <math.h>

#define APP_ADDR                (FLASH_BASE + 0x9000)
// NOTE: The page before APP_ADDR is for firmware update state indicator
#define FW_UPDATE_METADATA_ADDR (APP_ADDR - FLASH_PAGE_SIZE)
// NOTE: The last page (1KB = 0x400) is for non-volatile settings
#define APP_SIZE                (DEV_FLASH_SIZE - 0x9000 - FLASH_PAGE_SIZE) 

extern system_t sys;

volatile uint8_t host_com_port_open = 0;

volatile cmd_process_locker_t cmd_process_locker;
volatile cmd_process_unlocker_t cmd_process_unlocker;

bool MSA311_INT_triggered = false;
uint32_t MSA311_polling_ts = 0;
float reference_tilt = 3.07; // in unit of rad. Should be initialized when power up

volatile bool machine_power_on = false;

uint32_t last_motor_active = 0;

#define CURRENT_LED_PWM_POWER (htim3.Instance->CCR2)

#if DEBUG_USB_CDC_CONNECT
extern uint8_t ctrl_line_state_change;
extern uint16_t ctrl_line_state;
#endif
typedef struct {
  LedMode mode;
  uint32_t last_ts;
  uint32_t duty_cycle;
} LedState ;
LedState led_state;

void led_handler(uint32_t new_ts);

void flux_periodic_handling() {
  #if DEBUG_USB_CDC_CONNECT
  if (ctrl_line_state_change == 1) {
    debugString("ctrl_line_state:");
    debug_uint32_base10(ctrl_line_state);
    debugString("\n");
    ctrl_line_state_change = 0;
  }
  #endif
  // Detect change of orientation
  /*
  if (settings.disable_tilt_detect != true) {
    if (millis() - MSA311_polling_ts > 400) {
      if (MSA311_working()) {
        Adafruit_MSA311_read();
        // NOTE: We only care about the change in y-axis
        if (sys.state == STATE_CYCLE) {
          float tilt = MSA311_get_tilt_y();
          // About 15 degree = 3.14 * 10 / 180 ~ 0.17 rad
          if (tilt - reference_tilt > 0.85 || reference_tilt - tilt > 0.85) {
            bit_true(sys_rt_exec_state, EXEC_FEED_HOLD);
            printString("[DEBUG: ");
            printFloat(reference_tilt, 3);
            printString(",");
            printFloat(tilt, 3);
            printString("]\n");
            printString("[FLUX: tilt]\n");
          }
        }
      }
      MSA311_polling_ts = millis();
    }
  }
  */

  // Detect sudden active motion (shake or vibrate)
  if (MSA311_INT_triggered) {
    printString("[FLUX: act]\n");
    MSA311_INT_triggered = false;
    if (sys.state == STATE_CYCLE || is_in_fast_raster_mode()) {
      bit_true(sys_rt_exec_state, EXEC_FEED_HOLD);
    }
  }

  // Disable motor when idle for 5 seconds
  if (HAL_GPIO_ReadPin(STEP_DISABLE_GPIO_PORT, STEPPERS_DISABLE_PIN) == GPIO_PIN_RESET) {
    if (sys.state == STATE_IDLE || sys.state == STATE_ALARM) {
      // do nothing
    } else {
      last_motor_active = millis();
    }
    if (millis() - last_motor_active > 5000) {
      HAL_GPIO_WritePin(STEP_DISABLE_GPIO_PORT, STEPPERS_DISABLE_PIN, GPIO_PIN_SET);
      // disable both motor and laser
      reset_power_24v();
    }
  }

  // Led status control
  switch (sys.state) {
    case STATE_IDLE:
    case STATE_ALARM:
      set_led_mode(kBreath);
      break;
    case STATE_CYCLE:
      set_led_mode(kOn);
      break;
    case STATE_HOLD:
      set_led_mode(kFlash);
      break;
    default:
      break;
  }
  led_handler(millis());

}

void set_stepper_MS1() {
  HAL_GPIO_WritePin(MS1_GPIO_Port, MS1_Pin, GPIO_PIN_SET);
}

void reset_stepper_MS1() {
  HAL_GPIO_WritePin(MS1_GPIO_Port, MS1_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Power for both motor and laser
 * 
 */
void set_power_24v() {
  HAL_GPIO_WritePin(PWR_24V_GPIO_Port, PWR_24V_Pin, GPIO_PIN_SET);
}

/**
 * @brief Power for both motor and laser
 * 
 */
void reset_power_24v() {
  HAL_GPIO_WritePin(PWR_24V_GPIO_Port, PWR_24V_Pin, GPIO_PIN_RESET);
}

void set_led_mode(LedMode new_mode) {
  if (led_state.mode == new_mode) {
    return;
  }
  switch (new_mode) {
    case kOn:
      led_state.duty_cycle = 1000;
      break;
    case kOff:
      led_state.duty_cycle = 0;
      break;
    case kFlash:
      led_state.duty_cycle = 1000;
      break;
    case kBreath:
      led_state.duty_cycle = 0;
      break;
    case kManual:
    default:
      led_state.duty_cycle = 0;
      new_mode = kManual;
      break;
  }
  led_state.mode = new_mode;
  CURRENT_LED_PWM_POWER = led_state.duty_cycle;
  led_state.last_ts = millis();
}

void set_led_power(uint32_t val) {
  if (led_state.mode == kManual) {
    CURRENT_LED_PWM_POWER = val;
  }
}

void led_handler(uint32_t new_ts) {
  switch (led_state.mode) {
    case kOff:
      break;
    case kOn:
      break;
    case kFlash:
      if (new_ts - led_state.last_ts > 600) {
        if (led_state.duty_cycle == 0) {
          led_state.duty_cycle = 1000;
        } else {
          led_state.duty_cycle = 0;
        }
        led_state.last_ts = new_ts;
        CURRENT_LED_PWM_POWER = led_state.duty_cycle;
      }
      break;
    case kBreath:
      if (new_ts - led_state.last_ts > 60) {
        if (led_state.duty_cycle >= 2000) {
          led_state.duty_cycle = 0;
        } else {
          led_state.duty_cycle += 40;
        }
        led_state.last_ts = new_ts;
        CURRENT_LED_PWM_POWER = led_state.duty_cycle <= 1000 ? 
                                led_state.duty_cycle : 2000 - led_state.duty_cycle;
      }
      break;
    case kManual:
    default:
      break;
  }
}

void start_firmware_update() {
  // 1. Erase the first page at APP addr
  HAL_StatusTypeDef status;
  HAL_FLASH_Unlock();
  uint32_t PageError = 0;
  FLASH_EraseInitTypeDef eraseinitstruct;
  
  eraseinitstruct.TypeErase = FLASH_TYPEERASE_PAGES;
  eraseinitstruct.PageAddress = FW_UPDATE_METADATA_ADDR;
  eraseinitstruct.NbPages = 1;
  status = HAL_FLASHEx_Erase(&eraseinitstruct, &PageError);
  
  HAL_FLASH_Lock();
  if(status != HAL_OK)
  {
    return;
  }

  // 2. Software Reset
  NVIC_SystemReset();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
    case GPIO_PIN_1:
      if (sys.state != STATE_HOLD) {
        MSA311_INT_triggered = true;
      }
      break;
    case LIMIT_X_Pin:
    case LIMIT_Y_Pin:
      if (sys.state != STATE_ALARM) {  // Ignore if already in alarm state. 
        if (!(sys_rt_exec_alarm)) {
          // Check limit pin state. 
          if (limits_get_state()) {
            mc_reset(); // Initiate system kill.
            system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT); // Indicate hard limit critical event
          }
        }
      }
      break;
    case GPIO_PIN_9: // POWER BTN
      if (machine_power_on) {
        // Software reset (enter power off state)
        NVIC_SystemReset();
      }
      break;
    default:
      break;
  }
}
