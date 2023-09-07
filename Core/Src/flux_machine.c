
#include "flux_machine.h"
#include "main.h"
#include "sensors.h"
#include "system.h"
#include <math.h>

#define APP_ADDR                (FLASH_BASE + 0x9000)
// NOTE: The page before APP_ADDR is for firmware update state indicator
#define FW_UPDATE_METADATA_ADDR (APP_ADDR - FLASH_PAGE_SIZE)
// NOTE: The last page (1KB = 0x400) is for non-volatile settings
#define APP_SIZE                (DEV_FLASH_SIZE - 0x9000 - FLASH_PAGE_SIZE) 

extern system_t sys;

volatile uint8_t host_com_port_open = 0;

//volatile cmd_process_locker_t cmd_process_locker;
//volatile cmd_process_unlocker_t cmd_process_unlocker;

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
  return;
  // if (led_state.mode == new_mode) {
  //   return;
  // }
  // switch (new_mode) {
  //   case kOn:
  //     led_state.duty_cycle = 1000;
  //     break;
  //   case kOff:
  //     led_state.duty_cycle = 0;
  //     break;
  //   case kFlash:
  //     led_state.duty_cycle = 1000;
  //     break;
  //   case kBreath:
  //     led_state.duty_cycle = 0;
  //     break;
  //   case kManual:
  //   default:
  //     led_state.duty_cycle = 0;
  //     new_mode = kManual;
  //     break;
  // }
  // led_state.mode = new_mode;
  // CURRENT_LED_PWM_POWER = led_state.duty_cycle;
  // led_state.last_ts = millis();
}

void set_led_power(uint32_t val) {
  return;
  // if (led_state.mode == kManual) {
  //   CURRENT_LED_PWM_POWER = val;
  // }
}

void led_handler(uint32_t new_ts) {
  return;
  // switch (led_state.mode) {
  //   case kOff:
  //     break;
  //   case kOn:
  //     break;
  //   case kFlash:
  //     if (new_ts - led_state.last_ts > 600) {
  //       if (led_state.duty_cycle == 0) {
  //         led_state.duty_cycle = 1000;
  //       } else {
  //         led_state.duty_cycle = 0;
  //       }
  //       led_state.last_ts = new_ts;
  //       CURRENT_LED_PWM_POWER = led_state.duty_cycle;
  //     }
  //     break;
  //   case kBreath:
  //     if (new_ts - led_state.last_ts > 60) {
  //       if (led_state.duty_cycle >= 2000) {
  //         led_state.duty_cycle = 0;
  //       } else {
  //         led_state.duty_cycle += 40;
  //       }
  //       led_state.last_ts = new_ts;
  //       CURRENT_LED_PWM_POWER = led_state.duty_cycle <= 1000 ? 
  //                               led_state.duty_cycle : 2000 - led_state.duty_cycle;
  //     }
  //     break;
  //   case kManual:
  //   default:
  //     break;
  // }
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
