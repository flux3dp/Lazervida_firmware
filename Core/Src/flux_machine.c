
#include "flux_machine.h"
#include "main.h"
#include "sensors.h"
#include "stepper.h"
#include "system.h"
#include "fast_raster_print.h"

#define APP_ADDR                (FLASH_BASE + 0x9000)
// NOTE: The page before APP_ADDR is for firmware update state indicator
#define FW_UPDATE_METADATA_ADDR (APP_ADDR - FLASH_PAGE_SIZE)
// NOTE: The last page (1KB = 0x400) is for non-volatile settings
#define APP_SIZE                (DEV_FLASH_SIZE - 0x9000 - FLASH_PAGE_SIZE) 

extern system_t sys;
extern PCD_HandleTypeDef hpcd_USB_FS;

volatile cmd_process_locker_t cmd_process_locker;
volatile cmd_process_unlocker_t cmd_process_unlocker;

bool MSA311_INT_triggered = false;

uint32_t last_motor_active = 0;

#define CURRENT_LED_PWM_POWER (htim3.Instance->CCR2)

typedef struct {
  LedMode mode;
  uint32_t last_ts;
  uint32_t duty_cycle;
} LedState ;
LedState led_state;

void led_handler(uint32_t new_ts);

void flux_periodic_handling() {
  if (MSA311_INT_triggered) {
    printString("[DEBUG: MSA311 int]\n");
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

void set_power_24v() {
  HAL_GPIO_WritePin(PWR_24V_GPIO_Port, PWR_24V_Pin, GPIO_PIN_SET);
}

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
  delay_ms(1000);
  HAL_PCD_Stop(&hpcd_USB_FS);
  delay_ms(1000);
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
    default:
      break;
  }
}
