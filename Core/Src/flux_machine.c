
#include "flux_machine.h"
#include "main.h"
#include "sensors.h"
#include "stepper.h"
#include "system.h"

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

void flux_periodic_handling() {
  /*
  if (door_is_open()) {
    system_set_exec_state_flag(EXEC_SAFETY_DOOR);
  }
  if (base_is_open()) {
    system_set_exec_state_flag(EXEC_SAFETY_DOOR);
  }
  */
  if (MSA311_INT_triggered) {
    printString("[DEBUG: MSA311 int]\n");
    MSA311_INT_triggered = false;
    bit_true(sys_rt_exec_state, EXEC_FEED_HOLD);
  }
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
