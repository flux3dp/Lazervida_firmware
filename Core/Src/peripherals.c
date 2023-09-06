#include "peripherals.h"
#include "main.h"
#include "print.h"
#include <stdbool.h>
#include <stdio.h>
#include "flux_machine.h"

/**
 * @brief Initialize Power 24V (and fan on laser head)
 *                      NOTE: MUST enable 24V to enable motor/laser
 *                   I2C (for RGB-led and G-sensor)
 * 
 */
void peripherals_init() {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  return;
}

void I2C_scanning() {
  int i;
  HAL_StatusTypeDef ret;
  printf("I2C Scanning...\r\n");
  for(i=0; i<255; i++)
  {
    ret = HAL_I2C_IsDeviceReady(&hi2c1, i, 3, 5);
    if(ret == HAL_OK) {
      printf("Discovered %x\n", i >> 1);
    }
  }
  printf("I2C Scan Done!\r\n\r\n");
  /*--[ Scanning Done ]--*/
}


void enable_interrupt_for_collision() {
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

void disable_interrupt_for_collision() {
  /* EXTI interrupt init*/
  HAL_NVIC_DisableIRQ(EXTI1_IRQn);
}
