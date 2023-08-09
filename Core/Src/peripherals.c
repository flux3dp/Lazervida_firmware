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
  
  /*Configure GPIO pins : PWR_24V */
  GPIO_InitStruct.Pin = PWR_24V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // MSA311 Accelerometer: I2C and INT pin
  GPIO_InitStruct.Pin = MSA311_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MSA311_INT_GPIO_Port, &GPIO_InitStruct);
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
