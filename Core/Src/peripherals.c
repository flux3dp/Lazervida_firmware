#include "peripherals.h"
#include "main.h"
#include "print.h"
#include <stdbool.h>
#include <stdio.h>

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

  MX_I2C1_Init();
  if (!Adafruit_MSA311_begin(&hi2c1)) {
    printString("[ERROR: MSA311 not responding]\n");
  }
  enable_interrupt_for_collision();

}

void I2C_scanning() {
  int i;
  uint8_t Buffer[25] = {0};
  HAL_StatusTypeDef ret;

  /*-[ I2C Bus Scanning ]-*/
  printString("[Starting I2C Scanning: \r\n");
  for(i=1; i<128; i++)
  {
    ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5);
    if (ret != HAL_OK) /* No ACK Received At That Address */
    {
      printString(" - ");
    }
    else if(ret == HAL_OK)
    {
      sprintf((char *)Buffer, "0x%X", i);
      printString((const char*)Buffer);
    }
  }
  printString("Done!]\r\n\r\n");
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
