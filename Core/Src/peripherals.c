#include "peripherals.h"
#include "main.h"
#include <stdbool.h>

/**
 * @brief Initialize FAN
 *                   I2C (for RGB-led and G-sensor)
 * 
 */
void peripherals_init() {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  HAL_GPIO_WritePin(GPIOA, FAN_EN_Pin, GPIO_PIN_RESET);
  
  /*Configure GPIO pins : FAN_EN_Pin */
  GPIO_InitStruct.Pin = FAN_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  MX_I2C1_Init();
}

void turn_on_fan() {
  HAL_GPIO_WritePin(FAN_EN_GPIO_Port, FAN_EN_Pin, GPIO_PIN_SET);
}

void turn_off_fan() {
  HAL_GPIO_WritePin(FAN_EN_GPIO_Port, FAN_EN_Pin, GPIO_PIN_RESET);
}

void toggle_fan() {
  HAL_GPIO_TogglePin(FAN_EN_GPIO_Port, FAN_EN_Pin);
}

bool get_fan_state() {
  return (bool)(HAL_GPIO_ReadPin(FAN_EN_GPIO_Port, FAN_EN_Pin));
}