
#include "sensors.h"
#include "flux_machine.h"
#include "main.h"
#include <stdbool.h>

uint32_t adc_Buffer[1];

/**
 * @brief Initialize DOOR_DETECT, BASE_DETECT pins
 * 
 */
void sensors_init() {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /*Configure GPIO pins : BASE_DETECT_Pin*/
  GPIO_InitStruct.Pin = BASE_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DOOR_DETECT_Pin */
  GPIO_InitStruct.Pin = DOOR_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DOOR_DETECT_GPIO_Port, &GPIO_InitStruct);

  MX_ADC1_Init();
  HAL_ADC_Start_DMA(&hadc1, adc_Buffer, sizeof(adc_Buffer)/sizeof(adc_Buffer[0]));
}

bool door_is_open() {
  return (HAL_GPIO_ReadPin(DOOR_DETECT_GPIO_Port, DOOR_DETECT_Pin) != 0);
}

bool base_is_open() {
  return (HAL_GPIO_ReadPin(BASE_DETECT_GPIO_Port, BASE_DETECT_Pin) != 0);
}
