
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

  MX_ADC1_Init();
  HAL_ADC_Start_DMA(&hadc1, adc_Buffer, sizeof(adc_Buffer)/sizeof(adc_Buffer[0]));
}

