#include <stdint.h>
#include "cpu_map.h"
#include "main.h"
#include "nuts_bolts.h"

uint32_t SYSCLK_FREQ;
//uint32_t HCLK_FREQ;
uint32_t STEPPER_TIMER_TICK_FREQ;
uint32_t TICKS_PER_MICROSECOND; // "Stepper" timer tick frequency


/**
 * @brief Get the rcc clock config object
 *        WARNING: This function should be called prior to most of other functions
 */
void get_rcc_clock_config() {
  SYSCLK_FREQ = HAL_RCC_GetSysClockFreq(); // e.g. 72MHz
  //HCLK_FREQ = HAL_RCC_GetHCLKFreq();
  STEPPER_TIMER_TICK_FREQ = HAL_RCC_GetPCLK2Freq(); // e.g. 72MHz
  TICKS_PER_MICROSECOND = (STEPPER_TIMER_TICK_FREQ/1000000); // "Stepper" timer tick frequency
}