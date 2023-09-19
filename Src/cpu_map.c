#include <stdint.h>
#include "cpu_map.h"
#include "main.h"
#include "nuts_bolts.h"

uint16_t mock_spindle_direction_port;

// NOTE: XXX_PORT = gpio output register in stm32
//       XXX_DDR = gpio config register in stm32

// initialized in step_pin_mask_init()
GPIO_PIN_TYPE STEP_MASK;
GPIO_PIN_TYPE STEP_PIN[N_AXIS];
GPIO_BIT_TYPE STEP_BIT[N_AXIS]; 

// initialized in dir_pin_mask_init()
GPIO_PIN_TYPE DIRECTION_MASK;
GPIO_PIN_TYPE DIRECTION_PIN[N_AXIS];
GPIO_BIT_TYPE DIRECTION_BIT[N_AXIS]; 

// initialized in limit_pin_mask_init()
GPIO_PIN_TYPE LIMIT_MASK;
GPIO_PIN_TYPE LIMIT_PIN[N_AXIS];
GPIO_BIT_TYPE LIMIT_BIT[N_AXIS]; 

// initialized in stepper_disable_mask_init()
GPIO_BIT_TYPE STEPPERS_DISABLE_BIT;
GPIO_PIN_TYPE STEPPERS_DISABLE_PIN;

// initialized in laser_enable_mask_init()
GPIO_BIT_TYPE SPINDLE_ENABLE_BIT;
GPIO_PIN_TYPE SPINDLE_ENABLE_PIN;

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

void step_pin_mask_init() {
  int i = 0;
  int axis = 0;

  STEP_PIN[X_AXIS] = STEP_X_Pin;
  STEP_PIN[Y_AXIS] = STEP_Y_Pin;

  STEP_MASK = 0;
  for (axis = 0; axis < N_AXIS; axis++) {
    STEP_MASK |= STEP_PIN[axis];
    STEP_BIT[axis] = 0;
    for (i = 0; i < 16; i++) {
      if ((STEP_PIN[axis] >> i) & 0x1) {
        break;
      }
      STEP_BIT[axis] += 1;
    }
    // NOTE: if STEP_BIT[x] == 16 -> invalid
  }
}

void dir_pin_mask_init() {
  int i = 0;
  int axis = 0;

  DIRECTION_PIN[X_AXIS] = DIR_X_Pin;
  DIRECTION_PIN[Y_AXIS] = DIR_Y_Pin;

  DIRECTION_MASK = 0;
  for (axis = 0; axis < N_AXIS; axis++) {
    DIRECTION_MASK |= DIRECTION_PIN[axis];
    DIRECTION_BIT[axis] = 0;
    for (i = 0; i < 16; i++) {
      if ((DIRECTION_PIN[axis] >> i) & 0x1) {
        break;
      }
      DIRECTION_BIT[axis] += 1;
    }
    // NOTE: if DIRECTION_BIT[x] == 16 -> invalid
  }
}

void limit_pin_mask_init() {
  int i = 0;
  int axis = 0;

  LIMIT_PIN[X_AXIS] = LIMIT_X_Pin;
  LIMIT_PIN[Y_AXIS] = LIMIT_Y_Pin;

  LIMIT_MASK = 0;
  for (axis = 0; axis < N_AXIS; axis++) {
    LIMIT_MASK |= LIMIT_PIN[axis];
    LIMIT_BIT[axis] = 0;
    for (i = 0; i < 16; i++) {
      if ((LIMIT_PIN[axis] >> i) & 0x1) {
        break;
      }
      LIMIT_BIT[axis] += 1;
    }
    // NOTE: if DIRECTION_BIT[x] == 16 -> invalid
  }
}

uint16_t read_limit_port() {
  return LIMIT_X_GPIO_Port->IDR;
}

void stepper_disable_mask_init() {
  int i;
  STEPPERS_DISABLE_PIN = STEP_EN_INV_Pin;
  STEPPERS_DISABLE_BIT = 0;
  for (i = 0; i < 16; i++) {
    if ((STEPPERS_DISABLE_PIN >> i) & 0x1) {
      break;
    }
    STEPPERS_DISABLE_BIT += 1;
  }
}

void laser_enable_mask_init() {
  int i;
  SPINDLE_ENABLE_PIN = LASER_EN_Pin;
  SPINDLE_ENABLE_BIT = 0;
  for (i = 0; i < 16; i++) {
    if ((SPINDLE_ENABLE_PIN >> i) & 0x1) {
      break;
    }
    SPINDLE_ENABLE_BIT += 1;
  }
}