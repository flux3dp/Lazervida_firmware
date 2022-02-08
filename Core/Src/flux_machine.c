
#include "flux_machine.h"
#include "main.h"
#include "sensors.h"
#include "stepper.h"
#include "system.h"

extern system_t sys;

void flux_periodic_handling() {
  /*
  if (door_is_open()) {
    system_set_exec_state_flag(EXEC_SAFETY_DOOR);
  }
  if (base_is_open()) {
    system_set_exec_state_flag(EXEC_SAFETY_DOOR);
  }
  */
}

void set_stepper_MS1() {
  HAL_GPIO_WritePin(MS1_GPIO_Port, MS1_Pin, GPIO_PIN_SET);
}

void reset_stepper_MS1() {
  HAL_GPIO_WritePin(MS1_GPIO_Port, MS1_Pin, GPIO_PIN_RESET);
}

void set_stepper_power() {
  HAL_GPIO_WritePin(MOTOR_PWR_GPIO_Port, MOTOR_PWR_Pin, GPIO_PIN_SET);
}

void reset_stepper_power() {
  HAL_GPIO_WritePin(MOTOR_PWR_GPIO_Port, MOTOR_PWR_Pin, GPIO_PIN_RESET);
}
