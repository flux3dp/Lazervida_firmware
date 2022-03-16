
#include "flux_machine.h"
#include "main.h"
#include "sensors.h"
#include "stepper.h"
#include "system.h"

extern system_t sys;

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

void set_stepper_power() {
  HAL_GPIO_WritePin(MOTOR_PWR_GPIO_Port, MOTOR_PWR_Pin, GPIO_PIN_SET);
}

void reset_stepper_power() {
  HAL_GPIO_WritePin(MOTOR_PWR_GPIO_Port, MOTOR_PWR_Pin, GPIO_PIN_RESET);
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
