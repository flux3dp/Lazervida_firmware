/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx.h"
#include "stm32f1xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
uint32_t millis();
void _delay_ms(uint32_t ms);

extern void SystemClock_Config(void);
extern void MX_GPIO_Init(void);
extern void MX_DMA_Init(void);
extern void MX_ADC1_Init(void);
extern void MX_I2C1_Init(void);
extern void MX_USART1_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_TIM3_Init(void);
extern void MX_TIM1_Init(void);
extern void MX_TIM2_Init(void);
extern void MX_TIM4_Init(void);

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
//extern UART_HandleTypeDef huart1;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWR_24V_Pin GPIO_PIN_0
#define PWR_24V_GPIO_Port GPIOA
#define MSA311_INT_Pin GPIO_PIN_1
#define MSA311_INT_GPIO_Port GPIOA
#define MSA311_INT_EXTI_IRQn EXTI1_IRQn
#define LASER_EN_Pin GPIO_PIN_2
#define LASER_EN_GPIO_Port GPIOA
#define LIMIT_Y_Pin GPIO_PIN_5
#define LIMIT_Y_GPIO_Port GPIOA
#define LIMIT_Y_EXTI_IRQn EXTI9_5_IRQn
#define DETECT_3V3_Pin GPIO_PIN_6
#define DETECT_3V3_GPIO_Port GPIOA
#define LIMIT_X_Pin GPIO_PIN_7
#define LIMIT_X_GPIO_Port GPIOA
#define LIMIT_X_EXTI_IRQn EXTI9_5_IRQn
#define LASER_PWM_Pin GPIO_PIN_0
#define LASER_PWM_GPIO_Port GPIOB
#define DIR_Y_Pin GPIO_PIN_1
#define DIR_Y_GPIO_Port GPIOB
#define STEP_Y_Pin GPIO_PIN_2
#define STEP_Y_GPIO_Port GPIOB
#define USB_Detect_Pin GPIO_PIN_10
#define USB_Detect_GPIO_Port GPIOB
#define STEP_EN_INV_Pin GPIO_PIN_8
#define STEP_EN_INV_GPIO_Port GPIOA
#define MS1_Pin GPIO_PIN_15
#define MS1_GPIO_Port GPIOA
#define DIR_X_Pin GPIO_PIN_3
#define DIR_X_GPIO_Port GPIOB
#define STEP_X_Pin GPIO_PIN_4
#define STEP_X_GPIO_Port GPIOB
#define LED_PWM_Pin GPIO_PIN_5
#define LED_PWM_GPIO_Port GPIOB
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
#define POWER_BTN_Pin GPIO_PIN_9
#define POWER_BTN_GPIO_Port GPIOB
#define POWER_BTN_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
