/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern I2C_HandleTypeDef hi2c1;
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
void printUART(const char *message);
void printInt(uint16_t ints);
static uint16_t readADC(int channel);

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define adc_th_Pin GPIO_PIN_0
#define adc_th_GPIO_Port GPIOA
#define adc_br_Pin GPIO_PIN_1
#define adc_br_GPIO_Port GPIOA
#define adc_cl_Pin GPIO_PIN_2
#define adc_cl_GPIO_Port GPIOA
#define gpio_dir_Pin GPIO_PIN_5
#define gpio_dir_GPIO_Port GPIOA
#define gpio_mPhase_Pin GPIO_PIN_6
#define gpio_mPhase_GPIO_Port GPIOA
#define gpio_but1_Pin GPIO_PIN_0
#define gpio_but1_GPIO_Port GPIOB
#define gpio_but2_Pin GPIO_PIN_1
#define gpio_but2_GPIO_Port GPIOB
#define gpio_but3_Pin GPIO_PIN_7
#define gpio_but3_GPIO_Port GPIOA
#define gpio_but4_Pin GPIO_PIN_10
#define gpio_but4_GPIO_Port GPIOB
#define gpio_but5_Pin GPIO_PIN_11
#define gpio_but5_GPIO_Port GPIOB
#define gpio_but6_Pin GPIO_PIN_12
#define gpio_but6_GPIO_Port GPIOB
#define gpio_but7_Pin GPIO_PIN_13
#define gpio_but7_GPIO_Port GPIOB
#define gpio_but8_Pin GPIO_PIN_14
#define gpio_but8_GPIO_Port GPIOB
#define gpio_but9_Pin GPIO_PIN_15
#define gpio_but9_GPIO_Port GPIOB
#define gpio_but10_Pin GPIO_PIN_8
#define gpio_but10_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
