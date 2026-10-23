/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M1_DIR2_Pin GPIO_PIN_2
#define M1_DIR2_GPIO_Port GPIOE
#define M2_DIR1_Pin GPIO_PIN_3
#define M2_DIR1_GPIO_Port GPIOE
#define M2_DIR2_Pin GPIO_PIN_4
#define M2_DIR2_GPIO_Port GPIOE
#define M1_PWM_Pin GPIO_PIN_5
#define M1_PWM_GPIO_Port GPIOE
#define M2_PWM_Pin GPIO_PIN_6
#define M2_PWM_GPIO_Port GPIOE
#define I_ADC_Pin GPIO_PIN_3
#define I_ADC_GPIO_Port GPIOF
#define U_ADC_Pin GPIO_PIN_4
#define U_ADC_GPIO_Port GPIOF
#define ARMS1_PWM_Pin GPIO_PIN_6
#define ARMS1_PWM_GPIO_Port GPIOF
#define ARMS2_PWM_Pin GPIO_PIN_7
#define ARMS2_PWM_GPIO_Port GPIOF
#define ARMS3_PWM_Pin GPIO_PIN_8
#define ARMS3_PWM_GPIO_Port GPIOF
#define ARMS4_PWM_Pin GPIO_PIN_9
#define ARMS4_PWM_GPIO_Port GPIOF
#define M1_ENC_A_Pin GPIO_PIN_0
#define M1_ENC_A_GPIO_Port GPIOA
#define M1_ENC_B_Pin GPIO_PIN_1
#define M1_ENC_B_GPIO_Port GPIOA
#define BLUETOOTH_TX_Pin GPIO_PIN_2
#define BLUETOOTH_TX_GPIO_Port GPIOA
#define BLUETOOTH_RX_Pin GPIO_PIN_3
#define BLUETOOTH_RX_GPIO_Port GPIOA
#define M2_ENC_A_Pin GPIO_PIN_9
#define M2_ENC_A_GPIO_Port GPIOE
#define M2_ENC_B_Pin GPIO_PIN_11
#define M2_ENC_B_GPIO_Port GPIOE
#define ARME_DIR_Pin GPIO_PIN_12
#define ARME_DIR_GPIO_Port GPIOE
#define ARME_STEP_Pin GPIO_PIN_13
#define ARME_STEP_GPIO_Port GPIOE
#define ARMS5_PWM_Pin GPIO_PIN_15
#define ARMS5_PWM_GPIO_Port GPIOB
#define M3_DIR1_Pin GPIO_PIN_3
#define M3_DIR1_GPIO_Port GPIOG
#define M3_DIR2_Pin GPIO_PIN_4
#define M3_DIR2_GPIO_Port GPIOG
#define M4_DIR1_Pin GPIO_PIN_5
#define M4_DIR1_GPIO_Port GPIOG
#define M4_DIR2_Pin GPIO_PIN_6
#define M4_DIR2_GPIO_Port GPIOG
#define ARME_PWM_Pin GPIO_PIN_7
#define ARME_PWM_GPIO_Port GPIOC
#define M3_PWM_Pin GPIO_PIN_8
#define M3_PWM_GPIO_Port GPIOC
#define M4_PWM_Pin GPIO_PIN_9
#define M4_PWM_GPIO_Port GPIOC
#define M4_ENC_A_Pin GPIO_PIN_4
#define M4_ENC_A_GPIO_Port GPIOB
#define M4_ENC_B_Pin GPIO_PIN_5
#define M4_ENC_B_GPIO_Port GPIOB
#define M3_ENC_A_Pin GPIO_PIN_6
#define M3_ENC_A_GPIO_Port GPIOB
#define M3_ENC_B_Pin GPIO_PIN_7
#define M3_ENC_B_GPIO_Port GPIOB
#define M1_DIR1_Pin GPIO_PIN_1
#define M1_DIR1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
