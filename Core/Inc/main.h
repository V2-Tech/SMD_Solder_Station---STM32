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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PulsStart_Pin GPIO_PIN_13
#define PulsStart_GPIO_Port GPIOC
#define PulsStart_EXTI_IRQn EXTI15_10_IRQn
#define PulsStop_Pin GPIO_PIN_15
#define PulsStop_GPIO_Port GPIOC
#define PulsStop_EXTI_IRQn EXTI15_10_IRQn
#define HEATER_PWM_Pin GPIO_PIN_1
#define HEATER_PWM_GPIO_Port GPIOA
#define MAX6675_NSS_Pin GPIO_PIN_4
#define MAX6675_NSS_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOB
#define ENC_DT_Pin GPIO_PIN_4
#define ENC_DT_GPIO_Port GPIOB
#define ENC_CLK_Pin GPIO_PIN_5
#define ENC_CLK_GPIO_Port GPIOB
#define PulsEncoder_Pin GPIO_PIN_7
#define PulsEncoder_GPIO_Port GPIOB
#define PulsEncoder_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */
extern _Bool BlinkVar;
extern _Bool AlarmVar;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
