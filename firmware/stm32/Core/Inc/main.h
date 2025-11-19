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
#include "stm32u0xx_hal.h"

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
#define AN1_Pin GPIO_PIN_0
#define AN1_GPIO_Port GPIOC
#define AN2_Pin GPIO_PIN_1
#define AN2_GPIO_Port GPIOC
#define BAT_CTRL1_Pin GPIO_PIN_2
#define BAT_CTRL1_GPIO_Port GPIOC
#define BAT_CTRL2_Pin GPIO_PIN_3
#define BAT_CTRL2_GPIO_Port GPIOC
#define DBG_TX_Pin GPIO_PIN_0
#define DBG_TX_GPIO_Port GPIOA
#define DGB_RX_Pin GPIO_PIN_1
#define DGB_RX_GPIO_Port GPIOA
#define USB_ON_Pin GPIO_PIN_2
#define USB_ON_GPIO_Port GPIOA
#define USB_ON_EXTI_IRQn EXTI2_3_IRQn
#define STAT1_Pin GPIO_PIN_3
#define STAT1_GPIO_Port GPIOA
#define STAT2_Pin GPIO_PIN_4
#define STAT2_GPIO_Port GPIOA
#define EPD_SCK_Pin GPIO_PIN_5
#define EPD_SCK_GPIO_Port GPIOA
#define EPD_CS_Pin GPIO_PIN_6
#define EPD_CS_GPIO_Port GPIOA
#define EPD_DIN_Pin GPIO_PIN_7
#define EPD_DIN_GPIO_Port GPIOA
#define EPD_DC_Pin GPIO_PIN_4
#define EPD_DC_GPIO_Port GPIOC
#define EPD_RST_Pin GPIO_PIN_5
#define EPD_RST_GPIO_Port GPIOC
#define EPD_BUSY_Pin GPIO_PIN_0
#define EPD_BUSY_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_10
#define LED_GPIO_Port GPIOB
#define BUTTON_Pin GPIO_PIN_11
#define BUTTON_GPIO_Port GPIOB
#define BUTTON_EXTI_IRQn EXTI4_15_IRQn
#define RF_SCK_Pin GPIO_PIN_13
#define RF_SCK_GPIO_Port GPIOB
#define RF_MISO_Pin GPIO_PIN_14
#define RF_MISO_GPIO_Port GPIOB
#define RF_MOSI_Pin GPIO_PIN_15
#define RF_MOSI_GPIO_Port GPIOB
#define RF_GPIO1_Pin GPIO_PIN_6
#define RF_GPIO1_GPIO_Port GPIOC
#define RF_GPIO2_Pin GPIO_PIN_7
#define RF_GPIO2_GPIO_Port GPIOC
#define RF_GPIO0_Pin GPIO_PIN_8
#define RF_GPIO0_GPIO_Port GPIOC
#define RF_RST_Pin GPIO_PIN_9
#define RF_RST_GPIO_Port GPIOC
#define RF_CS_Pin GPIO_PIN_8
#define RF_CS_GPIO_Port GPIOA
#define RF_TX_Pin GPIO_PIN_9
#define RF_TX_GPIO_Port GPIOA
#define RF_RX_Pin GPIO_PIN_10
#define RF_RX_GPIO_Port GPIOA
#define PS1_Pin GPIO_PIN_10
#define PS1_GPIO_Port GPIOC
#define PS2_Pin GPIO_PIN_11
#define PS2_GPIO_Port GPIOC
#define PS3_Pin GPIO_PIN_12
#define PS3_GPIO_Port GPIOC
#define PS4_Pin GPIO_PIN_2
#define PS4_GPIO_Port GPIOD
#define GPO_Pin GPIO_PIN_4
#define GPO_GPIO_Port GPIOB
#define GPO_EXTI_IRQn EXTI4_15_IRQn
#define SCL_Pin GPIO_PIN_6
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOB
#define PULL_CNTR_Pin GPIO_PIN_8
#define PULL_CNTR_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
