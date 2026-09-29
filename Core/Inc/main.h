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
#include "stm32h5xx_hal.h"

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
#define LTE_PWR_Pin GPIO_PIN_13
#define LTE_PWR_GPIO_Port GPIOC
#define LTE_PWR_Key_Pin GPIO_PIN_14
#define LTE_PWR_Key_GPIO_Port GPIOC
#define GPS_PWR_Pin GPIO_PIN_15
#define GPS_PWR_GPIO_Port GPIOC
#define GPS_TX_Pin GPIO_PIN_2
#define GPS_TX_GPIO_Port GPIOA
#define GPS_RX_Pin GPIO_PIN_3
#define GPS_RX_GPIO_Port GPIOA
#define FLASH_SPI_SCK_Pin GPIO_PIN_5
#define FLASH_SPI_SCK_GPIO_Port GPIOA
#define FLASH_SPI_MISO_Pin GPIO_PIN_6
#define FLASH_SPI_MISO_GPIO_Port GPIOA
#define FLASH_SPI_MOSI_Pin GPIO_PIN_7
#define FLASH_SPI_MOSI_GPIO_Port GPIOA
#define Flash_PWR_Pin GPIO_PIN_4
#define Flash_PWR_GPIO_Port GPIOC
#define LOG_TX_Pin GPIO_PIN_10
#define LOG_TX_GPIO_Port GPIOB
#define LOG_RX_Pin GPIO_PIN_11
#define LOG_RX_GPIO_Port GPIOB
#define LTE_RX_Pin GPIO_PIN_14
#define LTE_RX_GPIO_Port GPIOB
#define LTE_TX_Pin GPIO_PIN_15
#define LTE_TX_GPIO_Port GPIOB
#define EN_DISCHARGE_Pin GPIO_PIN_12
#define EN_DISCHARGE_GPIO_Port GPIOD
#define EN_CHARGE_Pin GPIO_PIN_6
#define EN_CHARGE_GPIO_Port GPIOC
#define GPS_RST_Pin GPIO_PIN_8
#define GPS_RST_GPIO_Port GPIOA
#define RTC_EN_PW_Pin GPIO_PIN_3
#define RTC_EN_PW_GPIO_Port GPIOB
#define IMU_EN_PW_Pin GPIO_PIN_4
#define IMU_EN_PW_GPIO_Port GPIOB
#define I2C1_RESET_Pin GPIO_PIN_5
#define I2C1_RESET_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
