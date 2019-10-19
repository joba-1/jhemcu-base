/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f7xx_hal.h"

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
#define TARGET "JHEF7DUAL"
#define ID "JH7D"
#define GYRO_1_SPI_INSTANCE 1
#define GYRO_2_SPI_INSTANCE 1
#define BARO_I2C_INSTANCE 1
#define MAX7456_SPI_INSTANCE 2
#define MAX7456_SPI_CS_PIN "SPI2_NSS_PIN"
#define FLASH_SPI_INSTANCE 3
#define CURRENT_METER_SCALE_DEFAULT 450
#define FLASH_CS_Pin GPIO_PIN_13
#define FLASH_CS_GPIO_Port GPIOC
#define PINIO1_Pin GPIO_PIN_14
#define PINIO1_GPIO_Port GPIOC
#define BEEPER_Pin GPIO_PIN_15
#define BEEPER_GPIO_Port GPIOC
#define RSSI_ADC_Pin GPIO_PIN_0
#define RSSI_ADC_GPIO_Port GPIOC
#define CURRENT_METER_ADC_Pin GPIO_PIN_1
#define CURRENT_METER_ADC_GPIO_Port GPIOC
#define VBAT_ADC_Pin GPIO_PIN_2
#define VBAT_ADC_GPIO_Port GPIOC
#define GYRO_2_EXTI_Pin GPIO_PIN_3
#define GYRO_2_EXTI_GPIO_Port GPIOC
#define GYRO_2_EXTI_EXTI_IRQn EXTI3_IRQn
#define RECV_TX_Pin GPIO_PIN_2
#define RECV_TX_GPIO_Port GPIOA
#define RECV_RX_Pin GPIO_PIN_3
#define RECV_RX_GPIO_Port GPIOA
#define GYRO_2_CS_Pin GPIO_PIN_4
#define GYRO_2_CS_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define GYRO_1_EXTI_Pin GPIO_PIN_4
#define GYRO_1_EXTI_GPIO_Port GPIOC
#define GYRO_1_EXTI_EXTI_IRQn EXTI4_IRQn
#define GYRO_1_CS_Pin GPIO_PIN_2
#define GYRO_1_CS_GPIO_Port GPIOB
#define VTX_TX_Pin GPIO_PIN_10
#define VTX_TX_GPIO_Port GPIOB
#define VTX_RX_Pin GPIO_PIN_11
#define VTX_RX_GPIO_Port GPIOB
#define ESC_TX_Pin GPIO_PIN_6
#define ESC_TX_GPIO_Port GPIOC
#define ESC_RX_Pin GPIO_PIN_7
#define ESC_RX_GPIO_Port GPIOC
#define SDA_Pin GPIO_PIN_9
#define SDA_GPIO_Port GPIOC
#define SCL_Pin GPIO_PIN_8
#define SCL_GPIO_Port GPIOA
#define GPS_TX_Pin GPIO_PIN_9
#define GPS_TX_GPIO_Port GPIOA
#define GPS_RX_Pin GPIO_PIN_10
#define GPS_RX_GPIO_Port GPIOA
#define LED0_Pin GPIO_PIN_15
#define LED0_GPIO_Port GPIOA
#define SPI3_SCK_Pin GPIO_PIN_10
#define SPI3_SCK_GPIO_Port GPIOC
#define SPI3_MISO_Pin GPIO_PIN_11
#define SPI3_MISO_GPIO_Port GPIOC
#define SPI3_MOSI_Pin GPIO_PIN_5
#define SPI3_MOSI_GPIO_Port GPIOB
#define PINIO2_Pin GPIO_PIN_9
#define PINIO2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
