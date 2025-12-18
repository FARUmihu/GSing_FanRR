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
#include "stm32h7xx_hal.h"

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

/* USER CODE BEGIN Private defines */
//控制DX,RX的引脚宏定义
#define RS485_GPIO_Port GPIOD
#define RS485_Pin       GPIO_PIN_4

// 定义 RS485 收发控制宏
#define RS485_RxMode()    HAL_GPIO_WritePin(RS485_GPIO_Port, RS485_Pin, GPIO_PIN_RESET)
#define RS485_TxMode()    HAL_GPIO_WritePin(RS485_GPIO_Port, RS485_Pin, GPIO_PIN_SET)

//发送完毕的回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

//接收完毕的回调函数。准备写一个将接收的数据进行归类排布的函数，转存完就清空全局缓存区，11.30号，暂无此函数。12.3,make it over
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

//现在正在写CAN线的&hfdcan1读取的回调函数，如果想要实现fdcan2、3的，仿照进行补写就行
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
