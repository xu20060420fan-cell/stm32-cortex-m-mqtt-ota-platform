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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* 外设句柄统一在这里导出，业务层（user/Task）直接 extern 即可，
 * 不用每个模块各写一遍 extern，也方便 .ioc 重新生成代码时保持一致 */
extern SPI_HandleTypeDef hspi1;    /* W25Q128 外置 Flash（与 bootloader 同引脚） */
extern SPI_HandleTypeDef hspi2;    /* NRF24L01+ 无线模块 */
extern UART_HandleTypeDef huart2;  /* ESP8266 下载器串口 */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* 按键：KEY1=PA1、KEY2=PA12（原 PA3 让给 USART2_RX）、KEY3=PB4，均为上拉输入 */
#define KEY1_Pin GPIO_PIN_1
#define KEY1_GPIO_Port GPIOA
#define KEY2_Pin GPIO_PIN_12
#define KEY2_GPIO_Port GPIOA
/* PB4 原来是 SPI 屏的片选 CS，屏已不用，改接第三个按键（确定键） */
#define KEY3_Pin GPIO_PIN_4
#define KEY3_GPIO_Port GPIOB
/* LED：LED1=PA8（原 PA2 让给 USART2_TX）、LED2=PB0 */
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_0
#define LED2_GPIO_Port GPIOB
/* W25Q128 外置 Flash 片选：与 bootloader 的 SPI1 引脚完全一致（PA4），
 * W25Q 的 CS# 是低有效，空闲必须保持高电平 */
#define W25Q_CS_Pin GPIO_PIN_4
#define W25Q_CS_GPIO_Port GPIOA
/* NRF24L01+ 控制脚：CE / CSN 推挽输出（初值低 = 未选中/未发射），IRQ 上拉输入 */
#define NRF24_CE_Pin GPIO_PIN_9
#define NRF24_CE_GPIO_Port GPIOB
#define NRF24_CSN_Pin GPIO_PIN_12
#define NRF24_CSN_GPIO_Port GPIOB
#define NRF24_IRQ_Pin GPIO_PIN_1
#define NRF24_IRQ_GPIO_Port GPIOB
/* 蜂鸣器：复用原 LCD 背光 PB8，低电平触发，所以空闲态写高（不响） */
#define BUZZER_Pin GPIO_PIN_8
#define BUZZER_GPIO_Port GPIOB
/* 已下线的 TFT 屏 RES/DC：引脚保留占位（免得重排 Mcu.PinN），改成普通 GPIO 输出 */
#define NC_RES_Pin GPIO_PIN_3
#define NC_RES_GPIO_Port GPIOB
#define NC_DC_Pin GPIO_PIN_5
#define NC_DC_GPIO_Port GPIOB
/* MPU6050 的 INT 脚 → PA15（EXTI15）：跌落/碰撞中断，也是低功耗唤醒源
 *
 * ⚠ 为什么是 PA15 而不是 PC0：
 *   这块 STM32 核心板（2×20P）的排针上 **PC0 / PC1 / PC2 / PC3 / PC4 / PC5–PC12
 *   一个都没引出来**（只有 PC13/PC14/PC15 是晶振脚）。可用 IO 只有 29 个：
 *     PA0–PA12、PA15（14 个） + PB0、PB1、PB3–PB15（15 个）
 *   固件已占用 26 个（PA1–PA10、PA12 / PB0、PB1、PB3–PB15），
 *   空闲的只剩 **PA0 / PA11 / PA15** 三个：
 *     · PA0  → 电池分压（ADC12_IN0）
 *     · PA11 → 保留（USART0 CTS/DTR）
 *     · PA15 → 拿来做 MPU INT（EXTI15，原本是 SPI3_NSS，现在 SPI3 已不用）
 */
#define MPU_INT_Pin GPIO_PIN_15
#define MPU_INT_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
