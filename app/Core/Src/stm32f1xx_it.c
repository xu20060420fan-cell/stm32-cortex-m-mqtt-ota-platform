/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
#include "debug_log.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* 低功耗模块：RTC 句柄 hrtc 在这里给中断用 */
#include "lowpower.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_spi2_tx;
extern DMA_HandleTypeDef hdma_usart1_tx;   /* 日志串口 DMA */
extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim4;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/* 统一的异常报告：把故障状态寄存器和出错地址打出来，然后停机。
 * 原来这些处理器都是静默 while(1)，出问题只能看到"卡死"，什么都查不到。 */
static void fault_report(const char *name)
{
  dbg_fatal_puts("\r\n[FATAL] ");
  dbg_fatal_puts(name);
  dbg_fatal_puts("\r\n");
  dbg_fatal_hex("  CFSR =", SCB->CFSR);
  dbg_fatal_hex("  HFSR =", SCB->HFSR);
  dbg_fatal_hex("  BFAR =", SCB->BFAR);
  dbg_fatal_hex("  MMFAR=", SCB->MMFAR);
  dbg_fatal_puts("  CFSR 常见位: 1=DACCVIOL(非法写) 9=PRECISERR(访问非法地址)\r\n");
  dbg_fatal_puts("              10=IMPRECISERR 12=STKERR(压栈失败) 16=UNDEFINSTR\r\n");
  dbg_fatal_puts("              17=INVSTATE 24=UNALIGNED 25=DIVBYZERO\r\n");
  /* 【调用链扫描】栈上残留的返回地址都是代码地址（app 区 0x08006000~0x0800FFFF，
   * bootloader 区 0x08000000~0x08005FFF）。把它们按顺序打出来，就是出事时的调用链。
   * 嵌套 fault 会把标准异常帧冲掉，扫栈比读 frame 更可靠。 */
  {
    uint32_t *sp = (uint32_t *)__get_MSP();
    uint32_t  shown = 0U;

    dbg_fatal_puts("  --- stack code addresses (call chain) ---\r\n");
    for (uint32_t i = 0U; i < 80U; i++) {
      uint32_t v = sp[i];

      if (((v >= 0x08000000UL) && (v < 0x08010000UL)) || ((v & 1U) != 0U)) {
        if ((v >= 0x08000000UL) && (v < 0x08010000UL)) {
          dbg_fatal_hex("    [", v);
          shown++;
        }
      }
      if (shown > 14U) {
        break;
      }
    }
  }
  /* 【为什么不再永久停机】原来这里是 for(;;) "停机等调试器"。
   * 现场没接调试器时，这一停就等于板子永久死了 —— 而且现场最常见的情形
   * 恰恰是没接调试器。现在：诊断信息已经全部发完，空转约 3 秒
   *（接了调试器的人这段时间足够 halt 在这里细看），然后自复位。
   * 重启后如果故障还在，这一整段还会再打一遍，反复打就能看出是稳定复现
   * 还是偶发。 */
  for (volatile uint32_t i = 0U; i < 6000000UL; i++) { }
  NVIC_SystemReset();
  for ( ; ; )
  {
    /* 复位没成功才会停在这里 */
  }
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  fault_report("HARD FAULT");
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  fault_report("MEM MANAGE FAULT");
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  fault_report("BUS FAULT");
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  fault_report("USAGE FAULT");
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.  (USART1_TX 日志)
  */
void DMA1_Channel4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

/**
  * @brief This function handles USART1 global interrupt.
  *        日志用 DMA 发送，完成事件要靠这个中断收尾。
  */
void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
}

/**
  * @brief This function handles I2C2 event interrupt.
  * @note 屏幕已改为阻塞式 I2C 发送（见 ssd1306.c 的 oled_write），
  *       不再使用 I2C 中断。这里保留空实现只是为了让中断向量表有落点，
  *       同时避免链接器把 HAL_I2C_EV_IRQHandler（2224 字节）拖进固件。
  */
void I2C2_EV_IRQHandler(void)
{
  /* 故意留空：I2C2 走阻塞式，这里永远不会被触发 */
}

/**
  * @brief This function handles I2C2 error interrupt.
  */
void I2C2_ER_IRQHandler(void)
{
  /* 故意留空：见上 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/* ==========================================================================
 * 低功耗唤醒源（见 user/Task/lowpower.c）
 * ========================================================================== */

/**
  * @brief EXTI15 —— PA15 = MPU6050 的 INT 脚
  * @note  跌落/碰撞/数据就绪都靠它把 MCU 从 STOP 里叫醒
  *        （EXTI15 落在 EXTI15_10 这个共享向量里）
  */
void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(MPU_INT_Pin);
}

/**
  * @brief EXTI1 —— PB1 = nRF24L01 的 IRQ 脚
  * @note  收到下行命令（自检/复位/OTA/配置）时把 MCU 叫醒
  */
void EXTI1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

/**
  * @brief RTC 闹钟中断
  * @note  1 秒周期，负责「定时采样 + 定时上报」的节拍基准
  *        只在 STOP 模式打开时才编译 —— 否则会把整个 HAL RTC 驱动拖进固件。
  */
#if LP_ENABLE_STOP
void RTC_Alarm_IRQHandler(void)
{
  HAL_RTC_AlarmIRQHandler(&hrtc);
}
#endif

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
