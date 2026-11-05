/*
 * lowpower.h —— STM32 低功耗（STOP 模式）对外接口
 */
#ifndef __LOWPOWER_H
#define __LOWPOWER_H

#include <stdint.h>

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_rtc.h"

/*
 * 深度睡眠（STOP 模式）总开关 —— 默认 **关**
 * ---------------------------------------------------------------------------
 * 为什么默认关：STOP 模式整套代码（LSE+RTC 闹钟+EXTI+时钟恢复+tick 补偿）
 * 实测约 **2.1KB flash**，而 40K 分区在做完外设级省电后只剩 ~400 字节，
 * 硬编进去会直接链接失败。
 *
 * 想打开的话，先解决这两个前提：
 *
 *   ① flash 预算（二选一）
 *      · 把分区从 24K/40K 改成 **20K/44K**（bootloader 实占 17.2KB，够）：
 *        改 bootloaer/users/bootloader.h 的 bl_size、
 *        bootloaer/STM32F103C8TX_FLASH.ld 的 LENGTH、
 *        本工程的 .ld 的 ORIGIN/LENGTH、system_stm32f1xx.c 的 VECT_TAB_OFFSET，
 *        然后 **重刷 bootloader**（上位机写入地址也要跟着改）。
 *      · 或者把 STM32F103**C8**T6 换成 **CBT6**（128KB，同封装同引脚，PCB 不用改）。
 *
 *   ② 采样节奏
 *      F1 的 RTC 闹钟只有 **1 秒** 粒度（本工程 HAL 里没有唤醒定时器 API），
 *      而现在 imu_task 是 50ms、oled_task 是 40ms。直接进 STOP 会让
 *      FreeRTOS 的 tick 补偿误差过大、跌落检测也会漏掉。
 *      需要把周期采样改成 1Hz，并把跌落检测改成 **MPU 的自由落体中断唤醒**
 *      （INT_ENABLE 的 FALL 位），而不是 50ms 轮询。
 *
 * 打开方式：把下面的 LP_ENABLE_STOP 改成 1。
 */
#define LP_ENABLE_STOP  0

/* RTC 句柄：中断处理要用 */
extern RTC_HandleTypeDef hrtc;

/* 唤醒原因 */
#define LP_WAKE_NONE    0U   /* 不是被叫醒的（主动返回） */
#define LP_WAKE_ALARM   1U   /* RTC 闹钟（1 秒周期） */
#define LP_WAKE_EXTI    2U   /* EXTI：MPU INT / nRF24 IRQ */
#define LP_WAKE_UART    3U   /* USART1 收到字节（OTA / 调试） */

/* 一个 RTC 闹钟周期 = 1 秒，tick 补偿按这个值补 */
#define LP_CYCLE_MS     1000U

/** 初始化：EXTI 唤醒源（PC0 = MPU INT，PB1 = nRF24 IRQ）。
 *  LP_ENABLE_STOP=1 时还会顺带初始化 LSE + RTC + 闹钟。 */
void lp_init(void);

/** 只配两个 EXTI 唤醒源（不碰 RTC） */
void lp_init_exti(void);

/** 有没有 EXTI 事件挂着（MPU INT / nRF24 IRQ） */
uint8_t lp_exti_pending(void);
void     lp_exti_clear(void);

/** 进入 STOP 模式睡到下一次 RTC 闹钟（或被 EXTI/USART 叫醒）。
 *  返回后已经把系统时钟恢复到 72MHz，并把 STOP 期间丢失的 tick 补齐。 */
void lp_enter(void);

/** 上一次是被什么叫醒的 */
uint8_t lp_wake_reason(void);

/** 禁止/允许进入 STOP（OTA 刷机、射频收发期间必须禁止） */
void lp_block(void);
void lp_unblock(void);
uint8_t lp_can_sleep(void);

/** 统计：睡了多少次 / 各原因被叫醒多少次 */
uint32_t lp_sleep_count(void);
uint32_t lp_wake_count(uint8_t reason);

#endif /* __LOWPOWER_H */
