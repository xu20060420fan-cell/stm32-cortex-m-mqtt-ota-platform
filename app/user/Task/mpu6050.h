/*
 * mpu6050.h  ——  MPU6050 / MPU6500 / MPU9250 / MPU9255 六轴传感器驱动（I2C1）
 *
 *  Created on: Oct 24, 2026
 *      Author: Administrator
 *
 * 板上丝印是 "MPU-9250/6500/9255" 三选一，所以驱动**不写死型号**：
 * 初始化时先读 WHO_AM_I（0x75）自动识别，再配寄存器。
 *
 * ------------------------------------------------------------------
 * 【接线 —— 新加的接线，和 AHT20 / BMP280 共 I2C1 总线】
 *   I2C1 SCL = PB6
 *   I2C1 SDA = PB7
 *   MPU 的 AD0 接地 -> 7 位地址 0x68
 *   MPU 的 AD0 接 3.3V -> 7 位地址 0x69
 *   （总线上的其它器件：AHT20 = 0x38，BMP280 = 0x76/0x77，都不冲突）
 *
 *   VCC = 3.3V，GND 共地。模块一般自带稳压和上拉。
 *   注意：MPU6050 很多模块把 VCC 接成 5V 也能用，但 I2C 上拉有时是 5V 的，
 *         STM32 的 PB6/PB7 是 5V 容忍的，一般没事。
 * ------------------------------------------------------------------
 */
#ifndef TASK_MPU6050_H_
#define TASK_MPU6050_H_

#include "stm32f1xx_hal.h"

/* ===================== WHO_AM_I 识别码 ===================== */
#define MPU_ID_MPU6050   0x68U   /* MPU6050 */
#define MPU_ID_MPU6500   0x70U   /* MPU6500 */
#define MPU_ID_MPU9250   0x71U   /* MPU9250（部分批次读回 0x73） */
#define MPU_ID_MPU9255   0x73U   /* MPU9255 */

/* ===================== I2C 7 位地址（AD0 接地 / 接 VDD） ===================== */
#define MPU6050_ADDR_AD0_LOW   0x68U
#define MPU6050_ADDR_AD0_HIGH  0x69U

/* ===================== 错误码（mpu6050_last_error() 返回） ===================== */
#define MPU6050_ERR_NONE        0U   /* 正常 */
#define MPU6050_ERR_NO_ACK      1U   /* 0x68/0x69 都没人应答（没接/断线/上电没上） */
#define MPU6050_ERR_UNKNOWN_ID  2U   /* 有应答但 WHO_AM_I 不认识 */
#define MPU6050_ERR_CONFIG      3U   /* 写寄存器失败 */
#define MPU6050_ERR_VERIFY      4U   /* 写完读回 WHO_AM_I 不一致 */

/* ===================== 对外 API ===================== */

/**
 * @brief  初始化：探测 I2C 地址 -> 读 WHO_AM_I 认型号 -> 配寄存器 -> 回读校验
 * @retval 识别到的型号（0x68/0x70/0x71/0x73）  0:失败
 * @note   失败**不重试**，直接返回，由调用方决定怎么办；
 *         失败原因用 mpu6050_last_error() 取。
 *         必须在 FreeRTOS 任务上下文调用（里面用了 vTaskDelay 等器件起振）。
 */
uint8_t mpu6050_init(void);

/**
 * @brief  读一帧原始数据（加速度 + 角速度）
 * @param  ax: 加速度 X 原始值（可传 NULL 表示不要）
 * @param  ay: 加速度 Y 原始值（可传 NULL）
 * @param  az: 加速度 Z 原始值（可传 NULL）
 * @param  gx: 角速度 X 原始值（可传 NULL）
 * @param  gy: 角速度 Y 原始值（可传 NULL）
 * @param  gz: 角速度 Z 原始值（可传 NULL）
 * @retval 1:成功  0:失败（没初始化 / I2C 出错）
 */
uint8_t mpu6050_read(int16_t *ax, int16_t *ay, int16_t *az,
                     int16_t *gx, int16_t *gy, int16_t *gz);

/**
 * @brief  读加速度并按 ±2g 量程换算成 g
 * @param  ax: X 轴加速度 (g)  @param  ay: Y 轴 (g)  @param  az: Z 轴 (g)
 * @retval 1:成功  0:失败
 */
uint8_t mpu6050_read_accel_gravity(float *ax, float *ay, float *az);

/**
 * @brief  读角速度并按 ±250°/s 量程换算成 dps
 * @param  gx: X 轴角速度 (dps)  @param  gy: Y 轴 (dps)  @param  gz: Z 轴 (dps)
 * @retval 1:成功  0:失败
 */
uint8_t mpu6050_read_gyro_dps(float *gx, float *gy, float *gz);

/**
 * @brief  型号名字（"MPU6050" / "MPU6500" / "MPU9250" / "MPU9255" / "UNKNOWN"）
 */
const char *mpu6050_model_name(uint8_t id);

/**
 * @brief  上次初始化/读操作的错误码（MPU6050_ERR_*）
 */
uint8_t mpu6050_last_error(void);

/**
 * @brief  初始化是否成功（可以正常读数据了）
 * @retval 1:可以读  0:还不能
 */
uint8_t mpu6050_is_ready(void);

/**
 * @brief  开 INT 脚中断（数据就绪）
 * @note   INT 接到 PC0 → EXTI0，STOP 模式下靠它把 MCU 叫醒做跌落判定。
 * @retval 1:成功  0:失败
 */
uint8_t mpu6050_enable_int(void);

/**
 * @brief  进低功耗（只保留加速度计，陀螺仪停）
 * @note   ~60µA（正常模式 ~3.5mA）。进睡后 INT 不再产生数据就绪中断。
 * @retval 1:成功  0:失败
 */
uint8_t mpu6050_sleep(void);

/**
 * @brief  退出低功耗，恢复陀螺仪时钟
 * @retval 1:成功  0:失败
 */
uint8_t mpu6050_wake(void);

#endif /* TASK_MPU6050_H_ */
