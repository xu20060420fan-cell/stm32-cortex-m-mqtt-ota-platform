/*
 * mpu6050.c  ——  MPU6050 / MPU6500 / MPU9250 / MPU9255 六轴传感器驱动（I2C1）
 *
 *  Created on: Oct 24, 2026
 *      Author: Administrator
 *
 * 板上丝印 "MPU-9250/6500/9255" 三选一，所以靠 WHO_AM_I(0x75) 自动认型号：
 *   0x68 -> MPU6050      0x70 -> MPU6500
 *   0x71 -> MPU9250      0x73 -> MPU9255（也有 MPU9250 批次读回 0x73）
 * 这几个型号的加速度/角速度寄存器（0x3B~0x48）、量程位、时钟源位都是一样的，
 * 所以一套代码通吃，不用按型号分支。
 *
 * 初始化失败**不重试**：直接返回 0，错误原因放在 s_err 里，
 * 由调用方决定是重试还是跳过（不搞死循环，免得占死一个任务）。
 *
 * 延时用 vTaskDelay（让出 CPU），不用 HAL_Delay（会阻塞其它任务）。
 */
#include "mpu6050.h"
#include "debug_log.h"

#include "FreeRTOS.h"
#include "task.h"

extern I2C_HandleTypeDef hi2c1;

/* ===================== 寄存器 ===================== */
#define MPU_REG_SMPLRT_DIV      0x19U   /* 采样率分频 */
#define MPU_REG_CONFIG          0x1AU   /* DLPF 数字低通滤波 */
#define MPU_REG_GYRO_CONFIG     0x1BU   /* 陀螺量程 */
#define MPU_REG_ACCEL_CONFIG    0x1CU   /* 加速度量程 */
#define MPU_REG_ACCEL_XOUT_H    0x3BU   /* 加速度/角速度数据起点，连续 14 字节 */
#define MPU_REG_INT_ENABLE      0x38U   /* 中断使能：bit0=数据就绪 bit4=跌落 */
#define MPU_REG_PWR_MGMT_1      0x6BU   /* 电源/时钟源 */
#define MPU_REG_WHO_AM_I        0x75U   /* 型号 ID */
#define MPU_REG_WHO_AM_I_2      0x7AU   /* 型号 ID 第二字节（MPU9250 系列） */

/* ===================== 参数 ===================== */
#define MPU_I2C_TIMEOUT_MS      50U
#define MPU_POWER_ON_MS         100U    /* 上电到第一次 I2C 访问要 ≥100ms */
#define MPU_RESET_WAIT_MS       10U     /* 写 PWR_MGMT_1 后等器件复位完成 */
#define MPU_REG_WRITE_DELAY_MS  2U      /* 连续写寄存器之间的间隔（保险） */

/* 量程换算（ACCEL_CONFIG=0 -> ±2g，GYRO_CONFIG=0 -> ±250°/s） */
#define MPU_ACCEL_LSB_PER_G     16384.0f
#define MPU_GYRO_LSB_PER_DPS    131.0f

/* ===================== 内部状态 ===================== */
static uint8_t  s_addr8;      /* 实际使用的 8 位格式地址（7 位左移 1 位） */
static uint8_t  s_model;      /* 识别到的 WHO_AM_I */
static uint8_t  s_ready;      /* 1 = 初始化成功 */
static uint8_t  s_err;        /* 上次的错误码 */

/* ===================== 底层 I2C ===================== */

/**
 * @brief  读 WHO_AM_I
 * @param  addr8: 8 位格式 I2C 地址（输出，探测到的那个）
 * @param  id: WHO_AM_I 值（输出）
 * @retval 1:读到  0:I2C 失败
 */
static uint8_t mpu_read_who(uint16_t addr8, uint8_t *id)
{
    if (HAL_I2C_Mem_Read(&hi2c1, addr8, MPU_REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT,
                         id, 1U, MPU_I2C_TIMEOUT_MS) != HAL_OK) {
        return 0U;
    }
    return 1U;
}

/**
 * @brief  写一个寄存器
 * @retval 1:成功  0:失败
 */
static uint8_t mpu_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2];
    HAL_StatusTypeDef ret;

    tx[0] = reg;
    tx[1] = val;

    ret = HAL_I2C_Master_Transmit(&hi2c1, s_addr8, tx, 2U, MPU_I2C_TIMEOUT_MS);
    if (ret != HAL_OK) {
        return 0U;
    }
    vTaskDelay(pdMS_TO_TICKS(MPU_REG_WRITE_DELAY_MS));
    return 1U;
}

/**
 * @brief  把 WHO_AM_I 翻译成型号名
 */
const char *mpu6050_model_name(uint8_t id)
{
    switch (id) {
    case MPU_ID_MPU6050:
        return "MPU6050";
    case MPU_ID_MPU6500:
        return "MPU6500";
    case MPU_ID_MPU9250:
        return "MPU9250";
    case MPU_ID_MPU9255:
        return "MPU9255";
    default:
        return "UNKNOWN";
    }
}

/**
 * @brief  判断一个 WHO_AM_I 值是不是受支持的型号
 */
static uint8_t mpu_id_known(uint8_t id)
{
    return ((id == MPU_ID_MPU6050) || (id == MPU_ID_MPU6500) ||
            (id == MPU_ID_MPU9250) || (id == MPU_ID_MPU9255)) ? 1U : 0U;
}

/* ===================== 对外 API ===================== */

/**
 * @brief  初始化并识别型号
 * @retval 识别到的型号（0x68/0x70/0x71/0x73）  0:失败
 */
uint8_t mpu6050_init(void)
{
    static const uint8_t cand_addr[2] = {
        (uint8_t)(MPU6050_ADDR_AD0_LOW << 1),    /* AD0 接地 */
        (uint8_t)(MPU6050_ADDR_AD0_HIGH << 1)    /* AD0 接 VDD */
    };
    uint8_t i;
    uint8_t id = 0U;
    uint8_t who2 = 0U;

    s_ready = 0U;
    s_model = 0U;
    s_err   = MPU6050_ERR_NONE;

    /* 上电要等一会儿，MPU6050 从上电到能响应 I2C 需要 100ms */
    vTaskDelay(pdMS_TO_TICKS(MPU_POWER_ON_MS));

    /* ---- 1) 探测 I2C 地址 + 读 WHO_AM_I 认型号 ---- */
    for (i = 0U; i < 2U; ++i) {
        if (mpu_read_who(cand_addr[i], &id) == 0U) {
            continue;                       /* 这个地址没人应答，试下一个 */
        }
        if (mpu_id_known(id) == 0U) {
            dbg_printf("[MPU] addr 0x%02X WHO_AM_I=0x%02X (not MPU60/65/9250)\r\n",
                       (unsigned)(cand_addr[i] >> 1), (unsigned)id);
            s_err = MPU6050_ERR_UNKNOWN_ID;
            continue;
        }
        s_addr8 = cand_addr[i];
        s_model = id;
        break;
    }

    if (s_model == 0U) {
        dbg_printf("[MPU] no device on 0x68/0x69 (check wiring / AD0 / power)\r\n");
        s_err = MPU6050_ERR_NO_ACK;
        return 0U;
    }

    dbg_printf("[MPU] found %s at 7bit=0x%02X (WHO_AM_I=0x%02X)\r\n",
               mpu6050_model_name(s_model), (unsigned)(s_addr8 >> 1), (unsigned)s_model);

    /* MPU9250/9255 有第二个 ID 字节，读一下有助于确认是 MPU9250 还是 MPU9255 */
    if ((s_model == MPU_ID_MPU9250) || (s_model == MPU_ID_MPU9255)) {
        if (HAL_I2C_Mem_Read(&hi2c1, s_addr8, MPU_REG_WHO_AM_I_2, I2C_MEMADD_SIZE_8BIT,
                             &who2, 1U, MPU_I2C_TIMEOUT_MS) == HAL_OK) {
            dbg_printf("[MPU] WHO_AM_I_2 = 0x%02X\r\n", (unsigned)who2);
        }
    }

    /* ---- 2) 唤醒：PWR_MGMT_1 = 0x01 -> CLKSEL = PLL with X 轴陀螺 ---- */
    if (mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x01U) == 0U) {
        dbg_printf("[MPU] write PWR_MGMT_1 fail\r\n");
        s_err = MPU6050_ERR_CONFIG;
        return 0U;
    }
    vTaskDelay(pdMS_TO_TICKS(MPU_RESET_WAIT_MS));

    /* ---- 3) CONFIG：数字低通滤波，0x03 -> 陀螺带宽 44Hz / 加速度 44Hz ---- */
    if (mpu_write_reg(MPU_REG_CONFIG, 0x03U) == 0U) {
        s_err = MPU6050_ERR_CONFIG;
        return 0U;
    }

    /* ---- 4) SMPLRT_DIV：开了 DLPF 之后内部输出 1kHz，这里再二分频 -> 500Hz ---- */
    if (mpu_write_reg(MPU_REG_SMPLRT_DIV, 0x01U) == 0U) {
        s_err = MPU6050_ERR_CONFIG;
        return 0U;
    }

    /* ---- 5) GYRO_CONFIG = 0x00 -> ±250°/s ---- */
    if (mpu_write_reg(MPU_REG_GYRO_CONFIG, 0x00U) == 0U) {
        s_err = MPU6050_ERR_CONFIG;
        return 0U;
    }

    /* ---- 6) ACCEL_CONFIG = 0x00 -> ±2g ---- */
    if (mpu_write_reg(MPU_REG_ACCEL_CONFIG, 0x00U) == 0U) {
        s_err = MPU6050_ERR_CONFIG;
        return 0U;
    }

    /* ---- 7) 通信检查：WHO_AM_I 读回来必须还是原值 ---- */
    id = 0U;
    if ((mpu_read_who(s_addr8, &id) == 0U) || (id != s_model)) {
        dbg_printf("[MPU] verify fail: WHO_AM_I readback=0x%02X (expect 0x%02X)\r\n",
                   (unsigned)id, (unsigned)s_model);
        s_err = MPU6050_ERR_VERIFY;
        return 0U;
    }

    s_ready = 1U;
    dbg_printf("[MPU] init ok %s a=+-2g g=+-250dps 500Hz\r\n",
               mpu6050_model_name(s_model));
    return s_model;
}

/**
 * @brief  读一帧原始数据（14 字节连续读）
 * @retval 1:成功  0:失败
 */
uint8_t mpu6050_read(int16_t *ax, int16_t *ay, int16_t *az,
                     int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[14];
    int16_t v;

    if (s_ready == 0U) {
        return 0U;
    }
    if ((ax == NULL) && (ay == NULL) && (az == NULL) &&
        (gx == NULL) && (gy == NULL) && (gz == NULL)) {
        return 0U;
    }

    /* ACCEL_XOUT_H(0x3B) 起连续 14 字节：加速度 6 + 温度 2 + 角速度 6 */
    if (HAL_I2C_Mem_Read(&hi2c1, s_addr8, MPU_REG_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT,
                         buf, 14U, MPU_I2C_TIMEOUT_MS) != HAL_OK) {
        s_err = MPU6050_ERR_NO_ACK;
        return 0U;
    }

    v = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
    if (ax != NULL) { *ax = v; }
    v = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    if (ay != NULL) { *ay = v; }
    v = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
    if (az != NULL) { *az = v; }

    v = (int16_t)((uint16_t)buf[8] | ((uint16_t)buf[9] << 8));
    if (gx != NULL) { *gx = v; }
    v = (int16_t)((uint16_t)buf[10] | ((uint16_t)buf[11] << 8));
    if (gy != NULL) { *gy = v; }
    v = (int16_t)((uint16_t)buf[12] | ((uint16_t)buf[13] << 8));
    if (gz != NULL) { *gz = v; }

    return 1U;
}

/**
 * @brief  读加速度并换算成 g（±2g 量程）
 * @retval 1:成功  0:失败
 */
uint8_t mpu6050_read_accel_gravity(float *ax, float *ay, float *az)
{
    int16_t rx = 0;
    int16_t ry = 0;
    int16_t rz = 0;

    if (mpu6050_read(&rx, &ry, &rz, NULL, NULL, NULL) == 0U) {
        return 0U;
    }
    if (ax != NULL) { *ax = (float)rx / MPU_ACCEL_LSB_PER_G; }
    if (ay != NULL) { *ay = (float)ry / MPU_ACCEL_LSB_PER_G; }
    if (az != NULL) { *az = (float)rz / MPU_ACCEL_LSB_PER_G; }
    return 1U;
}

/**
 * @brief  读角速度并换算成 dps（±250°/s 量程）
 * @retval 1:成功  0:失败
 */
uint8_t mpu6050_read_gyro_dps(float *gx, float *gy, float *gz)
{
    int16_t rx = 0;
    int16_t ry = 0;
    int16_t rz = 0;

    if (mpu6050_read(NULL, NULL, NULL, &rx, &ry, &rz) == 0U) {
        return 0U;
    }
    if (gx != NULL) { *gx = (float)rx / MPU_GYRO_LSB_PER_DPS; }
    if (gy != NULL) { *gy = (float)ry / MPU_GYRO_LSB_PER_DPS; }
    if (gz != NULL) { *gz = (float)rz / MPU_GYRO_LSB_PER_DPS; }
    return 1U;
}

/**
 * @brief  上次的错误码
 */
uint8_t mpu6050_last_error(void)
{
    return s_err;
}

/**
 * @brief  能不能读数据
 */
uint8_t mpu6050_is_ready(void)
{
    return s_ready;
}

/* ---------------------------------------------------------------------------
 * 低功耗相关
 * ------------------------------------------------------------------------- */

/**
 * @brief  开 INT 脚中断（数据就绪）
 * @note   INT_ENABLE(0x38) bit0 = DATA_RDY，每采样一次就拉一次，
 *         MCU 用它做 EXTI0 唤醒源（跌落判定不用轮询）。
 */
uint8_t mpu6050_enable_int(void)
{
    if (s_ready == 0U) {
        return 0U;
    }
    /* bit0=DATA_RDY, bit4=FALL_DETECT_STATUS（跌落） */
    return mpu_write_reg(MPU_REG_INT_ENABLE, 0x11U);
}

/**
 * @brief  进低功耗：只留加速度计，陀螺仪停
 * @note   PWR_MGMT_1(0x6B)：SLEEP=1, CLKSEL=0 → ~60µA（正常模式 ~3.5mA）
 */
uint8_t mpu6050_sleep(void)
{
    if (s_ready == 0U) {
        return 0U;
    }
    return mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x40U);
}

/**
 * @brief  退出低功耗：CLKSEL=1（陀螺仪用 PLL 时钟）
 */
uint8_t mpu6050_wake(void)
{
    if (s_ready == 0U) {
        return 0U;
    }
    return mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x01U);
}
