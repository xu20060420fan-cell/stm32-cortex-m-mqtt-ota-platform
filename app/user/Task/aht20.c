/*
 * aht20.c  ——  AHT20 温湿度传感器驱动（I2C1：PB6=SCL / PB7=SDA）
 *
 *  Created on: Sep 7, 2026
 *      Author: Administrator
 *
 * 相对原始版本的修正：
 *  1. 补上**上电初始化/校准序列**（0xBE 0x08 0x00 + 校验状态字 bit3）。
 *     原版直接发测量命令，芯片没校准过时会偶发读不到数据。
 *  2. busy 位（bit7）不再"读一次就放弃"，改成等一会儿重读若干次，并且失败要打日志。
 *  3. 读写地址统一用 0x70（HAL 自己会置读写位），原来的 0x71 只是碰巧能跑。
 *  4. 删掉已无人引用的全局 msg / msg1 和 huart1 外部声明（占 RAM 又污染符号表）。
 *  5. 位域拼接改用按位或，和 datasheet 一致（原来的 + 结果对但容易被误读）。
 *
 * 本文件只在 FreeRTOS 任务里调用，所以等待一律用 vTaskDelay（让出 CPU），
 * 不用 HAL_Delay 忙等（AHT20 一次测量要等 85ms）。
 */
#include "aht20.h"
#include "debug_log.h"

#include "FreeRTOS.h"
#include "task.h"

extern I2C_HandleTypeDef hi2c1;

/* HAL 的 I2C 接口传的是"8 位格式"地址（7 位地址左移 1 位）：
 *   AHT20 的 7 位地址是 0x38 -> 0x38 << 1 = 0x70
 * 读的时候 HAL 会自己把最低位置 1（I2C_7BIT_ADD_READ），
 * 所以**读写都传 0x70**，不要传 0x71。 */
#define AHT20_ADDR      0x70U

/* 状态字（读回来的第 1 个字节）*/
#define AHT20_STATUS_BUSY        0x80U   /* 1 = 正在测量，数据无效 */
#define AHT20_STATUS_CALIBRATED  0x08U   /* 1 = 校准系数已就绪 */

/* 时序（单位 ms），来自 AHT20 datasheet */
#define AHT20_POWER_ON_MS   100U   /* 上电到第一次 I2C 访问要等 ≥100ms */
#define AHT20_INIT_WAIT_MS   10U   /* 发完初始化命令等 ≥10ms */
#define AHT20_MEASURE_MS     85U   /* 触发测量后等 ≥75ms（典型 80ms） */
#define AHT20_BUSY_RETRY      5U   /* busy 时最多重读几次 */
#define AHT20_BUSY_WAIT_MS    5U   /* 每次重读前等多久 */

#define AHT20_I2C_TIMEOUT_MS 100U

/* 上电延时只做一次；s_inited 表示当前是否已确认校准 */
static uint8_t s_power_on_done = 0U;
static uint8_t s_inited         = 0U;

/** 读 1 字节状态字 @retval 1:成功 0:I2C 失败 */
static uint8_t aht20_read_status(uint8_t *status)
{
    return (HAL_I2C_Master_Receive(&hi2c1, AHT20_ADDR, status, 1U,
                                   AHT20_I2C_TIMEOUT_MS) == HAL_OK) ? 1U : 0U;
}

/**
 * @brief  AHT20 初始化：上电稳定 -> 读状态 -> 未校准则发 0xBE 0x08 0x00 -> 确认校准位
 * @retval 1:已校准，可以测量  0:失败（串口会打出原因）
 */
uint8_t aht20_init(void)
{
    uint8_t cmd[3] = {0xBE, 0x08, 0x00};
    uint8_t status = 0U;
    uint8_t i;

    if (s_power_on_done == 0U) {
        vTaskDelay(pdMS_TO_TICKS(AHT20_POWER_ON_MS));
        s_power_on_done = 1U;
    }

    if (aht20_read_status(&status) == 0U) {
        dbg_printf("[AHT] init: read status fail\r\n");
        return 0U;
    }

    /* 状态字 bit3 = 0 说明校准系数还没加载，需要主动初始化一次 */
    if ((status & AHT20_STATUS_CALIBRATED) == 0U) {
        if (HAL_I2C_Master_Transmit(&hi2c1, AHT20_ADDR, cmd, 3U,
                                    AHT20_I2C_TIMEOUT_MS) != HAL_OK) {
            dbg_printf("[AHT] init: send 0xBE 0x08 0x00 fail\r\n");
            return 0U;
        }
        vTaskDelay(pdMS_TO_TICKS(AHT20_INIT_WAIT_MS));
    }

    /* 等校准位就绪 */
    for (i = 0U; i < AHT20_BUSY_RETRY; ++i) {
        if (aht20_read_status(&status) == 0U) {
            dbg_printf("[AHT] init: read status fail\r\n");
            return 0U;
        }
        if ((status & AHT20_STATUS_CALIBRATED) != 0U) {
            s_inited = 1U;
            dbg_printf("[AHT] init ok, status=0x%02X\r\n", (unsigned)status);
            return 1U;
        }
        vTaskDelay(pdMS_TO_TICKS(AHT20_INIT_WAIT_MS));
    }

    dbg_printf("[AHT] init fail: not calibrated, status=0x%02X\r\n", (unsigned)status);
    return 0U;
}

/**
 * @brief  读一次温湿度
 * @param  tem: 输出温度（℃）  hum: 输出湿度（%RH）
 * @retval 1:成功  0:失败（原因已打到串口）
 */
uint8_t aht20_cluate(float *tem, float *hum)
{
    uint8_t cmd[3] = {0xAC, 0x33, 0x00};
    uint8_t buf[6];
    uint8_t retry;
    uint32_t hum_raw;
    uint32_t tem_raw;

    if (tem == NULL || hum == NULL) {
        return 0U;
    }

    /* 没初始化成功过就先补一次：避免上电顺序不对导致整个运行期都读不到数 */
    if ((s_inited == 0U) && (aht20_init() == 0U)) {
        return 0U;
    }

    /* 1) 触发一次测量 */
    if (HAL_I2C_Master_Transmit(&hi2c1, AHT20_ADDR, cmd, 3U,
                                AHT20_I2C_TIMEOUT_MS) != HAL_OK) {
        dbg_printf("[AHT] trigger fail (i2c)\r\n");
        s_inited = 0U;              /* 总线异常，下一轮重新初始化 */
        return 0U;
    }

    /* 2) 等测量完成 */
    vTaskDelay(pdMS_TO_TICKS(AHT20_MEASURE_MS));

    /* 3) 读回 状态字 + 5 字节数据；busy 位还置着就再等等（原来直接 return 0 静默失败）*/
    for (retry = 0U; ; ++retry) {
        if (HAL_I2C_Master_Receive(&hi2c1, AHT20_ADDR, buf, 6U,
                                   AHT20_I2C_TIMEOUT_MS) != HAL_OK) {
            dbg_printf("[AHT] read data fail (i2c)\r\n");
            s_inited = 0U;
            return 0U;
        }
        if ((buf[0] & AHT20_STATUS_BUSY) == 0U) {
            break;                  /* 测量完成，数据有效 */
        }
        if (retry >= AHT20_BUSY_RETRY) {
            dbg_printf("[AHT] busy too long, status=0x%02X\r\n", (unsigned)buf[0]);
            return 0U;
        }
        vTaskDelay(pdMS_TO_TICKS(AHT20_BUSY_WAIT_MS));
    }

    /* 4) 解析：湿度 20bit = b1[19:12] | b2[11:4] | b3[7:4]
     *          温度 20bit = b3[3:0]<<16 | b4<<8 | b5          */
    hum_raw = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) | ((uint32_t)buf[3] >> 4);
    tem_raw = (((uint32_t)buf[3] & 0x0FU) << 16) | ((uint32_t)buf[4] << 8) | (uint32_t)buf[5];

    *hum = (float)hum_raw * 100.0f / 1048576.0f;             /* 100 / 2^20 */
    *tem = (float)tem_raw * 200.0f / 1048576.0f - 50.0f;     /* 200 / 2^20 - 50 */

    return 1U;
}
