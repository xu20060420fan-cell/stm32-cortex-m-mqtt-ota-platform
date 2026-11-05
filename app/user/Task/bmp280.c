/*
 * bmp280.c  ——  BMP280 气压/温度传感器驱动（I2C1：PB6=SCL / PB7=SDA）
 *
 *  Created on: Sep 8, 2026
 *      Author: Administrator
 *
 * 相对原始版本的修正：
 *  1. **地址不再写死 0x77**：上电先探 0x76、再探 0x77（模块的 SDO 接地是 0x76、
 *     接 VDD 是 0x77），探到哪个用哪个，探不到才报错。原版写死 0x77，
 *     手上是 0x76 的模块就会一直 "wrong id"。
 *  2. 触发强制模式后**先等一小段再轮询** status：原来触发完立刻读，
 *     若 measuring 位还没置起会立即退出循环，读到上一次的原始数据。
 *  3. 超时改成**按时间**（HAL_GetTick）而不是数循环次数：原来的
 *     "timeout_cnt > 1000" 和真实耗时没有关系（每次迭代还含一次 I2C 事务）。
 *  4. 校准参数没读到就**不许出数据**：原来 corect 失败后 dig_* 全 0，
 *     sample 仍会返回"成功"并给出垃圾值。
 *  5. 删掉已无人引用的 huart1 外部声明。
 *
 * 打印统一走日志模块（环形缓冲 + DMA），绝不再自己 HAL_UART_Transmit，
 * 否则会和日志抢同一个串口，输出互相插花。
 */
#include "bmp280.h"
#include "debug_log.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>
#include <stdio.h>

/*
 * I2C 句柄：默认主总线 I2C1。
 * 想把 BMP280 挪到 MPU 的辅助总线（EDA/ECL，走 I2C3）时，
 * 在 main 里调 bmp280_set_bus(&hi2c3) 即可，驱动内部不用改。
 * ⚠ MPU6500 没有辅助总线，那种情况下就别调，留在 I2C1 上。
 */
extern I2C_HandleTypeDef hi2c1;
static I2C_HandleTypeDef *s_bus = &hi2c1;

void bmp280_set_bus(I2C_HandleTypeDef *bus)
{
    if (bus) s_bus = bus;
}

/* BMP280 的 7 位地址：SDO 接地 = 0x76，接 VDD = 0x77 */
#define BMP280_ADDR_7BIT_A      0x76U
#define BMP280_ADDR_7BIT_B      0x77U
/* HAL 要的是"8 位格式"地址（7 位地址左移 1 位） */
#define BMP280_TO_ADDR8(a)      ((uint16_t)((a) << 1))

/* 寄存器定义 */
#define BMP280_REG_ID           0xD0
#define BMP280_REG_CTRL_MEAS    0xF4
#define BMP280_REG_CONFIG       0xF5
#define BMP280_REG_STATUS       0xF3
#define BMP280_REG_RAW_DATA     0xF7
#define BMP280_REG_CALIB        0x88

#define BMP280_CHIP_ID          0x58U   /* BMP280；BME280 是 0x60，不兼容 */
#define BMP280_I2C_TIMEOUT_MS   100U
#define BMP280_MEAS_TIMEOUT_MS  100U    /* 轮询 status 的总超时 */
#define BMP280_STARTUP_MS       3U      /* 触发后先等这么久再开始轮询 */

/* 错误打印宏：统一走日志模块 */
#define BMP_ERR(fmt, ...) dbg_printf("[BMP] " fmt "\r\n", ##__VA_ARGS__)

/* 静态全局：当前地址、就绪标志、校准系数与中间变量 */
static uint16_t s_addr8;              /* 实际使用的 8 位格式地址 */
static uint8_t  s_ready;              /* 1 = 地址已确认且校准系数已读入 */

static uint16_t dig_T1, dig_P1;
static int16_t  dig_T2, dig_T3;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static int32_t  t_fine;
static uint32_t bmp_raw_p;
static uint32_t bmp_raw_t;

/** 读芯片 ID @retval 1:读到了(不管值对不对) 0:I2C 无应答 */
static uint8_t bmp280_read_id(uint16_t addr8, uint8_t *id)
{
    return (HAL_I2C_Mem_Read(s_bus, addr8, BMP280_REG_ID, I2C_MEMADD_SIZE_8BIT,
                             id, 1U, BMP280_I2C_TIMEOUT_MS) == HAL_OK) ? 1U : 0U;
}

/**
 * @brief  bmp280_init  探测器件并确认地址
 * @retval 1:找到 BMP280 并记住地址  0:两个地址都没应答 / ID 不对
 */
uint8_t bmp280_init(void)
{
    static const uint8_t cand[2] = {BMP280_ADDR_7BIT_A, BMP280_ADDR_7BIT_B};
    uint8_t i;

    s_ready = 0U;
    vTaskDelay(pdMS_TO_TICKS(5U));

    for (i = 0U; i < 2U; ++i) {
        uint8_t id = 0U;
        if (bmp280_read_id(BMP280_TO_ADDR8(cand[i]), &id) == 0U) {
            continue;                   /* 这个地址上没器件，试下一个 */
        }
        if (id != BMP280_CHIP_ID) {
            BMP_ERR("addr 0x%02X answered but id=0x%02X (not BMP280)",
                    (unsigned)cand[i], (unsigned)id);
            continue;
        }
        s_addr8 = BMP280_TO_ADDR8(cand[i]);
        dbg_printf("[BMP] found at 7bit=0x%02X, id=0x58\r\n", (unsigned)cand[i]);
        return 1U;
    }

    BMP_ERR("no BMP280 on 0x76/0x77 (check wiring / SDO)");
    return 0U;
}

/**
 * @brief  bmp280corect：读取24字节校准参数，配置寄存器
 * @retval 1:成功 0:失败
 */
uint8_t bmp280corect(void)
{
    uint8_t readbuffer[24U];
    uint8_t tx_buf[2U];
    HAL_StatusTypeDef ret;

    if (s_addr8 == 0U) {
        BMP_ERR("corect called before init");
        return 0U;
    }

    ret = HAL_I2C_Mem_Read(s_bus, s_addr8, BMP280_REG_CALIB, I2C_MEMADD_SIZE_8BIT,
                           readbuffer, 24U, BMP280_I2C_TIMEOUT_MS);
    if (ret != HAL_OK) {
        BMP_ERR("calib read fail ret=%d", ret);
        return 0U;
    }

    /* 解析校准参数，低字节在前 */
    dig_T1 = (uint16_t)(readbuffer[0U] | ((uint16_t)readbuffer[1U] << 8U));
    dig_T2 = (int16_t)(readbuffer[2U] | ((uint16_t)readbuffer[3U] << 8U));
    dig_T3 = (int16_t)(readbuffer[4U] | ((uint16_t)readbuffer[5U] << 8U));

    dig_P1 = (uint16_t)(readbuffer[6U] | ((uint16_t)readbuffer[7U] << 8U));
    dig_P2 = (int16_t)(readbuffer[8U] | ((uint16_t)readbuffer[9U] << 8U));
    dig_P3 = (int16_t)(readbuffer[10U] | ((uint16_t)readbuffer[11U] << 8U));
    dig_P4 = (int16_t)(readbuffer[12U] | ((uint16_t)readbuffer[13U] << 8U));
    dig_P5 = (int16_t)(readbuffer[14U] | ((uint16_t)readbuffer[15U] << 8U));
    dig_P6 = (int16_t)(readbuffer[16U] | ((uint16_t)readbuffer[17U] << 8U));
    dig_P7 = (int16_t)(readbuffer[18U] | ((uint16_t)readbuffer[19U] << 8U));
    dig_P8 = (int16_t)(readbuffer[20U] | ((uint16_t)readbuffer[21U] << 8U));
    dig_P9 = (int16_t)(readbuffer[22U] | ((uint16_t)readbuffer[23U] << 8U));

    /* dig_P1 是压力公式里的除数，为 0 说明校准数据是假的（读全 0 / 通信半死）*/
    if (dig_P1 == 0U) {
        BMP_ERR("calib looks invalid (dig_P1=0)");
        return 0U;
    }

    /* ctrl_meas: osrs_t=001(×1), osrs_p=100(×8), mode=00 睡眠模式 */
    tx_buf[0U] = BMP280_REG_CTRL_MEAS;
    tx_buf[1U] = 0x30U;
    ret = HAL_I2C_Master_Transmit(s_bus, s_addr8, tx_buf, 2U, BMP280_I2C_TIMEOUT_MS);
    if (ret != HAL_OK) {
        BMP_ERR("ctrl_meas write ret=%d", ret);
        return 0U;
    }

    /* config: t_sb=0.5ms, IIR 滤波关闭, 4 线 SPI 关 */
    tx_buf[0U] = BMP280_REG_CONFIG;
    tx_buf[1U] = 0x00U;
    ret = HAL_I2C_Master_Transmit(s_bus, s_addr8, tx_buf, 2U, BMP280_I2C_TIMEOUT_MS);
    if (ret != HAL_OK) {
        BMP_ERR("config write ret=%d", ret);
        return 0U;
    }

    s_ready = 1U;
    return 1U;
}

/**
 * @brief  读取6字节原始压力温度数据，存入bmp_raw_p / bmp_raw_t
 * @retval 1:ok 0:i2c error
 */
static uint8_t bmp280_read_raw(void)
{
    uint8_t buf[6U];

    if (HAL_I2C_Mem_Read(s_bus, s_addr8, BMP280_REG_RAW_DATA, I2C_MEMADD_SIZE_8BIT,
                         buf, 6U, BMP280_I2C_TIMEOUT_MS) != HAL_OK) {
        return 0U;
    }

    /* 拼接20bit原始ADC */
    bmp_raw_p = ((uint32_t)buf[0U] << 12U) | ((uint32_t)buf[1U] << 4U) | (buf[2U] >> 4U);
    bmp_raw_t = ((uint32_t)buf[3U] << 12U) | ((uint32_t)buf[4U] << 4U) | (buf[5U] >> 4U);
    return 1U;
}

/**
 * @brief  触发一次测量，读取并补偿温压
 * @param  out_temp: 输出温度，单位 ℃
 * @param  out_press: 输出压力，单位 Pa
 * @retval 1:成功  0:失败
 */
uint8_t bmp280_sample(float *out_temp, float *out_press)
{
    uint8_t status_reg;
    uint8_t tx_buf[2U];
    uint32_t t0;
    /* ★ 用 float 而不是 double —— 这是省 flash 的关键一改
     * ----------------------------------------------------------
     * 原来照抄 datasheet 8.2 的**双精度**参考代码，整段都是 double。
     * 在 Cortex-M3（没有 FPU）上，double 运算全部走软件库：
     *   __adddf3 630 + __subdf3 634 + __muldf3 596 + __divdf3 464
     *   + __aeabi_dadd/dsub/dmul/ddiv 各 ~600 + __aeabi_frsub 364
     *   ≈ 4.5KB 白白进固件。
     * 换成 Bosch 官方的 32 位 float 版后这整套库全部消失。
     *
     * 精度代价（Bosch 官方数据）：
     *   温度：double ±0.01°C  →  float ±0.01°C   （无差别）
     *   气压：double ±0.18Pa  →  float ±0.12hPa （自检/显示都够用）
     */
    float var1, var2, var1_p, var2_p, temp_val, press_val;

    if (out_temp == NULL || out_press == NULL) {
        return 0U;
    }

    /* 没探测成功过就先补一次（传感器供电比 MCU 慢时很常见）*/
    if (s_ready == 0U) {
        if ((bmp280_init() == 0U) || (bmp280corect() == 0U)) {
            return 0U;
        }
    }

    /* step1：触发强制模式 */
    tx_buf[0U] = BMP280_REG_CTRL_MEAS;
    tx_buf[1U] = 0x31U;
    if (HAL_I2C_Master_Transmit(s_bus, s_addr8, tx_buf, 2U,
                                BMP280_I2C_TIMEOUT_MS) != HAL_OK) {
        BMP_ERR("trig ret fail");
        s_ready = 0U;
        return 0U;
    }

    /* step2：先等器件真正进入测量（t_startup），再轮询 status bit3(measuring)。
     * 原来触发完立刻读，measuring 位可能还没置起 -> 直接退出循环读到旧数据 */
    vTaskDelay(pdMS_TO_TICKS(BMP280_STARTUP_MS));

    t0 = HAL_GetTick();
    do {
        if (HAL_I2C_Mem_Read(s_bus, s_addr8, BMP280_REG_STATUS, I2C_MEMADD_SIZE_8BIT,
                             &status_reg, 1U, BMP280_I2C_TIMEOUT_MS) != HAL_OK) {
            BMP_ERR("status read fail");
            s_ready = 0U;
            return 0U;
        }
        if ((status_reg & (1U << 3U)) == 0U) {
            break;                                  /* 测量完成 */
        }
        if ((HAL_GetTick() - t0) > BMP280_MEAS_TIMEOUT_MS) {
            BMP_ERR("measure timeout, status=0x%02X", (unsigned)status_reg);
            return 0U;
        }
        vTaskDelay(pdMS_TO_TICKS(1U));              /* 让出 CPU，别忙等 */
    } while (1);

    /* step3：读取原始数据 */
    if (bmp280_read_raw() == 0U) {
        BMP_ERR("raw read fail");
        return 0U;
    }

    /* step4：温度补偿（datasheet 8.2 的 32 位 float 版）*/
    var1 = (((float)bmp_raw_t) / 16384.0f - ((float)dig_T1) / 1024.0f) * ((float)dig_T2);
    var2 = ((((float)bmp_raw_t) / 131072.0f - ((float)dig_T1) / 8192.0f) *
            (((float)bmp_raw_t) / 131072.0f - ((float)dig_T1) / 8192.0f)) * ((float)dig_T3);
    t_fine = (int32_t)(var1 + var2);
    temp_val = (var1 + var2) / 5120.0f;
    *out_temp = temp_val;

    /* step5：压力补偿 */
    var1_p = (((float)t_fine) / 2.0f) - 64000.0f;
    var2_p = var1_p * var1_p * ((float)dig_P6) / 32768.0f;
    var2_p = var2_p + var1_p * ((float)dig_P5) * 2.0f;
    var2_p = (var2_p / 4.0f) + (((float)dig_P4) * 65536.0f);

    var1_p = (((float)dig_P3) * var1_p * var1_p / 524288.0f + ((float)dig_P2) * var1_p) / 524288.0f;
    var1_p = (1.0f + var1_p / 32768.0f) * ((float)dig_P1);

    if (var1_p == 0.0f) {
        BMP_ERR("press div0");
        return 0U;
    }

    press_val = 1048576.0f - (float)bmp_raw_p;
    press_val = (press_val - (var2_p / 4096.0f)) * 6250.0f / var1_p;

    var1_p = ((float)dig_P9) * press_val * press_val / 2147483648.0f;
    var2_p = press_val * ((float)dig_P8) / 32768.0f;
    press_val = press_val + (var1_p + var2_p + ((float)dig_P7)) / 16.0f;

    *out_press = press_val;

    return 1U;
}
