/*
 * rf_protocol.h —— STM32 ↔ ESP32-S3 的 2.4GHz 帧协议
 * -----------------------------------------------------------------------------
 * 物理层：nRF24L01+PA，SPI2（STM32 侧）/ SPI3（ESP32-S3 侧）
 *   地址 'N','O','D','E','1'（5 字节，先进先出）、信道 76、1Mbps、CRC 开、ACK 开
 *
 * 帧格式（单包 ≤32 字节，头 4 字节 + 负载 ≤24 字节）：
 *
 *   偏移  0     1      2     3      4..27
 *        magic  type   seq   len    payload
 *         0xA5  u8     u8    u8     len 字节
 *
 *   magic 固定 0xA5：用来在噪声里快速筛出有效帧。
 *   seq   递增序号；分片的多帧（DIAG_DET / CFG）靠它排序。
 *   len   负载长度，0..24。
 *
 * 为什么不用变长帧：nRF24 硬件只收固定长度包，len 字段让我们能在
 * 32 字节里塞不同长度的负载，又不用发填充。
 *
 * 多字节数值一律**小端**（STM32 与 ESP32 都是小端，直接 memcpy 即可）。
 * -----------------------------------------------------------------------------
 */
#ifndef CAMPUS_RF_PROTOCOL_H
#define CAMPUS_RF_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 帧头 ---- */
#define RF_MAGIC        0xA5U
#define RF_MAX_PAYLOAD  24U
#define RF_HEADER_LEN   4U

/* ---- 帧类型 ---- */
#define RF_TYPE_SENSOR    0x01U   /* STM32→S3  传感器实时数据 */
#define RF_TYPE_STATUS    0x02U   /* STM32→S3  状态机/跌落/告警 */
#define RF_TYPE_DIAG_SUM  0x03U   /* STM32→S3  自检摘要（位图） */
#define RF_TYPE_DIAG_DET  0x04U   /* STM32→S3  自检明细（分片） */
#define RF_TYPE_CMD       0x05U   /* S3→STM32  命令 */
#define RF_TYPE_ACK       0x06U   /* STM32→S3  命令应答 */
#define RF_TYPE_CFG       0x07U   /* S3→STM32  配置（阈值，分片） */

/* ---- 设备状态枚举（与云端 status 字段一致）---- */
#define RF_STATE_STEADY   0x00U   /* 平稳 */
#define RF_STATE_MOVING   0x01U   /* 移动中 */
#define RF_STATE_FREEFALL 0x02U   /* 失重（跌落过程中） */
#define RF_STATE_IMPACT   0x03U   /* 冲击/碰撞 */
#define RF_STATE_ALARM    0x04U   /* 阈值告警 */

/* ---- SENSOR 负载布局（20 字节）---- */
typedef struct {
    int16_t  temp_c_x100;   /* 温度 ℃ ×100 */
    int16_t  hum_x100;      /* 湿度 %RH ×100 */
    uint32_t press_pa;      /* 气压 Pa */
    uint8_t  state;         /* RF_STATE_* */
    uint8_t  batt;          /* 电量 % */
    int16_t  pitch_x100;    /* 俯仰角 °×100 */
    int16_t  roll_x100;     /* 横滚角 °×100 */
    uint32_t ts;            /* 毫秒时间戳（本地 uptime） */
} __attribute__((packed)) rf_sensor_t;

/* ---- STATUS 负载布局（12 字节）----
 * 增加 fw_ver：S3 页面要显示节点固件版本。
 * 两端 rf_protocol.h 必须保持一致（tools_shared/check_rf_parity.py 会校验）。 */
#define RF_FW_VER_LEN   6U
typedef struct {
    uint8_t  state;         /* 状态机 */
    uint16_t fall_count;    /* 累计跌落次数 */
    uint8_t  alarm_bits;    /* 告警位：bit0 温度高 bit1 温度低 bit2 湿度高 bit3 湿度低 */
    uint16_t reset_count;   /* 复位次数 */
    char     fw_ver[RF_FW_VER_LEN];   /* 固件版本字符串，如 "1.0.4" */
} __attribute__((packed)) rf_status_t;

/* ---- DIAG_SUM 负载布局（10 字节）---- */
typedef struct {
    uint32_t health_mask;   /* bit i = 1 → 外设 i 正常 */
    uint32_t error_mask;    /* bit i = 1 → 外设 i 异常 */
    uint8_t  ok_count;
    uint8_t  bad_count;
} __attribute__((packed)) rf_diag_sum_t;

/* ---- CMD 命令字 ---- */
#define RF_CMD_PING        0x01U   /* S3 问 STM32 在不在 */
#define RF_CMD_SELFTEST    0x02U   /* S3 让 STM32 立刻重跑一次自检 */
#define RF_CMD_REBOOT      0x03U   /* S3 让 STM32 复位（进 bootloader 刷机窗口）*/
#define RF_CMD_GET_STATUS  0x04U   /* S3 要一份完整状态 */
#define RF_CMD_OTA_START   0x05U   /* S3 通知：开始走 OTA，让 STM32 复位 */

#ifdef __cplusplus
}
#endif

#endif /* CAMPUS_RF_PROTOCOL_H */
