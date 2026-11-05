/*
 * app_tasks.h —— 业务任务编排（传感 / 姿态 / 2.4G / 告警 / 自检）
 * -----------------------------------------------------------------------------
 * 与已有的 sensor_task / oled_task 的分工：
 *   sensor_task（既有）  采 AHT20 + BMP280，结果放进 g_sensor
 *   oled_task （既有）   刷 SSD1306
 *   本模块               姿态与跌落、2.4G 收发、阈值告警、自检上报
 *
 * 为什么还要一个 2.4G 任务组：
 *   ESP32-S3 那块彩屏要显示的数据来自这里；S3 自己没有传感器。
 *   两者用 nRF24 一对一对接，STM32 这边不碰 WiFi，功耗也就压得住。
 *
 * 低功耗策略见文件末尾的 app_lowpower_enter()。
 * -----------------------------------------------------------------------------
 */
#ifndef CAMPUS_APP_TASKS_H
#define CAMPUS_APP_TASKS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 全局状态（跨任务共享，读写都要走 app_state_lock） */
typedef struct {
    float    temp_c;        /* 温度 ℃ */
    float    hum;           /* 湿度 %RH */
    float    press;         /* 气压 Pa */
    uint8_t  batt;          /* 电量 % */
    uint8_t  state;         /* RF_STATE_* */
    uint16_t fall_count;    /* 累计跌落次数 */
    uint16_t reset_count;   /* 复位次数 */
    float    pitch_deg;
    float    roll_deg;
    uint8_t  alarm_bits;    /* 当前告警位 */
    uint32_t health_mask;   /* 自检健康位图 */
    uint32_t error_mask;    /* 自检错误位图 */
    uint8_t  link_up;       /* 2.4G 对端最近是否收到过 */
} app_state_t;

/* 阈值（由 S3 从云端取到后通过 CFG 帧下发） */
typedef struct {
    float    temp_max;      /* 温度上限 ℃，默认 35 */
    float    temp_min;      /* 温度下限 ℃，默认 5 */
    float    hum_max;       /* 湿度上限 %RH，默认 90 */
    float    hum_min;       /* 湿度下限 %RH，默认 20 */
    float    press_max;     /* 气压上限 hPa，默认 1200 */
    float    press_min;     /* 气压下限 hPa，默认 300 */
    float    fall_g;        /* 跌落冲击阈值 g，默认 2.5 */
    uint8_t  alarm_enable;  /* 告警使能 */
    uint32_t report_ms;     /* 上报周期（ms），默认 2000；可下发 500..3600000 */
} app_thresholds_t;

/* 初始化并创建全部业务任务（必须在 vTaskStartScheduler() 之前调用） */
void app_tasks_init(void);

/* 取全局状态快照（加锁拷贝，调用方拿到的副本是自洽的） */
void app_state_get(app_state_t *out);

/* 取当前阈值 */
void app_thresholds_get(app_thresholds_t *out);

/* 覆盖阈值（本地调试用；正常路径是收 CFG 帧） */
void app_thresholds_set(const app_thresholds_t *t);

/* 立即重跑一次自检并把报告发到 2.4G（后台命令用） */
void app_diag_trigger(void);

/* 设备状态 → 字符串（"steady"/"moving"/"freefall"/"impact"/"alarm"） */
const char *app_state_name(uint8_t state);

#ifdef __cplusplus
}
#endif

#endif /* CAMPUS_APP_TASKS_H */
