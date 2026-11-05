/*
 * sensor_task.h
 *
 * 传感器采集任务：AHT20（温度/湿度）+ BMP280（气压）。
 * 数据放在 g_sensor 里，显示层直接读。
 */
#ifndef TASK_SENSOR_TASK_H_
#define TASK_SENSOR_TASK_H_

#include <stdint.h>

typedef struct {
    float   temperature;   /**< 温度 ℃（AHT20） */
    float   humidity;      /**< 湿度 %RH（AHT20） */
    float   pressure;      /**< 气压 Pa（BMP280） */
    uint8_t aht_ok;        /**< AHT20 最近一次采样是否成功 */
    uint8_t bmp_ok;        /**< BMP280 最近一次采样是否成功 */
    uint8_t batt;          /**< 电池电量 %（PA0 = ADC12_IN0，接线表 ①-10） */
    uint8_t batt_ok;       /**< 电池采样链路可用（ADC 配起来了就是 1） */
} sensor_data_t;

extern volatile sensor_data_t g_sensor;

/** 传感器采集任务（FreeRTOS） */
void sensor_task(void *pvParameters);

/**
 * 电池 ADC 上电配置（PA0 = ADC12_IN0）。
 * 必须在调度器启动前调一次；失败不致命，只是 batt 一直报 0。
 */
void battery_adc_init(void);

/** 读一次电池电压（mV，已按分压比折算回电池端）；不可用返回 0 */
uint16_t battery_mv(void);

#endif /* TASK_SENSOR_TASK_H_ */
