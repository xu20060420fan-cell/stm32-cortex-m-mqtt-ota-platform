#ifndef __BMP280_H
#define __BMP280_H

#include "stm32f1xx_hal.h"

uint8_t bmp280_init(void);

/*
 * 指定 BMP280 挂在哪条 I2C 总线上（默认 I2C1）。
 * 用途：MPU9250/9255 的 EDA/ECL 是一条独立辅助 I2C，
 * 可以把 BMP280 挪过去，让主总线只跑 AHT20 + MPU。
 * ⚠ MPU6500 没有辅助总线，MPU6050 也没有。
 */
void bmp280_set_bus(I2C_HandleTypeDef *bus);
uint8_t bmp280corect(void);
/**
 * @brief  一次完整采样：触发测量→等待→读取原始→补偿计算
 * @param  out_temp: 输出温度，单位 ℃
 * @param  out_press: 输出压力，单位 Pa
 * @retval 1:成功  0:失败
 */
uint8_t bmp280_sample(float *out_temp, float *out_press);

#endif
