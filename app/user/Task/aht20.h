/*
 * aht20.h  ——  AHT20 温湿度传感器驱动
 *
 *  Created on: Sep 7, 2026
 *      Author: Administrator
 */
#ifndef AHT20_H_
#define AHT20_H_

#include "main.h"

/**
 * @brief  AHT20 初始化（含上电延时、状态字检查、校准命令 0xBE 0x08 0x00）
 * @retval 1:成功  0:失败
 * @note   建议上电后调一次；aht20_cluate 内部也会在没初始化成功时自动补一次
 */
uint8_t aht20_init(void);

/**
 * @brief  读一次温湿度
 * @param  tem: 输出温度，单位 ℃
 * @param  hum: 输出湿度，单位 %RH
 * @retval 1:成功  0:失败
 */
uint8_t aht20_cluate(float *tem, float *hum);

#endif /* AHT20_H_ */
