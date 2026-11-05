/*
 * buttom.c
 *
 *  Created on: Sep 4, 2026
 *      Author: Administrator
 */
#include"buttom.h"
void button(void *pvParmaemters){
   GPIO_PinState last =GPIO_PIN_SET;
	GPIO_PinState now =GPIO_PIN_SET;
	for(;;){
		/* 【按键在 PA12，不是 PA1】接线表 ①-9「按键 ×1」写死：
		 * KEY1 = PA12，内部上拉、按下接地；PA1 是 ADC1_IN1，
		 * 是整块板唯一剩下的模拟输入口，特意留给以后接模拟量。
		 * 原来这里读的是 main.h 的 KEY1_Pin(=PA1) —— 那个脚上根本没有按键，
		 * 读回来恒为高，所以按键怎么按都没反应。
		 * （main.h 里 KEY1/KEY2 的命名和接线表是反的，以接线表为准；
		 *   两个脚在 MX_GPIO_Init 里都配成了上拉输入，改读哪个都能读动。） */
		now=HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin);
		if(last==GPIO_PIN_RESET&&now==GPIO_PIN_SET){
			buzzer_beep();
		}
		last=now;
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}
