/*
 * buzzer.h
 *
 *  Created on: Sep 4, 2026
 *      Author: Administrator
 */

#ifndef TASK_BUZZER_H_
#define TASK_BUZZER_H_

#include"FreeRTOS.h"
#include"semphr.h"
#include"main.h"

void buzzer_init();
void buzzer_beep();

#endif /* TASK_BUZZER_H_ */
