/*
 * task_monitor.h
 *
 * 任务栈水位监控：记录各任务句柄，定期打印每个任务还剩多少栈。
 * 单位是"字"（1 字 = 4 字节），数值是历史最小剩余量——接近 0 就说明该加栈了。
 */
#ifndef TASK_TASK_MONITOR_H_
#define TASK_TASK_MONITOR_H_

#include "FreeRTOS.h"
#include "task.h"

/** 登记一个任务（建完任务后调用） */
void task_monitor_add(const char *name, TaskHandle_t handle);

/** 打印所有已登记任务的栈水位和剩余堆 */
void task_monitor_print(void);

#endif /* TASK_TASK_MONITOR_H_ */
