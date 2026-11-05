/*
 * oled_home.h
 *
 * 主页：显示关键信息，并带上从 KK_UI 移植过来的动画。
 * 目前不做换页，所有内容都在这一个页面上。
 */
#ifndef TASK_OLED_HOME_H_
#define TASK_OLED_HOME_H_

#include <stdint.h>

/** 复位主页与动画状态（上电时调用一次） */
void oled_home_init(uint32_t now);

/** 画一帧主页 */
void oled_home_draw(uint32_t now);

/** 显示任务：读三键 + 更新数据 + 画帧 */
void oled_task(void *pvParameters);

#endif /* TASK_OLED_HOME_H_ */
