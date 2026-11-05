/*
 * oled_anim.h
 *
 * 从 KK_UI 移植过来的动画引擎（缓动曲线 + 定点插值 + 动画状态）。
 * KK_UI 的"流畅无回弹非线性动画"就是靠这套东西实现的，代码很小，直接搬过来。
 */
#ifndef TASK_OLED_ANIM_H_
#define TASK_OLED_ANIM_H_

#include <stdint.h>

/* 定点格式：Q12 里 4096 = 1.0，Q8 里 256 = 1.0 */
#define OLED_Q12_ONE 4096U
#define OLED_Q8_ONE  256

/* 动画时长，沿用 KK_UI 的参考值 */
#define OLED_ANIM_ENTER_MS 300U   /* 整屏入场 */
#define OLED_ANIM_FOCUS_MS 160U   /* 焦点框飞行 */
#define OLED_ANIM_VALUE_MS 100U   /* 数值滚动 */
#define OLED_ANIM_TOAST_MS 120U   /* Toast 弹出 */

/** 缓动进度：elapsed/duration 映射到 0~4096（cubic-bezier(0.25,0,0.20,1)） */
uint16_t oled_ease_q12(uint32_t elapsed, uint32_t duration);

/** 按 Q12 进度在 from 和 to 之间插值 */
int32_t oled_lerp_q12(int32_t from, int32_t to, uint16_t progress);

/** Q8 转整数像素（四舍五入） */
int16_t oled_round_q8(int32_t value_q8);

/** 一个"从当前值平滑移动到目标值"的动画状态 */
typedef struct {
    int32_t  from;      /**< 起点（Q8） */
    int32_t  to;        /**< 目标（Q8） */
    uint32_t started;   /**< 起始时刻 ms */
    uint16_t duration;  /**< 时长 ms */
    uint8_t  active;    /**< 是否处于动画中 */
} oled_anim_t;

/** 直接设定值，不做动画 */
void oled_anim_set(oled_anim_t *anim, int32_t value, uint32_t now);

/** 平滑移动到新目标；目标没变则不重启动画，中途改目标会从当前位置接着走 */
void oled_anim_to(oled_anim_t *anim, int32_t to, uint16_t duration, uint32_t now);

/** 取当前动画值（Q8） */
int32_t oled_anim_value_q8(const oled_anim_t *anim, uint32_t now);

/** 取当前动画值（整数像素） */
int32_t oled_anim_value(const oled_anim_t *anim, uint32_t now);

/** 动画是否还在进行（用来决定这一帧要不要重画） */
uint8_t oled_anim_running(const oled_anim_t *anim, uint32_t now);

#endif /* TASK_OLED_ANIM_H_ */
