/*
 * oled_anim.c
 *
 * 动画引擎实现，曲线表和算法直接来自 KK_UI（MIT）：
 *   kk_ui/src/kk_ui_anim.c 的 KK_UI_EaseQ12 / KK_UI_LerpQ12 / KK_UI_RoundQ8
 * 保持同样的定点格式和采样方式，动画手感与 KK_UI 一致。
 */
#include "oled_anim.h"

/* cubic-bezier(0.25, 0, 0.20, 1)，按时间均匀采样 65 个点 */
static const uint16_t oled_ease_curve[65] = {
    0, 6, 23, 53, 98, 159, 236, 329,
    439, 565, 704, 853, 1010, 1170, 1331, 1490,
    1645, 1795, 1938, 2075, 2205, 2328, 2444, 2554,
    2658, 2756, 2848, 2936, 3018, 3095, 3168, 3237,
    3302, 3363, 3420, 3475, 3526, 3574, 3619, 3661,
    3701, 3738, 3773, 3806, 3836, 3864, 3891, 3915,
    3938, 3959, 3978, 3995, 4011, 4025, 4038, 4050,
    4060, 4069, 4076, 4082, 4087, 4091, 4094, 4095,
    4096
};

uint16_t oled_ease_q12(uint32_t elapsed, uint32_t duration)
{
    uint32_t reference_elapsed;
    uint32_t scaled;
    uint32_t index;
    uint32_t fraction;
    uint32_t value;

    if (duration == 0U || elapsed >= duration) {
        return OLED_Q12_ONE;
    }
    /* 把实际时长归一到 300 单位的参考时间轴，再线性插值查表 */
    reference_elapsed = elapsed * 300U / duration;
    scaled = reference_elapsed * 64U;
    index = scaled / 300U;
    fraction = scaled % 300U;
    value = oled_ease_curve[index];
    value += ((uint32_t)(oled_ease_curve[index + 1U] - value) * fraction) / 300U;
    return (uint16_t)value;
}

int32_t oled_lerp_q12(int32_t from, int32_t to, uint16_t progress)
{
    return from + (int32_t)(((int64_t)to - from) * progress / OLED_Q12_ONE);
}

int16_t oled_round_q8(int32_t value_q8)
{
    if (value_q8 >= 0) {
        return (int16_t)((value_q8 + 128) / 256);
    }
    return (int16_t)((value_q8 - 128) / 256);
}

void oled_anim_set(oled_anim_t *anim, int32_t value, uint32_t now)
{
    anim->from = value;
    anim->to = value;
    anim->started = now;
    anim->duration = 0U;
    anim->active = 0U;
}

void oled_anim_to(oled_anim_t *anim, int32_t to, uint16_t duration, uint32_t now)
{
    if (anim->to == to) {
        return;   /* 目标没变，不重启动画（否则每帧都会从头开始） */
    }
    /* 从"当前正在显示的位置"接着走，避免动画中途改目标时跳一下 */
    anim->from = oled_anim_value_q8(anim, now);
    anim->to = to;
    anim->started = now;
    anim->duration = duration;
    anim->active = (duration == 0U) ? 0U : 1U;
}

int32_t oled_anim_value_q8(const oled_anim_t *anim, uint32_t now)
{
    if (anim->active == 0U || anim->duration == 0U) {
        return anim->to;
    }
    if (now - anim->started >= anim->duration) {
        return anim->to;
    }
    return oled_lerp_q12(anim->from, anim->to,
                         oled_ease_q12(now - anim->started, anim->duration));
}

int32_t oled_anim_value(const oled_anim_t *anim, uint32_t now)
{
    return oled_round_q8(oled_anim_value_q8(anim, now));
}

uint8_t oled_anim_running(const oled_anim_t *anim, uint32_t now)
{
    if (anim->active == 0U || anim->duration == 0U) {
        return 0U;
    }
    return ((now - anim->started) < anim->duration) ? 1U : 0U;
}
