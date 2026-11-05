/*
 * fall_detect.h  ——  跌落 / 碰撞检测算法（纯计算，不碰硬件）
 *
 *  Created on: Oct 24, 2026
 *      Author: Administrator
 *
 * 输入：加速度（g）、角速度（dps）、时间戳（ms）
 * 输出：一个状态机 + 若干标志位
 *
 * 判据（两类都要覆盖）：
 *   1) 跌落（free fall）
 *      加速度模长持续 < freefall_g（默认 0.35g）持续 ≥ freefall_ms（默认 120ms），
 *      之后出现冲击峰值 > impact_g（默认 2.5g） -> 判定为**跌落**
 *   2) 碰撞 / 剧烈晃动
 *      角速度模长 > gyro_dps（默认 220dps）累计超过 gyro_ms（默认 300ms）
 *      -> 判定为**碰撞**（注意：这种没有自由落体阶段，所以不是跌落）
 *
 * 「设备静止但被晃动」和「真实跌落」靠两点区分：
 *   - 有没有经过**确认过的自由落体阶段**（ff_ready）
 *   - 冲击之后设备有没有**静止下来**（状态机走到 SETTLED）
 * 所以：真实跌落 = fall_detect_is_fall() == 1；
 *       只是被晃/被磕 = fall_detect_is_fall() == 0 且 fall_detect_is_impact() == 1。
 *
 * 坐标约定：模块右手坐标系，X 右、Y 前、Z 上；加速度单位 g，角速度单位 dps。
 *
 * 计算量很小，不依赖 libm 的 atan2f（newlib-nano 里没有，带进去会拖一大坨代码），
 * atan2 用手写多项式近似，sqrt 用 __builtin_sqrtf。
 */
#ifndef TASK_FALL_DETECT_H_
#define TASK_FALL_DETECT_H_

#include <stdint.h>

/* ===================== 状态机 ===================== */
typedef enum {
    FALL_STATE_IDLE     = 0,   /* 静止待命 */
    FALL_STATE_FREEFALL = 1,   /* 自由落体中（加速度很小） */
    FALL_STATE_IMPACT   = 2,   /* 落地/碰撞瞬间（冲击峰值刚过） */
    FALL_STATE_SETTLED  = 3    /* 冲击后已静止，等上位机确认后 reset */
} fall_state_t;

/* ===================== 可调参数 ===================== */
typedef struct {
    float    freefall_g;      /* 自由落体判定阈值（加速度模长，g），默认 0.35 */
    uint32_t freefall_ms;     /* 自由落体需持续的时长（ms），默认 120 */
    uint32_t freefall_max_ms; /* 自由落体最长时长（ms），超过视为一直没落地，默认 1500 */
    uint32_t impact_window_ms;/* 离开自由落体后等冲击峰值的窗口（ms），默认 300 */
    float    impact_g;        /* 冲击峰值阈值（g），默认 2.5 */
    float    gyro_dps;        /* 角速度阈值（dps），默认 220 */
    uint32_t gyro_ms;         /* 角速度超限需累计的时长（ms），默认 300 */
    float    settle_g;        /* 静止判定：加速度模长偏离 1g 的容差（g），默认 0.4 */
    float    settle_dps;      /* 静止判定：角速度阈值（dps），默认 60 */
    uint32_t settle_ms;       /* 冲击后需要保持静止多久（ms），默认 800 */
    uint32_t impact_max_ms;   /* 冲击后最长等待时间（ms），默认 3000 */
    float    gyro_alpha;      /* 互补滤波的陀螺权重（0~1），默认 0.98 */
} fall_config_t;

/* ===================== 对外 API ===================== */

/**
 * @brief  初始化（装默认参数并清空状态）。必须在 fall_detect_update() 之前调用。
 */
void fall_detect_init(void);

/**
 * @brief  清空所有状态和标志，回到 IDLE（上位机确认跌落后调用）
 */
void fall_detect_reset(void);

/**
 * @brief  覆盖判据参数（传 NULL 表示不修改）
 */
void fall_detect_set_config(const fall_config_t *cfg);

/**
 * @brief  取当前参数
 */
void fall_detect_get_config(fall_config_t *cfg);

/**
 * @brief  喂一帧数据，推进状态机
 * @param  now_ms: 当前时间戳（ms，建议用 HAL_GetTick()）
 * @param  ax: 加速度 X（g）
 * @param  ay: 加速度 Y（g）
 * @param  az: 加速度 Z（g）
 * @param  gx: 角速度 X（dps）
 * @param  gy: 角速度 Y（dps）
 * @param  gz: 角速度 Z（dps）
 */
void fall_detect_update(uint32_t now_ms, float ax, float ay, float az,
                        float gx, float gy, float gz);

/**
 * @brief  当前状态
 */
fall_state_t fall_detect_get_state(void);

/**
 * @brief  状态名字（"IDLE" / "FREEFALL" / "IMPACT" / "SETTLED"）
 */
const char *fall_detect_state_name(fall_state_t st);

/**
 * @brief  是否确认发生了跌落（经历过自由落体 + 冲击峰值）
 * @retval 1:是  0:否
 */
uint8_t fall_detect_is_fall(void);

/**
 * @brief  是否确认发生了碰撞 / 剧烈晃动（无自由落体阶段的冲击，或角速度持续超限）
 * @retval 1:是  0:否
 */
uint8_t fall_detect_is_impact(void);

/**
 * @brief  自上次 reset 以来的加速度峰值（g）
 */
float fall_detect_get_peak_g(void);

/**
 * @brief  自上次 reset 以来的角速度峰值（dps）
 */
float fall_detect_get_peak_dps(void);

/**
 * @brief  取当前倾角（互补滤波融合加速度 + 陀螺）
 * @param  pitch_deg: 输出俯仰角（度）  @param  roll_deg: 输出横滚角（度）
 */
void fall_detect_get_attitude(float *pitch_deg, float *roll_deg);

#endif /* TASK_FALL_DETECT_H_ */
