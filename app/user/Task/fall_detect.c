/*
 * fall_detect.c  ——  跌落 / 碰撞检测算法
 *
 *  Created on: Oct 24, 2026
 *      Author: Administrator
 *
 * 纯计算模块：不碰硬件、不用 HAL、不用 FreeRTOS，只吃 (加速度 g, 角速度 dps, 时间戳)。
 * 放在 fall_detect.h 里的参数都能调，判据说明见头文件。
 *
 * 状态机：
 *   IDLE ──加速度小──> FREEFALL ──冲击峰值>2.5g──> IMPACT ──静止──> SETTLED
 *     │                   │                          ↑
 *     └──冲击/剧烈晃动────┴──────────────────────────┘
 *
 * 数学：
 *   - 模长用 __builtin_sqrtf（newlib-nano 里有，单精度版本，很小）
 *   - atan2 自己写（newlib-nano **没有** atan2f，带进去会拖进一大坨 libm）
 *   - atan(x) 用 5 阶多项式近似（|x| ≤ 1，最大误差 ~1e-5 rad），再做象限归约
 */
#include "fall_detect.h"

/* ===================== 默认参数 ===================== */
#define FD_DEF_FREEFALL_G        0.35f
#define FD_DEF_FREEFALL_MS       120U
#define FD_DEF_FREEFALL_MAX_MS   1500U
#define FD_DEF_IMPACT_WINDOW_MS  300U
#define FD_DEF_IMPACT_G          2.5f
#define FD_DEF_GYRO_DPS          220.0f
#define FD_DEF_GYRO_MS           300U
#define FD_DEF_SETTLE_G          0.4f
#define FD_DEF_SETTLE_DPS        60.0f
#define FD_DEF_SETTLE_MS         800U
#define FD_DEF_IMPACT_MAX_MS     3000U
#define FD_DEF_GYRO_ALPHA        0.98f

#define FD_RAD2DEG               57.2957795f

/* ===================== 内部状态 ===================== */
static fall_config_t s_cfg;
static fall_state_t  s_state;

static uint8_t  s_fall;        /* 已确认跌落 */
static uint8_t  s_impact;      /* 已确认碰撞/剧烈晃动 */
static uint8_t  s_ff_ready;    /* 自由落体时长已经达标（确认是"摔"的前置条件） */

static uint32_t s_ff_ms;       /* 本次连续自由落体的时长 */
static uint32_t s_t_ff_last_low;/* 最后一次加速度还在自由落体阈值以下的时刻 */
static uint32_t s_shake_ms;    /* 角速度超限累计时长 */
static uint32_t s_t_impact;    /* 进入 IMPACT 的时刻 */

static uint32_t s_last_ms;     /* 上一帧的时间戳 */
static uint8_t  s_have_last;   /* 有没有上一帧 */

static float s_peak_g;         /* 自上次 reset 以来的加速度峰值 */
static float s_peak_dps;       /* 自上次 reset 以来的角速度峰值 */
static float s_pitch;          /* 俯仰角（度） */
static float s_roll;           /* 横滚角（度） */

/* ===================== 小工具 ===================== */

/** @brief 绝对值（不用 libm） */
static float fd_absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

/**
 * @brief  sqrtf 自算，不依赖 libm
 * 【为什么要自己写】Cortex-M3 没有硬件 FPU，__builtin_sqrtf 会落到
 * newlib 的 __ieee754_sqrtf（276 字节）。这里用"位 hacks 出初值 +
 * 两次牛顿迭代"，相对精度 ~1e-4，对姿态角显示绰绰有余，
 * 省下来的 Flash 够 uart_cmd 任务安家。
 */
static float fd_sqrtf(float x)
{
    union { float f; int32_t i; } u;

    if (x <= 0.0f) {
        return 0.0f;
    }
    u.f = x;
    u.i = (1 << 29) + (u.i >> 1) - (1 << 22);   /* fast sqrt 变体出初值 */
    u.f = 0.5f * (u.f + x / u.f);               /* 牛顿迭代两轮 */
    u.f = 0.5f * (u.f + x / u.f);
    return u.f;
}

/**
 * @brief  atan(x) 近似，|x| <= 1
 *        5 阶多项式，atan(1) 算出来 0.78541，和 pi/4 只差 1e-5 rad
 */
static float fd_atan1(float x)
{
    float x2 = x * x;

    return x * (0.9998660f + x2 * (-0.3302995f + x2 * (0.1801410f +
              x2 * (-0.0851330f + x2 * 0.0208351f))));
}

/**
 * @brief  atan2(y, x)，自己算，不依赖 libm 的 atan2f
 * @param  y: y  @param  x: x
 * @retval 角度（弧度，范围 -pi ~ pi]
 * @note   做法：先用 |y|/|x| 把角度归约到 [0, pi/2]，再按象限翻。
 *         归约时**只用绝对值**，符号统一在最后处理，免得象限符号搞反。
 */
static float fd_atan2f(float y, float x)
{
    const float PI      = 3.14159265f;
    const float HALF_PI = 1.57079633f;
    float ay = fd_absf(y);
    float ax = fd_absf(x);
    float a;

    if (ax > 1e-6f) {
        if (ay <= ax) {
            a = fd_atan1(ay / ax);                  /* [0, pi/4] */
        } else {
            a = HALF_PI - fd_atan1(ax / ay);        /* (pi/4, pi/2] */
        }
    } else {
        a = (ay > 1e-6f) ? HALF_PI : 0.0f;
    }

    /* x < 0 -> 翻到第二象限 [pi/2, pi] */
    if (x < 0.0f) {
        a = PI - a;
    }
    /* y < 0 -> 翻到下半平面 */
    if (y < 0.0f) {
        a = -a;
    }
    return a;
}

/**
 * @brief  角度归一化到 (-180, 180]，防止长时间积分跑飞
 * @note   用取模而不是 while 减法：万一陀螺读数异常大，循环次数也是可控的
 */
static float fd_wrap180(float deg)
{
    deg = deg - 360.0f * (float)((int32_t)((deg + 180.0f) / 360.0f));
    if (deg > 180.0f) {
        deg -= 360.0f;
    }
    if (deg <= -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

/**
 * @brief  姿态更新：加速度算倾角 + 陀螺积分，再用互补滤波融合
 * @param  ax: 加速度 X（g）  @param  ay: 加速度 Y（g）  @param  az: 加速度 Z（g）
 * @param  gx: 角速度 X（dps） @param  gy: 角速度 Y（dps）
 * @param  dt: 距上一帧的间隔（秒）
 */
static void fd_update_attitude(float ax, float ay, float az,
                               float gx, float gy, float dt)
{
    float alpha = s_cfg.gyro_alpha;
    float pitch_acc;
    float roll_acc;

    /* 加速度法：对重力敏感，静止时准，但一动就漂 */
    pitch_acc = fd_atan2f(-ax, fd_sqrtf(ay * ay + az * az)) * FD_RAD2DEG;
    roll_acc  = fd_atan2f(ay, az) * FD_RAD2DEG;

    /* 陀螺积分：短时间内准，但会积分漂移 */
    s_pitch += gx * dt;
    s_roll  += gy * dt;

    /* 互补滤波：陀螺占大头，加速度负责长期拉回来 */
    s_pitch = alpha * s_pitch + (1.0f - alpha) * pitch_acc;
    s_roll  = alpha * s_roll  + (1.0f - alpha) * roll_acc;

    s_pitch = fd_wrap180(s_pitch);
    s_roll  = fd_wrap180(s_roll);
}

/* ===================== 对外 API ===================== */

/**
 * @brief  初始化：装默认参数 + 清空状态
 */
void fall_detect_init(void)
{
    s_cfg.freefall_g        = FD_DEF_FREEFALL_G;
    s_cfg.freefall_ms       = FD_DEF_FREEFALL_MS;
    s_cfg.freefall_max_ms   = FD_DEF_FREEFALL_MAX_MS;
    s_cfg.impact_window_ms  = FD_DEF_IMPACT_WINDOW_MS;
    s_cfg.impact_g          = FD_DEF_IMPACT_G;
    s_cfg.gyro_dps          = FD_DEF_GYRO_DPS;
    s_cfg.gyro_ms           = FD_DEF_GYRO_MS;
    s_cfg.settle_g          = FD_DEF_SETTLE_G;
    s_cfg.settle_dps        = FD_DEF_SETTLE_DPS;
    s_cfg.settle_ms         = FD_DEF_SETTLE_MS;
    s_cfg.impact_max_ms     = FD_DEF_IMPACT_MAX_MS;
    s_cfg.gyro_alpha        = FD_DEF_GYRO_ALPHA;

    fall_detect_reset();
}

/**
 * @brief  清空状态和标志，回到 IDLE
 */
void fall_detect_reset(void)
{
    s_state          = FALL_STATE_IDLE;
    s_fall           = 0U;
    s_impact         = 0U;
    s_ff_ready       = 0U;
    s_ff_ms          = 0U;
    s_t_ff_last_low  = 0U;
    s_shake_ms       = 0U;
    s_t_impact       = 0U;
    s_last_ms        = 0U;
    s_have_last      = 0U;
    s_peak_g         = 0.0f;
    s_peak_dps       = 0.0f;
    s_pitch          = 0.0f;
    s_roll           = 0.0f;
}

/**
 * @brief  覆盖判据参数
 */
void fall_detect_set_config(const fall_config_t *cfg)
{
    if (cfg == 0) {
        return;
    }
    s_cfg = *cfg;
}

/**
 * @brief  取当前参数
 */
void fall_detect_get_config(fall_config_t *cfg)
{
    if (cfg == 0) {
        return;
    }
    *cfg = s_cfg;
}

/**
 * @brief  喂一帧数据，推进状态机
 */
void fall_detect_update(uint32_t now_ms, float ax, float ay, float az,
                        float gx, float gy, float gz)
{
    float a_mag = fd_sqrtf(ax * ax + ay * ay + az * az);
    float g_mag = fd_sqrtf(gx * gx + gy * gy + gz * gz);
    uint32_t dt_ms = 0U;
    float dt = 0.0f;

    /* 帧间隔 */
    if (s_have_last != 0U) {
        dt_ms = now_ms - s_last_ms;
        if (dt_ms > 1000U) {
            dt_ms = 1000U;      /* 太久没喂数据，别让积分跑飞 */
        }
    }
    s_last_ms   = now_ms;
    s_have_last = 1U;
    dt = (float)dt_ms * 0.001f;

    /* 峰值记录（自上次 reset 以来） */
    if (a_mag > s_peak_g) {
        s_peak_g = a_mag;
    }
    if (g_mag > s_peak_dps) {
        s_peak_dps = g_mag;
    }

    /* 姿态一直更新（不管在哪个状态） */
    fd_update_attitude(ax, ay, az, gx, gy, dt);

    switch (s_state) {
    /* ---------------- IDLE ---------------- */
    case FALL_STATE_IDLE:
        if (a_mag < s_cfg.freefall_g) {
            /* 开始自由落体 */
            s_state       = FALL_STATE_FREEFALL;
            s_ff_ms       = 0U;
            s_ff_ready    = 0U;
            s_t_ff_last_low = now_ms;
            s_shake_ms    = 0U;
        } else if (a_mag > s_cfg.impact_g) {
            /* 没有自由落体阶段的直接冲击 = 碰撞 */
            s_impact   = 1U;
            s_state    = FALL_STATE_IMPACT;
            s_t_impact = now_ms;
        } else if (g_mag > s_cfg.gyro_dps) {
            /* 角速度超限，累计时间够了就是「被猛晃/碰撞」 */
            s_shake_ms += dt_ms;
            if (s_shake_ms >= s_cfg.gyro_ms) {
                s_impact   = 1U;
                s_state    = FALL_STATE_IMPACT;
                s_t_impact = now_ms;
            }
        } else {
            s_shake_ms = 0U;
        }
        break;

    /* ---------------- FREEFALL ---------------- */
    case FALL_STATE_FREEFALL:
        if (a_mag < s_cfg.freefall_g) {
            /* 还在自由落体 */
            s_ff_ms += dt_ms;
            s_t_ff_last_low = now_ms;
            if (s_ff_ms >= s_cfg.freefall_ms) {
                s_ff_ready = 1U;     /* 自由落体时长达标 */
            }
        } else if (a_mag > s_cfg.impact_g) {
            /* 冲击峰值出现：有过自由落体 -> 跌落；没有 -> 只是碰撞 */
            if (s_ff_ready != 0U) {
                s_fall = 1U;
            } else {
                s_impact = 1U;
            }
            s_state    = FALL_STATE_IMPACT;
            s_t_impact = now_ms;
        } else {
            /* 已经离开自由落体但还没到冲击峰值：
             * 可能正处在落地过程中（加速度从 0 往上冲），
             * 等一等；超过窗口还没冲击就是被捡起来了，不算摔 */
            if ((now_ms - s_t_ff_last_low) > s_cfg.impact_window_ms) {
                s_state    = FALL_STATE_IDLE;
                s_ff_ready = 0U;
                s_ff_ms    = 0U;
            }
        }

        /* 一直悬空（自由落体超时） */
        if ((s_state == FALL_STATE_FREEFALL) && (s_ff_ms > s_cfg.freefall_max_ms)) {
            s_state    = FALL_STATE_IDLE;
            s_ff_ready = 0U;
            s_ff_ms    = 0U;
        }
        break;

    /* ---------------- IMPACT ---------------- */
    case FALL_STATE_IMPACT:
        if ((now_ms - s_t_impact) >= s_cfg.settle_ms) {
            /* 冲击之后已经静止下来：加速度回到 1g 附近、角速度很小 */
            float dev = a_mag - 1.0f;
            if (dev < 0.0f) {
                dev = -dev;
            }
            if ((dev <= s_cfg.settle_g) && (g_mag < s_cfg.settle_dps)) {
                s_state = FALL_STATE_SETTLED;
            }
        }
        /* 冲击后太久还没静止（还在滚/还在弹），也放行到 SETTLED，
         * 由上位机结合 fall_detect_is_fall()/is_impact() 判断 */
        if ((s_state == FALL_STATE_IMPACT) &&
            ((now_ms - s_t_impact) > s_cfg.impact_max_ms)) {
            s_state = FALL_STATE_SETTLED;
        }
        break;

    /* ---------------- SETTLED ---------------- */
    case FALL_STATE_SETTLED:
    default:
        /* 标志保持不变，等上位机调 fall_detect_reset() 才清 */
        break;
    }
}

/**
 * @brief  当前状态
 */
fall_state_t fall_detect_get_state(void)
{
    return s_state;
}

/**
 * @brief  状态名字
 */
const char *fall_detect_state_name(fall_state_t st)
{
    switch (st) {
    case FALL_STATE_IDLE:
        return "IDLE";
    case FALL_STATE_FREEFALL:
        return "FREEFALL";
    case FALL_STATE_IMPACT:
        return "IMPACT";
    case FALL_STATE_SETTLED:
        return "SETTLED";
    default:
        return "?";
    }
}

/**
 * @brief  是否确认跌落
 */
uint8_t fall_detect_is_fall(void)
{
    return s_fall;
}

/**
 * @brief  是否确认碰撞 / 剧烈晃动
 */
uint8_t fall_detect_is_impact(void)
{
    return s_impact;
}

/**
 * @brief  加速度峰值（g）
 */
float fall_detect_get_peak_g(void)
{
    return s_peak_g;
}

/**
 * @brief  角速度峰值（dps）
 */
float fall_detect_get_peak_dps(void)
{
    return s_peak_dps;
}

/**
 * @brief  取倾角
 */
void fall_detect_get_attitude(float *pitch_deg, float *roll_deg)
{
    if (pitch_deg != 0) {
        *pitch_deg = s_pitch;
    }
    if (roll_deg != 0) {
        *roll_deg = s_roll;
    }
}
