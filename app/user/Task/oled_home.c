/*
 * oled_home.c
 *
 * 主页：显示传感器信息 + 运行时长（不带按键交互）。
 *
 * 版面（128x64，12 号字）：
 *     传感器
 *     ─────────────────
 *     温度        25.3 C
 *     湿度        61.2 %
 *     气压     1013.2 hPa
 *     运行      00:12:34
 *
 * 动画（引擎见 oled_anim.c，曲线与 KK_UI 一致）：
 *   1. 入场      整屏内容从右侧平滑滑入       300ms
 *   2. 数值滚动  传感器数值平滑滚到新值        100ms
 */
#include "oled_home.h"

#include "ssd1306.h"
#include "oled_anim.h"
#include "sensor_task.h"
#include "uart_cmd.h"
#include "debug_log.h"
#include "task_monitor.h"
#include "app_version.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

/* ==================== 低功耗：OLED 分时点亮 ==================== */
/* OLED 点亮时 ~20mA，是整块板最大的耗电项；关掉只有 ~10µA。
 * 1 = 长时间没变化就关屏（现场用）；改成 0 = 屏幕常亮（开发调试用）。 */
#define OLED_AUTO_OFF        0                            /* 0 = 常亮（不再自动关屏） */
#define OLED_ON_AFTER_MS     5000U    /* 有变化后亮多久 */
#define OLED_WAKE_TEMP_X10   2        /* 温度变化 ≥0.2℃ 才算「有变化」 */
#define OLED_WAKE_HUM_X10    5        /* 湿度变化 ≥0.5%RH */
#define OLED_WAKE_PRESS_X10  10       /* 气压变化 ≥1hPa（0.1hPa 为单位） */

/* ==================== 主页显示的信息 ==================== */
static int32_t g_temp_x10;    /* 温度，单位 0.1℃ */
static int32_t g_hum_x10;     /* 湿度，单位 0.1% */
static int32_t g_press_x10;   /* 气压，单位 0.1hPa */
static int32_t g_uptime_s;    /* 运行时长，秒 */

typedef struct {
    const char *label;
    const char *unit;
    int32_t    *value;
    /* 该传感器最近一次采样是否成功。0 时这一行显示 "ERR"，
     * 而不是继续显示上一次的旧数值 —— 拔掉传感器后旧数值永远不变，
     * 看起来就像"屏幕不刷新"。 */
    const volatile uint8_t *ok;
} home_item_t;

static home_item_t g_items[] = {
    { "温度", "C",   &g_temp_x10,  &g_sensor.aht_ok },
    { "湿度", "%",   &g_hum_x10,   &g_sensor.aht_ok },
    { "气压", "hPa", &g_press_x10, &g_sensor.bmp_ok },
};
#define ITEM_ERR_STR "ERR"
#define ITEM_COUNT (sizeof(g_items) / sizeof(g_items[0]))

/* ---- 首页固定位置的版本标签 ----
 * 128x64 屏放 12 号字最多 5 行，数值 3 行 + 运行时间 1 行已经排满，
 * 所以版本号放在标题行右侧（"传感器" 左边是标题，右边是版本号）。
 * 版本字符串是编译期常量，放在 Flash 里，不占 RAM。 */
#define VERSION_X_RIGHT 126
static const char *const g_version_str = "Ver:" APP_VERSION_STRING;

/* ==================== 布局 ==================== */
#define TITLE_Y     0
#define SEP_Y       13
#define ROW0_Y      15
#define ROW_STEP    12
#define ROW_Y(i)    (ROW0_Y + (i) * ROW_STEP)   /* 15 / 27 / 39 */
#define TIME_ROW_Y  (ROW0_Y + ITEM_COUNT * ROW_STEP)   /* 51 */
#define LABEL_X     6
#define VALUE_RIGHT 126                          /* 数值右对齐到的 x */

#define FRAME_MS    33U                          /* 33ms 一帧 ≈ 30 FPS */

/* 数值滚动时长。原来是 100ms：40ms 一帧时只有 2~3 帧，肉眼就是"跳一下"。
 * 拉长到 300ms 后同样 30 FPS 能有 ~9 个中间帧，滚动的连续感明显更好。 */
#define OLED_HOME_VALUE_MS  300U

/* ==================== 动画状态 ==================== */
static oled_anim_t anim_enter;                    /* 入场横向偏移：128 -> 0 */
static oled_anim_t anim_value[ITEM_COUNT];        /* 三个传感器数值 */

/* 当前要显示的 OTA 状态（uart_cmd 从 8266 的 OA 行更新） */
static app_ota_ui_t g_ota;
/* "固件已写入 W25Q"先显示一会儿，再切换成"下载完成" */
#define OTA_WRITTEN_MS   4000U
static uint8_t  g_ota_stage;    /* 0 = 已写入W25Q，1 = 下载完成 */

void oled_home_init(uint32_t now)
{
    uint8_t i;

    oled_anim_set(&anim_enter, 128 * OLED_Q8_ONE, now);
    oled_anim_to(&anim_enter, 0, OLED_ANIM_ENTER_MS, now);

    for (i = 0U; i < ITEM_COUNT; ++i) {
        oled_anim_set(&anim_value[i], 0, now);
    }
    g_temp_x10 = 0;
    g_hum_x10 = 0;
    g_press_x10 = 0;
    g_uptime_s = 0;
}

/** 把 0.1 单位的定点数格式化成 "25.3 单位" */
static void fmt_fixed1(char *buf, uint16_t size, int32_t value_x10, const char *unit)
{
    long integer_part = (long)(value_x10 / 10);
    long frac_part = (long)(value_x10 % 10);

    if (frac_part < 0) {
        frac_part = -frac_part;
    }
    snprintf(buf, size, "%ld.%ld %s", integer_part, frac_part, unit);
}

/** 运行时长格式化成 时:分:秒 */
static void fmt_uptime(char *buf, uint16_t size, int32_t seconds)
{
    long s = (long)seconds;

    snprintf(buf, size, "%02ld:%02ld:%02ld",
             s / 3600L, (s / 60L) % 60L, s % 60L);
}

void oled_home_draw(uint32_t now)
{
    int32_t enter_off = oled_anim_value(&anim_enter, now);   /* 入场偏移，画完就是 0 */
    uint8_t i;
    uint16_t w;
    char buf[24];

    oled_clear();
    oled_set_font(u8g2_font_custom12);

    /* 标题 + 分隔线 */
    oled_print((uint8_t)enter_off, TITLE_Y, "传感器");
    u8g2_DrawHLine(&ssd1306, (u8g2_uint_t)enter_off, SEP_Y, 128);

    /* 版本号：固定在标题行右侧（跟着入场动画一起滑入） */
    w = oled_text_width(g_version_str);
    oled_print((uint8_t)(enter_off + VERSION_X_RIGHT - w), (uint8_t)TITLE_Y, g_version_str);

    /* 三行传感器数值（右对齐，数值平滑滚动；掉件的那一行显示 ERR） */
    for (i = 0U; i < ITEM_COUNT; ++i) {
        int32_t v = oled_anim_value(&anim_value[i], now);

        oled_print((uint8_t)(enter_off + LABEL_X), (uint8_t)ROW_Y(i), g_items[i].label);
        if ((g_items[i].ok != NULL) && (*g_items[i].ok == 0U)) {
            /* 传感器不在线：明确报错，别拿旧值冒充实时值 */
            w = oled_text_width(ITEM_ERR_STR);
            oled_print((uint8_t)(enter_off + VALUE_RIGHT - w), (uint8_t)ROW_Y(i), ITEM_ERR_STR);
        } else {
            fmt_fixed1(buf, sizeof(buf), v, g_items[i].unit);
            w = oled_text_width(buf);
            oled_print((uint8_t)(enter_off + VALUE_RIGHT - w), (uint8_t)ROW_Y(i), buf);
        }
    }

    /* 最后一行：有 OTA 任务时显示状态，否则显示运行时长。
     * 升级期间 STM32 会复位进 bootloader（那一刻屏幕由 bootloader 掌管，
     * 显示"升级中"+进度条），所以这里只需要覆盖"下载中/已就绪/失败"。 */
    if (g_ota.state == APP_OTA_DOWNLOAD) {
        snprintf(buf, sizeof(buf), "下载中 %u%%", (unsigned)g_ota.pct);
        oled_print((uint8_t)(enter_off + LABEL_X), (uint8_t)TIME_ROW_Y, buf);
    } else if (g_ota.state == APP_OTA_UPGRADE) {
        /* 下载完成分两段：先确认"已写入 W25Q"，几秒后再提示"下载完成" */
        oled_print((uint8_t)(enter_off + LABEL_X), (uint8_t)TIME_ROW_Y,
                   (g_ota_stage != 0U) ? "下载完成" : "已写入W25Q");
    } else if (g_ota.state == APP_OTA_FAILED) {
        oled_print((uint8_t)(enter_off + LABEL_X), (uint8_t)TIME_ROW_Y, "升级失败");
    } else {
        fmt_uptime(buf, sizeof(buf), g_uptime_s);
        oled_print((uint8_t)(enter_off + LABEL_X), (uint8_t)TIME_ROW_Y, "运行");
        w = oled_text_width(buf);
        oled_print((uint8_t)(enter_off + VALUE_RIGHT - w), (uint8_t)TIME_ROW_Y, buf);
    }

    oled_flush();
}

void oled_task(void *pvParameters)
{
    uint8_t i;
    uint32_t now = HAL_GetTick();

    uint32_t last_uptime = 0xFFFFFFFFU;
    uint8_t need_redraw;
    uint8_t force_redraw;
    uint8_t wake_redraw = 0U;
    /* 上一次的 OTA 状态：状态/百分比一变就立刻重画（不用等 1 秒跳秒） */
    uint8_t last_ota_state = 0xFFU;
    uint8_t last_ota_pct   = 0xFFU;
    uint32_t ota2_since    = now;   /* 进入"固件已就绪"的时刻 */
    /* 传感器在线状态：任何一路从「有数」变成「掉线」（或反过来）都强制重画一帧，
     * 这样插拔传感器时屏幕会在 ≤1 个刷新周期内切到 ERR / 恢复数值。 */
    uint8_t last_ok[ITEM_COUNT] = { 0xFFU, 0xFFU, 0xFFU };
    /* 低功耗：关屏/亮屏的状态机 */
    uint8_t  screen_on = 1U;
    uint32_t screen_on_until = 0U;
    int32_t  last_temp_x10 = 0x7FFFFFFF;
    int32_t  last_hum_x10  = 0x7FFFFFFF;
    int32_t  last_press_x10 = 0x7FFFFFFF;

    (void)pvParameters;

    oled_home_init(now);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        int32_t d;

        now = HAL_GetTick();

        /* 传感器值换成 0.1 单位的定点数（动画引擎用整数） */
        g_temp_x10  = (int32_t)(g_sensor.temperature * 10.0f);
        g_hum_x10   = (int32_t)(g_sensor.humidity * 10.0f);
        g_press_x10 = (int32_t)(g_sensor.pressure / 10.0f);   /* Pa -> 0.1hPa */
        g_uptime_s  = (int32_t)(now / 1000U);

        /* ---- 低功耗： OLED 分时点亮 ----
         * OLED 点亮 ~20mA，是整块板最大的耗电项；关掉只有 ~10µA。
         * 规则：传感器值有明显变化 → 点亮 OLED_ON_AFTER_MS；一直不变 → 关屏。
         * 运行时间每跳一秒不算「变化」，否则屏幕永远关不掉。
         */
        {
            uint8_t changed = 0U;

            if (last_temp_x10 == 0x7FFFFFFF) {
                changed = 1U;                       /* 第一次 */
            } else {
                d = g_temp_x10 - last_temp_x10;
                if ((d >= OLED_WAKE_TEMP_X10) || (d <= -OLED_WAKE_TEMP_X10)) { changed = 1U; }
                d = g_hum_x10 - last_hum_x10;
                if ((d >= OLED_WAKE_HUM_X10) || (d <= -OLED_WAKE_HUM_X10)) { changed = 1U; }
                d = g_press_x10 - last_press_x10;
                if ((d >= OLED_WAKE_PRESS_X10) || (d <= -OLED_WAKE_PRESS_X10)) { changed = 1U; }
            }
            if (changed != 0U) {
                last_temp_x10   = g_temp_x10;
                last_hum_x10    = g_hum_x10;
                last_press_x10  = g_press_x10;
                if (screen_on == 0U) {
                    screen_on = 1U;
                    oled_display_on();
                    wake_redraw = 1U;              /* 亮屏要重画一帧 */
                }
                screen_on_until = now + OLED_ON_AFTER_MS;
            }
        }

        /* 目标值变了就自动启动平滑滚动；掉线的那一路不动画（显示的是 ERR，
         * 数值动画保留在最后位置，等传感器回来再从这里滚过去）。 */
        force_redraw = 0U;
        for (i = 0U; i < ITEM_COUNT; ++i) {
            uint8_t ok = (g_items[i].ok != NULL) ? *g_items[i].ok : 1U;

            if (ok != last_ok[i]) {
                last_ok[i] = ok;
                force_redraw = 1U;
            }
            if (ok == 0U) {
                continue;
            }
            oled_anim_to(&anim_value[i], *g_items[i].value * OLED_Q8_ONE,
                         OLED_HOME_VALUE_MS, now);
        }

        /* ---- OTA 状态：变了就强制重画这一帧 ---- */
        app_ota_ui_get(&g_ota);
        if ((g_ota.updated_ms != 0U) &&
            ((now - g_ota.updated_ms) >= APP_OTA_STALE_MS)) {
            g_ota.state = APP_OTA_NONE;   /* 8266 长时间没更新：回退到运行时长 */
        }
        if ((g_ota.state != last_ota_state) || (g_ota.pct != last_ota_pct)) {
            if (g_ota.state == APP_OTA_UPGRADE) {
                ota2_since = now;         /* 重新计时"已写入 W25Q" */
            }
            last_ota_state = g_ota.state;
            last_ota_pct   = g_ota.pct;
            force_redraw   = 1U;
        }
        /* 已写入 W25Q 满 4 秒 → 换成"下载完成" */
        {
            uint8_t stage = 0U;
            if ((g_ota.state == APP_OTA_UPGRADE) &&
                ((now - ota2_since) >= OTA_WRITTEN_MS)) {
                stage = 1U;
            }
            if (stage != g_ota_stage) {
                g_ota_stage = stage;
                force_redraw = 1U;
            }
        }

        /* 按需重画：只在动画进行中或运行时长跳秒时才刷屏。
         * 空闲时约 1 FPS（原来固定 25 FPS，I2C 总线白占 57%）
         * ⚠ 屏幕关着的时候 **一个字节都不往 I2C 发** —— 这才是省电的关键，
         *   只发一条 0xAE 而还继续刷缓冲的话，I2C 电流省不掉。 */
        need_redraw = (g_uptime_s != (int32_t)last_uptime) ? 1U : 0U;
        /* 掉件/恢复、亮屏这两种事件必须立刻重画，不能被上面的跳秒判断覆盖 */
        if ((force_redraw != 0U) || (wake_redraw != 0U)) {
            need_redraw = 1U;
            force_redraw = 0U;
            wake_redraw  = 0U;
        }
        if (oled_anim_running(&anim_enter, now) != 0U) {
            need_redraw = 1U;
        }
        for (i = 0U; i < ITEM_COUNT; ++i) {
            if (oled_anim_running(&anim_value[i], now) != 0U) {
                need_redraw = 1U;
            }
        }

#if OLED_AUTO_OFF
        /* 亮着但一直没变化 → 关屏 */
        if ((screen_on != 0U) && (now >= screen_on_until)) {
            screen_on = 0U;
            oled_display_off();
            need_redraw = 0U;          /* 关屏这一刻不重画 */
        }
#else
        (void)screen_on;
        (void)screen_on_until;
#endif

        if (need_redraw != 0U) {
            last_uptime = (uint32_t)g_uptime_s;
            if (screen_on != 0U) {
                oled_home_draw(now);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(FRAME_MS));
    }
}
