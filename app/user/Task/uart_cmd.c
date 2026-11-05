/*
 * uart_cmd.c —— ESP8266 → APP 的串口命令解析
 *
 * 【协议】全部 ASCII 行，\n 结尾（8266 侧 ota_task.cpp 的
 * forwardThresholds / 状态行就是发这两类行）：
 *
 *   TH <tmax×100> <tmin×100> <hmax×100> <hmin×100> <fall×100> <alarm> <reportSec> [reportMs]
 *       阈值下发。全整数 ×100 —— 40K 固件里拉 float sscanf 太贵，
 *       strtol 就够；网页上填的 35.5℃ 到这里就是 3550。
 *   PB <pmax×100> <pmin×100>
 *       气压阈值。
 *   NET 0/1
 *       8266 联网状态。
 *   OA <state> <pct>
 *       OTA 进度：1=下载中 2=固件已写入 W25Q（下载完成）4=失败。
 *       收到 2 时顺便在 BKP->DR1 写 OTA 魔数：这样用户下一次按 RESET，
 *       bootloader 直接进"常驻升级"模式，会先画"升级中"再收固件；
 *       而普通 10 秒窗口里 bootloader 是不画屏的（屏幕会停在 APP 最后一帧）。
 *
 * 【OTA 触发不在这条链路上】
 *   8266 只负责"有新固件先下好、等 STM32 复位进 bootloader 时再推"。
 *   本板不接收任何"复位"命令；只把 "该升级了" 记进备份寄存器。
 *
 * 【接收方式：中断 + 环形缓冲（2026-11-03 从轮询改过来）】
 *   原来每 1ms 轮询一次 DR。8266 一整行是连着发的，只要字节间隔被
 *   WiFi 中断拉长一点，轮询循环就已经退出，后来的字节在 F1 的单字节
 *   DR 上直接溢出 —— 现场实锤：阈值回执 "9000" 变成 "900"，
 *   17 字节的触发串被打残，板子不复位，8266 干等 20 秒报 banner_timeout。
 *   改成"每来一个字节进一次中断 → 塞 256 字节环形缓冲 → 任务取干"，
 *   一个字节都不会丢。TX 侧的日志 DMA（debug_log.c）与 RX 完全独立。
 */
#include "uart_cmd.h"

#include <string.h>

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_tasks.h"
#include "debug_log.h"
#include "buzzer.h"

extern UART_HandleTypeDef huart1;

/* 与 bootloader 的约定：BKP->DR1 = 0x4F54（'OT'）→ 上电进常驻升级模式。
 * bootloader 读到后会立刻清掉，所以只生效一次。
 * 【为什么要写它】bootloader 的普通 10 秒窗口不画屏，屏幕会停在 APP
 * 最后一帧；只有常驻模式才会先画"升级中"再等固件。由 APP 在"固件已就绪"
 * 时落下这个标记，用户按 RESET 就能看到升级界面。 */
#define OTA_BKP_MAGIC   0x4F54U
#define OTA_BKP_NONE    0U

static void ota_bkp_set(uint8_t pending)
{
    static uint8_t mark = 0xFFU;   /* 上次写进去的状态，避免每 60 秒重复写 */
    uint8_t want = (pending != 0U) ? 1U : 0U;

    if (mark == want) {
        return;
    }
    /* F1 备份域默认写保护：要置 PWR_CR 的 DBP，否则写入被硬件静默丢弃 */
    PWR->CR |= PWR_CR_DBP;
    __HAL_RCC_BKP_CLK_ENABLE();
    BKP->DR1 = (want != 0U) ? OTA_BKP_MAGIC : OTA_BKP_NONE;
    mark = want;
}

#define CMD_BUF_LEN     64U

/* TL 遥测的发送周期（ms），TH 命令的 reportSec/reportMs 字段可以改它。
 * 合法范围与后台 THRESHOLD_META、8266 的 OTA_REPORT_MS_MIN/MAX 保持一致：
 * 500ms ~ 1 小时。这里只做范围保护，**不再把任何值强行改成 2 秒**。 */
#define REPORT_MS_MIN   500U
#define REPORT_MS_MAX   3600000U
static uint32_t s_report_ms = 2000;

/* 8266 的 NET 状态行不需要本板处理：联网提示由 8266 板载 LED 承担，
 * 这里不保留只写不读的标志位。 */

/* 【这里原来还有两个"传感器停更检测"的静态变量】
 * s_tl_sig / s_stale_since + SENSOR_STALE_MS 声明了却没有任何代码读写它们，
 * 编译器一直在报 defined but not used。真正的停更检测在 8266 那边
 * （esp8266_ota/src/ota_task.cpp 的 TL 指纹，超 30s 不变就发
 * EVT sensor_stale/sensor_recovered 上云），本板不需要重复做，故删除。
 * 留着只会让人以为"设备端也有一份停更检测"。 */

static char     s_line[CMD_BUF_LEN];
static uint16_t s_len = 0;

/* ---- OTA 显示状态：8266 发 "OA <state> <pct>" ----
 * APP 不参与升级流程，只把进度显示到屏幕上（实际刷写由 bootloader 完成）。 */
static volatile uint8_t  s_ota_state = APP_OTA_NONE;
static volatile uint8_t  s_ota_pct   = 0U;
static volatile uint32_t s_ota_ms    = 0U;

/* ---- 接收环形缓冲：中断里填，任务里取 ----
 * 【2026-11-03 现场实锤】原来的"每 1ms 轮询 DR"会丢字节：
 * 阈值回执里 "9000" 变成了 "900"；17 字节的触发串被打残，
 * 板子不复位，8266 干等 20 秒报 banner_timeout —— OTA 直接失败两次。
 * 1ms 一轮的轮询救不了这种"字节间隔被拉长"的丢字节，
 * 改成每来一个字节进一次中断，缓冲 256 字节，一行多长都吃得下。 */
#define RX_RING_SIZE 256U
static volatile uint8_t s_rx_buf[RX_RING_SIZE];
static volatile uint8_t s_rx_head = 0U;
static volatile uint8_t s_rx_tail = 0U;
static uint8_t s_it_byte;

/* 【不用 strtol】nano-libc 的 strtol 会拖几百字节进固件，
 * 这里手写一个十进制解析就够用（TH 行里全是普通整数）。
 * *ps 推进到数字后面；没有数字返回 0 且 *ok=0 */
static long parse_i(char **ps, int *ok)
{
    long    v   = 0;
    int     neg = 0;
    uint8_t got = 0;
    char   *p   = *ps;

    while (*p == ' ' || *p == '\t') p++;
    if (*p == '-') { neg = 1; p++; }
    while (*p >= '0' && *p <= '9') {
        v = v * 10 + (*p - '0');
        p++;
        got = 1;
    }
    *ps = p;
    if (ok != 0) *ok = got;
    return neg ? -v : v;
}

/* 阈值下发：TH <5 个 ×100 整数> <alarm 0/1> <reportSec> [reportMs]
 *
 * 【第 8 个字段是毫秒】
 *   老协议只有 reportSec（整数秒），最小 5 秒 —— 需求是"两秒就两秒，
 *   一秒、零点五秒都行"，5 秒起跳根本做不到。
 *   现在第 8 列是精确毫秒（100..60000），有它就用它；
 *   没有它就退回第 7 列的秒（但下限放宽到 1 秒）。
 *   ESP8266 侧（fw_download.cpp 的 readThresholds）按同样规则组装。
 */
static void handle_threshold(char *p)
{
    app_thresholds_t t;
    app_thresholds_t old;
    long v[8];
    int  i;
    int  ok;

    app_thresholds_get(&t);                 /* 没给的字段保持现值 */
    old = t;                                /* 用来判断这次下发的阈值是否真的变了 */

    for (i = 0; i < 8; i++) {
        v[i] = parse_i(&p, &ok);
        if (!ok) break;
    }
    if (i < 5) return;                      /* 前 5 个是必须的 */

    /* 域检查：明显不合理的值直接丢，不让一条坏行把阈值打穿 */
    if (v[0] < -5000 || v[0] > 10000) return;   /* -50.00 ~ 100.00 ℃ */
    if (v[1] < -5000 || v[1] > 10000) return;
    if (v[2] < 0     || v[2] > 10000) return;   /* 0 ~ 100.00 %RH */
    if (v[3] < 0     || v[3] > 10000) return;
    if (v[4] < 100   || v[4] > 1000)  return;   /* 灵敏度 1.0 ~ 10.0 */

    t.temp_max = (float)v[0] / 100.0f;
    t.temp_min = (float)v[1] / 100.0f;
    t.hum_max  = (float)v[2] / 100.0f;
    t.hum_min  = (float)v[3] / 100.0f;
    t.fall_g   = (float)v[4] / 100.0f;
    if (i > 5) t.alarm_enable = (v[5] != 0);

    /* 上报周期：优先用毫秒列（第 8 个），否则用秒列（第 7 个）。
     * 【这里原先把值双向夹死在 2000ms，导致后台怎么改都是 2 秒一报】
     *   现在按云端下发的值真跑：0.5 秒、1 秒、10 秒、1 小时都认。
     *   只做 500ms..3600s 的范围保护（低于 500ms 会把串口带宽打满）。 */
    if (i > 7 && v[7] > 0) {
        uint32_t ms = (uint32_t)v[7];
        if (ms < REPORT_MS_MIN) ms = REPORT_MS_MIN;
        if (ms > REPORT_MS_MAX) ms = REPORT_MS_MAX;
        t.report_ms = ms;
        s_report_ms = ms;
    } else if (i > 6 && v[6] >= 1) {
        uint32_t ms = (uint32_t)v[6] * 1000UL;
        if (ms < REPORT_MS_MIN) ms = REPORT_MS_MIN;
        if (ms > REPORT_MS_MAX) ms = REPORT_MS_MAX;
        t.report_ms = ms;
        s_report_ms = ms;
    }

    app_thresholds_set(&t);

    /* 阈值回执 THA：专门给 ESP8266 看的一行。
     * 【为什么多这一行】
     *   以前后台只能看到「我下发了阈值」，看不到「设备到底用没用」。
     *   8266 收到这行就会 POST /api/iot/event（threshold_ack），
     *   网页事件列表里出现「STM32 已确认阈值」——这才是阈值真实生效的证据。
     *   格式跟 TH 对称：THA <tmax×100> <tmin×100> <hmax×100> <hmin×100> <alarm> <report_ms> */
    dbg_printf("THA %ld %ld %ld %ld %u %lu\r\n",
               (long)(t.temp_max * 100.0f), (long)(t.temp_min * 100.0f),
               (long)(t.hum_max * 100.0f),  (long)(t.hum_min * 100.0f),
               (unsigned)t.alarm_enable, (unsigned long)s_report_ms);

    /* 只有阈值真的变了才"响一声确认"。
     * 8266 那边虽然有指纹去重，但重启后指纹可能丢；STM32 这里再兜一层，
     * 避免每分钟收到同一条 TH 就蜂鸣一次，被现场当成误报。 */
    if (old.temp_max != t.temp_max || old.temp_min != t.temp_min ||
        old.hum_max  != t.hum_max  || old.hum_min  != t.hum_min  ||
        old.fall_g   != t.fall_g   || old.alarm_enable != t.alarm_enable) {
        buzzer_beep();
    }
}

/* 气压阈值下发：PB <pmax×100> <pmin×100>（单位 hPa，范围 300..1200）
 *
 * 单独一行而不是塞进 TH —— TH 的 8 列位置早已被老固件固定解析，
 * 追加字段会让"发不出新协议"的两边对不上。单独一行只影响新固件。
 * 注意：这里**不叫蜂鸣器**，阈值没变时不该有声音。 */
static void handle_pressure(char *p)
{
    app_thresholds_t t;
    long vmax, vmin;
    int  ok;

    vmax = parse_i(&p, &ok);
    if (!ok) return;
    vmin = parse_i(&p, &ok);
    if (!ok) return;

    if (vmax < 30000 || vmax > 120000) return;   /* 300.00 ~ 1200.00 hPa */
    if (vmin < 30000 || vmin > 120000) return;

    app_thresholds_get(&t);
    t.press_max = (float)vmax / 100.0f;
    t.press_min = (float)vmin / 100.0f;
    app_thresholds_set(&t);
}

/* ---------------------------------------------------------------
 * 遥测上行：TL 行（APP → 8266 → 云）
 *
 * 【为什么走 dbg_printf】USART1 的 TX 是 debug_log 的 DMA 环形缓冲，
 * 8266 的 uart_task 会把收到的行全部看一遍：TL 开头的它拿去转发
 * （POST /api/iot/telemetry），其余当日志丢掉。一条通道两用，零新增外设。
 *
 * 【格式】TL <temp×100> <hum×100> <pressPa> <pitch×100> <roll×100> <batt> <alarm>
 * 全整数：8266 侧 strtol/手写解析，避免两边都拉浮点。
 * pitch/roll 挂在 accX/accY 字段上 —— 网页"姿态 X/Y/Z"列正好显示它们。
 * --------------------------------------------------------------- */
static void send_telemetry(void)
{
    app_state_t st;

    app_state_get(&st);
    /* 开头垫两个换行：这条线长空闲后**头 1~2 个字节**容易被采坏
     * （见 skip_line_junk 的实测说明）。垫在前面的字节就是拿来被采坏的 ——
     * 8266 的空行会被丢掉，真正的 "TL " 因此落在完好位置上。 */
    dbg_printf("\n\nTL %ld %ld %ld %ld %ld %u %u\r\n",
               (long)(st.temp_c * 100.0f),
               (long)(st.hum * 100.0f),
               (long)st.press,
               (long)(st.pitch_deg * 100.0f),
               (long)(st.roll_deg * 100.0f),
               (unsigned)st.batt,
               (unsigned)(st.alarm_bits != 0U));
}

/* 从行首最多挪开 3 个"非可打印 ASCII"字节再匹配命令。
 * 【为什么必须容忍】2026-11-01 从本板 RAM 里抓到实锤：
 *   每串数据的**头 1~2 个字节**会被采坏 —— 实测到一行
 *   `00 'T' 'H' ' ' '2' '6' '0' '0' ...`，即真命令前面多了一个 0x00。
 *   8266 那侧已经改成先垫两个换行（让坏字节去啃垫子），但垫子本身也可能
 *   被采成一个可见字符，所以这里再兜一层。
 * 【为什么必须按长度走而不是看 '\0'】收到的数据里**本来就可能含 0x00**
 *   （被采坏的首字节就是它！），用 C 字符串的方式一遇到 0 就停，
 *   等于永远卡在第一个坏字节上 —— 这就是第一版容错没生效的原因。
 *   只往后挪最多 3 字节，再多就会把普通日志行误判成命令。 */
static const char *skip_line_junk(const char *s, uint16_t len)
{
    uint16_t i = 0U;

    while ((i < len) && (i < 3U)) {
        uint8_t c = (uint8_t)s[i];

        if ((c >= 0x20U) && (c <= 0x7EU)) {
            break;      /* 落到可打印 ASCII 上就算正文开始 */
        }
        i++;            /* 垃圾字节：挪开一个再看 */
    }
    return s + i;
}

static void handle_line(void)
{
    /* 尾部可能的 \r 已经滤掉；先挪开被采坏的头几个字节，再容忍首尾空白 */
    char *s = (char *)skip_line_junk(s_line, s_len);
    while (*s == ' ' || *s == '\t') s++;

    if (strncmp(s, "TH ", 3) == 0) {
        handle_threshold(s + 3);
    } else if (strncmp(s, "PB ", 3) == 0) {
        handle_pressure(s + 3);
    } else if (strncmp(s, "NET ", 4) == 0) {
        /* 联网状态行：知道就行，不需要本地动作 */
    } else if (strncmp(s, "OA ", 3) == 0) {
        /* OA <state> <pct>：只更新屏幕状态，不做任何动作 */
        long v[2];
        int  i;
        int  ok;

        s += 3;
        for (i = 0; i < 2; i++) {
            v[i] = parse_i(&s, &ok);
            if (!ok) break;
        }
        if (i < 2) return;
        if (v[0] != APP_OTA_DOWNLOAD && v[0] != APP_OTA_UPGRADE &&
            v[0] != APP_OTA_FAILED && v[0] != APP_OTA_NONE) return;

        s_ota_pct = (v[1] <= 0) ? 0U : ((v[1] >= 100) ? 100U : (uint8_t)v[1]);
        s_ota_ms  = HAL_GetTick();
        s_ota_state = (uint8_t)v[0];

        /* 固件已就绪 → 落下 BKP 标记：下次按 RESET 直接进 bootloader 的
         * 常驻升级模式（会先画"升级中"）。其它状态就清掉。 */
        ota_bkp_set((v[0] == APP_OTA_UPGRADE) ? 1U : 0U);
    }
    /* 其它行当噪声丢掉（PA9 上还有 8266 意外漏过来的字节） */
}

void app_ota_ui_get(app_ota_ui_t *out)
{
    if (out == NULL) {
        return;
    }
    out->state      = s_ota_state;
    out->pct        = s_ota_pct;
    out->updated_ms = s_ota_ms;
}

void uart_cmd_task(void *argument)
{
    uint32_t last_tl  = 0;
    uint32_t now;

    (void)argument;

    for (;;) {
        now = xTaskGetTickCount();

        /* 遥测：按阈值里的上报周期发 TL 行（默认 2s，云端网页就"实时"了） */
        if ((now - last_tl) >= pdMS_TO_TICKS(s_report_ms)) {
            last_tl = now;
            send_telemetry();
        }

        /* ---- 收：中断已经把字节塞进环形缓冲，这里一次性取干 ----
         * 一个字节都不丢：中断是"每来一个就进一次"，不存在 1ms 轮询
         * 那段空窗期。取干后让出 CPU，别人（传感器/OLED）照样能跑。 */
        while (s_rx_tail != s_rx_head) {
            uint8_t c = s_rx_buf[s_rx_tail];

            s_rx_tail = (uint8_t)((s_rx_tail + 1U) & (RX_RING_SIZE - 1U));
            if (c == '\r') {
                /* 忽略 */
            } else if (c == '\n') {
                s_line[s_len] = '\0';
                if (s_len > 0) {
                    handle_line();
                }
                s_len = 0U;
            } else if (s_len < CMD_BUF_LEN - 1U) {
                s_line[s_len++] = (char)c;
            } else {
                s_len = 0U;                 /* 超长：丢整行，防错位 */
            }
        }

        /* 无条件让出 CPU：没数据时也保证 1ms 一轮，别人能跑 */
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void uart_cmd_init(void)
{
    TaskHandle_t h = NULL;

    /* 【先把中断接收挂上去】必须赶在任何一行到达之前，否则最先到的
     * 触发串/阈值行会进不了缓冲（8266 一开机就会发 TH）。 */
    (void)HAL_UART_Receive_IT(&huart1, &s_it_byte, 1U);

    /* 栈 160 words。创建失败必须报出来：这个任务负责「收 TH 阈值」和
     * 「发 TL 遥测」，它一死，云端就完全看不到这台设备（现象是设备在线但没数据）。 */
    if (xTaskCreate(uart_cmd_task, "ucmd", 160, NULL, 2, &h) != pdPASS) {
        dbg_printf("[BOOT] ucmd task FAILED (heap %u B)\r\n",
                   (unsigned)xPortGetFreeHeapSize());
    }
}

/* ---- USART1 接收完成回调：一个字节一进，绝不丢 ----
 * 中断里只做"塞缓冲 + 重新挂接收"，不调用任何 FreeRTOS API（ISR 优先级
 * 低于 configMAX_SYSCALL_INTERRUPT_PRIORITY，调了也不合法）。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uint8_t next = (uint8_t)((s_rx_head + 1U) & (RX_RING_SIZE - 1U));

        if (next != s_rx_tail) {            /* 满了就丢最新，宁可整行丢也不错位 */
            s_rx_buf[s_rx_head] = s_it_byte;
            s_rx_head = next;
        }
        (void)HAL_UART_Receive_IT(&huart1, &s_it_byte, 1U);
    }
}

/* ---- 错误回调：ORE（溢出）后必须把接收重新挂上 ----
 * 中断方式下 ORE 只可能在"重新挂接收"的那几十个周期里发生，
 * 概率极低；但一旦发生 HAL 会停掉接收，不重挂就永久收不到命令了。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        huart->RxState = HAL_UART_STATE_READY;
        (void)HAL_UART_Receive_IT(&huart1, &s_it_byte, 1U);
    }
}
