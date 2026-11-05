/*
 * uart_task.cpp —— 只做串口收发，把整行交给上层
 *
 * 【这个任务的三条铁律】
 *   1) 横幅之前收到的行一律丢掉。
 *      STM32 的 app 运行时，调试日志也从 PA9 打出来，
 *      bootloader 用的却是同一条线。不分清楚就会把日志当成协议应答。
 *      只有看到 "BL v..." 横幅，才说明对面已经换成 bootloader 了。
 *   2) 横幅之前绝对不发任何东西。
 *      发出去的 ISP 命令（? / O）要是撞进 app 的日志流里，
 *      会把 app 的命令行搞乱。所以"没进 ISP 模式"时，TX 一个字节都不发。
 *   3) 触发串是个例外。
 *      STM32 app 拿到云端 OTA 指令后会先发一行触发串再复位进 bootloader，
 *      所以触发串一定出现在横幅之前。它是应用层命令，不是 ISP 流量，
 *      认出来之后只是"通知一下 ota_task"，不往串口回任何东西。
 */
#include <Arduino.h>

#include "app_ctx.h"

/* 最新一条 TL 遥测行（见 app_ctx.h 说明） */
char g_lastTelemetry[128] = {0};
/* 最后一次收到 TL / THA 的时刻（millis），见 app_ctx.h 说明 */
uint32_t g_lastTlMs = 0;
uint32_t g_lastThAckMs = 0;
#include "config.h"

/* 行首匹配（只看前缀，容忍后面带参数） */
static bool startsWith(const char* s, const char* prefix)
{
    while (*prefix) {
        if (*s != *prefix) {
            return false;
        }
        s++;
        prefix++;
    }
    return true;
}

/* 从行首最多挪开 3 个"非可打印 ASCII"字节再认 token。
 * 【为什么要这一手】2026-11-01 在 STM32 侧抓到实锤：这条软串口链路
 * 每串数据的**头 1~2 个字节**会被采坏（'T''H' 变 0x00 0xa4、
 * "NET" 的 'E' 变 0x94），数字部分完好。反方向（STM32 → 本板）同理，
 * 一旦 "TL " 的头一两个字节被采坏，整行就当噪声丢了 ——
 * 表现就是"设备在线、阈值也在拉，后台永远没有遥测"。
 * STM32 侧同时给 TL 行垫了两个换行当牺牲字节，这边再兜一层。 */
static const char* skipLineJunk(const char* s, size_t len)
{
    size_t i = 0;

    /* 【按长度走，不能看 '\0'】被采坏的头一个字节常常就是 0x00，
     * 用 C 字符串一遇到 0 就停，等于永远卡在那个坏字节上。 */
    while ((i < len) && (i < 3)) {
        unsigned char c = (unsigned char)s[i];

        if (c >= 0x20 && c <= 0x7E) {
            break;
        }
        i++;
    }
    return s + i;
}

/* 往事件队列里丢一个事件。队列满了就丢（不能在这里阻塞太久，
 * 否则串口接收会被拖慢，把 MCU 的应答堵在硬件缓冲里）。 */
static void postEvent(uint8_t type)
{
    OtaEvent ev;

    ev.type = type;
    g_otaEventQueue.push(&ev);
}

void uartTask(void)
{
    static char   line[OTA_LINE_MAX];
    static size_t len = 0;
    static size_t n_line = 0;   /* 本行有效字节数（含 0x00 这类坏字节） */
    static uint32_t s_ispSince = 0;

    /* 【g_ispMode 的看门狗】
     * 这个标志只被三件事清除：收到一条**完整正确**的 TL 行、OTA 会话正常收尾、
     * 或放弃这条指令。于是链路一抖（软串口在负载下会打歪字节，实测过），
     * 开机那声横幅把 g_ispMode 置成 true 之后就再没人清它 ——
     * 后果是"设备明明在线、一直在拉阈值，后台却永远收不到遥测"，
     * 因为遥测转发和阈值转发都被 !g_ispMode 挡着。
     * 所以：不在 OTA 会话里、却已经"以为对面是 bootloader"超过 30 秒，
     * 就当它是残留状态，清掉并记一行。 */
    if (g_ispMode) {
        if (s_ispSince == 0) {
            s_ispSince = millis();
        } else if (!otaSessionActive() &&
                   (uint32_t)(millis() - s_ispSince) > 30000UL) {
            g_ispMode  = false;
            s_ispSince = 0;
            LOG_SERIAL.println("[uart] ISP 标志滞留超 30 秒，按残留清掉（恢复遥测/阈值转发）");
        }
    } else {
        s_ispSince = 0;
    }

    while (LINK_SERIAL.available()) {
        int c = LINK_SERIAL.read();

        if (c < 0) {
            break;
        }
        if (c == '\r' || c == '\n') {
            if (len > 0) {
                line[len] = '\0';
                n_line = len;                /* 先记下本行字节数：里面可能含 0x00 */
                len = 0;
                /* 头 1~3 个字节被采坏是常态，认 token 前先挪开（见 skipLineJunk） */
                const char* q = skipLineJunk(line, n_line);

                /* --- bootloader 横幅：切换到 ISP 模式 --- */
                if (startsWith(q, STM32_BANNER_PREFIX)) {
                    g_ispMode = true;
                    postEvent(OTA_EVT_BANNER);
                    LOG_SERIAL.printf("[uart] bootloader 横幅: %s\n", q);
                    continue;
                }

                /* --- TL 遥测行：存最新的，ota_task 定期转发上云 ---
                 * 【不判 g_ispMode】bootloader 阶段本来就没有 TL 行；
                 * 反过来 g_ispMode 是开机见到横幅置的，若一直不清，
                 * 这里会把 app 的 TL 全部吞掉（实际踩过的坑）。 */
                if (startsWith(q, "TL ")) {
                    /* 【TL = APP 活着的铁证】开机时 STM32 先跑 bootloader，
                     * 横幅会把 g_ispMode 置 true；10 秒后跳 APP，若没有任何
                     * 机制清掉它，遥测转发就被 !g_ispMode 挡死（实际踩过）。 */
                    if (g_ispMode) {
                        g_ispMode = false;
                        LOG_SERIAL.println("[uart] 收到 TL，ISP 模式退出（APP 在跑）");
                    }
                    strncpy(g_lastTelemetry, q, sizeof(g_lastTelemetry) - 1);
                    g_lastTelemetry[sizeof(g_lastTelemetry) - 1] = '\0';
                    g_lastTlMs = millis();
                    continue;
                }

                /* --- THA：STM32 的阈值回执（「我已把阈值用上了」） ---
                 * STM32 收到 TH 行、应用成功后会在同一路 USART1 上回一行
                 *   THA <tmax×100> <tmin×100> <hmax×100> <hmin×100> <alarm> <report_ms>
                 * 这是「阈值真实生效」唯一的硬证据：没有它，
                 * 后台只能看到「已下发」，看不到「设备真的用了」。
                 * 存成事件，ota_task 会 POST 到 /api/iot/event。 */
                if (startsWith(q, "THA ")) {
                    g_lastThAckMs = millis();
                    char ev[80];
                    snprintf(ev, sizeof(ev), "EVT threshold_ack %s", q + 4);
                    evqPush(ev);
                    continue;
                }

                /* --- EVT 事件行：STM32 app 的告警/状态事件，转发给后台 ---
                 * 【为什么不能当成普通日志丢掉】
                 *   app 越限时会打 "EVT alarm threshold bits=0x01"，
                 *   丢掉的话后台事件列表里永远只有「传感器插回来」这种
                 *   8266 自己造的事件，真正的越限告警不会留痕。
                 *   存的还是整行（带 "EVT " 前缀），cloudPostEvent 按
                 *   "EVT <type> <detail>" 来拆。
                 * 【进事件队列】不是单个变量 —— 多来源的事件（STM32 的 EVT、
                 *   阈值回执 THA、8266 自己判定的链路通断）不会互相顶掉。 */
                if (startsWith(q, "EVT ")) {
                    evqPush(q);
                    continue;
                }

                /* --- 其余的行：横幅之前是 app 日志，丢掉 --- */
                if (!g_ispMode) {
                    continue;
                }

                /* --- ISP 模式：整行交给 ota_task --- */
                {
                    UartLine q;
                    strncpy(q.text, line, OTA_LINE_MAX - 1);
                    q.text[OTA_LINE_MAX - 1] = '\0';
                    /* 队列满了就丢这一行并继续 —— 协议引擎会把不认识的行
                     * 当噪声跳过，丢一行不会让协议错位。 */
                    if (!g_uartLineQueue.push(&q)) {
                        LOG_SERIAL.println("[uart] 行队列满，丢弃一行");
                    }
                }
            }
            continue;
        }

        if (len < OTA_LINE_MAX - 1) {
            line[len++] = (char)c;
        } else {
            /* 行太长，丢掉整行（防止缓冲区溢出） */
            len = 0;
        }
    }
}

/* 首次进入时把串口起起来
 *
 * 【接线（最终版接线核对表 ①-2）】
 *   PA9 (STM32 TX) → D7 (GPIO13) = 本板的 RX
 *   PA10(STM32 RX) ← D8 (GPIO15) = 本板的 TX
 *
 * 【硬串口模式（默认）】ESP8266 的 UART0 通过 Serial.swap() 映射到
 *   TX=GPIO15(D8) / RX=GPIO13(D7)，正好是接线表要求的这对脚。
 *   日志已经全部搬到 Serial1(GPIO2)，不会混进协议流。
 *
 * 【顺序必须是 begin → swap，不能反】
 *   core 3.x 的 HardwareSerial::begin() 第一件事就是 end()（_uart = NULL），
 *   然后 uart_init() 重建 UART，而 uart_init() 里会执行
 *       IOSWAP &= ~(1 << IOSWAPU0);  pinMode(1, FUNCTION_0);  pinMode(3, SPECIAL);
 *   —— 也就是说 begin 会把 swap 位和引脚全部清回默认的 GPIO1/GPIO3。
 *   反过来，uart_swap() 第一行是 if (uart == NULL) return false;，
 *   没 begin 之前调 swap 是空操作，什么都不会发生。
 *   （2026-11-03 实测踩坑：写成 swap→begin，结果 UART 一直在 GPIO1/3，
 *    STM32 的 TL 一条都收不到。） */
void uartInit(void)
{
#if STM32_LINK_UART0
    Serial.begin(STM32_UART_BAUD);
    Serial.swap();                    /* UART0: TX→GPIO15(D8), RX→GPIO13(D7) */
    Serial.setTimeout(200);
#else
    g_stm32Serial.begin(STM32_UART_BAUD);
    g_stm32Serial.setTimeout(200);
#endif
    LOG_SERIAL.printf("[uart] STM32 %s就绪 RX=GPIO13(D7)←PA9, TX=GPIO15(D8)→PA10 @%d\n",
                  STM32_LINK_UART0 ? "硬件串口" : "软串口",
                  STM32_UART_BAUD);
}
