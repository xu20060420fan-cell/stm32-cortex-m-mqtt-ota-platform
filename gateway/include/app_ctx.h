/*
 * app_ctx.h —— 任务之间共享的东西
 *
 * 【为什么不放一堆全局可变变量】
 *   任务之间传消息一律走消息队列（MsgQueue）：
 *     g_uartLineQueue —— uart_task 把收到的行塞进来，ota_task 取出来解析
 *     g_otaEventQueue —— uart_task / mqtt_task 把"该升级了""看到横幅了"这类
 *                        事件塞进来，ota_task 取出来决定下一步做什么
 *   队列的存储区是静态数组，不动态分配。
 */
#ifndef APP_CTX_H
#define APP_CTX_H

#include <Arduino.h>

#include <SoftwareSerial.h>
#include "config.h"
#include "sched.h"

/* uart_task → ota_task 的一行 */
struct UartLine {
    char text[OTA_LINE_MAX];
};

/* 事件类型 */
enum OtaEventType {
    OTA_EVT_BANNER  = 1,  /* 看到 bootloader 横幅：对面已经进 bootloader 了 */
    OTA_EVT_MQTT    = 2   /* MQTT 下行：云端要求升级 */
};

struct OtaEvent {
    uint8_t type;
};

/* 两个队列 */
extern MsgQueue g_uartLineQueue;
extern MsgQueue g_otaEventQueue;

/* 与 STM32 通话的串口，脚位按最终版接线核对表 ①-2：
 *   RX = GPIO13 (D7) ← STM32 PA9 (USART1_TX)
 *   TX = GPIO15 (D8) → STM32 PA10(USART1_RX)
 * 【硬串口可行，但顺序必须是 begin → swap】
 *   core 3.x 的 uart_swap() 把 UART0 映射成 TX=GPIO15 / RX=GPIO13
 *   （见 framework 的 uart.cpp），正好是这对脚。
 *   关键是顺序：begin 之前 _uart 还是 NULL，swap() 第一行就 return false；
 *   而 begin 里的 uart_init 又会清掉 IOSWAP、把引脚设回 GPIO1/GPIO3。
 *   （2026-10-31 写的「swap 在 begin 前」正是踩了这个坑，被误判成
 *    硬件不支持才退回软串口；软串口 TL 头几个字节常被采坏。
 *    2026-11-03 改正顺序后走回硬件串口。）
 * 【GPIO15 当 TX 的 strap 疑虑】strap 只在复位瞬间采样，板上下拉保证
 *   能正常启动；固件跑起来后再驱动它当 TX 没有影响。
 * 【为什么没有互斥量】协作式调度单线程，uart_task 只在读、ota_task 只在写。 */
#if !STM32_LINK_UART0
extern SoftwareSerial g_stm32Serial;
#endif
/* 硬串口模式下直接用 UART0（LINK_SERIAL 在 config.h 里映射成 Serial），
 * 不再有 SoftwareSerial 对象。 */

/* uart_task 存下的最新一条 TL 遥测行（STM32 app 发的，ota_task 转发上云）。
 * 没有互斥：协作式调度单线程，uart_task 写、ota_task 读，不会同时。 */
extern char g_lastTelemetry[128];

/* 事件文本队列（4 条）。
 *   往里塞的：STM32 的阈值回执（THA）、8266 自己判定的链路通断、
 *             传感器恢复等。每条最终 POST 到 /api/iot/event。
 * 用队列而不是单个变量：多来源产生的事件不会互相顶掉。 */
void evqPush(const char* text);
bool evqPop(char* out, int cap);

/* 是不是已经进 bootloader 的 ISP 模式了。
 * 只有看到横幅才置 true —— 之前收到的行都是 app 的调试日志。
 * uart_task 负责置位，ota_task 靠它决定要不要发 ? 保活。 */
extern bool g_ispMode;

/* 连续快速复位保护（2026-11-03）。
 * 上电后若 RTC 里记到短时间内已经复位 >=4 次，这一轮进入安全模式：
 * 不连 MQTT、不主动触发 OTA，只保留 WiFi + 串口 + 2 秒遥测，
 * 避免"复位→重连 MQTT→再复位"的自我强化循环。稳定运行 60 秒后清零。 */
extern bool g_safeMode;

/* 现在是不是真的在跑一次 OTA 会话（ota_task 的状态机不在 IDLE）。
 * uart_task 用它判断 g_ispMode 是"真在刷机"还是"开机残留"。 */
bool otaSessionActive(void);

/* uart_task 的初始化（起串口） */
void uartInit(void);

/* 最后一次收到 STM32 的 TL 遥测行的时刻（millis）。
 * 0 = 从启动到现在一条 TL 都没收到过。
 * 【为什么需要它】STM32↔8266 的串口一旦不通（引脚错、swap 没生效、
 *   STM32 停在 bootloader），后台就一条遥测都收不到，
 *   而设备照样在线、照样拉阈值 —— 症状极具迷惑性。
 *   有了这个时间戳，8266 就能自己判断「STM32 链路断了」并上报事件，
 *   后台一眼就能定位到是哪一段。 */
extern uint32_t g_lastTlMs;

/* 最后一次收到 STM32 阈值回执（THA 行）的时刻（millis），0 = 从未收到。
 * STM32 应用完阈值会回一行 "THA ..."，这就是「阈值真实生效」的证据。 */
extern uint32_t g_lastThAckMs;

/* wifi_task 提供 */
bool wifiIsConnected(void);

/* mqtt_task 提供：把一条状态上报发给云端。
 * 没连上就丢弃，不阻塞 OTA 流程。 */
void mqttPublishStatus(const char* json);

#endif /* APP_CTX_H */
