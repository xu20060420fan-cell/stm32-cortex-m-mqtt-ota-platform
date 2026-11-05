/*
 * app_ctx.cpp —— 共享对象的定义
 *
 * 队列的存储区用静态数组：ESP8266 的堆被 WiFi 切得七零八落，
 * 这里一共 1KB 出头，放静态区最稳。
 */
#include "app_ctx.h"

#include <string.h>

/* 串口行队列：16 行 × 72 字节。
 * 为什么是 16 行：bootloader 的 ? 命令一次要回 12 行
 * （OK + 10 个 KV + END），保活一次就得装得下。 */
static uint8_t s_lineStore[OTA_QUEUE_LEN][OTA_LINE_MAX];
/* 事件队列 */
static uint8_t s_evtStore[8];

MsgQueue g_uartLineQueue(s_lineStore, OTA_QUEUE_LEN, OTA_LINE_MAX);
MsgQueue g_otaEventQueue(s_evtStore, 8, (uint16_t)sizeof(OtaEvent));

/* g_stm32Serial（SoftwareSerial RX=13/TX=15）的定义在 uart_bridge.cpp */

/* 看到 bootloader 横幅之前，一直是 false —— 这期间收到的行都是 app 日志 */
bool g_ispMode = false;

/* 连续快速复位保护，具体判定在 main.cpp 的 setup() 里 */
bool g_safeMode = false;

/* ================================================================
 * 事件文本队列
 *
 * 【为什么不用一个 g_lastEvent 单条】
 *   以前只有一个 g_lastEvent，谁写谁覆盖：STM32 的阈值回执（THA）
 *   和 8266 自己产生的「STM32 链路断了」很可能互相顶掉，
 *   一条都发不出去。现在改成 4 条的小队列，先到先发。
 * ================================================================ */
#define EVQ_LEN 4
#define EVQ_TEXT_MAX 96
static char s_evqStore[EVQ_LEN][EVQ_TEXT_MAX];
static int s_evqHead = 0;
static int s_evqTail = 0;

void evqPush(const char* text)
{
    if (text == NULL || text[0] == '\0') {
        return;
    }
    int next = (s_evqHead + 1) % EVQ_LEN;
    if (next == s_evqTail) {
        /* 队列满了：丢掉最老的一条，腾位置给新的 */
        s_evqTail = (s_evqTail + 1) % EVQ_LEN;
    }
    strncpy(s_evqStore[s_evqHead], text, EVQ_TEXT_MAX - 1);
    s_evqStore[s_evqHead][EVQ_TEXT_MAX - 1] = '\0';
    s_evqHead = next;
}

bool evqPop(char* out, int cap)
{
    if (s_evqTail == s_evqHead) {
        return false;
    }
    strncpy(out, s_evqStore[s_evqTail], cap - 1);
    out[cap - 1] = '\0';
    s_evqTail = (s_evqTail + 1) % EVQ_LEN;
    return true;
}
