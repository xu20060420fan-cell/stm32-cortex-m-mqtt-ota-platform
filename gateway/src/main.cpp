/*
 * main.cpp —— 入口：把四个模块起起来，然后交给协作式调度器
 *
 * 【为什么不写在 loop() 里各写各的】
 *   ESP8266 的 Arduino 核心（PlatformIO 这套是非 OS SDK）是单线程的，
 *   没有 FreeRTOS 任务可用（详见 src/sched.h 的说明）。
 *   所以这里用协作式调度：loop() 里轮流调用四个模块，
 *   谁在等待（等串口应答、等 HTTP 数据）就把调度器转起来，
 *   效果上就是"谁也不饿死谁"。
 *
 *   四个模块的分工：
 *     wifi_task —— 连 WiFi，断了退避重连
 *     mqtt_task —— MQTT over WebSocket，收下行指令
 *     uart_task —— 只做串口收发，把整行交给上层
 *     ota_task  —— 下载 + 按 ASCII 协议喂 STM32
 */
#include <Arduino.h>
#include <LittleFS.h>

#include "app_ctx.h"
#include "config.h"
#include "sched.h"
#include "prov.h"

void wifiInit(void);
void mqttInit(void);
void provEnsure(void);

/* ---------------- 连续复位计数 ----------------
 * ESP8266 的 RTC 内存能跨软件复位/看门狗复位保留。开机时把计数加一，
 * 稳定跑满 60 秒再清零。计数 >=4 说明一直在"复位→启动→又复位"，
 * 这一轮就降到安全模式（不连 MQTT、不主动触发 OTA），先把设备稳住。 */
#define BOOT_RTC_MAGIC  0x424F4F54UL    /* 'BOOT' */
struct BootRtc {
    uint32_t magic;
    uint32_t count;
};
static BootRtc s_bootRtc;
static bool    s_bootStable = false;

static void bootRtcRead(void)
{
    s_bootRtc.magic = 0;
    s_bootRtc.count = 0;
    if (ESP.rtcUserMemoryRead(0, (uint32_t*)&s_bootRtc, sizeof(s_bootRtc)) &&
        s_bootRtc.magic == BOOT_RTC_MAGIC) {
        s_bootRtc.count++;
    } else {
        s_bootRtc.magic = BOOT_RTC_MAGIC;
        s_bootRtc.count = 1;
    }
    ESP.rtcUserMemoryWrite(0, (uint32_t*)&s_bootRtc, sizeof(s_bootRtc));

    if (s_bootRtc.count >= 4) {
        g_safeMode = true;
    }
    LOG_SERIAL.printf("[boot] 连续快速复位计数=%lu%s\n",
                  (unsigned long)s_bootRtc.count,
                  g_safeMode ? "（安全模式：本轮不连 MQTT、不主动触发 OTA）" : "");
}

void setup()
{
    /* 【2026-11-03 日志搬家】UART0 按接线表 ①-2 用 Serial.swap() 映射到
     *   D8/D7 专供 STM32 协议（见 uart_task.cpp），日志改走 Serial1 = GPIO2(D4)。
     *   要看日志就把 USB-TTL 的 RX 接 D4；不接也不影响 OTA。
     *   绝对不能再用 Serial 打日志 —— 那样每个字都会混进 TL/ISP 字节流。 */
    LOG_SERIAL.begin(115200);
    LOG_SERIAL.setTimeout(200);
    delay(300);

    LOG_SERIAL.println("\n======================================");
    LOG_SERIAL.println("[boot] ESP8266 固件下载器");
    LOG_SERIAL.printf("[boot] 设备 %s, 串口 TX=GPIO15(D8) RX=GPIO13(D7) @%d\n",
                  DEVICE_ID, STM32_UART_BAUD);
    LOG_SERIAL.println("[boot] STM32 接 USART1: D8(GPIO15)→PA10, D7(GPIO13)←PA9 [最终接线表①-2]");
    LOG_SERIAL.println("======================================");

    bootRtcRead();

    /* ---- 文件系统：用来暂存待升级的固件 ----
     * 固件最大 40KB（app 分区 0x08006000~0x08010000），ESP8266 的 RAM 装不下，
     * 只能先落盘。顺手在下载的时候就把 CRC32 算出来（O 命令里要声明它）。 */
    if (LittleFS.begin()) {
        LOG_SERIAL.println("[boot] LittleFS 就绪");
    } else {
        LOG_SERIAL.println("[boot] LittleFS 挂载失败！没有它就没法暂存固件，"
                       "OTA 无法进行");
    }

    /* ---- flash 与分区对账 ----
     * 【为什么要专门打这一行】platformio.ini 用的是 nodemcuv2（4MB，
     * eagle.flash.4m1m.ld → 文件系统从 3MB 起）。如果这颗片子实际是
     * 1MB/2MB 的克隆片，LittleFS 的地址就在芯片外面 —— 写进去静默失败、
     * 读出来是垃圾，表现成"随机复位 + 固件下载老是坏"。
     * 两个数一比就判死，不用猜。 */
    {
        uint32_t nominal = ESP.getFlashChipSize();
        uint32_t real    = ESP.getFlashChipRealSize();

        LOG_SERIAL.printf("[boot] flash 分区表按 %lu 字节，芯片实际 %lu 字节 %s\n",
                      (unsigned long)nominal, (unsigned long)real,
                      (nominal > real) ? "← ⚠ 分区超出芯片，LittleFS 会写飞！" : "OK");
        FSInfo fi;
        if (LittleFS.info(fi)) {
            LOG_SERIAL.printf("[boot] LittleFS 总 %lu / 已用 %lu 字节\n",
                          (unsigned long)fi.totalBytes, (unsigned long)fi.usedBytes);
        }
    }

    /* ---- 上一次为什么重启 ----
     * 【这是 8266 侧最关键的一条诊断】Soft WDT、异常复位、欠压三种情况
     * 在外部表现完全一样（"过一会儿又活过来了"），不打这一行就永远只能猜。
     * 同时推一条 EVT 上云：后台事件流里能直接看到复位的节奏（30s？60s？10s？）
     * 和类型，节奏本身就能把根因区分开。 */
    {
        String reason = ESP.getResetReason();
        String info   = ESP.getResetInfo();
        char   evt[128];

        LOG_SERIAL.printf("[boot] 上次复位: %s | %s\n", reason.c_str(), info.c_str());
        LOG_SERIAL.printf("[boot] 空闲堆 %u 字节，最大可连续块 %u 字节\n",
                      (unsigned)ESP.getFreeHeap(),
                      (unsigned)ESP.getMaxFreeBlockSize());

        /* reason/info 里可能带空格和括号，EVT 的解析只切第一个空格做类型，
         * 剩下的整段进 detail —— 这里刻意不带引号，免得把 JSON 弄坏。 */
        snprintf(evt, sizeof(evt), "EVT gw_reset %s heap=%u boot=%u",
                 reason.c_str(), (unsigned)ESP.getFreeHeap(),
                 (unsigned)s_bootRtc.count);
        evqPush(evt);
    }

    /* ---- WiFi 配网：读配置。热点不再常开（AP+STA 会崩溃），
     * 断网 60s 后由 wifi_task 自动拉起配网热点 ---- */
    provEnsure();

    /* ---- 各模块初始化 ---- */
    uartInit();
    wifiInit();
    mqttInit();

    LOG_SERIAL.println("[boot] 启动");
}

void loop()
{
    uint32_t t0 = millis();
    uint32_t tPortal, tSched;

    /* 稳定运行 60 秒 → 连续复位计数清零，并退出安全模式。
     * 计数只在"快速反复复位"时才累积，正常长时间运行不会被误判。 */
    if (!s_bootStable && millis() >= 60000UL) {
        s_bootStable   = true;
        s_bootRtc.count = 0;
        ESP.rtcUserMemoryWrite(0, (uint32_t*)&s_bootRtc, sizeof(s_bootRtc));
        if (g_safeMode) {
            g_safeMode = false;
            mqttInit();
            LOG_SERIAL.println("[boot] 已稳定运行 60s，退出安全模式，恢复 MQTT");
        }
    }

    /* 常开的配网服务：处理 192.168.4.1 的请求 + 保存后的验证 */
    provPortalTask();
    tPortal = (uint32_t)(millis() - t0);

    /* 协作式调度：把四个模块依次转一遍。
     * 每个模块干完一段就立刻返回，不会互相卡住。 */
    t0 = millis();
    schedRun();
    tSched = (uint32_t)(millis() - t0);

    /* 【软狗复位前的最后一道线索】
     * Soft WDT 每 14.6 秒把整台 8266 复位一次（2026-11-02 云端事件流实测），
     * 光看复位原因定不出是谁把 loop 占住的。provPortalTask 不在调度器里，
     * sched.cpp 那条"单任务占用"探针盖不到它，所以这里补一层：
     * 一轮 loop 超过 1500ms 就把两半的耗时都打出来。
     * 复位前那一条就是凶手 —— 串口接上 30 秒就能定位，不用猜。 */
    if (tPortal + tSched >= 1500U) {
        LOG_SERIAL.printf("[loop] 一轮耗时 %lu ms（配网 %lu + 调度 %lu，堆 %u）\n",
                      (unsigned long)(tPortal + tSched),
                      (unsigned long)tPortal, (unsigned long)tSched,
                      (unsigned)ESP.getFreeHeap());
    }

    /* 稍微歇一下再转下一轮。
     * 【为什么要歇】
     * ESP8266 的软件看门狗靠 idle 任务喂，loop() 空转不 yield 会反复复位；
     * 而且 WiFi 协议栈也需要时间跑。 */
    delay(1);
}
