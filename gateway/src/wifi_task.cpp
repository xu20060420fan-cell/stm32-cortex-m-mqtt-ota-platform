/*
 * wifi_task.cpp —— 连 WiFi，断了退避重连
 *
 * 【为什么要单独一个模块】
 *   WiFi 断开是常态（信号弱、AP 重启）。如果连接逻辑混在主流程里，
 *   一次断线就会把整个 OTA 流程卡死。
 *   独立出来 + 退避重连，断网时其它模块照常跑。
 *
 * 【联网状态三处可见（2026-10-31）】
 *   1. 板载 LED：连上 = 常亮；断开 = 慢闪
 *   2. 状态变化时向 STM32 发一行 "NET 1 <ip>" / "NET 0"，
 *      APP 的 uart_cmd 解析后蜂鸣器提示（两短响=上线，一长响=掉线）
 *   3. 云端网页设备列表的在线/离线（90 秒心跳窗口，轮询自动续）
 */
#include <Arduino.h>
#include <ESP8266WiFi.h>

#include "app_ctx.h"
#include "config.h"
#include "prov.h"

/* 重连退避的状态 */
static uint32_t s_nextTry = 0;
static uint32_t s_backoff  = WIFI_RETRY_BASE_MS;
static bool     s_wasUp     = false;       /* 上一次的联网状态（用于边沿触发） */
static uint32_t s_downSince = 0;            /* 断流起始时刻（0=在线） */
static bool     s_portalMode = false;      /* 配网热点模式（未配置/断网超时） */
static uint32_t s_portalRetryAt = 0;        /* 热点期间下一次"回头试 STA"的时刻 */

/* 开着热点期间，每隔这么久回头试一次 STA。
 * 【为什么必须回头试】原来 s_portalMode 一旦置起就再也不会复位：
 *   热点开起来之后 STA 永远不再重连。路由器重启、家里断网一会儿这类
 *   最常见的场景，设备会一直卡在配网模式里 —— 后台永远显示离线，
 *   而且没人去拔电它就不会自己恢复。 */
#define PORTAL_STA_RETRY_MS 120000UL

bool wifiIsConnected(void)
{
    return WiFi.status() == WL_CONNECTED;
}

void wifiInit(void)
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);         /* 灭（低电平亮） */

    /* persistent(false) 必须在**任何** WiFi 动作之前：开着它的时候
     * 每次 WiFi.begin()/disconnect() 都会往 flash 写一次配置，
     * 配网页的"试连 15 秒"和这里的退避重连都是反复读写 ——
     * flash 反复擦写既慢又伤寿命（这也是协作式调度里最容易忽略的一笔）。
     * 原来这一句在无配置分支的 return 之后，那条路径上它根本没生效。 */
    WiFi.persistent(false);

    /* 【降峰值电流】现场两块板共用 3.3V，满功率发射会把电源拉塌，
     * 复位原因报 "External System"、STM32 也跟着复位。10dBm 足够室内使用。
     * modem sleep 让射频在不发数据时休眠，进一步压低平均电流。 */
    WiFi.setOutputPower(WIFI_TX_POWER_DBM);
    WiFi.setSleepMode(WIFI_MODEM_SLEEP);

    /* 【没配过网 → 开机立刻开热点】不用等 60 秒，手机马上能扫到 */
    if (!provHasConfig()) {
        LOG_SERIAL.println("[wifi] 无 WiFi 配置，立即开热点供配网");
        provPortalStart();
        s_portalMode = true;
        return;                              /* 配网完成会重启，走不到 STA */
    }

    /* 从没连上过的路径也要能"断网 60 秒开热点"：
     * s_downSince 原来只在 上线→掉线 的边沿赋值，从未连上的设备
     * 走的是 now-0，看着也对，但语义是蒙对的 —— 这里显式给起始时刻。 */
    s_downSince = millis();

    WiFi.mode(WIFI_STA);            /* 纯 STA —— AP+STA 共存会 Soft WDT 崩溃 */
    WiFi.setAutoReconnect(true);
    WiFi.hostname(DEVICE_ID);
    WiFi.begin(provSsid(), provPass());
    LOG_SERIAL.printf("[wifi] 连接 %s\n", provSsid());
}

void wifiTask(void)
{
    uint32_t now = millis();

    if (WiFi.status() == WL_CONNECTED) {
        if (!s_wasUp) {
            /* ---- 掉线 → 上线：常亮 + 通知 STM32 ---- */
            s_wasUp = true;
            s_backoff = WIFI_RETRY_BASE_MS;
            LOG_SERIAL.printf("[wifi] 已连接, IP=%s\n",
                          WiFi.localIP().toString().c_str());
            digitalWrite(LED_BUILTIN, LOW);  /* 常亮 */
            LINK_SERIAL.printf("NET 1 %s\n",
                               WiFi.localIP().toString().c_str());
        }
        return;
    }

    /* ---- 断线：慢闪 LED ---- */
    if (s_wasUp) {
        s_wasUp = false;
        s_downSince = now;
        LOG_SERIAL.println("[wifi] 断开");
        digitalWrite(LED_BUILTIN, HIGH);
        LINK_SERIAL.print("NET 0\n");
    }
    digitalWrite(LED_BUILTIN, (now / 400) % 2 ? LOW : HIGH);

    /* ---- 断网 60s（或配网模式）→ 开热点供配网 ----
     * 热点是纯 AP 模式（AP+STA 共存会 Soft WDT 崩溃，实测）。
     * 【比较必须全在 int32_t 里做】原来右边写的是 60000UL，
     *   `(int32_t)(...) > 60000UL` 会把左边又提升回无符号 ——
     *   millis() 49.7 天回绕那一刻差值变负，就变成"负数 > 60000"成立，
     *   热点会被莫名其妙打开。这类混符号比较是 ESP8266 上的经典坑。 */
    if (s_portalMode || (int32_t)(now - s_downSince) > (int32_t)60000L) {
        if (!provPortalRunning()) {
            LOG_SERIAL.println("[wifi] 60s 连不上，开热点 STM32-OTA-Setup 供配网");
            provPortalStart();
            s_portalMode = true;
            s_portalRetryAt = now + PORTAL_STA_RETRY_MS;
        }

        /* ---- 开着热点也要回头试一次 STA（只有配过网才有意义）----
         * 顺序是硬的：先 provPortalStop() 把 AP 摘掉，再开 STA；
         * 试不通再把热点拉回来。任何时刻只开一个模式，绝不共存。 */
        if (provHasConfig() && (int32_t)(now - s_portalRetryAt) >= 0) {
            uint32_t t0;

            s_portalRetryAt = millis() + PORTAL_STA_RETRY_MS;
            LOG_SERIAL.println("[wifi] 配网模式里回头试一次 STA");
            provPortalStop();
            WiFi.mode(WIFI_STA);
            WiFi.begin(provSsid(), provPass());

            /* 阻塞试连最多 10 秒：delay() 里核心会喂软狗，所以不会 WDT 复位。
             * 这里故意不调 schedPumpOthers() —— 避免 wifiTask 自己递归进来。 */
            t0 = millis();
            while (WiFi.status() != WL_CONNECTED && (uint32_t)(millis() - t0) < 10000UL) {
                delay(100);
            }

            if (WiFi.status() == WL_CONNECTED) {
                LOG_SERIAL.println("[wifi] 断网期间恢复了，关闭配网热点");
                s_portalMode = false;
                s_wasUp      = false;      /* 让上线边沿重走一遍：LED + NET 行 */
                s_downSince  = 0;
                s_backoff    = WIFI_RETRY_BASE_MS;
                return;
            }
            WiFi.disconnect();
            provPortalStart();             /* 还是连不上，热点得留着，不然没法配网 */
        }
        return;                              /* 配网期间不做 STA 重连 */
    }

    /* 退避重连：失败次数越多等得越久，避免疯狂重连把 AP 打死 */
    if ((int32_t)(now - s_nextTry) < 0) {
        return;
    }
    s_nextTry = now + s_backoff;

    LOG_SERIAL.printf("[wifi] 未连接, %u ms 后重连\n", (unsigned)s_backoff);
    WiFi.disconnect();
    WiFi.begin(provSsid(), provPass());

    s_backoff = (s_backoff * 2 > WIFI_RETRY_MAX_MS) ? WIFI_RETRY_MAX_MS
                                                     : s_backoff * 2;
}
