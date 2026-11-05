/*
 * mqtt_task.cpp —— MQTT over WebSocket
 *
 * 【为什么是 MQTT over WebSocket 而不是普通 MQTT】
 *   broker 暴露的是 ws://<server>/mqtt（同一台 nginx）。
 *   走 WebSocket 的好处是只用 80 端口，跟固件下载走同一个入口，
 *   不用额外开一个 1883 端口。
 *
 * 【PubSubClient 怎么塞进 WebSocket】
 *   PubSubClient 只认 Arduino 的 Client 接口。
 *   ArduinoWebsockets 的 WebsocketsClient **不是** Client，
 *   而且它收数据靠 onMessage 回调、没有 read()/peek()。
 *   所以这里写一个 WsClient 适配器：
 *     - 收：onMessage 回调把字节塞进环形缓冲，read() 从缓冲里取
 *     - 发：write() 把一段字节作为一个**二进制** WebSocket 帧发出去
 *            （MQTT 是二进制协议，用 binary 帧，别用 text）
 *     - connect()：真正去建 WebSocket 连接
 */

/* ================================================================
 * 精简构建开关：-D OTA_MINIMAL
 * ================================================================
 * 为什么这个节点能砍掉 MQTT：
 *   8266 的 OTA 指令本来就是走 HTTP /api/iot/config 拉的（见 fw_download.cpp，
 *   ota_task 一直在轮询），MQTT 在这个节点上**只负责上报下载进度**。
 * 所以砍掉 MQTT 之后：
 *     · flash 省 ~40KB —— PubSubClient + ArduinoWebsockets 两个库整个不链接
 *     · RAM 省 ~10KB   —— WebSocket 环形缓冲 + PubSubClient 的 packet buffer
 *     · 代价：后台看不到 8266 的实时进度，只剩串口日志能看到
 *   触发 OTA 改成轮询 /api/iot/config，本来就是这个流程，不受影响。
 *
 * 默认构建（不带 OTA_MINIMAL）现在也不再建立 MQTT 连接：
 *   config.h 里 GW_ENABLE_MQTT 默认 0，原因就是下面这条"每 15 秒 Soft WDT
 *   复位"的握手阻塞。要恢复秒级下行，编译时加 -D GW_ENABLE_MQTT=1。
 */
#include <Arduino.h>

#include "app_ctx.h"        /* 里面已经带了 config.h */

/* 两种关法都走同一套空实现：
 *   · -D OTA_MINIMAL   —— 编译期把 PubSubClient / ArduinoWebsockets 整个去掉
 *   · GW_ENABLE_MQTT=0 —— config.h 的默认值（现场"每 15 秒 Soft WDT 复位"的根治开关）
 * 关掉后阈值/OTA/遥测仍走 HTTP 轮询，见 config.h 里那段说明。 */
#if defined(OTA_MINIMAL) || !GW_ENABLE_MQTT

void mqttInit(void) {}
void mqttTask(void) {}
void mqttPublishStatus(const char* json) { (void)json; }

#else /* 带 MQTT（需要 -D GW_ENABLE_MQTT=1 才会编到这里） */

#include <ArduinoWebsockets.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>

#include "config.h"
#include "fw_download.h"

/* 收 WebSocket 数据的环形缓冲。
 * 【为什么用静态数组】MQTT 包最大 512 字节（MQTT_MAX_PACKET_SIZE），
 * 缓冲给 1KB 绰绰有余，而且不能走堆。 */
static uint8_t s_rxBuf[1024];
static uint16_t s_rxHead = 0;
static uint16_t s_rxTail = 0;

/* WebSocket 帧最大负载。MQTT 的一包不会超过 512，所以一帧就够。 */
#define WS_FRAME_MAX 1024

class WsClient : public Client {
public:
    void attach(websockets::WebsocketsClient* ws) { ws_ = ws; }

    /* WebSocket 收到一帧 → 塞进环形缓冲 */
    void onMessage(const websockets::WebsocketsMessage& msg) {
        const char* d = msg.c_str();
        uint32_t n = msg.length();
        uint32_t i;

        for (i = 0; i < n; i++) {
            uint16_t next = (uint16_t)((s_rxHead + 1) % sizeof(s_rxBuf));
            if (next == s_rxTail) {
                break;              /* 缓冲满了，丢弃后面的字节 */
            }
            s_rxBuf[s_rxHead] = (uint8_t)d[i];
            s_rxHead = next;
        }
    }

    void setConnected(bool c) { connected_ = c; }

    /* ---- Arduino Client 接口 ---- */
    int available() override {
        return (int)(uint16_t)(s_rxHead - s_rxTail);
    }

    int read() override {
        if (s_rxHead == s_rxTail) {
            return -1;
        }
        {
            int v = s_rxBuf[s_rxTail];
            s_rxTail = (uint16_t)((s_rxTail + 1) % sizeof(s_rxBuf));
            return v;
        }
    }

    int read(uint8_t* buf, size_t size) override {
        size_t n = 0;
        while (n < size) {
            int c = read();
            if (c < 0) {
                break;
            }
            buf[n++] = (uint8_t)c;
        }
        return (int)n;
    }

    int peek() override {
        if (s_rxHead == s_rxTail) {
            return -1;
        }
        return s_rxBuf[s_rxTail];
    }

    void flush() override {
        /* 清掉没收完的半包，防止下次连接时把残渣当成新数据 */
        s_rxHead = s_rxTail;
    }

    size_t write(uint8_t b) override {
        return write(&b, 1);
    }

    size_t write(const uint8_t* buf, size_t size) override {
        size_t off = 0;

        if (ws_ == NULL || !connected_) {
            return 0;
        }
        while (off < size) {
            size_t n = size - off;
            if (n > WS_FRAME_MAX) {
                n = WS_FRAME_MAX;
            }
            /* MQTT 是二进制协议，必须用 binary 帧 */
            if (!ws_->sendBinary((const char*)(buf + off), n)) {
                break;
            }
            off += n;
        }
        return off;
    }

    int connect(IPAddress ip, uint16_t port) override {
        (void)ip;
        return connect(CLOUD_HOST, port);
    }

    int connect(const char* host, uint16_t port) override {
        if (ws_ == NULL) {
            return 0;
        }
        if (connected_) {
            return 1;
        }
        s_rxHead = s_rxTail;

        /* WebSocket 握手是阻塞的，失败会返回 false */
        connected_ = ws_->connect(host, (int)port, CLOUD_WS_PATH);
        if (!connected_) {
            LOG_SERIAL.println("[mqtt] WebSocket 连接失败");
        }
        return connected_ ? 1 : 0;
    }

    void stop() override {
        /* PubSubClient 在连接丢失时会调 stop()。
         * 这里只清缓冲，不关 WebSocket —— 真正的重连由 connect() 负责，
         * 免得"刚断开就连"把 socket 折腾坏。 */
        flush();
    }

    uint8_t connected() override {
        return connected_ ? 1 : 0;
    }

    operator bool() override {
        return connected_ ? 1 : 0;
    }

private:
    websockets::WebsocketsClient* ws_ = NULL;
    bool connected_ = false;
};

/* ---------------- 任务内部状态 ---------------- */
static websockets::WebsocketsClient s_ws;
static WsClient          s_adapter;
static PubSubClient      s_mqtt(s_adapter);
static uint32_t          s_nextTry = 0;      /* WebSocket 重连退避 */
static uint32_t          s_nextMqttTry = 0;  /* MQTT(CONNACK) 重连退避 */

static void onWsMessage(websockets::WebsocketsClient& client,
                         const websockets::WebsocketsMessage& msg)
{
    (void)client;
    s_adapter.onMessage(msg);
}

static void onWsEvent(websockets::WebsocketsClient& client,
                       websockets::WebsocketsEvent ev,
                       websockets::WSInterfaceString data)
{
    (void)client;
    (void)data;

    switch (ev) {
        case websockets::WebsocketsEvent::ConnectionOpened:
            s_adapter.setConnected(true);
            LOG_SERIAL.println("[mqtt] WebSocket 已连接");
            break;
        case websockets::WebsocketsEvent::ConnectionClosed:
            s_adapter.setConnected(false);
            LOG_SERIAL.println("[mqtt] WebSocket 已断开");
            break;
        default:
            break;
    }
}

/* 下行消息 */
static void onMqttMessage(char* topic, byte* payload, unsigned int len)
{
    LOG_SERIAL.printf("[mqtt] 收到 %s\n", topic);

    if (strcmp(topic, TOPIC_OTA_DOWNLINK) == 0) {
        /* 云端让升级。真正的固件地址还是去问云端配置接口拿，
         * 这里只负责"通知一声"。 */
        OtaEvent ev;
        ev.type = OTA_EVT_MQTT;
        g_otaEventQueue.push(&ev);
        return;
    }

    if (strcmp(topic, TOPIC_CONFIG_DOWNLINK) == 0) {
        /* 云端推阈值来了 —— 立刻转给 STM32 app（秒级生效）。
         * 【为什么在这里就地解析】payload 不是以 '\0' 结尾的，
         *   得先拷进自己的缓冲再交给 JSON 解析器。
         * 【解析失败只打日志】绝不能因为一条坏消息把 MQTT 主循环带崩。 */
        static char s_cfgBuf[512];
        static OtaInstruction s_cfgIns;

        if (payload == NULL) {
            return;
        }
        if (len >= sizeof(s_cfgBuf)) {
            len = sizeof(s_cfgBuf) - 1;
        }
        memcpy(s_cfgBuf, payload, len);
        s_cfgBuf[len] = '\0';

        if (cloudParseConfigJson(s_cfgBuf, len, &s_cfgIns)) {
            forwardThresholds(&s_cfgIns);
        } else {
            LOG_SERIAL.println("[mqtt] config 解析失败，忽略这一条");
        }
        return;
    }
}

/* 上报状态。ota_task 调用。
 * 【为什么不用互斥量】
 *   协作式调度是单线程的，ota_task 和 mqtt_task 不会同时执行，
 *   PubSubClient 的缓冲区不会被撞。 */
void mqttPublishStatus(const char* json)
{
    if (json == NULL || !s_mqtt.connected()) {
        return;
    }
    if (!s_mqtt.publish(TOPIC_STATUS, json)) {
        LOG_SERIAL.printf("[mqtt] 状态上报失败: %s\n", json);
    }
}

void mqttInit(void)
{
    if (g_safeMode) {
        LOG_SERIAL.println("[mqtt] 连续复位保护生效，本轮不建立 MQTT 连接");
        return;
    }

    s_ws.onMessage(onWsMessage);
    s_ws.onEvent(onWsEvent);
    s_adapter.attach(&s_ws);

    s_mqtt.setServer(CLOUD_HOST, CLOUD_WS_PORT);
    s_mqtt.setCallback(onMqttMessage);
    s_mqtt.setBufferSize(512);
    /* 【连接/收发超时压到 2 秒】PubSubClient 默认等 CONNACK 15 秒，
     * 一次连不上就把整个协作式调度占住 15 秒 —— 遥测、串口全部停摆，
     * 看起来就像设备死了。2 秒足够局域网 + 云端往返。 */
    s_mqtt.setSocketTimeout(2);

    LOG_SERIAL.println("[mqtt] 启动");
}

void mqttTask(void)
{
    uint32_t now = millis();

    if (g_safeMode) {
        return;
    }

    /* 1) WebSocket 连上没
     * 【OTA 期间绝不发起新连接】WebSocket 握手是阻塞的，而且没有超时；
     * 它又会被 keepAlive → MsgQueue::pop 的嵌套调度拉进来执行。
     * 一次握手 + CONNACK 叠在 LittleFS 写/下载循环里，很容易超过
     * 6.3 秒的软看门狗，现场表现就是"下载固件时 8266 不断复位"。
     * MQTT 在 OTA 里只用来报进度，断一会不影响升级，等回 IDLE 再连。 */
    if (!s_adapter.connected()) {
        if (otaSessionActive()) {
            return;
        }
        if ((int32_t)(now - s_nextTry) < 0) {
            return;
        }
        s_nextTry = now + 15000;        /* 15 秒一次，别把射频/电源按在地上摩擦 */

        if (!wifiIsConnected()) {
            return;
        }

        /* 【先做一次带上限的 TCP 预检，再做 WebSocket 握手】
         * ws_->connect() 内部的 WiFiClient.connect() 默认超时 5 秒，
         * 握手阶段还有两段 1 秒的纯自旋读（不让出 CPU）。
         * 服务器不在时这一套就是 5~7 秒的阻塞，正好顶穿 6.3 秒软看门狗。
         * 预检把超时压到 2 秒：端口不通就直接跳过这一轮，绝不去做那个握手。 */
        {
            WiFiClient probe;

            probe.setTimeout(2000);
            if (!probe.connect(CLOUD_HOST, CLOUD_WS_PORT)) {
                probe.stop();
                return;
            }
            probe.stop();
        }
        ESP.wdtFeed();                  /* 预检没超限，但别让计时器贴着阈值走 */
        s_adapter.connect(CLOUD_HOST, CLOUD_WS_PORT);
        return;
    }

    /* 2) MQTT 会话
     * 【必须有退避】WebSocket 通了但 CONNACK 不来时，原来的代码每转一圈
     * （几毫秒）就再试一次 connect，每次都阻塞 2~15 秒 —— 遥测/串口全被
     * 饿死，射频持续满载，现场表现就是"设备活着但不干活，过一会儿复位"。
     * 现在失败后 5 秒再试。 */
    if (!s_mqtt.connected()) {
        if (otaSessionActive()) {
            return;
        }
        if ((int32_t)(now - s_nextMqttTry) < 0) {
            return;
        }
        s_nextMqttTry = now + 5000;
        if (s_mqtt.connect(DEVICE_ID, MQTT_USER, MQTT_PASS)) {
            /* 两个下行主题都要订阅：
             *   ota    —— 云端说"该升级了"
             *   config —— 云端推阈值，后台改完秒级生效（不等轮询） */
            s_mqtt.subscribe(TOPIC_OTA_DOWNLINK);
            s_mqtt.subscribe(TOPIC_CONFIG_DOWNLINK);
            LOG_SERIAL.printf("[mqtt] 已连接, 订阅 %s + %s\n",
                          TOPIC_OTA_DOWNLINK, TOPIC_CONFIG_DOWNLINK);
        } else {
            LOG_SERIAL.println("[mqtt] MQTT 连接失败");
        }
        return;
    }

    /* 3) 收发。poll() 必须先跑 —— onMessage 回调是在 poll 里触发的，
     *    不 poll 就永远收不到数据。 */
    s_ws.poll();
    s_mqtt.loop();
}

#endif /* OTA_MINIMAL || !GW_ENABLE_MQTT */
