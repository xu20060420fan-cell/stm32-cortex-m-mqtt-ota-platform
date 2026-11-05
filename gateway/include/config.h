/*
 * config.h —— 全局可调参数
 *
 * 【为什么集中放一个文件】
 * WiFi 账号、云端地址、设备密钥、串口引脚、协议超时……这些每换一个现场就要改。
 * 集中在一处，改参数不用翻业务代码，也方便在 README 里对照着填。
 */
#ifndef CONFIG_H
#define CONFIG_H

/* ================================================================
 * 一、WiFi
 * ================================================================ */
/* WiFi 凭据 = **出厂默认**：LittleFS 里没有存档时用它（prov.cpp 的
 * "使用编译期 WiFi" 分支）。填上真实值的好处很实在：
 * 整片擦除（erase_flash）会把 LittleFS 里存的配网结果一起抹掉，
 * 如果这里还是 your-wifi-ssid，擦完就变成"没配过网"、要连热点重配一遍。
 * 存档优先于这里，所以以后换网络仍然可以在配网页面改，不用重烧。 */
#define WIFI_SSID               "YOUR_WIFI_SSID"
#define WIFI_PASSWORD           "YOUR_WIFI_PASSWORD"
#define WIFI_CONNECT_TIMEOUT_MS 20000   /* 一次连接最多等多久 */
#define WIFI_RETRY_BASE_MS      2000    /* 重连退避的起始间隔 */
#define WIFI_RETRY_MAX_MS       30000   /* 退避的上限 */

/* 【发射功率压到 10dBm（2026-11-03）】
 *   现场是 ESP8266 + STM32 共用一路 3.3V。复位原因长期报 "External System"，
 *   而且两块板同时复位 —— 典型的 WiFi 发射瞬间把电源拉塌（欠压复位）。
 *   20.5dBm 的峰值电流比 10dBm 大一倍多，室内几米距离根本用不上满功率。
 *   压到 10dBm 后覆盖仍然够用，峰值电流显著下降，是软件侧最有效的缓解。 */
#define WIFI_TX_POWER_DBM       10.0f

/* ================================================================
 * 二、云端 / MQTT
 * ================================================================ */
#define CLOUD_HOST              "YOUR_SERVER_HOST"
#define CLOUD_HTTP_PORT         80
#define CLOUD_WS_PORT           80
#define CLOUD_WS_PATH           "/mqtt"
#define MQTT_USER               "iot"
#define MQTT_PASS               "YOUR_MQTT_PASSWORD"

/* ================================================================
 * MQTT over WebSocket 总开关（默认 0 = 关闭）
 * ================================================================
 * 【为什么默认关掉】
 *   2026-11-02 / 11-03 现场实测："8266 每 15 秒 Soft WDT 复位"。
 *   15 秒 = mqttTask 的 WebSocket 重连退避；每次重连都要做一次
 *   `ws_->connect()` —— 它内部的 TCP connect(默认 5s) + 握手读超时
 *   (1s×2) 全在协作式调度器里同步阻塞，且握手阶段是**不让出 CPU 的
 *   死循环**，直接顶穿 6.3 秒软看门狗 → 复位。复位后 STM32 也跟着重启，
 *   现场表现就是"设备定时重启"。
 *
 * 【关掉它为什么功能不丢】
 *   8266 真正的职责只有三件，全部走 HTTP 轮询，MQTT 只是"更快一点"的副本：
 *     · 阈值下发：ota_task 每 10 秒拉 /api/iot/config 并 forwardThresholds()
 *     · OTA 指令：同一次轮询里带回来的 pendingOta
 *     · 遥测上报：ota_task 每 2 秒 POST /api/iot/telemetry
 *   代价仅是"后台改阈值后最多 10 秒生效"，而不是"秒级"。
 *
 * 【想恢复 MQTT】编译时加 -D GW_ENABLE_MQTT=1（同时把 lib_deps 里
 *   PubSubClient / ArduinoWebsockets 留着）。恢复后 connect 前后会加
 *   一次带 2 秒上限的 TCP 预检，服务器不在就不做那 5 秒的握手。
 * ================================================================ */
#ifndef GW_ENABLE_MQTT
#define GW_ENABLE_MQTT          0
#endif

#define TOPIC_PREFIX            "campus/iot"

/* 【身份必须可配置】原来 DEVICE_ID 写死 dev01、DEVICE_KEY 也是固定值，
 * 五台节点烧同一份固件时后台只能看到一个 dev01：
 *   - telemetry 互相覆盖，页面数据跳来跳去；
 *   - deviceKey 反查不出唯一设备，鉴权会串。
 * 编译期可用 -DDEVICE_ID=\"dev02\" -DDEVICE_KEY=\"...\" 为每台单独生成，
 * 不传时保持旧默认值，避免影响现有单台调试。 */
#ifndef DEVICE_ID
#define DEVICE_ID               "dev01"
#endif

/* 8266 网关固件版本号。
 * 【为什么要有它】
 *   每次拉 /api/iot/config 都会把版本号带上去（&fw=），后台记到
 *   device.gatewayVersion，网页上直接能看到。
 *   现场最耗人的一次误判就是「以为刷了新固件，板上其实跑的还是旧的」，
 *   而旧固件的症状（只拉配置、没有遥测）和 STM32 掉线一模一样。
 *   改固件时把这个版本号往上加一位，页面上一眼就能对得上。
 *   【现在定义在哪】往下几十行，紧跟 STM32_LINK_UART0 那里 ——
 *   因为版本号要按"链路用硬串口还是软串口"分叉，两处写死迟早会对不上。 */

/* 设备密钥：云端要求带请求头 X-Iot-Key。
 * 【注意】这是设备自己的密钥，不等于 MQTT 账号密码。
 * 现场部署时用真实值替换掉。 */
#ifndef DEVICE_KEY
#define DEVICE_KEY              "YOUR_DEVICE_KEY"
#endif

/* 下行：云端把升级指令发到这个主题 */
#define TOPIC_OTA_DOWNLINK      TOPIC_PREFIX "/" DEVICE_ID "/ota"
/* 下行：云端把阈值/上报周期推到这个主题（后台改完秒级生效） */
#define TOPIC_CONFIG_DOWNLINK   TOPIC_PREFIX "/" DEVICE_ID "/config"
/* 上行：把进度和结果报回云端 */
#define TOPIC_STATUS            TOPIC_PREFIX "/" DEVICE_ID "/status"

/* ================================================================
 * 三、与 STM32 的串口
 *
 * 【接的是 USART1，不是 USART2】
 *   bootloader 的 ISP/OTA 命令状态机走的是 USART1
 *   （STM32 侧 users/appbl.c 里 extern UART_HandleTypeDef huart1，
 *     bootloaer.ioc 里 PA9=USART1_TX、PA10=USART1_RX，
 *     main.c 里 huart1.Init.BaudRate=115200）。
 *   USART2（PA2/PA3）是 app 的调试控制台，ESP8266 不接它。
 *   固件数据必须进 bootloader 的命令状态机，所以只能接 USART1。
 *
 * 【最终接线】
 *   ESP8266 GPIO13 (TX)  ──►  STM32 PA10 (USART1_RX)
 *   ESP8266 GPIO5  (RX)  ◄──  STM32 PA9  (USART1_TX)
 *   ESP8266 GND           ──   STM32 GND
 *   波特率 115200 8N1（跟 STM32 侧 main.c 的 huart1.Init.BaudRate 一致）
 *
 * 【为什么用 Serial.swap() 而不是 SoftwareSerial】
 *   SoftwareSerial 只能占 GPIO13/GPIO5 这类普通脚，
 *   而实际接线是 D8(GPIO15) / D7(GPIO13) —— GPIO5 在板上根本没接。
 *   所以必须用硬件 UART0 + Serial.swap()，把它从 GPIO1/GPIO3
 *   （板载 CP2102，板上留空只做手插调试）换到 GPIO15/GPIO13。
 *
 * 【swap() 必须在 begin() 之后】
 *   core 3.x 的 begin() 会先 end() 再重建 UART，并清掉 IOSWAP 位、
 *   把引脚设回 GPIO1/GPIO3；而 begin() 之前 _uart 还是 NULL，
 *   swap() 直接 return false。所以只能是 begin → swap。
 *   顺序反了 UART0 就还在 GPIO1/GPIO3 上，
 *   结果就是 STM32 的 TL 遥测一条都进不来。
 *   参见 src/uart_task.cpp 的 uartInit()。
 *
 * 【最终接线】
 *   ESP8266 GPIO15(D8) TX ──►  STM32 PA10 (USART1_RX)
 *   ESP8266 GPIO13(D7) RX ◄──  STM32 PA9  (USART1_TX)
 *   ESP8266 GND          ──   STM32 GND
 *   波特率 115200 8N1（跟 STM32 侧 main.c 的 huart1.Init.BaudRate 一致）
 * ================================================================ */
/* 串口：8266 → STM32 的 USART1，用来推固件。
 *
 * ⚠ 波特率必须两边一致（STM32 侧是 huart1.Init.BaudRate），
 *   不一致 = 推过去全是乱码，bootloader 会因 CRC32 不过而拒绝。
 *
 * 【为什么默认 115200 而不是更高】
 *   这里用的是 EspSoftwareSerial（软件串口），用软件中断模拟时序，
 *   在 ESP8266 上超过 115200 就会开始丢字节 —— 115200 已经是它的稳定上限。
 *   40KB 固件推完 = 40*10/115200 ≈ 3.5 秒，一次 OTA 而已，完全够用。
 *
 * 【真要再快】
 *   换硬件 UART1（TX=GPIO2 / RX=GPIO3），速率可上 460800/921600，
 *   40KB 变 0.9 秒。代价：GPIO3 不能再接 USB-TTL 的 TX（调试口变单向），
 *   而且要同步改 STM32 侧的 huart1.Init.BaudRate。
 */
/* 串口：8266 → STM32 的 USART1，用来推固件、收 TL 遥测。
 * 两条线固定是 D7(GPIO13) ← PA9、D8(GPIO15) → PA10。
 *
 * 【用哪条驱动收这两根线，是个真实的两难，所以做成开关】
 *   1 = 硬件 UART0 + Serial.swap()：swap 之后 U0TXD=GPIO15(D8)、
 *       U0RXD=GPIO13(D7)，正好是接线表 §①-2 与检查单要求的接法。
 *       优点：位时序由硬件保证，负载下不失真。
 *       实测证据（2026-11-01，从 STM32 内存里读的）：
 *         · APP 的 s_line 里最后收到的 TH 行是 "…TH 1600 10000 2 0 002 2…"
 *           —— 数字全对，但开头 'T'/'H' 和空格被采成 0x00 / 0xa4，
 *           解析在第 3 个字段前断掉 ⇒ s_thr 里只有 temp_max/temp_min 生效，
 *           hum_max、fall_g 一直是出厂默认；
 *         · 另一行 NET 里的 'E' 被采成 0x94。
 *       也就是软串口在两个方向都会把字节打歪（另一路嫌疑是 PA10 被两个
 *       推挽输出同时驱动，那是硬件问题，见接线表 §①-2 的"不接"要求）。
 *       代价 1：ESP8266 的启动 ROM 日志也从这根脚出去（74880），
 *       也就是每次 8266 复位都会往 STM32 的 PA10 灌一串乱码；
 *       而且 GPIO15(D8) 是开机 strapping 脚，板子要允许它被驱动高。
 *       代价 2（真正的门槛）：开了 swap 之后 `Serial` 就是这条协议线，
 *       而全工程几十处 `Serial.printf` 日志都写 `Serial` —— **不先把日志
 *       挪到 `Serial1`(GPIO2/GPIO4) 就置 1，日志会直接混进 TL/ISP 字节流里
 *       把每一行弄脏**。所以这不是"一行开关"，是连着日志出口一起的改造。
 *   0 = 软串口 SoftwareSerial(13,15)：日志走板载 USB，不污染协议流，
 *       但位时序在负载下不可靠（上面那两条实测证据就是它造成的）。
 *
 * 【默认 0 = 保持现状】要上硬串口，先做"日志搬家"那一步再置 1。
 * ⚠ 两种版本的固件靠 OTA_FW_VERSION 区分（-hw / -soft，精简版再多一个 -m0），
 *   后台"网关 x.y"那一格就能看出板上是哪一版 ——
 *   别再"以为刷了新固件，其实跑的还是旧的"。 */
#ifndef STM32_LINK_UART0
#define STM32_LINK_UART0        1
#endif

/* 两个串口出口，按上面对话分开：
 *   LINK_SERIAL —— 和 STM32 通话的协议线（硬串口模式的 UART0，或软串口的 SW Serial）
 *   LOG_SERIAL  —— 调试日志（硬串口模式搬到 Serial1/GPIO2，绝不能混进协议流） */
#if STM32_LINK_UART0
#define LINK_SERIAL             Serial
#define LOG_SERIAL              Serial1
#else
#define LINK_SERIAL             g_stm32Serial
#define LOG_SERIAL              Serial
#endif
/* 版本号按"链路用硬串口还是软串口"+"有没有带 MQTT"两维分叉。
 * 【为什么要区分到这么细】2026-11-02 查"8266 每 15 秒 Soft WDT 复位"，
 * 手上唯一的线索就是后台"网关 x.y"那一格 —— 板上到底是带 MQTT 的完整版
 * 还是砍掉它的精简版，必须一眼看出来，否则同一份现象要按两种可能各解释一遍。
 * -m0 = 精简版（OTA_MINIMAL，没有 PubSubClient / WebSocket）。 */
/* 两处都会关掉 MQTT：OTA_MINIMAL 是编译期砍库，GW_ENABLE_MQTT=0 是默认值。
 * 版本号统一按"有没有 MQTT"分叉，后台一眼就能看出板上跑的是哪一版。 */
#if defined(OTA_MINIMAL) || !GW_ENABLE_MQTT
#define OTA_MQTT_DISABLED       1
#else
#define OTA_MQTT_DISABLED       0
#endif

#if STM32_LINK_UART0
#if OTA_MQTT_DISABLED
#define OTA_FW_VERSION          "2.4-hw2-m0"
#else
#define OTA_FW_VERSION          "2.4-hw2"
#endif
#else
#if OTA_MQTT_DISABLED
#define OTA_FW_VERSION          "2.4-soft-m0"
#else
#define OTA_FW_VERSION          "2.4-soft"
#endif
#endif
/* 软串口降级档：默认 115200；完整率 <99% 时用 -DSTM32_UART_BAUD=57600，
 * 同时 STM32 端用 -DSTM32_LINK_BAUD=57600，两侧必须一致（单向切换）。 */
#ifndef STM32_UART_BAUD
#define STM32_UART_BAUD         115200
#endif
#define STM32_UART_TX_PIN       15    /* D8 → STM32 PA10 (USART1_RX) */
#define STM32_UART_RX_PIN       13    /* D7 ← STM32 PA9  (USART1_TX) */

/* 【已删除触发串】
 * 原 STM32_TRIGGER_PREFIX("RF_CMD_OTA_START") / STM32_TRIGGER_ALIAS("OTA?")
 * 是 8266 让 STM32 app 复位进 BL 的兜底路径。单一路径下不再使用：
 * 8266 只预下载 + 等 STM32 自己复位，绝不主动复位 STM32。 */

/* bootloader 开机横幅，形如 "BL v1.2  wait 10s"。
 * 【为什么非看到它不可】
 *   app 运行时 PA9 上全是调试日志，bootloader 也用同一条线。
 *   只有看到横幅才说明"对面已经换成 bootloader 了"，
 *   这之前收到的行一律当应用日志丢掉，一个字节也不回。 */
#define STM32_BANNER_PREFIX     "BL v"

/* ================================================================
 * 四、OTA 协议参数
 * ================================================================ */
/* MCU 的单包上限。bootloader.h 里 RX_CHUNK_MAX = uart_rec_len = 512，
 * 大于这个值 MCU 会 clamp 回 512，于是双方分帧错位 —— 必须 <= 512。 */
#define OTA_CHUNK_MAX           512
#define OTA_DEFAULT_CHUNK       512

/* 单行缓冲长度。MCU 最长的一行是
 * "ER:crc fail calc=0x12345678 recv=0x12345678" = 42 字符，留足余量。 */
#define OTA_LINE_MAX            72

/* 串口行队列长度。bootloader 的 ? 命令一次要回 12 行（OK + 10 个 KV + END），
 * 所以队列不能太小，否则保活一次就把队列塞爆。 */
#define OTA_QUEUE_LEN           16

/* HTTP 下载缓冲（静态，1KB）。ESP8266 内存金贵，绝不把整个固件读进 RAM。 */
#define OTA_BUF_SIZE            1024

/* 【2026-11-04 增量下载】每次 otaTask 被调度到，最多搬这么多字节就返回。
 * 下载不再一口气跑到底：主循环每转一圈都能继续收 TL、继续上报遥测，
 * 后台看设备时"更新下载中"和"正常运行"没有区别。
 * 1KB 在 115200 软串口下不到 10ms 就搬完，对调度几乎没有占用。 */
#define OTA_DL_SLICE_BYTES      1024

/* ---- 超时（毫秒），数值对齐 STM32 侧上位机 tools/firmware_tool.py ----
 *   T_ERASE=30s  T_CHUNK=10s  T_OTA_END=120s
 * 【为什么等 READY 给 30 秒】
 *   MCU 收到 O 之后要先擦 W25Q（44KB → 11 个扇区），要好几秒才回 READY。 */
#define OTA_T_READY_MS          30000
#define OTA_T_CHUNK_MS          10000
#define OTA_T_VERIFY_MS         30000

/* 【保活周期 —— 这是最容易踩的坑】
 *   bootloader 的刷机窗口 BOOT_WAIT_MS 只有 10 秒，而且窗口是
 *   "从最后一次收到整行算起"的：只要在窗口内收到一条认识的命令，
 *   就留在命令行继续等下一条。
 *   固件下载要好几秒到几十秒，下载期间必须定期发 ? 给它续命，
 *   否则 MCU 以为"没人理我"，直接跳去跑 app，OTA 就废了。
 *   ? 的应答是 12 行（OK + 10 个 KV + END），必须一次读干净，
 *   不然残留的行会把后面 O 命令的应答搞乱。 */
#define OTA_T_KEEPALIVE_MS      2000
#define OTA_T_INFO_MS           5000    /* ? 命令的应答超时 */

/* 看到横幅之后，最多等多久还能等到横幅（STM32 复位到出横幅的时间） */
#define OTA_T_BANNER_MS         20000
/* 【开机窗口接管】见到 bootloader 横幅后多久之内，PH_IDLE 允许"绕过 APP
 * 直接进下载"。必须小于 bootloader 的 BOOT_WAIT_MS(10 秒)：
 * 超过这个年龄还去发 ISP 命令，对面其实已经跳 app 了，白忙一场。
 * 留 1 秒余量给串口/调度。 */
#define OTA_BOOT_TAKEOVER_MS    9000

/* 空闲时轮询 /api/iot/config 的间隔（阈值下发 + 云端自主发现升级指令）
 * 【为什么 10 秒而不是 60 秒】
 *   MQTT 下行是主路径（秒级生效），这个轮询只是兜底
 *   （MQTT 断了 / 后台还没推），所以 10 秒足够，别太慢。 */
#define OTA_POLL_MS             10000

/* ---- 遥测转发节奏 ----
 * 【已改为事件驱动】8266 只在收到 STM32 新的一行 TL 时转发一次，
 * 不再按固定 2 秒把缓存里的旧 TL 重复 POST 上云。
 * 设备上报周期由云端 reportIntervalMs 决定，转发节奏自动跟随。
 * （原 OTA_TL_POST_MS=2000 的固定节奏已删除：它会在后台伪造
 *   "每 2 秒一条新遥测"的假象，掩盖 STM32 断流。） */

/* ---- 上报周期（毫秒）的合法范围 ----
 *   云端下发 reportIntervalMs（500..3600000），与后台 THRESHOLD_META
 *   以及 STM32 uart_cmd.c 的 REPORT_MS_MIN/MAX 保持一致。
 *   这个值决定两件事：TH 第 8 列发什么、以及 8266 多久没收到 TL 算断流。 */
#define OTA_REPORT_MS_MIN       500
#define OTA_REPORT_MS_MAX       3600000

/* ================================================================
 * 五、固件暂存
 * ================================================================ */
/* 【为什么要暂存】
 *   O 命令里必须先声明整包的 len 和 crc32 —— MCU 收完才会拿 crc32 比对。
 *   而 crc32 只有把整个文件过一遍才知道。
 *   固件最大 44KB，ESP8266 可用 RAM 只有 40KB 上下，根本装不下。
 *   所以先下载到 LittleFS，顺手把 CRC32 算出来，再从文件里分块喂给 MCU。 */
#define FW_TMP_PATH             "/fw_pending.bin"

/* 暂存文件的"指纹"小文件：记录这半截固件是给哪一版固件续的。
 *   内容："<整包大小> <云端给的CRC32>\n"
 *   下载续传前先对一下：云端现在要传的固件跟本地这半截不是同一份
 *   （换了固件 / 被换过文件），就不能接着下，必须从头来。 */
#define FW_TMP_META_PATH        "/fw_pending.meta"

/* 是否允许 https。
 *   默认 0：ESP8266 的 BearSSL 完整 TLS 栈吃 RAM，40KB 机型上不划算。
 *   【2026-11-02 复测：这个开关不需要打开了】
 *   早年 README 里写的"nginx 把 80 端口全部 301 到 https"已经不成立：
 *   实测 http://<server>/api/iot/config 直接回 401、
 *   http://…/admin/api/iot/firmware/download 直接回 403（都是后端答的，
 *   没有 301）；只有 http://IP/ 这条根路径还 301。
 *   所以设备侧继续走明文 http，TLS 这条路留着当备选，不要默认打开。 */
#ifndef OTA_ALLOW_TLS
#define OTA_ALLOW_TLS           0
#endif

/* 云端配置接口的响应缓冲。里面要装下整个 JSON（含带签名的长 URL）。 */
/* 配置接口响应缓冲。
 * 【2026-11-03 从 1024 提到 2048】后端 config 响应随阈值字段和签名直链增长，
 * 实测已到 1098 字节：1024 的缓冲会把 JSON 尾巴截掉，deserializeJson 直接失败，
 * 表现为「设备在线、遥测照常，但阈值不更新、OTA 永远不下发」。
 * 留一倍余量，以后再加字段也不会再踩。 */
#define CLOUD_RESP_MAX          2048
/* 固件直链 URL 的最大长度（带 HMAC 签名，可能一两百字符）。 */
#define FW_URL_MAX              320

/* ---- 单次 HTTP 请求的超时（连接 + 每次读，单位毫秒）----
 * 【为什么从 15s/8s/20s 收到 4 秒】
 *   这台调度器是协作式的：otaTask 里一次 http 调用卡住，
 *   遥测转发、串口收发、MQTT 全都在原地等 —— 卡 15 秒就是设备
 *   "失联 15 秒"，看上去和死了没区别。
 *   后台 nginx 有限流，慢响应是常态，不是异常；慢的时候正确的做法是
 *   这轮放弃、下一轮（OTA_POLL_MS=10s）再问，而不是把一个请求耗到底。
 *   4 秒 = 比 2 秒的遥测节奏宽裕，又远小于会出问题的十秒级。 */
#define CLOUD_HTTP_TIMEOUT_MS   4000

#endif /* CONFIG_H */
