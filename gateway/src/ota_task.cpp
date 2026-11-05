/*
 * ota_task.cpp —— OTA 总调度
 *
 * ================================================================
 * 一、整体时序（单一路径）
 * ================================================================
 *   ESP8266:
 *     1) 周期拉 /api/iot/config              → 有 pending 指令就开始
 *     2) 下载到 LittleFS + 写 /fw_ready.txt   → 掉电/重启都不丢
 *     3) 等 STM32 **自己**复位进 bootloader   → 看到 BL 横幅，唯一触发条件
 *     4) 发 "O <len> 0x<crc32> 512"          → 等 READY
 *     5) 每 512 字节发一包，每包等一个 OK    → 最后等 verify ok
 *
 *   【只有这一条路径】
 *     · 8266 不主动复位 STM32（不发触发串）；
 *     · 不靠发 ? 探测对面是不是 bootloader；
 *     · 收到新固件先下好等着，设备下一次复位进 BL 时直接推。
 *   这样后台点"下发升级"之后，不需要人和设备同步时机：
 *   固件先落地，什么时候复位，什么时候刷。
 *
 * ================================================================
 * 二、两个最容易踩的坑
 * ================================================================
 * 【坑 1：刷机窗口只有 10 秒】
 *   bootloader 的 BOOT_WAIT_MS 是 10 秒，窗口是"从最后一次**真命令**算起"的
 *   —— 问候行、空行、线路噪声都不算（2026-11-01 修正，之前它们也能续窗口，
 *   结果窗口永远关不上、app 永远进不去）。
 *   固件下载要好几秒到几十秒，下载期间必须定期发 ? 给它续命，
 *   否则 MCU 以为"没人理我"，直接跳去跑 app，OTA 就废了。
 *   ? 的应答有 12 行，必须一次读干净，否则残留的行会把后面 O 的应答搞乱。
 *   ★ 发了 O 之后就不能再发 ? 了 —— 那时候 MCU 正在收数据，
 *     混进去的字节会被当成固件内容，CRC 必错。
 *
 * 【坑 2：O 命令里必须先声明 crc32】
 *   MCU 收完整个镜像才会算 CRC32 再比对，所以 crc32 必须在发 O 之前就知道。
 *   而 crc32 只有把整个文件过一遍才知道 —— 所以必须先把文件落到本地 Flash。
 */
#include <Arduino.h>
#include <LittleFS.h>

#include "app_ctx.h"
#include "config.h"
#include "fw_download.h"
#include "uart_bridge.h"

/* 状态机 */
enum OtaPhase {
    PH_IDLE = 0,
    PH_PREPARE,      /* 问云端、发起下载 */
    PH_DOWNLOAD,     /* 每次只搬一小片固件，搬完就返回（不阻塞设备） */
    PH_SEND,         /* 发 O + 喂数据 */
    PH_RETRY         /* 失败退避 */
};

/* 增量下载器：静态放这里，避免占 otaTask 的栈。
 * 下载期间每轮只搬 OTA_DL_SLICE_BYTES，然后立刻返回 ——
 * STM32 的传感器、屏幕、遥测转发都不受影响。 */
static FwDownloader s_dl;

/* 上报的 JSON 缓冲。静态，不走堆。 */
static char s_status[160];

/* 云端指令结构体有 400 多字节，放栈上太奢侈，挪到静态区。 */
static OtaInstruction s_ins;

/* 保活时读 ? 的应答。静态，避免占栈。 */
static char s_info[256];

/* 引擎是文件级静态：协议引擎内部有个 512 字节的分包缓冲，
 * 放在这里避免占 ota_task 的栈。 */
static OtaEngine s_engine(g_serialSink);

/* 上次发 ? 保活的时间 */
static uint32_t s_lastKeepAlive = 0;
/* 上次 bootloader 明确应答 ? 的时刻（0=还没确认过）。
 * 推固件前必须凭"新鲜横幅"或"刚应答的 ?"确认对面确实还在 BL，
 * 不能只信粘滞的 g_ispMode。 */
static uint32_t s_blAliveMs = 0;
/* 重试退避计数 */
static uint8_t  s_retryCount = 0;
/* 下次空闲轮询 /api/iot/config 的时间（阈值 + 升级指令都靠它） */
static uint32_t s_nextPoll = 0;
/* 上次转发过的 TL 样本时刻（等于当时的 g_lastTlMs）。
 * 同一条 TL 只转发一次，不再按固定 2 秒重复上报。 */
static uint32_t s_lastPostedTlMs = 0;
/* 当前云端配置的上报周期（ms），默认 2 秒。
 * 用于判断「TL 断流」和「传感器数值停更」的窗口 —— 上报周期调慢后，
 * 固定 30 秒的断流门槛会把正常设备误判成链路故障。 */
static uint32_t s_tlExpectedMs = 2000;
/* 下次必须"从头下载"（本地暂存文件和云端指纹对不上时置位） */
static bool     s_forceFullDownload = false;
/* 传感器停更检测：TL 数值指纹 + 停更起始时刻（0=没停更） */
static uint32_t s_tlSig     = 0;
static uint32_t s_tlStaleMs = 0;
/* EVT 组装缓冲 */
static char s_evtBuf[160];
/* STM32 链路是否已经报过「断了」 */
static bool s_linkDown = false;

/* ================================================================
 * OTA 指令的"账"—— 设备端自己记一份
 *
 * 【为什么必须记】后台的 pendingOta 只有收到 success=true 才清空，
 * 而这条指令在 config 里每 10 秒就会被再返回一次。没有本地这份账的话，
 * "同一条指令"和"新下发的一条"看起来一模一样，于是：
 *   轮询命中 → 发触发串 → STM32 复位进 bootloader → 下载 → 推送
 *   → 成功（或失败）→ 回 IDLE → 10 秒后又命中同一条 → 又来一遍
 * 表现就是「STM32 永远起不来看着像死了」「8266 一直在下载刷机器」。
 * 这是本项目最坑的一种失败，因为它每一轮都是"正常"的，日志里全是成功字样。
 *
 *   s_workKey   本轮正在处理的指令
 *   s_doneKey   已经成功刷进去的指令（再来就不重复刷机，只补报结局）
 *   s_quitKey   连续失败后已放弃的指令（等后台重新下发一个新 id 才会再动）
 *   s_failCount 当前指令已连续失败几次
 * ================================================================ */
static char    s_workKey[40]  = {0};
static char    s_doneKey[40]  = {0};
static char    s_quitKey[40]  = {0};
static uint8_t s_failCount    = 0;
static uint32_t s_retireTryMs = 0;   /* 补报结局的节流时刻 */
#define OTA_MAX_FAIL 3U

/* ---- 账本持久化 ----
 * 原来 s_workKey/s_doneKey/s_quitKey 只在 RAM：8266 一旦掉电或软狗复位，
 * 同一条指令会被当成"新指令"再刷一遍。用 LittleFS 存住，重启后先读回来。 */
#define OTA_LEDGER_PATH "/ota_ledger.txt"

static void ledgerSave(void)
{
    File f = LittleFS.open(OTA_LEDGER_PATH, "w");
    if (!f) {
        return;
    }
    f.printf("work=%s\ndone=%s\nquit=%s\nfail=%u\n",
             s_workKey, s_doneKey, s_quitKey, (unsigned)s_failCount);
    f.close();
}

static void ledgerLoad(void)
{
    File f = LittleFS.open(OTA_LEDGER_PATH, "r");
    if (!f) {
        return;
    }
    while (f.available()) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.startsWith("work=")) {
            snprintf(s_workKey, sizeof(s_workKey), "%s", line.c_str() + 5);
        } else if (line.startsWith("done=")) {
            snprintf(s_doneKey, sizeof(s_doneKey), "%s", line.c_str() + 5);
        } else if (line.startsWith("quit=")) {
            snprintf(s_quitKey, sizeof(s_quitKey), "%s", line.c_str() + 5);
        } else if (line.startsWith("fail=")) {
            s_failCount = (uint8_t)line.substring(5).toInt();
        }
    }
    f.close();
    LOG_SERIAL.printf("[ota] 账本恢复 work=%s done=%s quit=%s fail=%u\n",
                  s_workKey[0] ? s_workKey : "-",
                  s_doneKey[0] ? s_doneKey : "-",
                  s_quitKey[0] ? s_quitKey : "-", (unsigned)s_failCount);
}

/* ---- 预下载固件记录 ----
 * 【为什么要把"已经下好了"单独记账】收到更新就先把固件下进 LittleFS，
 * 等 STM32 进 bootloader 时直接推 —— 而不是"进了 BL 才开始下载"。
 * 这样 BL 的 10 秒窗口 / 120 秒保活不用陪着下载，8266 掉电或断网也不丢
 * 已经下好的固件，下次上电看到横幅就能直接推。
 * 记录里必须带指令身份（key）+ 版本 + 大小 + CRC：只有和后台这一条
 * 指令完全对上才算数，后台换了固件就作废重下。
 */
#define FW_READY_PATH "/fw_ready.txt"
static bool s_fwReady = false;
static char s_fwReadyKey[40]  = {0};
static char s_fwReadyVer[32]  = {0};
static uint32_t s_fwLen = 0;
static uint32_t s_fwCrc = 0;

static void fwReadySave(const char* key, const char* ver, uint32_t len, uint32_t crc)
{
    File f = LittleFS.open(FW_READY_PATH, "w");
    if (!f) {
        return;
    }
    f.printf("key=%s\nver=%s\nlen=%u\ncrc=%u\n",
             key, ver, (unsigned)len, (unsigned)crc);
    f.close();
    snprintf(s_fwReadyKey, sizeof(s_fwReadyKey), "%s", key);
    snprintf(s_fwReadyVer, sizeof(s_fwReadyVer), "%s", ver);
    s_fwReady = true;
    LOG_SERIAL.printf("[ota] 固件已预下载就绪 %s (%u 字节)\n",
                      ver, (unsigned)len);
}

static void fwReadyLoad(void)
{
    File f = LittleFS.open(FW_READY_PATH, "r");
    if (!f) {
        return;
    }
    uint32_t len = 0;
    uint32_t crc = 0;
    while (f.available()) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.startsWith("key=")) {
            snprintf(s_fwReadyKey, sizeof(s_fwReadyKey), "%s", line.c_str() + 4);
        } else if (line.startsWith("ver=")) {
            snprintf(s_fwReadyVer, sizeof(s_fwReadyVer), "%s", line.c_str() + 4);
        } else if (line.startsWith("len=")) {
            len = (uint32_t)line.substring(4).toInt();
        } else if (line.startsWith("crc=")) {
            crc = (uint32_t)line.substring(4).toInt();
        }
    }
    f.close();
    if (len > 0) {
        s_fwLen  = len;
        s_fwCrc  = crc;
        s_fwReady = true;
        LOG_SERIAL.printf("[ota] 恢复预下载固件 ver=%s len=%u（等 STM32 进 BL 直接推）\n",
                          s_fwReadyVer, (unsigned)len);
    }
}

static void fwReadyClear(void)
{
    s_fwReady = false;
    s_fwReadyKey[0] = '\0';
    s_fwReadyVer[0] = '\0';
    LittleFS.remove(FW_READY_PATH);
}

/* 一条指令的身份串：后台有 id 就用 id（每次 dispatch 一个新的 UUID），
 * 没有 id（老后台）就退化成 ver+大小+CRC 拼出来的指纹，同样能认出"是同一条"。 */
static void otaKeyOf(const OtaInstruction* ins, char* out, size_t cap)
{
    if (ins->id[0] != '\0') {
        snprintf(out, cap, "%s", ins->id);
    } else {
        snprintf(out, cap, "v%s-%lu-%08lX", ins->ver,
                 (unsigned long)ins->size, (unsigned long)ins->crc32);
    }
}

/* 让 bootloader 放手去跑 app（G 命令）。
 * 只在"已经进了 BL、但这一轮提前结束"（下载失败/放弃）时用；
 * 升级成功时 bootloader 自己跳去 app，不需要发。 */
static void releaseBootloader(void)
{
    if (!g_ispMode) {
        return;
    }
    g_serialSink.write((const uint8_t*)"G\n", 2);
    LOG_SERIAL.println("[ota] 已发 G 让 bootloader 去跑 app");
}

/* 把云端阈值转发给 STM32 app。
 * 【协议】"TH <tmax×100> <tmin×100> <hmax×100> <hmin×100> <fall×100> <alarm> <reportSec> <reportMs>\n"
 * 【只整数不带小数点】app 侧用 strtol 解析，不把 float scanf 拉进 40K 固件。
 * 【第 7 列是整数秒、给旧固件看；第 8 列是毫秒、新的精确值】
 *   亚秒上报（0.5 秒）靠第 8 列表达：reportSec=1、reportMs=500。
 *   第 8 列只有在毫秒值合法（100..60000）时才带，
 *   否则只发 7 列（老固件解析到第 7 个数就停，多余的字段它会忽略）。
 * 【只在 app 在跑时发】g_ispMode=true 说明对面是 bootloader，
 * TH 不是 ISP 命令，发过去只会换来 ER:unknown cmd 把它踢去跑 app。 */
/* 【T30 阈值指纹】阈值没变就不要每 10 秒重发一次 TH：
 * STM32 每次收到 TH 都会蜂鸣，现场表现就是"每隔 10 秒响一声"，
 * 同时也会把 8266→STM32 的软串口带宽白占掉。 */
static uint32_t s_thFp = 0;
static bool     s_thFpValid = false;
static uint32_t s_thLastSendMs = 0;

/* 指纹没变也定期重发。STM32 单独复位（OTA/掉电/看门狗）后阈值会回到
 * 它自己的默认值，而 8266 这边的指纹还记着上一次 —— 原来就再也不发，
 * 现场表现正是「网页上温度已超阈值，蜂鸣器却不响」。
 * 重发一条 TH 只有几十字节，60 秒一次完全可以忽略。 */
#define TH_RESEND_MS    60000UL

static uint32_t thFingerprint(const OtaInstruction* ins)
{
    const int32_t v[10] = {
        (int32_t)ins->thTempMax,  (int32_t)ins->thTempMin,
        (int32_t)ins->thHumMax,   (int32_t)ins->thHumMin,
        (int32_t)ins->thPressMax, (int32_t)ins->thPressMin,
        (int32_t)ins->thFall,     (int32_t)ins->thAlarmEn,
        (int32_t)ins->thReportSec, ins->thReportMs
    };
    uint32_t h = 2166136261UL;          /* FNV-1a */
    const uint8_t* p = (const uint8_t*)v;
    for (uint32_t i = 0; i < (uint32_t)sizeof(v); i++) {
        h ^= p[i];
        h *= 16777619UL;
    }
    return h;
}

void forwardThresholds(const OtaInstruction* ins)
{
    char line[96];
    char pline[48];
    int n, np;
    uint32_t fp;
    uint32_t now;

    if (ins == NULL || !ins->hasTh) {
        return;
    }

    /* 记住云端配置的上报周期：TL 转发与断流检测都按它来 */
    if (ins->thReportMs >= OTA_REPORT_MS_MIN && ins->thReportMs <= OTA_REPORT_MS_MAX) {
        s_tlExpectedMs = (uint32_t)ins->thReportMs;
    }

    now = millis();
    fp = thFingerprint(ins);
    if (s_thFpValid && fp == s_thFp && (uint32_t)(now - s_thLastSendMs) < TH_RESEND_MS) {
        return;     /* 值没变且还没到重发周期 */
    }
    if (g_ispMode) {
        /* 对面是 bootloader，不是 app —— TH 发过去没有意义 */
        return;
    }

    /* 【开头垫两个换行 —— 这两个字节就是拿来被采坏的】
     * 实测这条软串口链路每串数据的**头 1~2 个字节**会失真（STM32 侧抓到过
     * "TH 1000 400 …" 的 'T''H' 被采成 0x00 0xa4，数字全完好）。
     * 收端光"跳过垃圾字节"救不回来 —— 坏掉的正是 token 本身。
     * 所以由发端先送两个换行：STM32 的空行会被丢掉，被采坏的也是这两个空行，
     * 真正的 "TH " 就落在完好位置上。STM32 发 TL 时做了同样的处理。 */
    if (ins->thReportMs >= OTA_REPORT_MS_MIN && ins->thReportMs <= OTA_REPORT_MS_MAX) {
        n = snprintf(line, sizeof(line),
                     "\n\nTH %d %d %d %d %d %d %d %ld\n",
                     (int)ins->thTempMax, (int)ins->thTempMin,
                     (int)ins->thHumMax, (int)ins->thHumMin,
                     (int)ins->thFall, (int)ins->thAlarmEn,
                     (int)ins->thReportSec, (long)ins->thReportMs);
    } else {
        n = snprintf(line, sizeof(line),
                     "\n\nTH %d %d %d %d %d %d %d\n",
                     (int)ins->thTempMax, (int)ins->thTempMin,
                     (int)ins->thHumMax, (int)ins->thHumMin,
                     (int)ins->thFall, (int)ins->thAlarmEn,
                     (int)ins->thReportSec);
    }
    if (n > 0) {
        g_serialSink.write((const uint8_t*)line, (size_t)n);
        LOG_SERIAL.printf("[ota] 阈值已转发: %s", line);

        /* 气压阈值单独一行 PB（详见 uart_cmd.c handle_pressure 的说明）。 */
        np = snprintf(pline, sizeof(pline), "\n\nPB %ld %ld\n",
                      (long)ins->thPressMax, (long)ins->thPressMin);
        if (np > 0) {
            g_serialSink.write((const uint8_t*)pline, (size_t)np);
            LOG_SERIAL.printf("[ota] 气压阈值已转发: %s", pline);
        }

        s_thFp = fp;            /* 只有真正写出去才记账，失败下一轮会重试 */
        s_thFpValid = true;
        s_thLastSendMs = now;
    }
}

/* ================================================================
 * 给 APP 屏幕的 OTA 状态行
 *
 * OA <state> <pct>：1=下载中 2=固件已就绪(等复位升级) 4=失败
 * 【只在 app 在跑时发】g_ispMode=true 说明对面是 bootloader，
 * 那时串口上走的是 ISP 协议，混入 ASCII 行会被当固件字节。
 * bootloader 自己也会显示"升级中"+进度条，所以真正刷写阶段不靠这条。
 * 相同状态不重复发，避免每帧刷屏占串口。
 * ================================================================ */
static uint8_t s_appOtaState = 0U;
static uint8_t s_appOtaPct   = 0xFFU;
static uint32_t s_appOtaLastMs = 0;

static void notifyAppStatus(uint8_t state, uint8_t pct)
{
    char line[32];
    uint32_t now = millis();

    if (g_ispMode) {
        return;
    }
    /* 相同状态 60 秒内不重发；超过 60 秒补一条，
     * 让 APP 那边的"状态超时"不会把"固件已就绪"误清掉。 */
    if ((state == s_appOtaState) && (pct == s_appOtaPct) &&
        ((uint32_t)(now - s_appOtaLastMs) < 60000UL)) {
        return;
    }
    s_appOtaState  = state;
    s_appOtaPct    = pct;
    s_appOtaLastMs = now;
    snprintf(line, sizeof(line), "\n\nOA %u %u\n", (unsigned)state, (unsigned)pct);
    g_serialSink.write((const uint8_t*)line, strlen(line));
}

/* 当前阶段与几个截止时刻 */
static OtaPhase  s_phase = PH_IDLE;

/* uart_task 要用它区分"真在刷机"和"开机残留的 ISP 标志" */
bool otaSessionActive(void)
{
    return (s_phase != PH_IDLE);
}
static uint32_t  s_notBefore = 0;     /* 退避结束时间 */
static uint32_t  s_wifiDeadline = 0;  /* 等 WiFi 的截止时刻 */
/* 【开机窗口接管】上次收到 bootloader 横幅的时刻。
 * bootloader 每次上电都会打 "BL v2.1 wait 10s"（不只是 OTA hold 分支），
 * 这条横幅落到 uart_task 里就是 OTA_EVT_BANNER。有了这个时间戳，
 * PH_IDLE 就能分清"对面此刻真的是 bootloader（可以绕过 APP 直接刷）"
 * 和"只是 30 秒前那条残留的 ISP 标志"。 */
static uint32_t  s_bannerMs = 0;
static bool      s_bannerSeen = false;

/* 【诊断用】最近一次失败的阶段 + 现场。
 * 原来放弃时只报一句 "abandoned after retries"，后台看不出到底是
 * 下载失败、等横幅超时、还是 bootloader 回了 ER —— 只能拔板子接串口。
 * 把这一行带进 /api/iot/ota/result 的 message 和事件 detail，
 * 下一次失败在网页上就能直接看到原因。 */
static char      s_lastFail[128];

/* 【2026-11-03】bootloader 在会话里回的最后一行 + 累计行数。
 * 现场症状：协议报 st=3（等 READY 超时）、lastError 为空 —— 连 ER 都没收到。
 * 光看 st 分不清是「O 没送到 STM32」还是「送到了但 bootloader 没理」。
 * 把引擎收到的每一行都留一条，失败时随 message 上报：
 *   bl=0:...     → 一条都没收到（RX 通路的问题）
 *   bl=N:BL:xxx  → 收到了 N 行，最后一行是 BL:xxx（说明通路通，是命令内容/时序的问题）*/
static char      s_blLine[48];
static uint16_t  s_blLines;
/* 【单独留 ER 行】s_blLine 只保最后一行，而 bootloader 回完 ER 之后
 * 往往还会接着打 "BL:cmd" 之类的行把它覆盖掉（现场就是这样，上报里
 * 只能看到 BL:cmd，定罪全靠猜）。失败原因一律以 ER 开头，单独存一份。 */
static char      s_blEr[52];

static void reportStatus(const char* state, const char* extra)
{
    if (extra != NULL && extra[0] != '\0') {
        snprintf(s_status, sizeof(s_status),
                 "{\"dev\":\"%s\",\"state\":\"%s\",\"info\":\"%s\"}",
                 DEVICE_ID, state, extra);
    } else {
        snprintf(s_status, sizeof(s_status),
                 "{\"dev\":\"%s\",\"state\":\"%s\"}", DEVICE_ID, state);
    }
    LOG_SERIAL.printf("[ota] %s\n", s_status);
    mqttPublishStatus(s_status);
}

/* 协议引擎每读到一行都会调这个：打日志 + 留最后一条给失败上报 */
static void onMcuLine(const char* line, void* ctx)
{
    (void)ctx;
    s_blLines++;
    strncpy(s_blLine, line, sizeof(s_blLine) - 1);
    s_blLine[sizeof(s_blLine) - 1] = '\0';
    /* 【只留第一条 ER】失败链路常常会连打两行，比如
     *   ER:verify fail calc=0x... recv=0x...
     *   ER:rollback
     * 后一行会把真正的原因盖掉。第一条才是"哪一步挂的"。 */
    if (s_blEr[0] == '\0' && strncmp(line, "ER", 2) == 0) {
        strncpy(s_blEr, line, sizeof(s_blEr) - 1);
        s_blEr[sizeof(s_blEr) - 1] = '\0';
    }
    LOG_SERIAL.printf("[mcu] %s\n", line);
}

/* ================================================================
 * 保活：发一次 ? 并把 12 行应答读干净
 * ================================================================ */
static void keepAlive(void)
{
    uint32_t now = millis();

    if (!g_ispMode) {
        return;
    }
    if ((int32_t)(now - s_lastKeepAlive) < OTA_T_KEEPALIVE_MS) {
        return;
    }
    s_lastKeepAlive = now;

    /* queryDeviceInfo 会一直读到 END，正好把 ? 的整份应答清干净。 */
    if (s_engine.queryDeviceInfo(s_info, sizeof(s_info), OTA_T_INFO_MS)) {
        s_blAliveMs = millis();          /* 对面确实是 BL，续上 120s 窗口 */
        LOG_SERIAL.println("[ota] 保活 ? 已应答");
    } else {
        LOG_SERIAL.println("[ota] 保活 ? 没应答");
    }
}

/* ================================================================
 * 下载进度回调：喂看门狗 + 保活
 * ================================================================ */
static bool onDownloadProgress(uint32_t done, uint32_t total, void* ctx)
{
    (void)ctx;

    /* 每次进度回调都喂一次看门狗：下载路径会嵌套调度，
     * 单次 LittleFS 写入 + 串口保活加起来不能保证始终低于 6.3 秒。 */
    ESP.wdtFeed();

    /* 【保活必须放在这里】
     * 下载是唯一一段"长时间不碰串口"的空档，
     * 而 bootloader 的刷机窗口只有 10 秒 —— 窗口就在这里过期。 */
    keepAlive();

    if (total > 0) {
        uint8_t pct = (uint8_t)((done * 100UL) / total);

        if (pct > 100U) {
            pct = 100U;
        }
        /* 每 5% 推一次屏幕状态，别让 100 条小行把串口占满 */
        notifyAppStatus(1U, (uint8_t)((pct / 5U) * 5U));
        LOG_SERIAL.printf("[ota] 下载 %u/%u\n", (unsigned)done, (unsigned)total);
    } else {
        LOG_SERIAL.printf("[ota] 下载 %u 字节\n", (unsigned)done);
    }
    return true;   /* 继续下载 */
}

/* ================================================================
 * 喂完数据之后的"收尾观察"
 *
 * 【为什么要多看一会儿】
 *   当前 bootloader 的 "verify ok" 只代表"W25Q 里那份的 CRC32 对上了"，
 *   后面还有备份旧 app、擦内部 Flash、搬运、回读校验三步。
 *   成功时 MCU 不再输出任何东西（它直接跳去跑 app 了），
 *   失败时会打 "ER:backup ..." / "ER:flash ..." / "ER:verify fail ..."。
 *   所以这里再读一会儿，把失败信息抓出来，
 *   别把"没报错"当成"升级成功"报上去。
 * ================================================================ */
static bool settleAndCheck(char* errOut, size_t errCap)
{
    uint32_t start = millis();
    bool sawBackupOk = false;

    errOut[0] = '\0';

    while (millis() - start < 60000UL) {
        char line[OTA_LINE_MAX];

        if (!g_serialSink.readLine(line, sizeof(line), 1000)) {
            /* 1 秒没动静。多等一会儿就收工 —— MCU 成功时本来就没输出。 */
            if (sawBackupOk && (millis() - start > 20000UL)) {
                break;
            }
            continue;
        }
        LOG_SERIAL.printf("[mcu] %s\n", line);

        if (strncmp(line, "ER", 2) == 0) {
            strncpy(errOut, line, errCap - 1);
            errOut[errCap - 1] = '\0';
            return false;
        }
        if (strncmp(line, "backup ok", 9) == 0) {
            sawBackupOk = true;
        }
    }
    return true;
}

/* ================================================================
 * 遥测转发 + 传感器停更检测 + 链路通断检测
 *
 * 【为什么单独抽成一个函数，而不是留在 PH_IDLE 里】
 *   下载固件那几十秒里状态机处在 PH_PREPARE / PH_DOWNLOAD，
 *   如果这段代码只写在 PH_IDLE 分支里，下载期间就一次都不会执行 ——
 *   后台看到的表现正是"一下发更新，传感器就像停了"。
 *   现在不管处于哪个阶段，每轮调度都会调它，
 *   设备在下载更新的同时照常上报遥测、照常刷新屏幕。
 * ================================================================ */
static void serviceTelemetry(void)
{
    if (!wifiIsConnected() || g_ispMode || g_lastTelemetry[0] == '\0') {
        return;
    }

    /* 指纹：TL 的前 5 个整数简单 hash（值不变 hash 不变） */
    uint32_t fp = 0;
    int nv = 0;
    const char* q = g_lastTelemetry + 3;
    while (nv < 5) {
        char* end;
        long val = strtol(q, &end, 10);
        if (end == q) break;
        fp = fp * 31UL + (uint32_t)val;
        q = end;
        nv++;
    }
    uint32_t staleWin = s_tlExpectedMs * 2UL + 5000UL;
    if (staleWin < 30000UL) staleWin = 30000UL;

    if (fp != s_tlSig) {
        if (s_tlSig != 0 && s_tlStaleMs != 0 &&
            (int32_t)(millis() - s_tlStaleMs) > (int32_t)staleWin) {
            /* 停更超过窗口后又动了 = 传感器插回来了 */
            snprintf(s_evtBuf, sizeof(s_evtBuf),
                     "EVT sensor_recovered tl=%s", g_lastTelemetry);
            evqPush(s_evtBuf);
            LOG_SERIAL.println("[tl] 传感器恢复，已记 EVT");
        }
        s_tlSig     = fp;
        s_tlStaleMs = 0;
    } else if (s_tlStaleMs == 0) {
        s_tlStaleMs = millis();        /* 开始停更计时 */
    }

    /* 同一条 TL 只转发一次；新样本到达（g_lastTlMs 变了）才发。 */
    if (g_lastTlMs != 0 && g_lastTlMs != s_lastPostedTlMs) {
        s_lastPostedTlMs = g_lastTlMs;
        if (cloudPostTelemetry(g_lastTelemetry)) {
            LOG_SERIAL.println("[tl] 遥测已上云");
        }
        /* 失败不重试到死：下一条 TL 到达时自然再发 */
    }

    /* ---- STM32 链路通断检测 ----
     * 串口不通时（引脚错/swap 没生效/STM32 停在 bootloader），
     * 后台一条遥测都收不到，但设备照样在线、照样拉阈值 —— 极易误判。
     * 这里自己判断并上报事件：
     *   · 启动后一条 TL 都没有 → stm32_link_down
     *   · 曾经有、现在断流超窗口 → stm32_link_down
     *   · 恢复 → stm32_link_up */
    {
        uint32_t now = millis();
        /* 断流窗口跟随上报周期：默认 2 秒时仍是 30 秒；
         * 配成 60 秒一报时窗口是 125 秒，不会把正常设备误判成断链。 */
        uint32_t linkWin = s_tlExpectedMs * 2UL + 5000UL;
        if (linkWin < 30000UL) linkWin = 30000UL;
        bool linkUp = (g_lastTlMs != 0) && ((int32_t)(now - g_lastTlMs) < (int32_t)linkWin);
        if (!linkUp && now > (linkWin + 30000UL) && !s_linkDown) {
            s_linkDown = true;
            snprintf(s_evtBuf, sizeof(s_evtBuf),
                     "EVT stm32_link_down last_tl_ms=%lu",
                     (unsigned long)g_lastTlMs);
            evqPush(s_evtBuf);
            LOG_SERIAL.println("[link] STM32 串口没数据（TL 断流）");
        } else if (linkUp && s_linkDown) {
            s_linkDown = false;
            snprintf(s_evtBuf, sizeof(s_evtBuf),
                     "EVT stm32_link_up tl=%s", g_lastTelemetry);
            evqPush(s_evtBuf);
            LOG_SERIAL.println("[link] STM32 串口恢复");
        }
    }
}

/* ================================================================
 * 任务主体：每次被调度到就干一段，然后立刻返回
 * ================================================================ */
void otaTask(void)
{
    OtaEvent ev;

    /* 【只做一次】把协议引擎收到的每一行都打进日志。
     * 断点续传时最重要的几行（PROG=... / RESUME <written> <chunk>）就靠它看。 */
    static bool s_cbReady = false;
    if (!s_cbReady) {
        s_engine.setLineCallback(onMcuLine, NULL);
        s_cbReady = true;
    }

    /* 账本只从 LittleFS 读一次；必须在任何指令判断之前完成。 */
    static bool s_ledgerReady = false;
    if (!s_ledgerReady) {
        ledgerLoad();
        /* 预下载记录同样只读一次：8266 复位后文件还在 LittleFS，
         * 读回来就能在下次看到 BL 横幅时直接推，不必重下。 */
        fwReadyLoad();
        s_ledgerReady = true;
    }

    /* ---------------- 收事件 ---------------- */
    while (g_otaEventQueue.pop(&ev, 0)) {
        if (ev.type == OTA_EVT_BANNER) {
            LOG_SERIAL.println("[ota] bootloader 已就绪");
            /* 记下"上次亲眼见到 bootloader 的时刻"，PH_IDLE 靠它判断
             * 「现在是不是开机那 10 秒窗口」——见下面 PH_IDLE 的接管分支。 */
            s_bannerMs   = millis();
            s_bannerSeen = true;
        } else if (ev.type == OTA_EVT_MQTT) {
            if (s_phase == PH_IDLE) {
                s_phase = PH_PREPARE;
                s_wifiDeadline = millis() + 30000;   /* 等 WiFi 的截止时间 */
                s_blLines = 0;                       /* 新会话：从零开始记收到的行 */
                s_blLine[0] = '\0';
                s_blEr[0]   = '\0';
                LOG_SERIAL.println("[ota] 收到升级请求");
            }
        }
    }

    /* ---------------- 每轮都先转发遥测 ----------------
     * 【必须放在 switch 之前】下载/退避期间状态机不是 PH_IDLE，
     * 放在分支里就轮不到执行；放在这里，不管设备在下载更新还是
     * 在正常待机，云端都能按 STM32 的真实节奏收到遥测。
     * 进 bootloader 之后 serviceTelemetry 自己会返回（串口上是 ISP 协议）。 */
    serviceTelemetry();

    /* ---------------- 退避没到就先歇着 ---------------- */
    if ((int32_t)(millis() - s_notBefore) < 0) {
        /* 退避期间也要保活 —— bootloader 的刷机窗口只有 10 秒 */
        keepAlive();
        return;
    }

    switch (s_phase) {
        case PH_IDLE: {
            /* 空闲时把队列里残留的行丢掉，别让它们堆着占地方 */
            g_serialSink.flushInput();
            /* 顺手等一小会儿事件（等待期间别的任务照常跑） */
            g_otaEventQueue.pop(&ev, 50);

            /* 【预下载件 + BL 横幅 = 立刻推】不等云端、不等下一轮轮询。
             * bootloader 的等待窗口只有 10 秒，慢一拍就只能再等下一次复位。
             * 横幅由 uart_task 置 s_bannerSeen/g_ispMode，这里 50ms 一轮就看得到。 */
            if (s_fwReady && !g_safeMode && s_bannerSeen && g_ispMode &&
                (uint32_t)(millis() - s_bannerMs) < OTA_BOOT_TAKEOVER_MS) {
                LOG_SERIAL.println("[ota] 预下载件就绪且看到 BL 横幅，立即推送");
                s_blLines       = 0;
                s_blLine[0]     = '\0';
                s_blEr[0]       = '\0';
                s_phase         = PH_SEND;
                break;
            }

            /* 【周期轮询】S3 不在时的云端自主升级链路：
             * 定期拉 /api/iot/config —— 阈值转发给 app；发现待升级固件
             * 就先把固件下到本地，等 STM32 下一次复位进 BL 时直接推。 */
            if (wifiIsConnected() &&
                (int32_t)(millis() - s_nextPoll) >= 0) {
                s_nextPoll = millis() + OTA_POLL_MS;

                if (cloudFetchOta(&s_ins)) {
                    if (s_ins.hasTh) {
                        forwardThresholds(&s_ins);
                    }
                    if (s_ins.valid) {
                        char key[40];
                        static bool s_quitLogged = false;

                        otaKeyOf(&s_ins, key, sizeof(key));

                        if (strcmp(key, s_doneKey) == 0) {
                            /* 这条已经刷成功了，只是后台那边还没销掉。
                             * 再发一次触发串就是把已经在跑的 STM32 打回复位重刷 ——
                             * 补报结局（60 秒一次），报丢了就下一轮再补，直到它消失。 */
                            if ((int32_t)(millis() - s_retireTryMs) >= 0) {
                                s_retireTryMs = millis() + 60000UL;
                                LOG_SERIAL.println("[ota] 该指令已成功过，补报结局销账");
                                notifyAppStatus(0U, 0U);   /* 屏幕：清掉升级状态 */
                                cloudPostOtaResult(s_ins.id, true, s_ins.ver,
                                                   "already applied");
                                releaseBootloader();
                            }
                        } else if (strcmp(key, s_quitKey) == 0) {
                            /* 为这条已经连败放弃过了。后台重新下发会换一个新 id，
                             * 那时才会再动。 */
                            if (!s_quitLogged) {
                                s_quitLogged = true;
                                LOG_SERIAL.println("[ota] 该指令已放弃，等后台重新下发");
                            }
                            notifyAppStatus(4U, 0U);   /* 屏幕：升级失败（此时 app 在跑，能收到） */
                            releaseBootloader();
                        } else {
                            s_quitLogged = false;
                            if (strcmp(key, s_workKey) != 0) {
                                /* 换了一条新指令 → 失败计数从头算 */
                                s_failCount = 0;
                                snprintf(s_workKey, sizeof(s_workKey), "%s", key);
                                ledgerSave();
                            }

                            /* 本地预下载件是不是后台现在这条指令？
                             * 不是（后台换了固件 / 换了新 id）就作废重下，
                             * 绝不拿旧文件去刷。 */
                            if (s_fwReady && strcmp(s_fwReadyKey, key) != 0) {
                                LOG_SERIAL.println("[ota] 预下载件与当前指令不符，作废重下");
                                fwReadyClear();
                                fwDiscardPending();
                                s_forceFullDownload = true;
                            }

                            /* 【安全模式下不碰 STM32 的复位】连续复位保护期间
                             * 只保持遥测/阈值，等设备稳定 60 秒退出安全模式再说。 */
                            if (g_safeMode) {
                                LOG_SERIAL.println("[ota] 安全模式：跳过本轮升级");
                            } else if (s_bannerSeen && g_ispMode &&
                                       (uint32_t)(millis() - s_bannerMs) < OTA_BOOT_TAKEOVER_MS) {
                                /* 【bootloader 刚进窗口 → 直接推】
                                 * 只认横幅。不发 ? 探测、不发触发串：
                                 * 设备复位进 BL 是唯一入口，时机完全由用户/设备决定。
                                 * BL 的保活由 PH_PREPARE 下载期间的 ? 承担。 */
                                LOG_SERIAL.println("[ota] bootloader 在场，直接推固件");
                                g_ispMode       = true;
                                s_blLines       = 0;
                                s_blLine[0]     = '\0';
                                s_blEr[0]       = '\0';
                                if (s_fwReady) {
                                    /* 已经预下载好了：不碰网络，直接 O/W */
                                    s_phase = PH_SEND;
                                } else {
                                    /* 还没下：趁 BL 在等，先下载（? 保活续命） */
                                    s_phase        = PH_PREPARE;
                                    s_wifiDeadline = millis() + 30000;
                                }
                                ledgerSave();
                            } else if (!s_fwReady) {
                                /* 【关键：先下载，跟 STM32 在不在 BL 无关】
                                 * 收到更新就把固件下进 LittleFS，下好记账。
                                 * 这样下一次 STM32 复位进 BL 时是"直接推"，
                                 * 而不是让 BL 干等我们下 40KB。 */
                                LOG_SERIAL.println("[ota] 云端有新固件，先预下载到本地");
                                s_phase        = PH_PREPARE;
                                s_wifiDeadline = millis() + 30000;
                            } else {
                                /* 【单一路径的等待点】
                                 * 固件已经在本地了：不发触发串、不主动复位 STM32。
                                 * 设备什么时候复位进 BL，就什么时候刷。 */
                                LOG_SERIAL.println("[ota] 固件已就绪，等 STM32 下一次复位进 bootloader");
                                notifyAppStatus(2U, 100U);   /* 定期续一下"升级中"，别让 APP 超时清掉 */
                            }
                        }
                    }
                }
            }

            /* ---- 事件转发：把队列里的事件逐条 POST 到 /api/iot/event ---- */
            if (wifiIsConnected() && !g_ispMode) {
                char evt[96];
                while (evqPop(evt, (int)sizeof(evt))) {
                    if (!cloudPostEvent(evt)) {
                        /* 发送失败：塞回队列头，下个周期再试（最多留一条） */
                        evqPush(evt);
                        break;
                    }
                    LOG_SERIAL.printf("[evt] 已上云: %s\n", evt);
                }
            }
            break;
        }

        case PH_PREPARE:
            if (!wifiIsConnected()) {
                if ((int32_t)(millis() - s_wifiDeadline) >= 0) {
                    LOG_SERIAL.println("[ota] 等 WiFi 超时");
                    s_phase = PH_RETRY;
                }
                break;                      /* 下次调度再来 */
            }

            if (!cloudFetchOta(&s_ins)) {
                /* HTTP/JSON 失败 —— 值得重试 */
                reportStatus("cloud_error", "");
                s_phase = PH_RETRY;
                break;
            }
            if (!s_ins.valid) {
                /* 云端说没有待升级的固件 —— 正常情况，不是错误。
                 * 顺带把这次拿到的阈值转给 app（app 在跑才有意义） */
                if (s_ins.hasTh) {
                    forwardThresholds(&s_ins);
                }
                reportStatus("idle", "no pending firmware");
                s_phase = PH_IDLE;
                break;
            }

            reportStatus("downloading", s_ins.ver);
            notifyAppStatus(1U, 0U);       /* 屏幕：下载中 0% */

            /* 【非阻塞下载的第一步】这里只发起 HTTP 请求，不搬数据。
             *   真正的字节搬运在 PH_DOWNLOAD：每轮调度只搬 OTA_DL_SLICE_BYTES，
             *   搬完立刻返回，所以下载期间 uart_task 照常收 TL、
             *   serviceTelemetry() 照常上报，STM32 的传感器和屏幕完全不受影响。
             * 【断点续传】本地已有半截固件时 begin() 会发 Range 续下；
             *   s_forceFullDownload 为真（上次发现文件和云端指纹对不上）就从头来。 */
            if (!s_dl.begin(s_ins.url, s_ins.size, s_ins.crc32,
                            s_forceFullDownload, onDownloadProgress, NULL)) {
                LOG_SERIAL.println("[ota] 发起下载失败");
                snprintf(s_lastFail, sizeof(s_lastFail), "download_failed");
                reportStatus("download_failed", s_ins.ver);
                s_phase = PH_RETRY;
                break;
            }
            s_forceFullDownload = false;
            s_phase = PH_DOWNLOAD;         /* 下一轮开始一片一片地搬 */
            break;

        case PH_DOWNLOAD: {
            /* 【下载期间设备完全无感】
             *   · step() 只搬 1KB 就返回，主循环照常转 uart/mqtt/遥测；
             *   · 不碰 STM32：不进 bootloader、不改 BKP、不复位；
             *   · 固件只是落在 8266 自己的 LittleFS 里，等用户按 RESET 才刷。 */
            FwDownloader::Result r = s_dl.step(OTA_DL_SLICE_BYTES);

            if (r == FwDownloader::BUSY) {
                break;                     /* 让出去，下一轮继续 */
            }

            s_fwLen = s_dl.done();
            s_fwCrc = s_dl.crc32();

            if (r == FwDownloader::FAILED) {
                LOG_SERIAL.println("[ota] 下载失败（半截已落盘，下次自动续传）");
                snprintf(s_lastFail, sizeof(s_lastFail), "download_failed");
                reportStatus("download_failed", s_ins.ver);
                s_phase = PH_RETRY;
                break;
            }

            /* 云端给了大小就必须对得上，对不上说明拿到的不是那个文件 */
            if (s_ins.size != 0 && s_ins.size != s_fwLen) {
                LOG_SERIAL.printf("[ota] 大小不符: 云端 %u, 实下 %u\n",
                              (unsigned)s_ins.size, (unsigned)s_fwLen);
                fwDiscardPending();
                s_forceFullDownload = true;
                snprintf(s_lastFail, sizeof(s_lastFail),
                         "size_mismatch cloud=%u got=%u",
                         (unsigned)s_ins.size, (unsigned)s_fwLen);
                reportStatus("size_mismatch", s_ins.ver);
                s_phase = PH_RETRY;
                break;
            }

            /* 云端给过 CRC32（上传时算好入库的）就和本地算的对账，
             * 对不上说明文件在传输中损坏 —— 别把它喂给 MCU 白忙一场。
             * 【顺手把坏文件丢掉】下一次从头下载，别拿同一份坏文件续传。 */
            if (s_ins.crc32 != 0 && s_ins.crc32 != s_fwCrc) {
                LOG_SERIAL.printf("[ota] CRC32 不符: 云端 0x%08X, 实算 0x%08X\n",
                              (unsigned)s_ins.crc32, (unsigned)s_fwCrc);
                fwDiscardPending();
                s_forceFullDownload = true;
                snprintf(s_lastFail, sizeof(s_lastFail),
                         "crc_mismatch cloud=0x%08X got=0x%08X",
                         (unsigned)s_ins.crc32, (unsigned)s_fwCrc);
                reportStatus("crc_mismatch", s_ins.ver);
                s_phase = PH_RETRY;
                break;
            }

            LOG_SERIAL.printf("[ota] 固件就绪 %u 字节 CRC32=0x%08X\n",
                          (unsigned)s_fwLen, (unsigned)s_fwCrc);
            /* 【记账】把"已经下好"持久化：key + 版本 + 大小 + CRC。
             * 8266 掉电/重启后读回来，看到 BL 横幅就能直接推，不用重下。 */
            fwReadySave(s_workKey, s_ins.ver, s_fwLen, s_fwCrc);
            notifyAppStatus(2U, 100U);     /* 屏幕：已写入 W25Q，等复位升级 */
            /* 只有确认 bootloader 还在，才进 PH_SEND。
             * 确认条件：① 横幅还在 10 秒窗口内；② 或最近 6 秒内 ? 得到过应答。
             * 光看 g_ispMode 不够 —— 它可能还留着上一次 BL 的粘滞状态。 */
            {
                bool bannerFresh = s_bannerSeen &&
                                   (uint32_t)(millis() - s_bannerMs) < OTA_BOOT_TAKEOVER_MS;
                bool aliveFresh  = (s_blAliveMs != 0) &&
                                   (uint32_t)(millis() - s_blAliveMs) < 6000UL;
                if (g_ispMode && (bannerFresh || aliveFresh)) {
                    s_phase = PH_SEND;      /* BL 正在等命令，直接推 */
                } else {
                    /* 不在 BL 里：下好就回 IDLE，安静等 STM32 下一次复位。
                     * 【这就是用户要的语义】固件已经写进 W25Q 暂存区，
                     * 但 APP 照常跑、传感器照常采、屏幕照常刷；
                     * 直到用户按 RESET / 下次上电，bootloader 才真正刷写。 */
                    s_nextPoll = millis() + 1000;
                    s_phase    = PH_IDLE;
                }
            }
            break;
        }

        case PH_SEND: {
            FlashFwSource src;
            OtaStatus st;
            char errBuf[OTA_LINE_MAX];

            /* ---- 先问一次断点（Q 命令）----
             * bootloader 那边记着"上次传到哪儿了"（槽/大小/CRC/已写字节/分包）。
             * 指纹（大小+CRC）跟我们要传的固件一致 → 说明是同一份文件，
             *   可以让 bootloader 回 RESUME 接着传；
             * 不一致 → 它自己会擦槽重传，我们什么都不用做。
             * 【为什么必须用 Q 查到的槽】bootloader 只在"O 命令里的槽"
             *   和断点记录的槽一致时才回 RESUME，所以要把槽设成断点的槽。 */
            {
                OtaResumeInfo ri;

                s_engine.clearTargetSlot();
                if (s_engine.queryResumeInfo(&ri, OTA_T_INFO_MS) && ri.valid) {
                    LOG_SERIAL.printf("[ota] 断点: slot=%u size=%u crc=0x%08X "
                                  "written=%u chunk=%u\n",
                                  (unsigned)ri.slot, (unsigned)ri.size,
                                  (unsigned)ri.crc, (unsigned)ri.written,
                                  (unsigned)ri.chunk);
                    if (ri.size == s_fwLen && ri.crc == s_fwCrc) {
                        if (ri.written > s_fwLen) {
                            /* MCU 那边写得比我们手里的文件还多 —— 本地文件被换过/
                             * 截断了。续传只会喂错字节：重新下载整个固件。 */
                            LOG_SERIAL.println("[ota] 断点比本地文件还长，放弃续传，重新下载");
                            fwDiscardPending();
                            s_forceFullDownload = true;
                            snprintf(s_lastFail, sizeof(s_lastFail),
                                     "resume_invalid mcu_written=%u fw=%u",
                                     (unsigned)ri.written, (unsigned)s_fwLen);
                            reportStatus("resume_invalid", s_ins.ver);
                            s_phase = PH_RETRY;
                            break;
                        }
                        /* 同一份文件 → 指定槽，bootloader 会回 RESUME 接着传 */
                        s_engine.setTargetSlot(ri.slot);
                        LOG_SERIAL.printf("[ota] 指纹一致，可续传（已写 %u 字节）\n",
                                      (unsigned)ri.written);
                    } else {
                        LOG_SERIAL.println("[ota] 断点指纹与本次固件不同，全新传输");
                    }
                } else {
                    LOG_SERIAL.println("[ota] 没有断点，全新传输");
                }
            }

            if (!src.open(FW_TMP_PATH)) {
                LOG_SERIAL.println("[ota] 打开暂存文件失败");
                snprintf(s_lastFail, sizeof(s_lastFail), "file_error");
                reportStatus("file_error", "");
                s_phase = PH_RETRY;
                break;
            }

            reportStatus("flashing", s_ins.ver);

            /* ★ 协议核心在这里 ★
             * 发 O → 等 READY（或 RESUME 续传）→ 每 512 字节发一包并等一个 OK
             * → 等 verify ok。等待期间协议引擎会通过队列把别的任务转起来。 */
            {
                /* 版本号：在线时用云端这条指令的；离线推预下载件时用记账里的。
                 * bootloader 拿它写 .app_tag / 上报，留空也能刷，但有更清楚。 */
                const char* verArg = s_ins.valid ? s_ins.ver : s_fwReadyVer;

                st = s_engine.run(src, s_fwLen, s_fwCrc,
                                  (verArg[0] != '\0') ? verArg : NULL);
            }
            src.close();
            s_engine.clearTargetSlot();

            if (st == OTA_OK) {
                /* verify ok 之后还有备份 + 搬运 + 回读校验，再看一会儿 */
                if (settleAndCheck(errBuf, sizeof(errBuf))) {
                    LOG_SERIAL.println("[ota] 升级成功");
                    reportStatus("done", s_ins.ver);
                    s_retryCount = 0;
                    s_failCount  = 0;
                    /* 【先记账，再上报】记账是本地一定成功的，上报可能失败；
                     * 有本地这条在，就算后台永远没销账也不会再重刷第二次。 */
                    snprintf(s_doneKey, sizeof(s_doneKey), "%s", s_workKey);
                    ledgerSave();       /* 先落盘，再上报：掉电也不会重刷 */
                    /* 预下载件已经用掉：删记录 + 删本地暂存文件，
                     * 免得下次轮询还拿它去推。 */
                    fwReadyClear();
                    fwDiscardPending();
                    cloudPostOtaResult(s_ins.id, true, s_ins.ver, "");
                    g_ispMode = false;      /* 会话结束，回到"app 在跑"的假设 */
                    s_phase = PH_IDLE;
                } else {
                    LOG_SERIAL.printf("[ota] 升级失败: %s\n", errBuf);
                    snprintf(s_lastFail, sizeof(s_lastFail), "flash: %s", errBuf);
                    reportStatus("flash_failed", errBuf);
                    s_phase = PH_RETRY;
                }
            } else {
                LOG_SERIAL.printf("[ota] 协议失败 status=%d %s\n",
                              (int)st, s_engine.lastError());
                snprintf(s_lastFail, sizeof(s_lastFail),
                         "proto st=%d %s bl=%u:%s er=%s",
                         (int)st, s_engine.lastError(),
                         (unsigned)s_blLines, s_blLine, s_blEr);
                if (st == OTA_ERR_SHORT) {
                    /* 本地文件比声明的长度短（被截断了）——丢掉它，下次从头下。
                     * 不然每一轮都会在同一个地方断。 */
                    LOG_SERIAL.println("[ota] 本地固件不完整，丢弃后重新下载");
                    fwDiscardPending();
                    s_forceFullDownload = true;
                } else if (strstr(s_engine.lastError(), "crc") != NULL) {
                    /*
                     * 【CRC 失败 = 我们手里这份文件是坏的】
                     *   bootloader 从 W25Q 槽里读回来重算的 CRC 跟声明的对不上，
                     *   说明**传到槽里的那份**有问题 —— 要么本地文件本身坏，
                     *   要么传输过程被污染了。
                     *   光重传同一份文件没有意义（会无限重试同一个坏文件）：
                     *   必须把本地暂存文件丢掉、重新从云端下一份。
                     *   （bootloader 侧也已经在 CRC 失败时清掉断点，
                     *     所以下一轮会是全新传输，不会续到同一份坏数据上。）
                     */
                    LOG_SERIAL.println("[ota] CRC 校验失败，丢弃本地固件重新下载");
                    fwDiscardPending();
                    s_forceFullDownload = true;
                }
                reportStatus("protocol_failed", s_engine.lastError());
                s_phase = PH_RETRY;
            }
            break;
        }

        case PH_RETRY: {
            /* 退避重试。下载失败、协议失败都走这里。
             *
             * 【但必须有终点】以前这里是无限重试：指令在后台一直挂着，
             * 于是每一轮都要复位一次 STM32、重下一遍固件、重推一遍 ——
             * 板子永远起不来，8266 永远在忙（这也是它"定时重启"的来源）。
             * 现在最多 OTA_MAX_FAIL 次就放弃这一条：记进 s_quitKey、
             * 把失败结局报给后台、并用 G 把 bootloader 放回去跑 app。
             * 后台再点一次"下发"会换一个新的 id，那时才会重新动手。 */
            uint32_t wait;

            /* 本轮结束：把可能还挂着的下载连接/文件句柄关掉。
             * 已经落盘的半截文件保留着，下一轮从断点续传。 */
            s_dl.abort();

            s_retryCount++;
            s_failCount++;
            ledgerSave();       /* 失败计数也要落盘，否则重启后又从头刷 */
            notifyAppStatus(4U, 0U);   /* 屏幕：升级失败 */

            /* 【每一次失败都立刻报后台，不再攒到第 3 次】
             * 原来只有放弃时才上报，结果前两次失败在现场完全不可见 ——
             * 网页上 pendingOta 一直挂着、failures 还是 0，只能猜。
             * 现在每失败一次就发一条，后台 events 里能直接看到
             *   try1: proto st=4 ... er=ER:verify fail ...
             * 一眼就知道是下载、CRC、备份还是跳转那一步挂的。 */
            {
                char msg[160];
                snprintf(msg, sizeof(msg), "try%u: %s",
                         (unsigned)s_failCount,
                         (s_lastFail[0] != '\0') ? s_lastFail : "unknown");
                reportStatus("failed", msg);
                cloudPostOtaResult(s_ins.id, false, s_ins.ver, msg);
            }

            /* 【失败先放板子回 APP】
             * 这一轮已经结束了，STM32 没有理由继续停在 bootloader 里等。
             * 不放手会导致板子一直卡在 bootloader（屏幕黑、遥测断，看着像死机）。 */
            releaseBootloader();
            g_ispMode = false;
            ledgerSave();

            if (s_failCount >= OTA_MAX_FAIL) {
                snprintf(s_quitKey, sizeof(s_quitKey), "%s", s_workKey);
                ledgerSave();
                LOG_SERIAL.printf("[ota] 连续 %u 次没成，放弃这条指令（后台重新下发才会再动）\n",
                              (unsigned)s_failCount);
                /* 把最后一次失败原因一起带走，网页的"升级失败"里就能看到 */
                {
                    char msg[128];
                    snprintf(msg, sizeof(msg), "abandoned x%u: %s",
                             (unsigned)s_failCount,
                             (s_lastFail[0] != '\0') ? s_lastFail : "unknown");
                    reportStatus("abandoned", msg);
                    cloudPostOtaResult(s_ins.id, false, s_ins.ver, msg);
                }
                releaseBootloader();
                g_ispMode = false;
                s_failCount = 0;
                s_phase     = PH_IDLE;
                break;
            }

            wait = 5000UL << ((s_retryCount > 4) ? 4 : (s_retryCount - 1));
            if (wait > 60000UL) {
                wait = 60000UL;
            }
            LOG_SERIAL.printf("[ota] %u 秒后重试\n", (unsigned)(wait / 1000));
            s_notBefore    = millis() + wait;
            s_wifiDeadline = millis() + 30000;
            /* 【回到 IDLE，而不是直接 PREPARE】
             * PREPARE 只下载固件，它自己不会去触发 STM32 复位；
             * 而这一轮失败时我们已经用 G 把板子放回 app 了（g_ispMode=false），
             * 直接进 PREPARE 只会下载完去等一个永远不会来的横幅。
             * 回 IDLE 才会走「检查账本 → 再发一次触发串」那条路。 */
            s_phase        = PH_IDLE;
            break;
        }
    }
}
