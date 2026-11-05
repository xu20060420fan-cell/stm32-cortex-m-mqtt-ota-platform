/*
 * fw_download.h —— 云端接口 + 固件下载
 *
 * 【为什么要先落盘再喂数据】
 *   O 命令里必须先声明整包的 len 和 crc32，MCU 收完才会拿 crc32 比对。
 *   而 crc32 只有把整个文件过一遍才知道 —— 也就是说，"发 O 命令"必须发生在
 *   "文件全部过完一遍"之后。
 *   固件最大 44KB，ESP8266 可用 RAM 只有 40KB 上下，装不下整个文件。
 *   所以：先下载到 LittleFS（顺手算 CRC32），再从文件里分块喂给 MCU。
 *
 *   顺带的好处：喂数据时是纯本地读盘，速度稳定。
 *   MCU 每包只给 5 秒超时（appbl.c 的 RX_TIMEOUT_MS），
 *   要是边下边喂，网络一抖就超时失败。
 */
#ifndef FW_DOWNLOAD_H
#define FW_DOWNLOAD_H

#include <Arduino.h>
#include <FS.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>

#include "config.h"
#include "ota_proto.h"

/* 云端下发的待升级指令 + 阈值（一次 /api/iot/config 全带回） */
struct OtaInstruction {
    char    url[FW_URL_MAX];   /* 固件直链（带 HMAC 签名，config 响应时现签） */
    char    id[40];            /* 本次下发指令的唯一号（后台 dispatch 时 randomUUID）
                                * 【为什么必须带回来】后台 pendingOta 只有在这条指令
                                * 被"结掉"之前会一直出现在 /api/iot/config 里。
                                * 没有 id 就没法区分"新下发的一条"和"同一条在重复轮询"，
                                * 于是每 10 秒轮询一次就复位一次 STM32 —— 死循环。 */
    char    ver[32];
    char    sha256[80];
    uint32_t size;             /* 云端给的字节数，0 = 云端没给 */
    uint32_t crc32;            /* 云端算好的 CRC32（0 = 云端没给，自己算） */
    bool    valid;             /* true = 有一条待升级指令 */

    /* 阈值（云端 config 字段，原样转发给 STM32 app）。
     * 全部 ×100 存整数，串口上好解析（app 侧不用拉 float scanf 进固件）。 */
    bool    hasTh;
    int16_t thTempMax;   /* ×100 ℃ */
    int16_t thTempMin;   /* ×100 ℃ */
    int16_t thHumMax;    /* ×100 %RH */
    int16_t thHumMin;    /* ×100 %RH */
    int32_t thPressMax;  /* ×100 hPa，单独走 PB 行。必须 32 位：
                          * 1200.00 hPa = 120000，int16_t 最大 32767 会溢出成负数 */
    int32_t thPressMin;  /* ×100 hPa，单独走 PB 行 */
    int16_t thFall;      /* 跌落灵敏度 ×100（app 的 fall_g 阈值） */
    int16_t thAlarmEn;   /* 0/1 */
    int16_t thReportSec; /* 上报间隔（整数秒，给旧固件看的那一列） */
    int32_t thReportMs;  /* 上报间隔（毫秒，精确值）；0 = 没有，不下发第 8 列 */
};

/* 下载过程中的回调：每下一块调一次。
 * 返回 false 表示要中止下载。 */
typedef bool (*FwProgressCb)(uint32_t done, uint32_t total, void* ctx);

/* 拉云端配置（阈值 + 待升级指令）。
 *   GET http://<host>/api/iot/config?deviceId=<id>   头 X-Iot-Key: <密钥>
 * 返回 true = HTTP 和 JSON 都成功（无论有没有待升级指令）；
 * out->valid = 有待升级指令；out->hasTh = 带回了阈值。
 * 【返回值语义 2026-10-31 变了】调用方要同时看返回值和 out->valid。 */
bool cloudFetchOta(OtaInstruction* out);

/* 解析云端推下来的阈值 JSON（MQTT 的 campus/iot/<dev>/config 主题）。
 *   payload 形如 {"deviceId":"dev01","config":{...},"serverTime":...}
 *   阈值也可能直接挂在顶层（容错）。
 * 返回 true = 解析成功且带回了阈值（out->hasTh = true）。
 * 解析失败只返回 false，绝不崩。 */
bool cloudParseConfigJson(const char* payload, size_t len, OtaInstruction* out);

/* 把阈值转成 STM32 app 的 TH 行发出去（在 ota_task.cpp 里实现）。
 *   8 个字段：TH <tmax×100> <tmin×100> <hmax×100> <hmin×100> <fall×100>
 *              <alarm0/1> <reportSec> <reportMs>
 *   最后一列（毫秒）只有在 thReportMs 合法（100..60000）时才带。
 *   对面是 bootloader（g_ispMode）时不发 —— TH 不是 ISP 命令。 */
void forwardThresholds(const OtaInstruction* ins);

/* ================================================================
 * 增量固件下载（非阻塞）
 *
 * 【为什么要拆成一片一片】
 *   ESP8266 是单线程协作调度（没有 FreeRTOS 任务），如果下载在一个
 *   otaTask 调用里一口气跑到底，那么下载的几十秒里 otaTask 始终占着
 *   调度器，遥测转发那一段根本轮不到执行 —— 后台看到的就是
 *   "一下发更新，传感器就不动了"。
 *   拆成"每次只搬一小片、然后立刻返回"之后，loop() 每转一圈都会
 *   收一次 TL、报一次遥测；下载再慢，设备看起来也一直在正常运行。
 *
 * 【用法】
 *   if (dl.begin(url, size, crc, forceFull, cb, ctx)) {
 *       Result r = dl.step(OTA_DL_SLICE_BYTES);
 *       // BUSY = 还没完，调用方直接 return，下一轮再 step 一次
 *       // DONE / FAILED = 本片是最后一片
 *   }
 *   【断点续传】本地已有半截、且这半截跟云端现在要传的固件是同一份
 *   （/fw_pending.meta 里记着大小+CRC，对得上）时，发
 *   "Range: bytes=<已有>-" 续下（服务器返回 206）；
 *   服务器不支持 Range（返回 200）就自动退化成从头下载；
 *   指纹对不上（换了固件/文件被换过）就丢弃半截、从头下载。
 * ================================================================ */
class FwDownloader {
public:
    enum Result { BUSY = 0, DONE, FAILED };

    /* 发起下载（会做一次 HTTP GET，最多阻塞一个请求超时）。
     *   url 原样使用 —— 直链的签名算在文件名上，改一个字就 403。
     *   expectSize = 云端给的整包大小（0 = 不知道）。
     *   expectCrc  = 云端给的 CRC32（0 = 云端没给）。
     *   forceFull  = true 时无视本地已有的半截文件，从头下载。 */
    bool begin(const char* url, uint32_t expectSize, uint32_t expectCrc,
               bool forceFull, FwProgressCb cb = NULL, void* ctx = NULL);

    /* 搬一小段（最多 maxBytes 字节）就返回。
     *   BUSY   = 还没下完，下一轮继续 step
     *   DONE   = 整包下完，CRC 已算好
     *   FAILED = 失败（可能是断网/服务器错误/超时；半截文件留着，下次续传） */
    Result step(uint32_t maxBytes);

    /* 放弃本次会话（关连接、关文件）。已经落盘的半截仍可用于续传。 */
    void abort(void);

    uint32_t done(void) const  { return total_; }
    uint32_t total(void) const { return expectTotal_; }
    uint32_t crc32(void) const { return crc_ ^ 0xFFFFFFFFU; }
    bool     resumed(void) const { return resumed_; }

private:
    void finish(bool ok);

    WiFiClient   client_;
    HTTPClient   http_;
    File         file_;
    uint32_t     total_;
    uint32_t     crc_;
    uint32_t     expectSize_;
    uint32_t     expectTotal_;
    uint32_t     deadline_;
    uint32_t     lastDataMs_;
    FwProgressCb cb_;
    void*        ctx_;
    bool         active_;
    bool         resumed_;
    bool         failed_;
};

/* 丢掉 LittleFS 里的暂存固件（下次下载从头来）。
 *   用于"本地文件和云端指纹对不上"的情况。 */
void fwDiscardPending(void);

/* LittleFS 里暂存固件现在有多少字节（没有文件返回 0）。 */
uint32_t fwPendingSize(void);

/* 把一条 TL 遥测行转发到云端（POST /api/iot/telemetry）。
 * TL 格式：TL <temp×100> <hum×100> <pressPa> <pitch×100> <roll×100> <batt> <alarm>
 * 成功返回 true。 */
bool cloudPostTelemetry(const char* tlLine);

/* 把一条 EVT 事件行上报到云端（POST /api/iot/event）。
 * EVT 格式：EVT <类型> <详情>，如 "EVT sensor_recovered aht20"。 */
bool cloudPostEvent(const char* evtLine);

/* 把一次 OTA 的**结局**回报给云端（POST /api/iot/ota/result）。
 *   id       = 本次指令的号（云端只结对应的那条，防止误清新指令）
 *   success  = true 刷机成功 / false 失败
 *   version  = 刷进去的版本号（成功时云端记为设备当前版本）
 *   message  = 失败原因（成功时可为空）
 * 【为什么这条非有不可】云端的 pendingOta 只有收到 success=true 才会清空。
 *   没人报 → 指令永远挂着 → 每次轮询都再触发一次复位刷机（死循环）。
 * 返回 true = 云端收到并处理了（HTTP 200 + success）。 */
bool cloudPostOtaResult(const char* id, bool success,
                        const char* version, const char* message);

/* 从 LittleFS 里读固件（喂给协议引擎用） */
class FlashFwSource : public FwSource {
public:
    bool open(const char* path);
    void close();
    size_t read(uint8_t* buf, size_t max) override;
    bool seek(uint32_t pos) override;     /* 断点续传定位 */
private:
    File f_;
    bool  open_;
};

#endif /* FW_DOWNLOAD_H */
