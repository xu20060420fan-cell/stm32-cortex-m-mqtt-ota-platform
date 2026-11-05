/*
 * fw_download.cpp —— 云端接口 + HTTP 固件下载
 *
 * 【关于 TLS】
 *   ESP8266 只有 40KB 级别可用 RAM，完整的 BearSSL TLS 栈会把内存吃光，
 *   固件直链按设计走**明文 http**。
 *   （当前服务器 nginx 把 80 端口除了 /mqtt 以外全 301 到 https，
 *     这件事在 README 里单独写了，需要运维改 nginx。）
 */
#include "fw_download.h"

#include <ArduinoJson.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>
#include <LittleFS.h>

#include <stdlib.h>

/* 遥测里带的固件版本。
 * 【注意这是 8266 网关自己的版本，不是 STM32 app 的版本】
 *   所以字段名用 gatewayVersion，后台分开存 —— 以前叫 firmwareVersion，
 *   会把 STM32 app 的真实版本盖成网关版本。
 * 精确的 app 版本以 ota/result 上报为准。 */
#ifndef FW_VERSION_STR
#define FW_VERSION_STR OTA_FW_VERSION
#endif

#include "crc32.h"
#include "sched.h"

#if OTA_ALLOW_TLS
#include <BearSSL.h>
#endif

/* 静态缓冲，不走堆。ESP8266 的堆被 WiFi 切得七零八落，
 * 到处 new String 迟早 OOM。 */
static char s_respBuf[CLOUD_RESP_MAX];
static uint8_t s_dlBuf[OTA_BUF_SIZE];

/* 【常驻 HTTP 连接（2026-11-03）】
 *   2 秒一次遥测、10 秒一次配置、事件上报如果每次都 new 一个 WiFiClient，
 *   ESP8266 每 2 秒就要建一次 TCP + 分配一次 socket/缓冲：
 *   堆碎片和射频峰值电流都被放大，是"过一会儿就复位"的重要推手。
 *   这里复用同一条到 CLOUD_HOST 的连接（HTTP keep-alive），
 *   只有下载固件仍用独立连接（下载期间不能被别的请求打扰）。 */
static WiFiClient s_http;

/* ================================================================
 * 阈值解析（/api/iot/config 的响应和 MQTT config 主题共用一套）
 * ================================================================ */

/* 字段名容错：后台可能给 camelCase，也可能给 snake_case。 */
static JsonVariant pickCfg(JsonObject cfg, const char* a, const char* b)
{
    JsonVariant v = cfg[a];

    if (!v.isNull()) {
        return v;
    }
    return cfg[b];
}

/* 把一段阈值 JSON 读进 out。
 * 返回 true = 读到了阈值（out->hasTh = true）。
 *
 * 【上报周期的两条来源】
 *   reportIntervalMs  —— 精确值，优先
 *   reportIntervalSec —— 可能带小数（0.5 / 1 / 2），换算成毫秒
 * 以前这里把值钳到 2000ms，导致后台怎么改设备都是 2 秒一报。
 * 现在只做 500ms..3600s 的合法范围保护，云端下发多少就发多少。 */
static bool readThresholds(JsonObject cfg, OtaInstruction* out)
{
    JsonVariant vs;
    JsonVariant vm;
    double sec = 2.0;                       /* 默认 2 秒 */
    bool haveSec = false;

    if (cfg.isNull()) {
        return false;
    }

    out->hasTh       = true;
    out->thTempMax   = (int16_t)((pickCfg(cfg, "tempMax", "temp_max")         | 60.0)  * 100);
    out->thTempMin   = (int16_t)((pickCfg(cfg, "tempMin", "temp_min")         | 0.0)   * 100);
    out->thHumMax    = (int16_t)((pickCfg(cfg, "humMax", "hum_max")           | 100.0) * 100);
    out->thHumMin    = (int16_t)((pickCfg(cfg, "humMin", "hum_min")           | 0.0)   * 100);
    out->thPressMax  = (int32_t)((pickCfg(cfg, "pressMax", "press_max")       | 1200.0) * 100);
    out->thPressMin  = (int32_t)((pickCfg(cfg, "pressMin", "press_min")       | 300.0)  * 100);
    out->thFall      = (int16_t)((pickCfg(cfg, "fallSensitivity", "fall_sensitivity") | 2.0) * 100);
    out->thAlarmEn   = (pickCfg(cfg, "alarmEnabled", "alarm_enabled") | true) ? 1 : 0;

    vs = pickCfg(cfg, "reportIntervalSec", "report_interval_sec");
    vm = pickCfg(cfg, "reportIntervalMs", "report_interval_ms");

    if (!vs.isNull()) {
        sec = vs.as<double>();
        haveSec = true;
    }
    if (sec < 0.0) {
        sec = 0.0;
    }

    /* 先按秒算，毫秒字段存在就以后者为准 */
    long ms = haveSec ? (long)(sec * 1000.0 + 0.5) : 2000L;
    if (!vm.isNull()) {
        ms = vm.as<long>();
    }
    if (ms < OTA_REPORT_MS_MIN) {
        ms = OTA_REPORT_MS_MIN;
    }
    if (ms > OTA_REPORT_MS_MAX) {
        ms = OTA_REPORT_MS_MAX;
    }
    out->thReportMs = (int32_t)ms;
    /* 第 7 列是给只会解析整数秒的旧固件的兼容值：0.5s 向上取整成 1s。
     * 新固件会优先用第 8 列的毫秒值，所以亚秒周期不会丢精度。 */
    out->thReportSec = (int16_t)((ms + 999L) / 1000L);
    if (out->thReportSec < 1) {
        out->thReportSec = 1;
    }
    return true;
}

/* 解析云端推下来的阈值 JSON（MQTT config 主题）。
 *   形如 {"deviceId":"dev01","config":{...},"serverTime":...}
 *   阈值也可能直接挂在顶层（容错）。 */
bool cloudParseConfigJson(const char* payload, size_t len, OtaInstruction* out)
{
    JsonDocument doc;
    JsonObject root;
    JsonObject cfg;

    if (payload == NULL || out == NULL) {
        return false;
    }
    memset(out, 0, sizeof(*out));

    if (len == 0) {
        len = strlen(payload);
    }
    if (deserializeJson(doc, payload, len)) {
        LOG_SERIAL.println("[cloud] config JSON 解析失败");
        return false;
    }

    root = doc.as<JsonObject>();
    if (root.isNull()) {
        LOG_SERIAL.println("[cloud] config 不是 JSON 对象");
        return false;
    }

    cfg = root["config"].as<JsonObject>();
    if (cfg.isNull()) {
        cfg = root;                /* 容错：阈值直接挂在顶层 */
    }
    if (!readThresholds(cfg, out)) {
        return false;
    }
    LOG_SERIAL.printf("[cloud] 收到阈值 tmax=%d tmin=%d hmax=%d hmin=%d pmax=%d pmin=%d fall=%d "
                    "alarm=%d sec=%d ms=%ld\n",
                    (int)out->thTempMax, (int)out->thTempMin,
                    (int)out->thHumMax, (int)out->thHumMin,
                    (int)out->thPressMax, (int)out->thPressMin,
                    (int)out->thFall, (int)out->thAlarmEn,
                    (int)out->thReportSec, (long)out->thReportMs);
    return true;
}

/* ================================================================
 * 一、云端配置接口
 * ================================================================ */
bool cloudFetchOta(OtaInstruction* out)
{
    char url[128];
    HTTPClient http;
    int code;
    int total = 0;
    JsonDocument doc;
    JsonObject ota;
    const char* u;

    if (out == NULL) {
        return false;
    }
    memset(out, 0, sizeof(*out));

    /* 带上自己的固件版本：后台会记成 device.gatewayVersion。
     * 【为什么必须带】
     *   现场最耗人的一次误判就是「以为刷了新固件，其实板上跑的还是旧的」，
     *   而旧固件的症状（只拉配置、没有遥测）和 STM32 挂了一模一样。
     *   网页上能直接看到网关版本号，就不用再靠猜。 */
    snprintf(url, sizeof(url), "http://%s/api/iot/config?deviceId=%s&fw=%s",
             CLOUD_HOST, DEVICE_ID, OTA_FW_VERSION);

    http.setTimeout(CLOUD_HTTP_TIMEOUT_MS);
    /* 【不让 HTTPClient 自己跟重定向】
     * 固件直链如果被 301 到 https，HTTPClient 会拿明文 WiFiClient 去连 443，
     * 结果是莫名其妙的负数错误码，看不出原因。
     * 我们自己判断 301，好把真正的原因报出来。 */
    http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    http.setReuse(true);
    http.begin(s_http, url);
    http.addHeader("X-Iot-Key", DEVICE_KEY);

    ESP.wdtFeed();
    code = http.GET();
    if (code != HTTP_CODE_OK) {
        LOG_SERIAL.printf("[cloud] 配置接口返回 %d\n", code);
        http.end();
        return false;
    }

    /* 把响应体读进静态缓冲。
     * 【为什么要自己控制读多少】
     *   Stream::readBytes() 只有"读满指定长度或超时"这一个语义，
     *   没有"有多少读多少"的超时重试版本。
     *   这里按 available() 给的长度读，读完立刻把调度器转一转。 */
    {
        WiFiClient& stream = http.getStream();
        int32_t     want_len = http.getSize();   /* Content-Length，-1 = 不知道 */
        int         idleRounds = 0;

        stream.setTimeout(CLOUD_HTTP_TIMEOUT_MS);

        while (total < (int)CLOUD_RESP_MAX - 1) {
            int avail = stream.available();
            size_t want;
            int n;

            /* 【收够就停】HTTPClient 知道 Content-Length，这是最干净的出口。
             * 原来只有"读满缓冲或连接断了"两种出口 —— 服务器 keep-alive 且
             * 响应比缓冲小的时候，avail<=0 与 connected() 能同时成立几十秒，
             * 这个循环就一直空转（delay 喂着狗所以不复位，但 otaTask 再也不返回，
             * 一直到 nginx 空闲断开为止）。 */
            if (want_len > 0 && total >= (int)want_len) {
                break;
            }

            if (avail <= 0) {
                if (!http.connected()) {
                    break;
                }
                /* 没长度、也没数据：连续 40 轮(每轮 ~5ms+调度)就认定"到齐了" */
                if (++idleRounds > 40) {
                    break;
                }
                schedPumpOthers();
                delay(5);
                continue;
            }
            idleRounds = 0;
            want = (size_t)avail;
            if (want > (size_t)(CLOUD_RESP_MAX - 1 - total)) {
                want = (size_t)(CLOUD_RESP_MAX - 1 - total);
            }
            n = (int)stream.readBytes(s_respBuf + total, want);
            if (n <= 0) {
                break;
            }
            total += n;
            schedPumpOthers();
        }
    }
    http.end();

    s_respBuf[total] = '\0';
    if (total <= 0) {
        LOG_SERIAL.println("[cloud] 配置接口响应为空");
        return false;
    }

    if (deserializeJson(doc, s_respBuf, (size_t)total)) {
        LOG_SERIAL.printf("[cloud] 配置接口 JSON 解析失败, 原文: %s\n", s_respBuf);
        return false;
    }

    /* ota 字段可能挂在顶层，也可能挂在 data 下面，两种都认一下 */
    ota = doc["ota"].as<JsonObject>();
    if (ota.isNull()) {
        JsonObject data = doc["data"].as<JsonObject>();
        if (!data.isNull()) {
            ota = data["ota"].as<JsonObject>();
        }
    }

    /* ---- 阈值：config 字段，每次都顺手带回（有没有 OTA 都要） ----
     * 全部 ×100 收进整数，串口上好解析（app 侧不拉 float scanf 进固件）；
     * 字段缺失就 hasTh=false 不转发。 */
    {
        JsonObject cfg = doc["config"].as<JsonObject>();
        readThresholds(cfg, out);
    }

    if (ota.isNull()) {
        /* 没有待升级的固件 —— 正常状态。HTTP/JSON 都成功，valid=false */
        LOG_SERIAL.println("[cloud] 没有待升级的固件");
        out->valid = false;
        return true;
    }

    u = ota["url"] | "";
    if (u[0] == '\0') {
        LOG_SERIAL.println("[cloud] ota 里没有 url（后端太老？config 应现签直链）");
        return false;
    }

    strncpy(out->url, u, FW_URL_MAX - 1);
    out->url[FW_URL_MAX - 1] = '\0';

    {
        /* 本次下发的指令号。云端每次 dispatch 生成一个新 UUID，
         * 靠它把"新的一条"和"同一条在反复轮询"区分开（见 ota_task.cpp 的结掉逻辑）。 */
        const char* i = ota["id"] | "";
        strncpy(out->id, i, sizeof(out->id) - 1);
        out->id[sizeof(out->id) - 1] = '\0';
    }
    {
        /* 后端 payload 用 "version"，容错读一下 "ver" */
        const char* v = ota["ver"] | (ota["version"] | "");
        strncpy(out->ver, v, sizeof(out->ver) - 1);
        out->ver[sizeof(out->ver) - 1] = '\0';
    }
    {
        /* crc32：后端给 "0x........" 字符串（上传时算好入库的）。
         * 有它 8266 不用重新算也能和本地算的对账。 */
        const char* c = ota["crc32"] | "";
        out->crc32 = (c[0] != '\0') ? (uint32_t)strtoul(c, NULL, 16) : 0;
    }
    {
        const char* s = ota["sha256"] | "";
        strncpy(out->sha256, s, sizeof(out->sha256) - 1);
        out->sha256[sizeof(out->sha256) - 1] = '\0';
    }
    {
        JsonVariant sv = ota["size"];
        out->size = sv.is<uint32_t>() ? sv.as<uint32_t>() : 0;
    }
    out->valid = true;

    LOG_SERIAL.printf("[cloud] 待升级 id=%s ver=%s size=%u crc32=0x%08X url=%s\n",
                  out->id, out->ver, (unsigned)out->size,
                  (unsigned)out->crc32, out->url);
    return true;
}

/* ================================================================
 * 二、固件下载（明文 http，分块落盘 + 顺带算 CRC32）
 *
 * 【断点续传】
 *   本地 LittleFS 里已经有一截（/fw_pending.bin），而且比整包小的时候，
 *   发 "Range: bytes=<已有>-" 让服务器只发剩下的那一段（206）。
 *   服务器不支持 Range（回 200 = 整包）就自动退化成从头下载，
 *   绝不会因为服务器不认 Range 就卡死。
 * ================================================================ */

/* LittleFS 里暂存文件现在多大（没有文件返回 0） */
uint32_t fwPendingSize(void)
{
    File f = LittleFS.open(FW_TMP_PATH, "r");
    uint32_t n = 0;

    if (f) {
        n = (uint32_t)f.size();
        f.close();
    }
    return n;
}

/* 丢掉暂存文件（下次下载从头来） */
void fwDiscardPending(void)
{
    if (LittleFS.exists(FW_TMP_PATH)) {
        LittleFS.remove(FW_TMP_PATH);
        LOG_SERIAL.println("[dl] 已丢弃本地暂存固件");
    }
    if (LittleFS.exists(FW_TMP_META_PATH)) {
        LittleFS.remove(FW_TMP_META_PATH);
    }
}

/* 记下"这半截固件是给哪一版固件续的" */
static void writePendingMeta(uint32_t size, uint32_t crc)
{
    File m = LittleFS.open(FW_TMP_META_PATH, "w");
    char buf[40];
    int n;

    if (!m) {
        return;
    }
    n = snprintf(buf, sizeof(buf), "%u %u\n", (unsigned)size, (unsigned)crc);
    m.write((const uint8_t*)buf, (size_t)n);
    m.close();
}

/* 读回指纹；对不上就返回 false（不能拿这半截去续别的固件） */
static bool readPendingMeta(uint32_t* size, uint32_t* crc)
{
    File m = LittleFS.open(FW_TMP_META_PATH, "r");
    char buf[40];
    int  n;                    /* 【必须是 int】FS::read 的返回类型是 int，
                                * LittleFS 把 lfs_file_read 的**负错误码**原样透传
                                * （flash 读错/文件损坏/被并发写坏都会给负数）。
                                * 原来这里接进 size_t，负数变成 ~4G 的巨值，
                                * 下一行 buf[n] 就是往野地址写一个 0 ——
                                * 现场表现成"莫名其妙的异常复位"，最难查的那种。 */
    unsigned long s = 0;
    unsigned long c = 0;

    if (!m) {
        return false;
    }
    n = m.read((uint8_t*)buf, sizeof(buf) - 1);
    m.close();
    if (n <= 0) {
        return false;
    }
    buf[n] = '\0';
    if (sscanf(buf, "%lu %lu", &s, &c) != 2) {
        return false;
    }
    *size = (uint32_t)s;
    *crc  = (uint32_t)c;
    return true;
}

/* 算已下载那部分的 CRC32（中间态）。
 * 续传时要把它的 CRC 接在后面继续算，否则整包 CRC 算不对。 */
static uint32_t crcPendingPrefix(uint32_t len)
{
    File f = LittleFS.open(FW_TMP_PATH, "r");
    uint32_t crc = 0xFFFFFFFFU;
    uint32_t left = len;
    static uint8_t buf[OTA_BUF_SIZE];

    if (!f) {
        return crc;
    }
    while (left > 0) {
        size_t want = (left > (uint32_t)sizeof(buf)) ? sizeof(buf) : (size_t)left;
        int n = f.read(buf, want);

        /* 同上：负错误码不能被当成"读到了 4G 字节"。
         * 那样 crc32Update 会拿野长度扫内存，而 left -= (uint32_t)n
         * 不减反增，这个 while 就再也出不来了。 */
        if (n <= 0) {
            break;
        }
        crc = crc32Update(crc, buf, (size_t)n);
        left -= (uint32_t)n;
    }
    f.close();
    return crc;
}

/* ================================================================
 * 增量下载：begin() 发起一次请求，step() 每轮只搬一小片
 *
 * 【2026-11-04 为什么改成这样】
 *   原来这个函数是一个 while 循环，一口气把 40KB 全下完才返回。
 *   虽然循环里每片都调 schedPumpOthers()，串口还能收字节，
 *   但遥测转发那一段代码在 otaTask 的 PH_IDLE 分支里 ——
 *   下载期间 s_phase 是 PH_PREPARE，那段根本执行不到，
 *   于是后台看到的是"一下发更新，传感器就像停了一样"。
 *   现在每次只搬 OTA_DL_SLICE_BYTES 就 return，
 *   主循环每一圈都会重新进入 otaTask，遥测转发照常进行。
 *
 *   ○ 下载期间 STM32 完全无感：固件只是落在 8266 的 LittleFS 里，
 *     不碰 STM32、不进 bootloader、不改 BKP。
 *   ○ 真正刷写只发生在用户按 RESET/重新上电、STM32 主动进
 *     bootloader 的那一刻（见 ota_task.cpp 的 PH_SEND）。
 * ================================================================ */

void FwDownloader::abort(void)
{
    if (active_) {
        http_.end();
        if (file_) {
            file_.close();
        }
    }
    active_ = false;
}

void FwDownloader::finish(bool ok)
{
    uint32_t len = total_;

    http_.end();
    if (file_) {
        file_.close();
    }
    active_ = false;
    failed_ = !ok;

    if (!ok) {
        if (len == 0U) {
            LOG_SERIAL.println("[dl] 一个字节都没下到");
        } else if (expectSize_ > 0U) {
            /* 半截留在 LittleFS 里，下一轮带 Range 从断点接着下 */
            LOG_SERIAL.printf("[dl] 还没下完 %u/%u，留着下次续传\n",
                          (unsigned)len, (unsigned)expectSize_);
        } else {
            LOG_SERIAL.printf("[dl] 下载中断，已收到 %u 字节\n", (unsigned)len);
        }
        return;
    }
    LOG_SERIAL.printf("[dl] 下载完成 %u 字节, CRC32=0x%08X\n",
                  (unsigned)len, (unsigned)crc32());
}

bool FwDownloader::begin(const char* url, uint32_t expectSize, uint32_t expectCrc,
                         bool forceFull, FwProgressCb cb, void* ctx)
{
    int code;
    int contentLen;
    uint32_t have = 0;

    abort();

    cb_          = cb;
    ctx_         = ctx;
    expectSize_  = expectSize;
    expectTotal_ = 0;
    total_       = 0;
    crc_         = 0xFFFFFFFFU;
    resumed_     = false;
    failed_      = false;

    if (url == NULL || url[0] == '\0') {
        return false;
    }

    /* ---- 要不要续传？ ----
     * 三个条件全满足才续：
     *   1) 本地确实有半截（0 < have < expectSize）
     *   2) /fw_pending.meta 里记的指纹跟云端现在这份固件一致
     *      （大小相同 + CRC 相同；云端没给 CRC 时只比大小）
     *   3) 不是被强制要求从头下
     * 否则宁可多下一次，也不要接错字节。 */
    if (!forceFull && expectSize > 0) {
        uint32_t metaSize = 0;
        uint32_t metaCrc = 0;

        have = fwPendingSize();
        if (have > 0 && have < expectSize) {
            bool metaOk = readPendingMeta(&metaSize, &metaCrc);
            bool sameFw = metaOk && metaSize == expectSize &&
                          (expectCrc == 0 || metaCrc == expectCrc);

            if (sameFw) {
                resumed_ = true;
                crc_ = crcPendingPrefix(have);
                total_ = have;
                LOG_SERIAL.printf("[dl] 本地已有 %u/%u 字节，发 Range 续传\n",
                              (unsigned)have, (unsigned)expectSize);
            } else {
                LOG_SERIAL.println("[dl] 本地半截固件跟云端这份对不上，丢弃后重下");
                fwDiscardPending();
                have = 0;
            }
        } else {
            have = 0;
        }
    }

    /* 记下这次的指纹：下次续传时要用它核对 */
    if (expectSize > 0) {
        writePendingMeta(expectSize, expectCrc);
    }

    /* 【为什么这里也只有 4 秒】这是"两次读到数据之间"的空档上限，
     * 不是整个下载的时限（总时限是 step() 里的 120 秒 deadline）。
     * 空档超过 4 秒就当这次失败退出 —— 半截文件已经写在 LittleFS 里，
     * 下一轮带 Range 从断点接着下，不会白下。 */
    http_.setTimeout(CLOUD_HTTP_TIMEOUT_MS);
    http_.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    http_.setReuse(false);
    /* 【URL 原样使用】
     * 直链带 HMAC 签名，文件名和签名参数都算进签名里，
     * 改一个字节就 403。这里不做任何 URL 编码或改写。 */
    http_.begin(client_, url);

    if (resumed_) {
        char range[32];
        snprintf(range, sizeof(range), "bytes=%u-", (unsigned)have);
        http_.addHeader("Range", range);
    }

    code = http_.GET();
    if (code == HTTP_CODE_MOVED_PERMANENTLY || code == HTTP_CODE_FOUND ||
        code == HTTP_CODE_SEE_OTHER || code == HTTP_CODE_TEMPORARY_REDIRECT) {
        /* 这就是 README 里说的 nginx 矛盾：80 端口把下载也 301 到 https，
         * 而 ESP8266 走明文 http 下载不了。直接把原因报出来，别让人猜。 */
        LOG_SERIAL.printf("[dl] 服务器把固件下载 301 到了 https（HTTP %d）。\n", code);
        LOG_SERIAL.println("[dl] ESP8266 明文 http 下载不了，需要运维放开 80 端口的下载路径。");
        http_.end();
        return false;
    }

    if (code == HTTP_CODE_OK) {
        if (resumed_) {
            /* 服务器不认 Range（回 200 = 整包）→ 退化成从头下载，
             * 覆盖写文件，不能报错卡死。 */
            LOG_SERIAL.println("[dl] 服务器不支持 Range（返回 200），退化成从头下载");
            resumed_ = false;
            have = 0;
            total_ = 0;
            crc_ = 0xFFFFFFFFU;
        }
    } else if (code == HTTP_CODE_PARTIAL_CONTENT) {
        if (!resumed_) {
            /* 没发 Range 却收到 206：服务器行为异常，当失败处理 */
            LOG_SERIAL.println("[dl] 服务器回了 206 但我们没请求 Range，放弃");
            http_.end();
            return false;
        }
    } else if (code == HTTP_CODE_RANGE_NOT_SATISFIABLE) {
        /* 416：服务器上的文件比我们本地那半截还短 —— 本地半截没用了，
         * 丢掉它，下次从头下（不能一直拿它去试）。 */
        LOG_SERIAL.println("[dl] 服务器返回 416（本地半截无效），丢弃后重下");
        fwDiscardPending();
        http_.end();
        return false;
    } else {
        LOG_SERIAL.printf("[dl] 下载失败 HTTP %d\n", code);
        http_.end();
        return false;
    }

    contentLen = http_.getSize();
    expectTotal_ = resumed_ ? expectSize
                            : (contentLen > 0 ? (uint32_t)contentLen : 0);

    /* 续传用追加模式（文件里已经有 have 个字节了），
     * 全新下载用 "w" 覆盖写 —— 半截的旧文件不能留着。 */
    file_ = LittleFS.open(FW_TMP_PATH, resumed_ ? "a" : "w");
    if (!file_) {
        LOG_SERIAL.println("[dl] 打不开 LittleFS 暂存文件");
        http_.end();
        return false;
    }

    deadline_   = millis() + 120000UL;   /* 整体最多 2 分钟 */
    lastDataMs_ = millis();
    active_     = true;
    return true;
}

FwDownloader::Result FwDownloader::step(uint32_t maxBytes)
{
    int    avail;
    size_t want;
    int    n;

    if (!active_) {
        return failed_ ? FAILED : DONE;
    }

    ESP.wdtFeed();

    if ((int32_t)(millis() - deadline_) >= 0) {
        LOG_SERIAL.println("[dl] 整体下载超时");
        finish(false);
        return FAILED;
    }
    if ((int32_t)(millis() - lastDataMs_) >= (int32_t)CLOUD_HTTP_TIMEOUT_MS) {
        LOG_SERIAL.println("[dl] 连续 4 秒没收到数据，本轮放弃");
        finish(false);
        return FAILED;
    }

    avail = http_.getStream().available();
    if (avail <= 0) {
        bool closed = !http_.connected();
        if (closed || (expectTotal_ > 0 && total_ >= expectTotal_)) {
            bool ok = (total_ > 0U) &&
                      ((expectSize_ == 0U) || (total_ == expectSize_));
            finish(ok);
            return ok ? DONE : FAILED;
        }
        /* 数据还没到：立刻返回，别占着调度器 */
        return BUSY;
    }

    want = (size_t)avail;
    if (want > (size_t)maxBytes) {
        want = (size_t)maxBytes;
    }
    if (want > sizeof(s_dlBuf)) {
        want = sizeof(s_dlBuf);
    }

    n = (int)http_.getStream().readBytes((char*)s_dlBuf, want);
    if (n <= 0) {
        return BUSY;        /* 这一下没读到，下一轮再试 */
    }

    if (file_.write(s_dlBuf, (size_t)n) != (size_t)n) {
        LOG_SERIAL.println("[dl] 写 LittleFS 失败");
        finish(false);
        return FAILED;
    }
    crc_ = crc32Update(crc_, s_dlBuf, (size_t)n);
    total_ += (uint32_t)n;
    lastDataMs_ = millis();

    if (cb_ != NULL && !cb_(total_, expectTotal_, ctx_)) {
        LOG_SERIAL.println("[dl] 回调要求中止");
        finish(false);
        return FAILED;
    }

    if (expectTotal_ > 0 && total_ >= expectTotal_) {
        finish(true);
        return DONE;
    }
    return BUSY;
}


/* ================================================================
 * TL 遥测转发：POST /api/iot/telemetry
 *
 * 【为什么走 HTTP 而不是 MQTT】精简版（esp8266_ota_min）没有 PubSubClient；
 * HTTP 两个环境都能用，而且 telemetry 本来就是后端的 REST 接口。
 * ================================================================ */
bool cloudPostTelemetry(const char* tlLine)
{
    HTTPClient http;
    char url[96];
    char body[192];
    int code;
    bool ok = false;

    /* 解析 TL 行的 7 个整数（后 4 个可能缺） */
    long v[7] = {0, 0, 101325, 0, 0, 0, 0};
    int  n = 0;
    const char* p = tlLine + 3;                 /* 跳过 "TL " */

    while (n < 7) {
        char* end;
        long val = strtol(p, &end, 10);
        if (end == p) break;
        v[n++] = val;
        p = end;
    }
    if (n < 7) return false;                    /* 字段不全，不转发 */

    snprintf(body, sizeof(body),
             "{\"temp\":%ld.%02ld,\"hum\":%ld.%02ld,\"press\":%ld,"
             "\"accX\":%ld.%02ld,\"accY\":%ld.%02ld,\"accZ\":0,"
             "\"batt\":%ld,\"status\":\"%s\",\"gatewayVersion\":\"%s\","
             "\"gwHeap\":%u}",
             v[0] / 100, labs(v[0]) % 100,
             v[1] / 100, labs(v[1]) % 100,
             v[2],
             v[3] / 100, labs(v[3]) % 100,
             v[4] / 100, labs(v[4]) % 100,
             v[5],
             v[6] ? "alarm" : "normal",
             FW_VERSION_STR,
             (unsigned)ESP.getFreeHeap());

    snprintf(url, sizeof(url), "http://%s/api/iot/telemetry", CLOUD_HOST);

    http.setTimeout(CLOUD_HTTP_TIMEOUT_MS);
    http.setReuse(true);
    http.begin(s_http, url);
    http.addHeader("X-Iot-Key", DEVICE_KEY);
    http.addHeader("Content-Type", "application/json");
    ESP.wdtFeed();
    code = http.POST(body);
    http.end();

    ok = (code == HTTP_CODE_OK);
    if (!ok) {
        LOG_SERIAL.printf("[tl] 上报失败 HTTP %d\n", code);
    }
    return ok;
}


/* EVT 事件上报：POST /api/iot/event。
 * 【谁来用】STM32 app 检测到"传感器掉线后又恢复"这类动作时发 EVT 行，
 * 8266 转发上来，后台事件列表里留痕。 */
bool cloudPostEvent(const char* evtLine)
{
    HTTPClient http;
    char url[96];
    char body[256];
    char detail[112];
    int code;
    bool ok = false;

    /* evtLine = "EVT <type> <detail>" → JSON */
    const char* rest = evtLine + 4;
    const char* sp = strchr(rest, ' ');
    char typeBuf[32] = "device_event";
    if (sp != NULL) {
        size_t n = (size_t)(sp - rest);
        if (n >= sizeof(typeBuf)) n = sizeof(typeBuf) - 1;
        memcpy(typeBuf, rest, n);
        typeBuf[n] = '\0';
        rest = sp + 1;
    }
    snprintf(detail, sizeof(detail), "%s", rest);

    snprintf(body, sizeof(body),
             "{\"type\":\"%s\",\"level\":\"info\",\"detail\":\"%s\"}",
             typeBuf, detail);
    snprintf(url, sizeof(url), "http://%s/api/iot/event", CLOUD_HOST);

    http.setTimeout(CLOUD_HTTP_TIMEOUT_MS);
    http.setReuse(true);
    http.begin(s_http, url);
    http.addHeader("X-Iot-Key", DEVICE_KEY);
    http.addHeader("Content-Type", "application/json");
    ESP.wdtFeed();
    code = http.POST(body);
    http.end();
    ok = (code == HTTP_CODE_OK);
    if (!ok) {
        LOG_SERIAL.printf("[evt] 上报失败 HTTP %d\n", code);
    }
    return ok;
}

/* ================================================================
 * 三、OTA 结局上报
 * ================================================================ */

/* 把任意字符串过成能安全拼进 JSON 的形式：丢掉引号/反斜杠/控制字符。
 * 失败原因是 MCU 打回来的原始一行，里面有什么都不奇怪，
 * 直接拼进 JSON 会把整条报文弄坏（后台 deserializeJson 失败 = 这条结果永久丢失）。 */
static void jsonSafe(char* dst, size_t cap, const char* src)
{
    size_t n = 0;

    if (src != NULL) {
        for (size_t i = 0; src[i] != '\0' && n < cap - 1; i++) {
            char ch = src[i];
            if (ch == '"' || ch == '\\' || (unsigned char)ch < 0x20U) {
                continue;
            }
            dst[n++] = ch;
        }
    }
    dst[n] = '\0';
}

/* OTA 结局上报：POST /api/iot/ota/result。
 * 【这是整条链路的"销账"动作，缺了它整条 OTA 就是死循环】
 *   后台的 pendingOta 只在收到 success=true 时才清空，而设备端以前
 *   根本没有这个调用 —— 指令永远销不掉，于是每 10 秒轮询都命中同一条指令、
 *   每次都把 STM32 复位进 bootloader 重刷一遍：板子看起来"直接卡死"
 *   （其实一直在被反复复位+刷机，APP 永远起不来、屏幕永远不亮），
 *   8266 则一直在下载+推送（长时间阻塞 → 定时重启）。
 * 失败也必须报：后台据此记事件，并能在连续失败后停发这条指令。 */
bool cloudPostOtaResult(const char* id, bool success,
                        const char* version, const char* message)
{
    HTTPClient http;
    char url[96];
    char body[320];
    char safeVer[40];
    char safeMsg[128];
    char safeId[48];
    int code;
    bool ok;

    jsonSafe(safeId,  sizeof(safeId),  id);
    jsonSafe(safeVer, sizeof(safeVer), version);
    jsonSafe(safeMsg, sizeof(safeMsg), message);

    snprintf(body, sizeof(body),
             "{\"deviceId\":\"%s\",\"id\":\"%s\",\"success\":%s,"
             "\"version\":\"%s\",\"message\":\"%s\"}",
             DEVICE_ID, safeId, success ? "true" : "false", safeVer, safeMsg);
    snprintf(url, sizeof(url), "http://%s/api/iot/ota/result", CLOUD_HOST);

    http.setTimeout(CLOUD_HTTP_TIMEOUT_MS);
    http.setReuse(true);
    http.begin(s_http, url);
    http.addHeader("X-Iot-Key", DEVICE_KEY);
    http.addHeader("Content-Type", "application/json");
    ESP.wdtFeed();
    code = http.POST(body);
    http.end();
    ok = (code == HTTP_CODE_OK);
    LOG_SERIAL.printf("[ota] 结局已上报 %s (HTTP %d)\n",
                  success ? "success" : "failed", code);
    return ok;
}

/* ================================================================
 * 四、从 LittleFS 读固件
 * ================================================================ */
bool FlashFwSource::open(const char* path)
{
    close();
    f_ = LittleFS.open(path, "r");
    open_ = (bool)f_ && f_.size() > 0;
    return open_;
}

void FlashFwSource::close()
{
    if (open_) {
        f_.close();
    }
    open_ = false;
}

size_t FlashFwSource::read(uint8_t* buf, size_t max)
{
    if (!open_ || max == 0) {
        return 0;
    }
    /* FS::read 返回 int：LittleFS 会把 lfs_file_read 的负错误码原样透传。
     * 直接当 size_t 返回，负数就成了"读到了 4G 字节"——
     * 上面那层只把长度夹到 want，于是**把一整块没初始化过的缓冲当固件数据
     * 喂给 bootloader**（表现为 CRC 对不上，却查不到是谁读错了）。
     * 出错就返回 0，让上层按"提前读完"走 OTA_ERR_SHORT：丢文件、重下。 */
    int n = f_.read(buf, max);
    return (n > 0) ? (size_t)n : 0;
}

/* 断点续传：MCU 回 RESUME <written> 时，从 written 处接着读 */
bool FlashFwSource::seek(uint32_t pos)
{
    if (!open_) {
        return false;
    }
    return f_.seek(pos);
}
