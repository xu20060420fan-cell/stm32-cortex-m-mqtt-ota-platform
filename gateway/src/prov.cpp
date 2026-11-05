/*
 * prov.cpp —— WiFi 配网（**纯 AP**，按需开启）
 *
 * 【为什么不能 AP+STA 共存】热点常开那阵子用的就是共存，WiFi 内存翻倍并且
 *   频繁 Soft WDT 复位（实测贴过崩溃日志）。现在任何时刻只开一个模式：
 *     · 没配过网 → 开机直接开热点
 *     · 连不上持续 60s → 开热点（开之前 STA 已经停了）
 *     · 要试连 STA → 先 provPortalStop() 把热点摘掉
 *   配完网保存 → 当场验证一次（15 秒）→ 成功才重启。
 * 【扫描】WiFi.scanNetworks() 内部会 enableSTA(true)，所以它只在进 AP 之前
 *   或用户点「重新扫描」时调，扫完收回纯 AP；页面本身只渲染缓存。
 * 【安全】热点无密码，连上就能改 WiFi 配置；调试期接受，正式部署再加密码。
 */
#include <Arduino.h>
#include <new>          /* new (std::nothrow) —— 开热点时的两个分配要能判空 */
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <LittleFS.h>

#include "config.h"
#include "prov.h"

static const char* WIFI_CFG_PATH   = "/wifi.json";
static const char* WIFI_RESET_PATH = "/wifi_reset";
static const char* AP_NAME         = "STM32-OTA-Setup";
static const byte   DNS_PORT       = 53;

/* 生效的凭据（provEnsure 之后有效） */
static char s_ssid[33]  = {0};   /* 802.11 SSID 最长 32 字节 */
static char s_pass[65]  = {0};   /* WPA2 最长 64 字节 */
static bool s_hasCfg    = false;

/* 常开配网服务的句柄 */
static ESP8266WebServer* s_web = nullptr;
static DNSServer*        s_dns = nullptr;
static volatile bool     s_verify = false;   /* 保存后待验证 */
static String            s_err;               /* 配网页顶部错误提示 */

/* ================================================================
 * 扫描结果缓存
 *
 * 【为什么绝不能"在热点里现场扫"】
 *   WiFi.scanNetworks() 的第一件事就是 WiFi.enableSTA(true)
 *   （本机 core：libraries/ESP8266WiFi/src/ESP8266WiFiScan.cpp:76）。
 *   也就是说在纯 AP 的配网阶段调它，等于把模式偷偷升回 AP+STA ——
 *   正是本文件实测过"频繁 Soft WDT 复位"的那个组合。
 *   以前这里每 30 秒无条件扫一次，等于每 30 秒给自己埋一次复位，
 *   节奏和「8266 好像定时重启」严丝合缝对得上。
 *
 *   规则：扫描只在**进 AP 之前**做一次，或者由用户点「重新扫描」触发，
 *   且扫完立刻把模式收回纯 AP。页面只渲染缓存，不碰扫描 API。
 * ================================================================ */
#define PROV_SCAN_MAX 12
static char    s_scanSsid[PROV_SCAN_MAX][33];
static int8_t  s_scanRssi[PROV_SCAN_MAX];
static bool    s_scanEnc[PROV_SCAN_MAX];
static int     s_scanCount = 0;
static bool    s_scanDone  = false;
static volatile bool s_rescan = false;        /* 用户点了「重新扫描」 */

static void scanIntoCache(void)
{
    int n = WiFi.scanNetworks();

    s_scanDone = true;
    if (n <= 0) {
        s_scanCount = 0;
        return;
    }
    if (n > PROV_SCAN_MAX) {
        n = PROV_SCAN_MAX;
    }
    for (int i = 0; i < n; i++) {
        String ssid = WiFi.SSID(i);
        s_scanSsid[i][0] = '\0';
        if (ssid.length() > 0) {
            ssid.toCharArray(s_scanSsid[i], sizeof(s_scanSsid[i]));
        }
        s_scanRssi[i] = (int8_t)WiFi.RSSI(i);
        s_scanEnc[i]  = (WiFi.encryptionType(i) != ENC_TYPE_NONE);
    }
    s_scanCount = n;
}

const char* provSsid(void) { return s_ssid[0] ? s_ssid : WIFI_SSID; }
const char* provPass(void) { return s_pass[0] ? s_pass : WIFI_PASSWORD; }
/* 【为什么这里也要认编译期凭据】provSsid/provPass 都会回落到 config.h，
 * 但这个判据只看"文件里有没有存档" —— 于是整片擦除之后：
 *   provEnsure 打 "[prov] 使用编译期 WiFi: xxx"
 *   wifi_task 问 provHasConfig() 拿到 false → "无 WiFi 配置，立即开热点"
 * 两句互相打脸，表现成"我明明把 WiFi 写进固件了，它还是不连"。
 * 出厂写死凭据的那条路（换板子/擦除后免配网）必须和上面两个回落一致。 */
bool provHasConfig(void)
{
    if (s_hasCfg) {
        return true;
    }
    return (WIFI_SSID[0] != '\0') && (String(WIFI_SSID) != "your-wifi-ssid");
}

static bool loadCfg(void)
{
    File f = LittleFS.open(WIFI_CFG_PATH, "r");
    if (!f) return false;

    /* 极简 JSON 抠字段：{"ssid":"...","pass":"..."}，不引 ArduinoJson */
    String body = f.readString();
    f.close();

    int si = body.indexOf("\"ssid\":\"");
    int pi = body.indexOf("\"pass\":\"");
    if (si < 0 || pi < 0) return false;

    /* 【坑】"ssid":" 一共 8 个字符，必须 +8 才落到"值"上。
     * 以前写 +7，正好停在值的左引号上，往后找引号找到的还是它自己，
     * 抠出来的 ssid 是空串 → loadCfg 永远失败 → 保存重启后永远回配网。
     * （这就是"保存并连接之后连不上目标 WiFi"的根因。） */
    si += 8; pi += 8;
    int se = body.indexOf('"', si);
    int pe = body.indexOf('"', pi);
    if (se < 0 || pe < 0) return false;

    String ssid = body.substring(si, se);
    String pass = body.substring(pi, pe);
    if (ssid.length() == 0 || ssid.length() >= sizeof(s_ssid) ||
        pass.length() >= sizeof(s_pass)) return false;

    ssid.toCharArray(s_ssid, sizeof(s_ssid));
    pass.toCharArray(s_pass, sizeof(s_pass));
    s_hasCfg = true;
    return true;
}

static bool saveCfg(const String& ssid, const String& pass)
{
    if (ssid.length() == 0 || ssid.length() >= sizeof(s_ssid) ||
        pass.length() >= sizeof(s_pass)) return false;

    File f = LittleFS.open(WIFI_CFG_PATH, "w");
    if (!f) return false;
    /* SSID/密码里的 " 和 \ 会把 JSON 弄坏，落盘前转义 */
    String es = ssid, ep = pass;
    es.replace("\\", "\\\\"); es.replace("\"", "\\\"");
    ep.replace("\\", "\\\\"); ep.replace("\"", "\\\"");
    f.print("{\"ssid\":\"");
    f.print(es);
    f.print("\",\"pass\":\"");
    f.print(ep);
    f.println("\"}");
    f.close();
    return true;
}

/* ---------------------------------------------------------------
 * 配网页面
 * --------------------------------------------------------------- */

/* 【HTML 转义别省】SSID 里什么字符都有，直接拼进 HTML 会被截断 */
static String htmlEsc(const String& s)
{
    String r = s;
    r.replace("&", "&amp;");
    r.replace("<", "&lt;");
    r.replace(">", "&gt;");
    r.replace("\"", "&quot;");
    return r;
}

static String portalPage(void)
{
    String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>STM32 OTA 配网</title><style>"
        "body{font:15px/1.6 system-ui;margin:16px;max-width:420px}"
        "label{display:block;margin:12px 0 4px}"
        "input[type=password]{width:100%;padding:8px;box-sizing:border-box}"
        "button{padding:10px 24px;background:#2563eb;color:#fff;border:0;"
        "border-radius:6px;margin-top:16px;width:100%}"
        ".ap{display:block;padding:8px;border:1px solid #ddd;border-radius:6px;"
        "margin:4px 0;cursor:pointer}"
        ".ap b{float:right;font-size:12px;color:#888}"
        ".err{background:#fee2e2;color:#b91c1c;padding:8px;border-radius:6px}"
        ".st{background:#dcfce7;color:#166534;padding:8px;border-radius:6px;"
        "font-size:13px}"
        "</style></head><body><h2>STM32 OTA 配网</h2>";

    if (s_err.length()) {
        html += "<p class='err'>" + htmlEsc(s_err) + "</p>";
    }
    /* 【热点常开】页头永远显示当前 STA 状态 —— 调试时一眼看到联网没有 */
    if (WiFi.status() == WL_CONNECTED) {
        html += "<p class='st'>当前已连接：" + htmlEsc(WiFi.SSID()) +
                " · IP " + WiFi.localIP().toString() +
                "（改下面会切信道，热点瞬断属正常）</p>";
    } else {
        html += "<p class='err'>当前未连上 WiFi（设备 ID：" +
                String(DEVICE_ID) + "）</p>";
    }

    html += "<p>选择 WiFi 并输入密码，保存后会先试连 15 秒，"
            "成功才重启。</p><form method='POST' action='/save'>";

    /* 只渲染缓存。这里原来直接调 WiFi.scanNetworks()，
     * 而它会把 STA 半边打开（见上面 scanIntoCache 的说明）——
     * 每打开/刷新一次配网页就制造一次 AP+STA 共存。 */
    for (int i = 0; i < s_scanCount; i++) {
        if (s_scanSsid[i][0] == '\0') continue;   /* 隐藏网络手输不了，跳过 */
        html += "<label class='ap'><input type='radio' name='ssid' value='" +
                htmlEsc(String(s_scanSsid[i])) + "' required> " +
                htmlEsc(String(s_scanSsid[i])) +
                " <b>" + String((int)s_scanRssi[i]) + " dBm" +
                (s_scanEnc[i] ? " &#128274;" : "") +
                "</b></label>";
    }
    if (!s_scanDone) {
        html += "<p>还没扫过（进热点前没扫到），可以点下面的重新扫描。</p>";
    } else if (s_scanCount == 0) {
        html += "<p>没扫到 WiFi。</p>";
    }
    html += "<form method='GET' action='/rescan'><button type='submit'"
            " style='background:#e5e7eb;color:#111;width:auto;padding:6px 14px'>"
            "重新扫描</button></form>";
    html += "<label>密码</label>"
            "<input type='password' name='pass' placeholder='WiFi 密码（开放式留空）'>"
            "<button type='submit'>保存并验证</button></form>"
            "<p style='color:#888;font-size:12px'>只支持 2.4GHz · 设备：" +
            String(DEVICE_ID) + "</p></body></html>";
    return html;
}

static void handleRoot(void)
{
    s_web->send(200, "text/html; charset=utf-8", portalPage());
}

/* 「重新扫描」：只置标志，扫描本身放到 provPortalTask 里做。
 * 【为什么不在 handler 里扫】handler 是在 handleClient() 里面被调的，
 * 那里做同步扫描会把整个 loop 占住好几秒（Soft WDT 的另一路），
 * 而且扫完还得把模式收回纯 AP —— 收不到就是 AP+STA 共存。 */
static void handleRescan(void)
{
    s_rescan = true;
    s_web->send(200, "text/html; charset=utf-8",
                "<meta charset='utf-8' http-equiv='refresh' content='3;url=/'>"
                "<p>正在重新扫描，约 2 秒后自动返回（期间热点会瞬断一下，正常）</p>");
}

static void handleSave(void)
{
    String ssid = s_web->arg("ssid");
    String pass = s_web->arg("pass");
    if (ssid.length() == 0) {
        s_web->send(400, "text/html; charset=utf-8",
                    "<meta charset='utf-8'>SSID 不能为空，<a href='/'>返回</a>");
        return;
    }
    if (!saveCfg(ssid, pass)) {
        s_web->send(500, "text/html; charset=utf-8",
                    "<meta charset='utf-8'>保存失败（文件系统？），<a href='/'>返回</a>");
        return;
    }
    /* 【先回话再验证】应答发出去、留时间冲掉，然后 provPortalTask 里
     * 会被 s_verify 接力去真连一次：连上才重启，连不上回到页面报错。 */
    loadCfg();                              /* 存完立刻装进 provSsid/provPass */
    s_verify = true;
    s_web->send(200, "text/html; charset=utf-8",
                "<meta charset='utf-8'><p>已保存：" + htmlEsc(ssid) +
                "，正在验证连接（约 15 秒）…<br>成功会自动重启，"
                "失败请稍后重新打开本页。</p>");
}

/* 保存后的连接验证：成功 true。
 * 【为什么要验证】以前"保存→重启→连不上→又回配网"，用户完全不知道
 * 是密码错还是 WiFi 是 5G 的（ESP8266 只支持 2.4G）。当场试连一次，
 * 错误立刻暴露在手机上。 */
static bool verifyConnect(void)
{
    uint32_t t0;

    /* 【必须先收回纯 STA】WiFi.begin() 内部会 enableSTA(true)。
     * 热点还开着的时候就 begin()，等于当场把模式变成 AP+STA ——
     * 就是本文件上面写死"实测频繁 Soft WDT 复位"的那个组合，
     * 而它偏偏在"用户刚点保存"这一刻必走到。 */
    WiFi.mode(WIFI_STA);

    WiFi.begin(provSsid(), provPass());
    LOG_SERIAL.printf("[prov] 验证连接 %s …\n", provSsid());

    t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000UL) {
        delay(100);           /* delay 里有 yield，软狗照样喂 */
    }
    if (WiFi.status() == WL_CONNECTED) {
        LOG_SERIAL.printf("[prov] 验证通过 IP=%s，重启进入正式流程\n",
                      WiFi.localIP().toString().c_str());
        return true;
    }
    WiFi.disconnect();
    LOG_SERIAL.println("[prov] 验证失败（密码错？WiFi 是 5G？）");
    return false;
}

/* ---------------------------------------------------------------
 * 配网服务（纯 AP，按需开启）
 * --------------------------------------------------------------- */
bool provPortalRunning(void)
{
    return (s_web != nullptr);
}

/* 关热点并释放服务对象。
 * 【为什么必须有这一步】
 *   1. 保存配置后验证连接、以及"路由恢复后重试 STA"这两条路都必须先摘掉 AP
 *      再开 STA —— AP+STA 共存是实测会把 8266 打回复位的组合。
 *   2. 服务对象（web + dns）不 delete 就一直占着堆，配网热点常开时
 *      这是最容易把堆拖碎的一块。
 * 用 softAPdisconnect(true) 而不是 WiFi.disconnect(true)：
 *   后者会把已保存的 WiFi 配置一起清掉。 */
void provPortalStop(void)
{
    if (s_web != nullptr) {
        s_web->close();
        delete s_web;
        s_web = nullptr;
    }
    if (s_dns != nullptr) {
        s_dns->stop();
        delete s_dns;
        s_dns = nullptr;
    }
    WiFi.softAPdisconnect(true);
    s_rescan = false;
    LOG_SERIAL.println("[prov] 配网热点已关闭");
}

void provPortalStart(void)
{
    if (s_web != nullptr) return;              /* 已在跑 */

    /* 【2026-11-01 改回纯 AP 模式】热点常开用的 AP+STA 共存会把 WiFi 内存
     * 翻倍，实测频繁 Soft WDT 复位（贴过崩溃日志）——"之前不会这样"的那版
     * 就是纯 STA。现在：连不上 WiFi 持续 60s 才开热点（由 wifi_task 调这里，
     * 那时 STA 已经停了），配完网保存重启回 STA。
     * ⚠ 同一原因：扫描必须放在**进 AP 之前**（scanNetworks 内部会 enableSTA(true），
     *   在 AP 里调它就等于把共存模式又打开）。 */
    if (!s_scanDone) {
        scanIntoCache();
    }
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_NAME);
    delay(100);

    /* 两个 new 必须判空：这是长跑后堆最容易碎的时候（下载/MQTT 都过了一遍），
     * 拿不出 2KB 就是往空指针上写 → 异常复位，看起来还像是"随机重启"。 */
    s_dns = new (std::nothrow) DNSServer();
    s_web = new (std::nothrow) ESP8266WebServer(80);
    if (s_dns == nullptr || s_web == nullptr) {
        LOG_SERIAL.println("[prov] 开热点失败：堆不够（内存不足）");
        delete s_dns; s_dns = nullptr;
        delete s_web; s_web = nullptr;
        WiFi.softAPdisconnect(true);
        return;
    }

    s_dns->start(DNS_PORT, "*", WiFi.softAPIP());   /* 劫持域名 → 配网页 */
    s_web->on("/", handleRoot);
    s_web->on("/save", HTTP_POST, handleSave);
    s_web->on("/rescan", handleRescan);
    s_web->onNotFound(handleRoot);             /* captive portal 探测都引到首页 */
    s_web->begin();

    LOG_SERIAL.printf("[prov] 调试热点 %s 常开，配网页 http://%s/ （任何时候可改 WiFi）\n",
                  AP_NAME, WiFi.softAPIP().toString().c_str());
}

/* 主循环里反复调：处理配网页 + 保存后的验证。
 * 【协作式】每轮只干一小段，绝不阻塞别的任务。 */
void provPortalTask(void)
{
    if (s_web == nullptr) return;

    if (s_verify) {
        static uint32_t flushAt = 0;
        if (flushAt == 0) {
            flushAt = millis();                /* 先让应答冲出去 */
        }
        if (millis() - flushAt < 500) {
            return;                            /* 这半秒内继续跑别的任务 */
        }
        flushAt = 0;

        /* 【先摘热点再试连】verifyConnect 里的 WiFi.begin() 会把 STA 打开；
         * 热点还开着就是 AP+STA 共存 —— 实测的 Soft WDT 复位组合，
         * 而且偏偏在"用户刚点保存"这一步必走到。 */
        provPortalStop();

        if (verifyConnect()) {
            LOG_SERIAL.println("[prov] 配置生效，重启…");
            delay(500);
            ESP.restart();
        }
        /* 失败：把热点重新拉起来，手机重连后就能看到错误提示 */
        s_err = "连接失败：请检查密码；ESP8266 只支持 2.4GHz WiFi。";
        s_verify = false;
        provPortalStart();

        /* provPortalStop 已经把两个句柄置空了；如果重启热点没成功（堆不够），
         * 下面两行就是往空指针上跳 —— 必须在这里再挡一道。 */
        if (s_web == nullptr || s_dns == nullptr) {
            return;
        }
    }

    /* 用户点了「重新扫描」才扫，而且扫完立刻把模式收回纯 AP。
     * 【原来这里是每 30 秒无条件 scanNetworks(true)】
     *   扫描内部会 enableSTA(true) → 配网阶段变成 AP+STA 共存 →
     *   实测的 Soft WDT 复位组合，而且节奏是固定的 30 秒一次，
     *   正是"8266 疑似定时重启"最像的来源。 */
    if (s_rescan) {
        s_rescan = false;
        scanIntoCache();
        WiFi.mode(WIFI_AP);                    /* 把扫描带出来的 STA 半边摘掉 */
        if (WiFi.softAPIP() == IPAddress(0, 0, 0, 0)) {
            WiFi.softAP(AP_NAME);              /* AP 被带掉了，重新拉起来 */
        }
    }

    s_dns->processNextRequest();
    s_web->handleClient();
}

/* ---------------------------------------------------------------
 * 对外入口
 * --------------------------------------------------------------- */
void provEnsure(void)
{
    /* 主动清配置：放一个 /wifi_reset 文件再上电 */
    if (LittleFS.exists(WIFI_RESET_PATH)) {
        LittleFS.remove(WIFI_RESET_PATH);
        if (LittleFS.exists(WIFI_CFG_PATH)) {
            LittleFS.remove(WIFI_CFG_PATH);
        }
        LOG_SERIAL.println("[prov] 检测到 wifi_reset，已清除已保存的 WiFi");
    }

    if (loadCfg()) {
        LOG_SERIAL.printf("[prov] 使用已保存的 WiFi: %s\n", s_ssid);
        return;
    }

    /* 编译期写死了真实 WiFi（不是占位符）也算有配置 —— 兼容老用法 */
    if (String(WIFI_SSID) != "your-wifi-ssid" && WIFI_SSID[0] != '\0') {
        LOG_SERIAL.printf("[prov] 使用编译期 WiFi: %s\n", WIFI_SSID);
        return;
    }

    LOG_SERIAL.println("[prov] 尚未配置 WiFi —— 连热点 STM32-OTA-Setup，"
                   "浏览器开 192.168.4.1 配网");
    /* 热点由 provPortalStart 常开，这里不用阻塞等了 */
}
