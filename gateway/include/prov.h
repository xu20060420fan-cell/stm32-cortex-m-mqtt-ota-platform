/*
 * prov.h —— WiFi 配网（**纯 AP**，按需开启）
 *
 * 【为什么是纯 AP】AP+STA 共存会把 WiFi 内存翻倍并频繁触发 Soft WDT 复位
 * （实测）。所以热点只在"没配置"或"连不上超过 60 秒"时开，
 * 并且任何要开 STA 的动作（验证连接、恢复重试）都必须先用 provPortalStop()
 * 把热点摘掉。两个模式任何时刻只开一个。
 *
 * 【怎么进配网】手机连热点 STM32-OTA-Setup，浏览器打开 192.168.4.1。
 * 也可以往 LittleFS 放一个名为 wifi_reset 的空文件再上电（清配置重来）。
 */
#ifndef PROV_H
#define PROV_H

#include <stddef.h>

/* 当前生效的 WiFi 凭据（/wifi.json 里的；没有配置时回退到 config.h 的宏） */
const char* provSsid(void);
const char* provPass(void);

/* 是否已经配过网 */
bool provHasConfig(void);

/* 上电调一次：读配置（wifi_reset 清配置）。热点由 provPortalStart 常开。 */
void provEnsure(void);

/* 开热点 + 配网服务（纯 AP）。断网 60s 后由 wifi_task 调用。 */
void provPortalStart(void);
bool provPortalRunning(void);

/* 关热点并释放服务对象。任何要打开 STA 的动作前都必须先调它
 * （AP+STA 共存 = 实测的 Soft WDT 复位）。 */
void provPortalStop(void);

/* 主循环里反复调：处理配网页请求 + 保存后的连接验证。 */
void provPortalTask(void);

#endif /* PROV_H */
