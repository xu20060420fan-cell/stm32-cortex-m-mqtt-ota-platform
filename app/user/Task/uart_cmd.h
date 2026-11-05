/*
 * uart_cmd.h —— ESP8266 → APP 的串口命令通道
 *
 * 【为什么需要它】云端→设备的链路只有一条：
 *   网页/后端 → ESP8266（联网）→ USART1 → APP。
 *   8266 在 USART1 上发这几类 ASCII 行：
 *     TH <tmax×100> <tmin×100> <hmax×100> <hmin×100> <fall×100> <alarm> <reportSec> [reportMs]
 *        → 设置阈值（网页"阈值配置"下发的值经 8266 转到这里）
 *     PB <pmax×100> <pmin×100>
 *        → 气压阈值
 *     NET 0/1
 *        → 8266 联网状态
 *     OA <state> <pct>
 *        → OTA 进度，供 OLED 显示：1=下载中 2=升级中(固件已就绪) 4=失败
 *
 * 【OTA 触发不在这条链路上】
 *   8266 只负责"有新固件先下好、等 STM32 复位进 bootloader 时再推"。
 *   本板不接收任何"进 OTA / 复位"命令；升级时机由 RESET 键/上电决定。
 *
 * 【注意冲突】bootloader 也用 USART1 —— 但它们俩不同时在场：
 *   app 在跑时 8266 发的是 TH/PB/NET/OA，进 bootloader 后 8266 发的是 ISP 协议。
 */
#ifndef UART_CMD_H_
#define UART_CMD_H_

#include <stdint.h>

/* OTA 在 OLED 上的显示状态 */
#define APP_OTA_NONE      0U   /* 无升级任务 */
#define APP_OTA_DOWNLOAD  1U   /* 下载中 */
#define APP_OTA_UPGRADE   2U   /* 固件已写入 W25Q（下载完成，等复位升级） */
#define APP_OTA_FAILED    4U   /* 升级/下载失败 */

/* OTA 状态超时：8266 上报后长时间没有新状态（比如它自己掉电/重启），
 * OLED 就回退显示运行时长，避免永远停在"下载完成"。 */
#define APP_OTA_STALE_MS  600000UL   /* 10 分钟 */

typedef struct {
    uint8_t  state;        /* APP_OTA_xxx */
    uint8_t  pct;          /* 0..100 */
    uint32_t updated_ms;   /* 最近一次收到 OA 的时刻（HAL_GetTick） */
} app_ota_ui_t;

/* 创建串口命令任务（在 app_tasks_init 里调用一次） */
void uart_cmd_init(void);

/* 取当前 OTA 显示状态；没有任务时 state=APP_OTA_NONE */
void app_ota_ui_get(app_ota_ui_t *out);

#endif /* UART_CMD_H_ */
