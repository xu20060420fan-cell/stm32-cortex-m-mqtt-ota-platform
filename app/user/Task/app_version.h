/*
 * app_version.h
 *
 * APP 固件版本号。
 *
 * 两处要用：
 *   1. 启动日志 / OLED 首页显示 "x.y.z"
 *   2. 给 bootloader 读的固定地址版本标签（见 app_version.c 里的 app_tag[]）
 *
 * 打包规则：major<<16 | minor<<8 | patch
 */
#ifndef TASK_APP_VERSION_H_
#define TASK_APP_VERSION_H_

#include <stdint.h>

/* 版本号（整数，不带 U 后缀 —— 下面要靠它拼字符串） */
#define APP_VERSION_MAJOR  1
#define APP_VERSION_MINOR  0
/* 2 = 本轮全量修复（VTOR、SPI MISO/CSN 争用、阈值回执 THA、电池 ADC、
 * 错误复位打印……）。OTA 实测就靠这个号从 1.0.1 变成 1.0.2 来判定
 * "新固件真的落地了"，所以每次改动只要想让后台看见，就得动它。
 * 16 = 串口 ORE 修复（清溢出标志不再吞掉 DR 里那个有效字节）。
 * 17 = bootloader OTA 进度界面（"升级中"+进度条）配套验证版本。
 * 18 = 搬运阶段（92%以后）也按百分比"滴"，避免以为升级卡住。
 * 19 = 跳 app 前还原 OLED 窗口 + 进度 1% 一刷（修主页右侧残留进度条）。
 * 20 = 进度数字不再整块清、阶段切换改成平滑走过去。
 * 21 = 蜂鸣器改成调用者直接写脚（去掉信号量+任务这条静默失效的链）；
 *      USART1 接收从 1ms 轮询改成中断+256B 环形缓冲（修"阈值回执变 900"、
 *      触发串被打残导致 banner_timeout 刷不进去）；
 *      PA11 推挽输出开机置高，用来演示"后台 OTA 刷完固件 → LED 亮"。
 * 22 = 演示版固件（DEMO_PA11=1）：刷完这一版 PA11 就是高电平，LED 亮。
 *      1.0.21 是对应的基线版（DEMO_PA11=0），两版其它代码完全一样。
 * 23 = OLED 更流畅（30FPS + 300ms 数值滚动）；新增 OTA 状态显示：
 *      8266 下发 "OA <state> <pct>"，最后一行显示"下载中 xx% / 升级中 /
 *      升级失败"，无任务时回到运行时长。
 * 24 = 补全字库（下载/升级/失败所需汉字），并精简 OLED 初始化日志腾出空间。
 * 25 = 同一版本的验机版本，用来在真机上走一遍"下载中 → 升级中"显示。
 * 26 = 屏幕状态改成 下载中 → 已写入W25Q → 下载完成；
 *      固件就绪时写 BKP 魔数，让用户按 RESET 后 bootloader 进常驻升级模式，
 *      先显示"升级中"再收固件（普通 10 秒窗口不画屏，会停在 APP 最后一帧）。
 */
#define APP_VERSION_PATCH  26

/* 打包成 bootloader 用的版本号：major<<16 | minor<<8 | patch */
#define APP_VERSION_PACKED  ((APP_VERSION_MAJOR << 16) | (APP_VERSION_MINOR << 8) | APP_VERSION_PATCH)

/* 版本标签的魔数：'APPT' 小端（0x41 0x50 0x50 0x54） */
#define APP_TAG_MAGIC  0x54545041U

/* 把数字宏变成字符串，例如 APP_VERSION_STR(APP_VERSION_MAJOR) -> "1" */
#define APP_VERSION_STR_HELPER(x)  #x
#define APP_VERSION_STR_EXPAND(x)   APP_VERSION_STR_HELPER(x)

/* "1.0.0" —— 编译期常量，不占 RAM、不用 sprintf */
#define APP_VERSION_STRING  APP_VERSION_STR_EXPAND(APP_VERSION_MAJOR) "." \
                             APP_VERSION_STR_EXPAND(APP_VERSION_MINOR) "." \
                             APP_VERSION_STR_EXPAND(APP_VERSION_PATCH)

/* 返回版本字符串（"1.0.0"），字符串在 Flash 里 */
const char *app_version_str(void);

#endif /* TASK_APP_VERSION_H_ */
