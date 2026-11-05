/*
 * appbl.h
 *
 *  Created on: Sep 16, 2026
 *      Author: Administrator
 *
 * 【注意】不再包含 <stdio.h> / <stdlib.h> / <math.h>。
 * printf 走 bootloader.h 里的宏转到 bl_printf，atoi/strtol 用 bl_parse_u32 替代。
 * 一旦再引入 stdio.h，printf 宏会把 stdio.h 里的声明改写成 bl_printf 的
 * 重复声明，直接编译报错。
 */

#ifndef APPBL_H_
#define APPBL_H_

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "main.h"
#include "bootloader.h"

/* OTA 目标镜像的 CRC32（命令里给的） */
extern uint32_t crc32;

/* 1. 等命令 + 命令分发（ISP 命令在这里同步处理完） */
void app_bl_init(void);

/* 2. OTA：擦 W25Q，回 READY */
void app_bl_run(void);

/* 3. OTA：收数据写 W25Q */
void app_bl_recbuf(void);

/* 4. OTA：从 W25Q 读回来算 CRC32 比对 */
void app_bl_check(void);

/* 5. OTA：把当前 app 备份到 W25Q 备份区 + 回读校验（不通过就中止升级） */
void app_bl_backup(void);

/* 6. OTA：回滚 —— 从 W25Q 备份区把旧 app 写回内部 flash */
void app_bl_restore(void);

/* 7. OTA：擦内部 Flash + 搬运 + 回读校验 + 置完成标志 */
void app_bl_flash(void);

/* 8. 跳转 app */
void app_bl_jump(void);

/* 9. 状态机，放在 main 的 while 里反复调用 */
void app_bl_status(void);

#endif /* APPBL_H_ */
