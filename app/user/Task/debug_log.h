/*
 * debug_log.h
 *
 * 诊断日志，走 UART1（115200 8N1）。
 * - 普通日志：写进环形缓冲，由 DMA 后台发送，**不阻塞任务**；缓冲满了丢新数据
 * - 致命日志：dbg_fatal_puts 直接轮询写寄存器，中断关了也能打出来
 * 日志一律 ASCII，避免串口终端中文乱码。
 */
#ifndef TASK_DEBUG_LOG_H_
#define TASK_DEBUG_LOG_H_

#include "main.h"

/** 普通日志：入队后由 DMA 发送，不阻塞（几乎不占栈） */
void dbg_puts(const char *s);

/** 普通日志：格式化版本（内部用 128 字节缓冲 + vsnprintf） */
void dbg_printf(const char *fmt, ...);

/** 致命日志：先停掉 DMA，再直接轮询写 USART1，中断关闭时也能输出 */
void dbg_fatal_puts(const char *s);

/** 扫描一条 I2C 总线上的所有 7 位地址，打印有应答的器件 */
void dbg_i2c_scan(I2C_HandleTypeDef *hi2c, const char *bus_name);

/** 打一个 32 位十六进制数（异常上下文安全） */
void dbg_fatal_hex(const char *tag, uint32_t value);

/** 打印本次复位原因（NRST / 上电 / 软件 / 看门狗）并清除标志 */
void dbg_report_reset_cause(void);

#endif /* TASK_DEBUG_LOG_H_ */
