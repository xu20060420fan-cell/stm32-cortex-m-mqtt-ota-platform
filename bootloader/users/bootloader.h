/*
 * bootloader.h —— 分区定义 + 轻量串口输出/解析
 *
 * 【为什么不用 libc 的 printf】
 * 一个 printf 会把整个 stdio 家族拖进固件（nano-vfprintf、fflush、
 * mallocr、findfp、wsetup、wbuf……约 3.4KB），20K 分区装不下。
 * 所以这里自己实现 bl_printf，只支持 bootloader 真正用到的格式，
 * 并在本文件末尾用宏把 printf(...) 自动转到 bl_printf，调用处零改动。
 *
 * 【前提】任何 .c 文件都不能再 #include <stdio.h>。
 * 一旦引入，stdio.h 里的 printf 声明会被这个宏改写成 bl_printf 的
 * 重复声明，直接编译报错。
 */

#ifndef BOOTLOADER_H_
#define BOOTLOADER_H_

#include "main.h"
#include <string.h>
#include <stdint.h>
#include <stddef.h>

/* ================================================================
 * Flash 分区
 *
 *   bootloader  0x08000000 ~ 0x08006000   24K   只读（上位机不许擦）
 *   app         0x08006000 ~ 0x08010000   40K   可擦写
 *
 * 【必须】bl_size 和 STM32F103C8TX_FLASH.ld 里 FLASH 的 LENGTH 保持一致！
 * app_bl_addr 是自动算的（FLASH_BASE + bl_size），改 bl_size 一处即可。
 *
 * 【为什么回到 24K（2026-11-02）】
 * 协议扩展后（各槽版本号 / 运行中版本号 / W25Q 槽读回校验 / 显式续传语义）
 * bootloader 体积又涨了一截，21K 分区只剩几百字节余量。
 * 24K 分区给 bootloader 留足余量，app 侧 40K 也够（当前 text 约 37K）。
 * 【同步改过的】bootloader 的 ld、app 的 ld/VTOR(0x6000)、
 * app 的 .app_tag 段地址、云端上传校验常量（STM32_APP_BASE / STM32_APP_MAX）。
 * ================================================================ */
#define bl_size      (24*1024)                   /* bootloader 区大小 24K */
#define app_bl_addr  (FLASH_BASE + bl_size)      /* app 起始地址 0x08006000 */
#define flash_end    (FLASH_BASE + 64*1024)      /* flash 末尾"地址" 0x08010000 */
#define page_size    1024                        /* F103C8 一页 1K */
#define uart_rec_len 512                         /* HAL 每一轮接收的最大字节数 */

/* ================================================================
 * 接收环形缓冲
 *
 * 【为什么必须有它】
 * 上位机 ser.write(512) 一次，MCU 这边**不等于**"收到一包"。
 * USB 串口芯片（CH340 / CP2102 / FT232）是按 USB 包（典型 64 字节）
 * 转发数据的，包与包之间有毫秒级间隙；而 HAL_UARTEx_ReceiveToIdle_IT
 * 的硬件 IDLE 中断，只要线路空闲"一个字符时间"（115200 下约 87us）
 * 就会触发一次。所以一次 512 字节的写入，在 MCU 眼里常常是 2~8 个
 * 独立事件。
 *
 * 老实现是"一个事件 = 一包"，上一包还没被主循环取走时，新到的事件
 * 整包丢掉 —— 而上位机却以为发到了，于是后面全部错位（症状是
 * CRC 不过 / 一直等到超时）。
 *
 * 现在中断只往这个环里塞、永不丢；主循环按"命令里声明的每包长度"
 * 精确攒够 want 个字节再处理。这跟 AN3155 里 Write Memory 的
 * "N + (N+1) 字节数据 + 每块 ACK"是同一个思路：靠**长度**分帧，
 * 不靠**时间**。
 *
 * 【必须是 2 的幂】取模用 & 实现，不用除法。
 * 【为什么 1024】一包最大 512，写 Flash 期间上位机还可能再来一包，
 * 所以要留 2 倍余量。
 * ================================================================ */
#define RX_RING_SIZE 1024U
#define RX_CHUNK_MAX uart_rec_len                /* 单包上限，会写进 ? 命令的 MAXCHUNK */

/* app 区实际可用字节数（用地址算，改分区会自动跟着变） */
#define APP_MAX_SIZE (flash_end - app_bl_addr)   /* 40K = 40960 */

/* ================================================================
 * 运行中 app 的版本标签
 *
 * 【为什么需要】
 *   W25Q 元数据里记的是「槽里那份固件的版本」，那是**还没激活**的。
 *   用户问「我现在跑的这版是哪个版本」时，得能答出来。
 *   办法：app 在自己的**固定偏移**上放一个 8 字节标签
 *   （magic + 打包版本号），bootloader 直接读内部 flash 那个地址。
 *
 * 【偏移为什么是 0x110 而不是 0x100】
 *   F103 中容量器件（STM32F103xB/C）的向量表实际占 0x10C 字节
 *   （64 个中断向量 + 3 个 RAM 启动保留字），
 *   放在 +0x100 会压在向量表尾巴上 —— 那 8 个字节是启动代码要用的，
 *   覆盖它等于改掉复位流程。
 *   所以 app 侧链接脚本把标签定在 ORIGIN+0x110 = 0x08006110。
 *   ⚠ 两边必须一致：改了一边，另一边一定要跟着改。
 *
 * 【为什么放在靠前位置而不是末尾】
 *   放末尾的话，链接脚本会把 app bin 填充到 40K 撑满分区，
 *   每次 OTA 都要传 40K（多一倍流量，而且 bin 体积失去意义）。
 *
 * 【约定】app 侧 app_version.h 里必须用同名同值：
 *   APP_TAG_MAGIC = 0x54545041（'APPT' 小端）
 *   APP_VERSION_PACKED = (major<<16)|(minor<<8)|patch
 * ================================================================ */
#define APP_TAG_ADDR  (app_bl_addr + 0x110U)     /* 0x08006110 */
#define APP_TAG_MAGIC 0x54545041UL               /* 'APPT' 小端 */

/* RAM 范围，用来校验 app 的栈顶合不合法。
 * 要跟 STM32F103C8TX_FLASH.ld 里 RAM 的 LENGTH 保持一致（C8 是 20K） */
#define ram_start (0x20000000U)
#define ram_end   (0x20000000U + 20*1024U)       /* 0x20005000 */

/* ================================================================
 * 上电后等命令的时间
 *
 * 这段时间里收到命令 → 留在命令行等你刷机；
 * 收到无法识别的命令、或者一直没收到 → 认为只是正常上电，去跑 app。
 *
 * 【注意：这个值直接决定上电到 app 跑起来要等多久】
 * 现在设成 10 秒，意味着每次冷启动（或按复位）都会在 bootloader 里
 * 干等 10 秒才跳 app。开发阶段方便，量产固件嫌慢的话可以调小，
 * 或者改成"只有某个按键/某个备份寄存器的魔数才进 bootloader"。
 *
 * 【必须和上位机对齐】tools/firmware_tool.py 里握手窗口 TOTAL 是 20 秒，
 * 它靠"每 0.2 秒重试一次"来抓这个窗口，所以只要 TOTAL > BOOT_WAIT_MS
 * 就能抓到。把这里改到 20 秒以上时，TOTAL 也要跟着放大。
 *
 * 设成 0xFFFFFFFF 就是"永远等命令"（不会再自动跳 app）。
 * ================================================================ */
#define BOOT_WAIT_MS 10000U

/* ================================================================
 * 轻量串口输出 / 数字解析（替代 libc 的 printf / atoi / strtol）
 * ================================================================ */
int  bl_printf(const char *fmt, ...);
void bl_puts(const char *s);

/* 原样吐字节，不做 '\n' → '\n' 转换。
 * 给 R 命令回读裸数据用，绝对不能做任何字符替换 */
void bl_write(const uint8_t *d, uint32_t n);

/* 把字符串里的数字解析成 u32。支持 0x 前缀十六进制、纯十进制。
 * *ps 会被推进到数字后面。返回 1 = 解析成功，0 = 没解析到数字 */
uint8_t bl_parse_u32(const char **ps, uint32_t *out);

/* 在 hay 里找 needle，找不到返回 NULL（替代 strstr） */
const char *bl_strstr(const char *hay, const char *needle);

/* ================================================================
 * 全局状态
 * ================================================================ */
extern volatile uint32_t uart_rec_full_len;  /* 累计收到的字节数 */
extern volatile uint32_t rx_errors;          /* 串口出错次数（溢出/帧错），正常恒为 0 */
extern volatile uint32_t rx_overflow;        /* 环形缓冲被迫丢掉的字节数，正常恒为 0 */

extern uint32_t flash_write_offset;          /* OTA 时已写进 W25Q 的字节数 */

extern UART_HandleTypeDef huart1;

/* ================================================================
 * 接口
 * ================================================================ */
void init_bootloader(void);      /* 打开中断接收，上电调一次 */

/* ---- OTA 进度界面（SSD1306 128x64，软件 I2C：PB10=SCL / PB11=SDA）----
 * 只有 APP 写了 BKP 魔数、bootloader 进 OTA 常驻时才会点亮屏幕；
 * 普通上电这两句即使被调用也是空操作，不碰 I2C。 */
void ota_ui_begin(void);         /* 进 OTA 常驻时调一次：清屏 + 画"升级中" + 0% */
void ota_ui_set_pct(uint8_t pct);/* 进度 0~100，变 1% 就刷 */
void ota_ui_ramp(uint8_t target);/* 阶段切换用：1%/20ms 走过去，别直接跳台阶 */
void ota_ui_restore(void);       /* 跳 app 前必须调：清屏 + 还原整屏窗口（u8g2 不重设窗口） */

/* OTA 收数据：攒够一包（chunk 字节）→ 写 W25Q → 回 OK。
 * base  = W25Q 里的写入基址（A/B 槽制下 = 目标槽起始地址）
 * total = 整个镜像长度，chunk = 上位机声明的每包字节数。
 * 已写字节数记在 flash_write_offset（相对 base 的偏移），断点续传
 * 就是把它拨到断点位置再接着调本函数。
 * 返回 0 = 正常，-1 = 失败（超时/短包/写不进） */
int  bootloader_process(uint32_t base, uint32_t total, uint32_t chunk, uint32_t timeout_ms);

/* 擦除 [start, end) 覆盖到的页。返回 0 = 成功 */
int erase_page(uint32_t start, uint32_t end);

/* 跳转到 app。返回 0 = 成功（其实不会再返回了），-1 = 校验没过 */
int bl_jump_app(void);

/* 环形缓冲里现在有多少字节可读 */
uint32_t bl_rx_avail(void);

/* 清空环形缓冲（丢掉所有还没被处理的数据） */
void bl_rx_reset(void);

/* 从环形缓冲里取一个字节。1 = 取到，0 = 环空。
 * 命令阶段（等一行命令）用它代替轮询 DR —— 打印期间也能收，不丢字节 */
uint8_t bl_rx_get(uint8_t *c);

/* 从环形缓冲精确取 want 个字节。
 *   timeout_ms == 0  → 非阻塞：不够就返回已取到的（可能是 0）
 *   timeout_ms > 0   → 一直等到攒够 want 个，或超时
 * 返回实际取到的字节数；只有 == want 才算成功。
 * 【为什么按 want 取】want 就是上位机在命令里声明的每包长度。
 * 靠它分帧，才不会被 USB 串口的空闲间隙把一包切成好几段 */
uint32_t bl_read_exact(uint8_t *buf, uint32_t want, uint32_t timeout_ms);

/* ================================================================
 * 让原来所有的 printf(...) 调用自动走 bl_printf
 *
 * 用可变参数宏，这样 printf("abc") 和 printf("%d", x) 都能正确处理。
 * ================================================================ */
#define printf(...) bl_printf(__VA_ARGS__)

#endif /* BOOTLOADER_H_ */
