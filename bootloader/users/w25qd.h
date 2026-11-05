/*
 * w25qd.h —— W25Q SPI Flash 驱动 + 升级完成标志 + A/B 槽位元数据
 *
 * 【驱动方式】用 HAL_SPI（SPI1），不是裸寄存器。
 * 命令/地址这些几个字节的小传输走阻塞模式（HAL_SPI_TransmitReceive），
 * 真正搬数据（页编程 256 字节、读回 512 字节）走 DMA
 * （HAL_SPI_TransmitReceive_DMA / HAL_SPI_Receive_DMA），
 * 这样 CPU 不用一个字节一个字节等 SPI 的 TXE/RXNE 标志。
 *
 * 【为什么片选用 HAL_GPIO_WritePin 而不是直接写 BSRR】
 * 可读性优先。CS 翻转一次只有几十纳秒的差别，对 W25Q 完全无影响。
 *
 * 接线：PA4=CS  PA5=SCK  PA6=MISO  PA7=MOSI
 * 参数：SPI1 主模式，8bit，Mode0（CPOL=0 CPHA=0），MSB first，72M/4 = 18MHz
 */

#ifndef W25QD_H_
#define W25QD_H_

#include "main.h"
#include "bootloader.h"     /* 为了拿到 printf 宏和 bl_parse_u32 */

/* ---- 片选：PA4（CS_Pin / CS_GPIO_Port 在 main.h 里由 CubeMX 定义） ---- */
#define CS_HIGH()   HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET)
#define CS_LOW()    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET)

/* ---- W25Q 容量：上电读 JEDEC 自动识别，不再写死 ----
 * JEDEC 第 3 字节（容量）对照：
 *   0x14 = W25Q80  1MB   0x15 = W25Q16  2MB
 *   0x16 = W25Q32  4MB   0x17 = W25Q64  8MB
 *   0x18 = W25Q128 16MB  （板① 接线表用的是 W25Q128）
 * 识别失败（不在这张表里）w25qd_init() 返回 0 并打印实测值。
 * 实际容量用 w25q_total_size() 拿（init 成功之前它是 0）。 */
uint32_t w25q_total_size(void);

/* ================================================================
 * W25Q 分区规划（A/B 双槽 OTA）
 * ================================================================
 *   0x000000 ~ 0x00FFFF   槽 A（APP A）        64K
 *   0x010000 ~ 0x01FFFF   槽 B（APP B）        64K
 *   0x020000 ~ 0x02FFFF   旧 APP 备份区        64K（升级前备份内部 app，
 *                                              失败可用 B 命令回滚）
 *   0x0FE000              槽位元数据（4K 扇区，掉电保存，带 CRC）
 *   0x0FF000              升级完成标志（不变）
 *
 * 【为什么槽给 64K】app 区只有 40K，64K = 16 个 4K 扇区，
 * 整数个扇区、按扇区对齐，以后 app 再大也有余量。
 * 两个槽 + 备份区 + 元数据全部落在 1MB 以内，
 * 所以 W25Q80（1MB）和 W25Q128（16MB）都能用这套布局。
 *
 * 【active_slot 的含义】0 = APP A，1 = APP B，0xFF = 无记录
 * （比如元数据还没写过，或者 app 是 ST-Link 直接烧的）。
 * 它记录的是"当前内部 flash 里跑的固件对应哪个槽"，
 * 切换版本 = A 命令把另一个槽搬进内部 flash。
 */

#define W25Q_SLOT_SIZE      (64*1024UL)
#define W25Q_SLOT_A_ADDR    0x000000UL
#define W25Q_SLOT_B_ADDR    0x010000UL
#define W25Q_BACKUP_ADDR    0x020000UL   /* 备份区起始（原 0x010000，让位给槽 B） */

#define W25Q_BACKUP_MAGIC   0x314B5042UL   /* 'BPK1' 小端 */
#define W25Q_BACKUP_HDR_LEN 16UL           /* 备份头长度：magic/len/crc/reserved */

#define W25Q_META_ADDR      0x0FE000UL     /* 槽位元数据（独立 4K 扇区） */

/* 槽号 → 槽起始地址。slot 必须是 0 或 1（调用方保证） */
#define W25Q_SLOT_ADDR(slot)  ((slot) == 0 ? W25Q_SLOT_A_ADDR : W25Q_SLOT_B_ADDR)

/* 版本号打包：version = (major<<16) | (minor<<8) | patch */
#define W25Q_VER(maj, min, pat) \
    ((((uint32_t)(maj)) << 16) | (((uint32_t)(min)) << 8) | ((uint32_t)(pat)))

/* ================================================================
 * 槽位元数据（存在 W25Q_META_ADDR，掉电保存）
 *
 * 【为什么整机只有一份、为什么每次全量重写】
 * 元数据很小（几十字节），但 NOR flash 只能 1→0 写，
 * 改任何一个字段都得先擦整个扇区。所以干脆每次 load 到 RAM、
 * 改完 save 全量重写 + 回读校验。写入频率被刻意压到最低：
 * 断点进度每 4KB 才记一次（见 appbl.c），每次升级全程也就十来次擦写，
 * 4K 扇区 10 万次寿命绰绰有余。
 * ================================================================ */
typedef struct {
    uint32_t magic;             /* W25Q_META_MAGIC */
    uint8_t  active_slot;       /* 0=A 1=B 0xFF=无 */
    uint8_t  rsv[3];
    struct {
        uint8_t  valid;         /* 1 = 槽内固件 CRC 校验通过，可激活/可查版本 */
        uint8_t  rsv[3];
        uint32_t version;       /* W25Q_VER(maj,min,pat) */
        uint32_t size;          /* 固件字节数 */
        uint32_t crc;           /* 固件内容 CRC32 */
    } slot[2];
    struct {
        uint32_t slot;          /* 正在往哪个槽传，0xFFFFFFFF = 没有进行中的传输 */
        uint32_t size;          /* 本次要传的文件大小 */
        uint32_t crc;           /* 本次文件的 CRC32（和 size 一起当作"文件指纹"） */
        uint32_t version;       /* 本次文件的版本号 */
        uint32_t written;       /* 已经写进槽的字节数 = 断点位置 */
        uint32_t chunk;         /* 断点时用的每包字节数（续传要沿用同一个分帧） */
    } xfer;
    uint32_t crc;               /* 本结构体前面所有字节的 CRC32（不含 crc 字段本身） */
} w25q_meta_t;

#define W25Q_META_MAGIC 0x4154454DUL    /* 'META' 小端 */

/* 读元数据。合法（magic + CRC 都对）返回 1 并填 *m；
 * 不合法（没写过/写坏了）填一份"干净默认值"（active=0xFF、槽全无效、
 * xfer 无进行中）并返回 0。调用方拿到的 *m 任何情况下都能直接用 */
uint8_t meta_load(w25q_meta_t *m);

/* 保存元数据：擦 META 扇区 → 写入 → 回读比对。返回 1 = 成功 */
uint8_t meta_save(const w25q_meta_t *m);

/* ================================================================
 * SPI1 收尾
 * ================================================================ */
/* 拉高片选 + HAL_SPI_DeInit，跳转 app 前调用。
 * 【注意】没有 w25q_spi_init()：SPI1 的初始化交给 CubeMX 生成的
 * MX_SPI1_Init()（main.c）和 HAL_SPI_MspInit()（stm32f1xx_hal_msp.c），
 * 那里已经把 GPIO、时钟、DMA 通道都配好了 */
void w25q_spi_deinit(void);

/* ================================================================
 * W25Q 驱动
 * ================================================================ */
uint8_t w25qd_init(void);                                            /* 1 = 成功 */
uint8_t w25qd_erase(uint32_t addr, uint32_t num_sectors);            /* 1 = 成功 */
uint8_t w25qd_write(uint32_t addr, uint8_t *data, uint32_t length);  /* 1 = 成功 */
uint8_t w25qd_read(uint32_t addr, uint8_t *buf, uint32_t len);       /* 1 = 成功 */

/* ================================================================
 * 升级完成标志（掉电保护）
 *
 * 三态设计，避免"从没用过 OTA 的板子被永久拦在 bootloader"：
 *   0xFFFFFFFF  从没用过 OTA（ST-Link 烧的）→ 允许跳转
 *   0x5AA5A55A  上次升级成功               → 允许跳转
 *   0x00000000  升级中 / 上次升级失败       → 拒绝跳转
 *
 * 【和 active_slot 的分工】ota flag 管"内部 flash 里的 app 完不完整"
 * （防跳进残缺固件），active_slot 管"这份固件对应哪个槽"。
 * 两者互补，缺一不可：ST-Link 烧的 app 有 flag 没 slot 记录；
 * 只写了槽还没激活的固件有 slot 记录但不在内部 flash 里。
 *
 * 【为什么失败时故意不把它置回"可信"】
 * 写内部 Flash 的流程是"先置升级中 → 再擦 app 区"，所以一旦走到擦除，
 * 旧固件就已经没了。此时若把标志置回"可信"，bootloader 会跳进一个
 * 写了一半的固件直接跑飞 —— 正是这个标志要防的事。
 *
 * 【那卡住了怎么办】用 ISP 的 F 命令手工置回"可信"，见 appbl.c 里
 * cmd_force() 的说明。最容易踩的场景是"OTA 失败后改用 ST-Link 烧 app，
 * 但 ST-Link 不碰 W25Q，标志还挂在升级中"。
 * ================================================================ */
#define OTA_FLAG_ADDR   0x000FF000UL
#define OTA_FLAG_MAGIC  0x5AA5A55AUL

uint8_t  ota_flag_app_trusted(void);  /* 1 = app 可信，可以跳转 */
uint8_t  ota_flag_begin(void);        /* 擦内部 Flash 之前调用，置"升级中" */
uint8_t  ota_flag_commit(void);       /* 回读校验通过后调用，置"成功" */
uint32_t ota_flag_raw(void);          /* 读原始值，给 ? 命令回显用 */

#endif /* W25QD_H_ */
