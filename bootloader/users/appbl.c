/*
 * appbl.c —— 串口 OTA + ISP 命令集
 *
 * ================================================================
 * 零、上电后的行为（刷机窗口）
 *
 *   上电 / 复位
 *      │
 *      ├─ BOOT_WAIT_MS（默认 10 秒，在 bootloader.h）内收到**认识的**命令
 *      │     → 留在命令行陪你刷机（之后每敲一条认识的命令就继续留着）
 *      │
 *      ├─ 收到**不认识**的命令
 *      │     → 回 ER:unknown cmd, boot app，然后跳 app
 *      │
 *      └─ 一直没人理（超时）
 *            → 认为只是正常上电，跳 app
 *
 *   跳转失败（空片 / 固件坏了 / ota flag 不可信）就留在命令行等刷机，
 *   而且**只试一次**，不会每轮重试刷屏。
 *
 *   【副作用】跳 app 之前固定要等满 BOOT_WAIT_MS，所以冷启动会慢这么多。
 *   嫌慢就把 BOOT_WAIT_MS 调小，或者改成"按键才进 bootloader"。
 *
 *   【跳之前一定先判断 app 到底有没有东西】
 *   上面每一条"跳 app"都只是把状态机切到 bl_jump，真正跳不跳由
 *   bl_jump_app() 决定 —— 它会先校验 ota 标志、栈顶地址、复位向量，
 *   空片（读出全 0xFF）会被直接拦下，留在命令行等你刷机，
 *   **不会跳进一片空 Flash 把板子跑死**。详见 bootloader.c 里的说明。
 *
 * ================================================================
 * 一、命令一览（全部 ASCII，以 \r 或 \n 结尾；大小写不敏感）
 *
 *   单行回复：OK\n  或  ER:<原因>\n
 *   多行回复：第一行 OK，中间若干行，最后一行 END
 *   专用行：READY（数据流开始）/ RESUME <written> <chunk>（续传）
 *
 *   ?                              读设备信息（含 BLVER / ACTIVE / APPVER /
 *                                  RUNVER / SLOTA_BOOTABLE / SLOTB_BOOTABLE）
 *   H                              命令菜单
 *   P                              读分区表
 *   S                              槽位查询：A/B 槽的版本、大小、CRC、
 *                                  是否有效、是否可跳、哪个是激活槽、有没有断点
 *   Q                              断点续传进度查询（写到哪、下一包序号、
 *                                  哪个槽、文件多大、有没有可用断点 RESUME=0/1）
 *   E <addr> <len>                 擦除内部 flash [addr, addr+len)
 *   W <addr> <len> [chunk]         写入内部 flash（直写，不经 W25Q，调试用）
 *   R <addr> <len>                 读出内部 flash 裸数据
 *   V <addr> <len> <crc32>         校验该范围的 CRC32
 *   C <slot> [len]                 从 W25Q 槽里读回固件重算 CRC32（确认槽里那份
 *                                  就是你要的文件）
 *   O <slot> <len> <crc32> <ver> [chunk] [mode]
 *                                  OTA：固件下载到 W25Q 的 A/B 槽。
 *                                  ver 形如 1.2.3（可省略 = 0.0.0）。
 *                                  mode 省略 = 自动：与断点记录（槽/大小/CRC 一致）
 *                                  → 回 RESUME，从断点继续，不擦除；
 *                                  否则擦槽重传，回 READY。
 *                                  mode = 0 → 显式续传：指纹不一致回 ER:no resume
 *                                            且不擦槽；
 *                                  mode ≠ 0 → 显式新传：忽略断点，擦掉重收。
 *                                  旧格式 O <len> <crc32> [chunk] 仍兼容
 *                                  （槽自动选"非激活槽"）
 *   A <slot>                       激活槽位：把该槽固件搬进内部 flash、
 *                                  校验、记录 active_slot，然后跳转运行
 *   G                              跳转 app
 *   B                              回滚到备份区的旧 app（B 命令）
 *   F                              强制信任当前 app 并跳转（逃生口）
 *
 *   数字格式：0x 开头 = 十六进制，否则 = 十进制
 *   兼容旧写法：start:<十进制长度>:<十六进制CRC32>
 *
 * ================================================================
 * 二、OTA 状态流转（固件先落 W25Q 的 A/B 槽，校验通过才动内部 Flash）
 *
 *   bl_init      等命令
 *   bl_run       擦目标槽 → 回 READY
 *   bl_recdata   逐包收固件写进槽（每 4KB 记一次断点到元数据区）
 *   bl_checkdata 从槽读回来算 CRC32 比对 → 给槽记上"有效+版本"
 *   bl_backup    把当前内部 app 备份到 W25Q 备份区（可回滚）
 *   bl_activate / bl_flash
 *                擦内部 Flash → 从槽搬运 → 回读校验 → 记 active_slot
 *   bl_restore   回滚：从 W25Q 备份区恢复旧 app
 *   bl_jump      跳转运行
 *
 * 【核心思想】校验通过之前，内部 Flash 一个字节都不动。
 *            传输出错、CRC 不对、对端发错文件时，旧固件完好无损，
 *            设备重启照样能跑；断点留在槽里，重连后接着传。
 * ================================================================
 */

#include "appbl.h"
#include "bootloader.h"
#include "w25qd.h"
#include "crc32.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

typedef enum {
	bl_init,
	bl_run,
	bl_recdata,
	bl_checkdata,
	bl_backup,      /* 备份当前 app 到 W25Q 备份区 + 回读校验 */
	bl_restore,     /* 回滚：从 W25Q 备份区恢复旧 app */
	bl_flash,
	bl_jump
} bl_status;

static bl_status now_status = bl_init;

/* OTA 目标镜像的 CRC32 和长度（命令里给的） */
uint32_t crc32 = 0;
static uint32_t app_len = 0;

/* ================================================================
 * 本次 OTA 会话的目标槽（A/B 槽制）
 *
 * s_xfer_slot  槽号 0/1（active_slot 同一套编号）
 * s_xfer_base  该槽在 W25Q 里的起始地址（收数据、校验、搬运都用它）
 * s_xfer_ver   本次固件的版本号（W25Q_VER 打包格式），收完写进元数据
 * ================================================================ */
static uint32_t s_xfer_slot = 0;
static uint32_t s_xfer_base = W25Q_SLOT_A_ADDR;
static uint32_t s_xfer_ver  = 0;
static uint32_t s_last_ckpt = 0;    /* 上次记断点时的 4KB 边界 */

/* 【断点记到哪算一次】元数据的保存要"擦 4K 扇区 + 写 + 回读"，
 * 约 50~100ms，每包都记会把传输拖慢一倍。折中：
 * 第一包就记一次（覆盖"刚连上就断"的场景），
 * 之后每跨过一个 4KB 边界记一次 —— 掉电最多丢最后一个 4KB 的进度 */
#define XFER_CKPT_FIRST  s_chunk
#define XFER_CKPT_MASK   0xFFFFF000UL

/* ================================================================
 * 备份区（旧 APP）
 *
 * 升级流程里多出来的一步：在擦内部 flash 之前，
 * 先把当前 app 原样备份到 W25Q 的备份区，再读回来算 CRC 确认备份完整。
 * 备份不完整就**中止升级** —— 内部 flash 一个字节都没动，设备照常能跑。
 * ================================================================ */
typedef struct {
	uint32_t magic;      /* W25Q_BACKUP_MAGIC */
	uint32_t len;        /* 备份的字节数 */
	uint32_t crc;        /* 备份内容的 CRC32 */
	uint32_t reserved;   /* 对齐用 */
} backup_hdr_t;



/* 命令行缓冲。W/V 命令最长形如 "V 0x08005000 0xB000 0x12345678" 约 32 字节，
 * 留 64 足够，也给手工用串口助手敲命令留余量 */
#define CMD_BUF_SIZE 64
static uint8_t  cmd_buf[CMD_BUF_SIZE];
static uint16_t cmd_len = 0;

/* ISP 读写用的中转缓冲（512 字节） */
static uint8_t io_buf[uart_rec_len];

/* ================================================================
 * 备份区（旧 APP）相关实现
 *
 * 放在 io_buf 之后，因为下面几个函数都要用它做中转。
 * ================================================================ */

/* 读回备份内容算 CRC，和头里记的比。0=一致，-1=不一致或读失败 */
static int backup_crc_check(const backup_hdr_t *h)
{
	uint32_t off  = 0;
	uint32_t left = h->len;
	uint32_t crc  = 0xFFFFFFFFU;

	while (left > 0) {
		uint32_t n = (left > uart_rec_len) ? uart_rec_len : left;

		if (!w25qd_read(W25Q_BACKUP_ADDR + W25Q_BACKUP_HDR_LEN + off, io_buf, n)) {
			return -1;
		}
		crc = crc32_update(crc, io_buf, n);
		off  += n;
		left -= n;
	}
	crc ^= 0xFFFFFFFFU;

	return (crc == h->crc) ? 0 : -1;
}

/* 备份区状态：0=没有，1=有且校验通过，2=有但校验不过 */
static uint8_t backup_state(void)
{
	backup_hdr_t h;

	if (!w25qd_read(W25Q_BACKUP_ADDR, (uint8_t *)&h, sizeof(h))) {
		return 0;
	}
	if (h.magic != W25Q_BACKUP_MAGIC) {
		return 0;
	}
	if (h.len == 0 || h.len > APP_MAX_SIZE) {
		return 2;
	}
	/* 头对了还不够，内容也要重新算一遍 CRC —— 备份区可能被后来的
	 * 擦写踩坏，只信头会骗自己 */
	if (backup_crc_check(&h) != 0) {
		return 2;
	}
	return 1;
}

/* 内部 Flash 顺序写的续写状态 */
static uint32_t s_wr_addr    = 0;
static uint8_t  s_pend_byte  = 0;
static uint8_t  s_pend_valid = 0;

/* 接收超时（毫秒） */
#define RX_TIMEOUT_MS   5000U

/* 上位机在 W / O 命令里声明的"每包字节数"。
 *
 * 【为什么必须让上位机声明】
 * 上位机 write() 一次，MCU 收到的却可能是好几段（USB 串口芯片按
 * USB 包转发，包间有空隙，硬件 IDLE 中断 87us 就触发一次）。
 * MCU 只有知道"这一包该有多少字节"，才能按长度分帧，
 * 才能做到"收够一包回一个 OK"跟上位机的"发一包等一个 OK"对上。
 * 见 bootloader.c 里 bl_read_exact 的说明。 */
static uint32_t s_chunk = uart_rec_len;

/* 把上位机声明的每包大小收进合法范围。
 * 老上位机不带这个字段时给 0，那就退回 uart_rec_len —— 只要它每包
 * 不超过 512 就还是能跑（向后兼容） */
static uint32_t clamp_chunk(uint32_t c)
{
	if (c == 0 || c > RX_CHUNK_MAX) {
		return uart_rec_len;
	}
	return c;
}


/* ================================================================
 * 版本号：解析 "1.2.3" ↔ 打包 u32
 *
 * 打包格式 (major<<16)|(minor<<8)|patch，O 命令带进来、
 * 元数据存这个、S / ? 命令按点分格式打出去。
 * 对端（ESP8266 / 调试串口助手）只需要会拼 "1.2.3" 这个字符串。
 * ================================================================ */

/* 解析形如 1.2.3 / 1.2 / 1 的点分版本号。
 * 【为什么不用 bl_parse_u32 连着解析】它只认 0x/十进制，
 * 点号会终止数字 —— 正好利用这一点：数字后面跟 '.' 就继续取下一段 */
static uint8_t parse_version(const char **ps, uint32_t *ver)
{
	uint32_t maj, min = 0, pat = 0;

	if (!bl_parse_u32(ps, &maj)) {
		return 0;
	}
	if (**ps == '.') {
		(*ps)++;
		if (!bl_parse_u32(ps, &min)) {
			return 0;
		}
		if (**ps == '.') {
			(*ps)++;
			if (!bl_parse_u32(ps, &pat)) {
				return 0;
			}
		}
	}
	if (maj > 0xFFU || min > 0xFFU || pat > 0xFFU) {
		return 0;
	}
	*ver = W25Q_VER(maj, min, pat);
	return 1;
}

/* 按点分格式打印版本号（不换行，调用方自己收尾） */
static void print_version(uint32_t v)
{
	printf("%u.%u.%u",
	       (unsigned)((v >> 16) & 0xFFU),
	       (unsigned)((v >> 8) & 0xFFU),
	       (unsigned)(v & 0xFFU));
}


/* ================================================================
 * 一、范围与参数校验
 * ================================================================ */

/* 地址范围必须完整落在 app 区内。
 * 【关键】这一句是防变砖的第一道闸：上位机永远别想碰到 bootloader 自己 */
static uint8_t range_ok(uint32_t addr, uint32_t len)
{
	if (len == 0) {
		return 0;
	}
	if (addr < app_bl_addr) {
		return 0;                       /* 不许碰 bootloader 区 */
	}
	if ((addr + len) < addr) {
		return 0;                       /* 加法溢出 */
	}
	if ((addr + len) > flash_end) {
		return 0;
	}
	return 1;
}

/* 从字符串里连读两个数：addr 和 len */
static uint8_t parse_addr_len(const char *p, uint32_t *addr, uint32_t *len)
{
	if (!bl_parse_u32(&p, addr)) {
		return 0;
	}
	if (!bl_parse_u32(&p, len)) {
		return 0;
	}
	return 1;
}


/* ================================================================
 * 二、命令行接收
 * ================================================================ */

/* 阻塞等一行命令。返回 1 = 收到，0 = 超时。
 *
 * 【必加】先把 IT 接收停掉：否则阻塞版 HAL_UARTEx_ReceiveToIdle 会因为
 * RxState=BUSY_RX 直接返回 HAL_BUSY，而 HAL_BUSY 会走"超时"分支去跳 app ——
 * 表现为"CRC 校验失败后静默跑回旧固件，什么错都不报" */
/* ---- 收一行命令（两个口都收）--------------------------
 * 为什么不用 HAL_UARTEx_ReceiveToIdle：F1 HAL 的这个阻塞接收在超时上并不可靠，
 * 它会一直等到有数据才返回——现场表现就是卡在等命令的时候
 * bootloader 永远不会跳 app。这里改成直接轮询 DR 寄存器，
 * 时间到就返回，行为确定。
 *
 * 两个口：USART2（PA2/PA3，板上「调试UART 4P」，手动命令走这里）
 *         USART1（PA9/PA10，ESP8266 的 OTA 通道）
 *
 * 返回 1 = 收到完整一行（已放进 cmd_buf），0 = 超时 */
/* 【上电按键进 OTA】这颗空闲按键(PA12)的全程监听开关。
 * 1 = 还在开机等待窗口里，随时按一下都能进 OTA 常驻；
 * 0 = 已经收到真命令（协议开始了）或已经进过 OTA，不再监听。
 * 【为什么是"全程"而不是只在开机那 200ms】
 *   现场第一次验证就是这样失败的：开机瞬间采样 200ms，用户是等板子上电、
 *   听到 APP 自检的两声"哔"以后才去按 —— 那已经是 10 秒窗口的第 10 秒了，
 *   采样早结束了。所以把监听放进 wait_command 的等待循环里，
 *   整个窗口期间按一下都算数。 */
static uint8_t  s_key_watch  = 0U;
static uint32_t s_key_low_ms = 0U;      /* 0 = 当前没按住 */

static void key_takeover(void);         /* 实现在 buzzer_init 之后（那里才有 BUZZER_ON/OFF） */

/* 返回 1 = 收到完整一行（已放进 cmd_buf），0 = 超时 */
static uint8_t cmd_collect(uint8_t c)
{
	if ((c == '\r') || (c == '\n')) {
		if (cmd_len == 0U) {
			return 0;                       /* 空行，忽略 */
		}
		cmd_buf[cmd_len] = '\0';
		s_key_watch = 0U;                   /* 协议都开始了，别再监听按键 */
		return 1;
	}
	if (cmd_len < (CMD_BUF_SIZE - 1U)) {
		cmd_buf[cmd_len++] = c;
	}
	return 0;
}

static uint8_t wait_command(uint32_t timeout_ms)
{
	uint32_t t0 = HAL_GetTick();
	uint8_t  c;

	/* 【为什么这里不再轮询 DR（2026-11-03 现场定位）】
	 * 原来的写法每进一次本函数就 AbortReceive + 轮询 huart1->SR/RXNE。
	 * 问题在于"上一条命令的收尾打印"这段时间里没人读 DR：
	 *   `?` 应答最后一行 END 发完 → dispatch 返回 → 这里补打 "BL:cmd"（9 字节）
	 * 上位机（8266）一看到 END 就立刻发下一条 O，O 的头部正好落进盲窗，
	 * 硬件置 ORE —— 37 字节的命令只捡回第一个字节 'O'，其余全丢。
	 * 结果：命令没进 dispatch，既没有 ER 也没有 READY，
	 *      上位机干等 30 秒（st=3），现场就是"一升级就卡在 bootloader"。
	 *
	 * 现在中断接收（HAL_UARTEx_ReceiveToIdle_IT + 环形缓冲）从 app_bl_init
	 * 一进来就开着，打印期间照样往环里塞；本函数只从环里取字节，
	 * 盲窗彻底不存在。USART2（板上调试口，手动敲命令）仍然轮询。 */
	memset(cmd_buf, 0, sizeof(cmd_buf));
	cmd_len = 0;

	for (;;) {
		/* 【整个开机等待窗口都监听那颗空闲按键(PA12)】
		 * 不去抖不行：按键是机械的，按下瞬间会弹几下，20ms 连续低电平才算按。
		 * 按住了就进 OTA 常驻（第二声"哔"落地），然后就不再重复触发。 */
		if (s_key_watch != 0U) {
			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_RESET) {
				uint32_t now_t = HAL_GetTick();

				if (s_key_low_ms == 0U) {
					s_key_low_ms = (now_t != 0U) ? now_t : 1U;
				} else if ((now_t - s_key_low_ms) >= 20U) {
					key_takeover();
					continue;
				}
			} else {
				s_key_low_ms = 0U;
			}
		}

		/* 调试口（USART2）优先：手工敲命令用 */
		if ((huart2.Instance->SR & USART_SR_RXNE) != 0U) {
			c = (uint8_t)(huart2.Instance->DR & 0xFFU);
			if (cmd_collect(c) != 0U) {
				return 1;
			}
			continue;
		}

		/* OTA 通道（USART1）：从环形缓冲取，不碰 DR */
		if (bl_rx_get(&c) != 0U) {
			if (cmd_collect(c) != 0U) {
				return 1;
			}
			continue;
		}

		if ((HAL_GetTick() - t0) >= timeout_ms) {
			return 0;                       /* 整体超时 */
		}
	}
}
/* ================================================================
 * 三、ISP 命令实现
 * ================================================================ */

/* ================================================================
 * 运行中 app 的版本（从内部 flash 的固定标签读）
 *
 * 【为什么不能只靠 W25Q 元数据】
 *   元数据记的是「槽里那份固件的版本」，那是还没激活的。
 *   内部 flash 里现在跑的这一版，只有它自己的标签说了算。
 *
 * 【读不到怎么办】
 *   app 是用 ST-Link 直接烧的（没经过 OTA）→ 没有标签 → 返回 0，
 *   上位机显示 unknown，这是正常情况，不是故障。
 * ================================================================ */
static uint8_t read_running_version(uint32_t *ver)
{
	uint32_t tag[2];

	/* 标签在 app 区偏移 0x100 处。app 区被擦过就会读出 0xFF，
	 * magic 对不上 → 直接判无效，不会误读 */
	memcpy(tag, (const void *)(uintptr_t)APP_TAG_ADDR, sizeof(tag));
	if (tag[0] != APP_TAG_MAGIC) {
		return 0;
	}
	*ver = tag[1];
	return 1;
}

/* 某槽能不能跳（0=不能，1=能）
 *
 * 【语义】"这个槽里的固件经过校验、随时可以搬进内部 flash 跑"。
 * 判据 = 槽记录 valid（CRC 校验通过过）+ 大小合理。
 * 注意：这跟"槽是不是激活槽"是两回事 —— 非激活槽也能跳
 * （那正是 A 命令干的事：把另一个槽搬进去）。
 *
 * 【为什么要有这个字段】
 *   上位机要回答"这台设备现在能升到哪个版本"，
 *   光看 valid 不够：valid=1 只说明"曾经校验通过"，
 *   还要大小在 app 区放得下才算真能跳。 */
static uint8_t slot_bootable(const w25q_meta_t *m, uint32_t slot)
{
	if (!m->slot[slot].valid) {
		return 0;
	}
	if (m->slot[slot].size == 0U || m->slot[slot].size > APP_MAX_SIZE) {
		return 0;
	}
	return 1;
}

/* ---- ? 读设备信息 ---- */
static void cmd_info(void)
{
	w25q_meta_t m;
	uint32_t    runver = 0;

	bl_puts("OK\n");
	printf("DEV=STM32F103C8\n");
	printf("BLVER=2.1\n");
	printf("FLASH=%u\n",     (unsigned)(flash_end - FLASH_BASE));
	printf("PAGE=%u\n",      (unsigned)page_size);
	printf("APPADDR=0x%08X\n", app_bl_addr);
	printf("APPSIZE=%u\n",   (unsigned)APP_MAX_SIZE);
	printf("MAXCHUNK=%u\n",  (unsigned)RX_CHUNK_MAX);
	printf("W25QSIZE=%u\n",  (unsigned)w25q_total_size());
	/* 升级完成标志的原始值。
	 * 【为什么值得回显】它是"跳不过去"最常见的原因之一，
	 * 而且不看这个值根本没法判断：0xFFFFFFFF=从没用过（放行）、
	 * 0x5AA5A55A=可信（放行）、0x00000000=升级中（拒绝跳转，要重刷或用 F） */
	printf("OTFLAG=0x%08X\n", ota_flag_raw());
	/* 备份区状态：有没有一份"校验通过"的旧 APP 备份。
	 * 0=没有，1=有（可回滚），2=有但校验不过（备份区坏了，别指望它） */
	printf("BKUP=%u\n", (unsigned)backup_state());

	/* 激活槽 + 正在运行的固件版本（来自元数据，OTA 装机时记录的）。
	 * 0xFF/unknown = 没有记录（比如 app 是 ST-Link 直接烧的） */
	meta_load(&m);
	if (m.active_slot <= 1U) {
		printf("ACTIVE=%u\n", (unsigned)m.active_slot);
		if (m.slot[m.active_slot].valid) {
			printf("APPVER=");
			print_version(m.slot[m.active_slot].version);
			printf("\n");
		} else {
			bl_puts("APPVER=unknown\n");
		}
	} else {
		bl_puts("ACTIVE=none\n");
		bl_puts("APPVER=unknown\n");
	}

	/* RUNVER = 内部 flash 里**现在正在跑**的这一版的版本号。
	 * 直接读 app 自己的标签，跟"槽里那份待激活的固件"是两回事 */
	if (read_running_version(&runver)) {
		bl_puts("RUNVER=");
		print_version(runver);
		bl_puts("\n");
	} else {
		bl_puts("RUNVER=unknown\n");
	}

	/* 每个槽能不能跳 —— 上位机据此决定"能升到哪个版本" */
	printf("SLOTA_BOOTABLE=%u\n", (unsigned)slot_bootable(&m, 0));
	printf("SLOTB_BOOTABLE=%u\n", (unsigned)slot_bootable(&m, 1));

	bl_puts("END\n");
}

/* ---- S 槽位查询 ----
 * A/B 两个槽各自是什么版本、多大、CRC 多少、有没有校验通过的固件、
 * 能不能跳、哪个是激活槽、有没有没传完的断点 —— 对端决定"要不要升级、传哪个槽"
 * 之前先看这里。各字段含义见 w25qd.h 的 w25q_meta_t 注释
 *
 * 【字段格式固定】SLOT=<n>,VER=<x.y.z|none>,SIZE=<n>,CRC=0x%08X,
 *              VALID=<0|1>,BOOTABLE=<0|1>,ACTIVE=<0|1>
 * 对端按 KEY=VALUE 逗号分隔解析，不要按位置解析。 */
static void cmd_slots(void)
{
	w25q_meta_t m;

	meta_load(&m);

	bl_puts("OK\n");
	for (uint32_t i = 0; i < 2U; i++) {
		printf("SLOT=%u,VER=", (unsigned)i);
		if (m.slot[i].valid) {
			print_version(m.slot[i].version);
		} else {
			bl_puts("none");
		}
		printf(",SIZE=%u,CRC=0x%08X,VALID=%u,BOOTABLE=%u,ACTIVE=%u\n",
		       (unsigned)m.slot[i].size,
		       (unsigned)m.slot[i].crc,
		       (unsigned)m.slot[i].valid,
		       (unsigned)slot_bootable(&m, i),
		       (unsigned)(m.active_slot == i ? 1U : 0U));
	}
	if (m.xfer.slot == 0xFFFFFFFFUL) {
		bl_puts("XFER=none\n");
	} else {
		/* 断点：正在进行的那次传输。SIZE+CRC 是文件指纹，
		 * 对端拿它和手里要传的文件一比就知道能不能续 */
		printf("XFER=SLOT=%u,VER=", (unsigned)m.xfer.slot);
		print_version(m.xfer.version);
		printf(",SIZE=%u,CRC=0x%08X,WRITTEN=%u,CHUNK=%u\n",
		       (unsigned)m.xfer.size,
		       (unsigned)m.xfer.crc,
		       (unsigned)m.xfer.written,
		       (unsigned)m.xfer.chunk);
	}
	bl_puts("END\n");
}

/* ---- Q 断点续传进度查询 ----
 * 比 S 里那行 XFER 更针对"接着传"这个动作。
 *
 * 【字段格式固定】PROG=SLOT=<n>,SIZE=<n>,CRC=0x%08X,WRITTEN=<字节>,
 *              NEXTSEQ=<下一包序号>,CHUNK=<每包字节>,RESUME=<0|1>
 *
 * 【NEXTSEQ 怎么算】
 *   NEXTSEQ = WRITTEN / CHUNK（整除）。对端按它对齐自己发包的计数：
 *   已经写进去 WRITTEN 字节 = NEXTSEQ 个整包，所以下一个包就是第 NEXTSEQ 号。
 *
 * 【RESUME 字段的语义 —— 这是"收包 0 表示续传"的落点】
 *   RESUME=1 → 有可用断点，且文件指纹（SIZE+CRC）有效，对端应该续传，
 *              从 WRITTEN 继续发。
 *   RESUME=0 → 没有断点（written=0）或没有进行中的传输，对端应该新传
 *              （bootloader 会擦槽重收）。
 *   对端不必自己猜：读 Q 拿 RESUME，1 就续、0 就新传。
 *
 * 【WRITTEN 是字节数，不是包数】
 *   写 Flash 的粒度是字节，断点也记字节。下一包要从第 WRITTEN 个字节开始。 */
static void cmd_prog(void)
{
	w25q_meta_t m;

	meta_load(&m);

	bl_puts("OK\n");
	if (m.xfer.slot == 0xFFFFFFFFUL || m.xfer.written == 0U || m.xfer.chunk == 0U) {
		/* 没有可用断点：对端应当新传。
		 * 【格式必须正好是 PROG=none】
		 *   对端（ESP8266 的 OtaEngine::queryResumeInfo）认这一行，
		 *   认成"没断点"而不是"解析失败"。别的写法它会当成一条
		 *   字段缺失的坏记录。 */
		bl_puts("PROG=none\n");
	} else {
		printf("PROG=SLOT=%u,SIZE=%u,CRC=0x%08X,WRITTEN=%u,NEXTSEQ=%u,CHUNK=%u,PKTS=%u,RESUME=1\n",
		       (unsigned)m.xfer.slot,
		       (unsigned)m.xfer.size,
		       (unsigned)m.xfer.crc,
		       (unsigned)m.xfer.written,
		       (unsigned)(m.xfer.written / m.xfer.chunk),
		       (unsigned)m.xfer.chunk,
		       (unsigned)m.xfer.written / m.xfer.chunk);
	}
	bl_puts("END\n");
}

/* ---- H 命令菜单 ----
 * 【给谁用的】一是调试时拿串口助手连上来忘了命令；
 * 二是 ESP8266 这类对端上电后可以先要一份菜单做协议自检。
 * 每行都带命令字母开头，上位机按 OK/ER/END 框架解析时当普通数据行 */
static void cmd_menu(void)
{
	bl_puts("OK\n");
	bl_puts("CMD ?        device info (BLVER/ACTIVE/APPVER/RUNVER/BOOTABLE)\n");
	bl_puts("CMD S        slot query (A/B version,size,crc,valid,bootable,xfer)\n");
	bl_puts("CMD Q        resume progress (written,nextseq,slot,size,resume)\n");
	bl_puts("CMD O        ota download: O <slot> <len> <crc32> <ver> [chunk] [mode]\n");
	bl_puts("CMD C        w25q slot check: C <slot> [len] -> crc,size,match\n");
	bl_puts("CMD A        activate slot: A <slot> (flash+jump)\n");
	bl_puts("CMD E        erase internal: E <addr> <len>\n");
	bl_puts("CMD W        write internal: W <addr> <len> [chunk]\n");
	bl_puts("CMD R        read internal: R <addr> <len>\n");
	bl_puts("CMD V        verify internal: V <addr> <len> <crc32>\n");
	bl_puts("CMD P        partition table\n");
	bl_puts("CMD G        boot app\n");
	bl_puts("CMD B        restore backup (rollback)\n");
	bl_puts("CMD F        force trust app and boot\n");
	bl_puts("CMD H        this menu\n");
	bl_puts("END\n");
}

/* ---- A 激活槽位 ----
 * 把槽里那份校验过的固件搬进内部 flash 并跳转 —— 版本切换就是它。
 * 【走 bl_backup 先备份】激活会覆盖内部 flash，先把现在的 app
 * 备份到备份区（和 O 流程同一条路），万一新版本有问题还能 B 回滚。
 * 真正的动作放状态机里做，这里只应答，免得对端以为板子死了 */
static void cmd_activate(const char *p)
{
	uint32_t    slot;
	w25q_meta_t m;

	if (!bl_parse_u32(&p, &slot)) {
		bl_puts("ER:need slot\n");
		return;
	}
	if (slot > 1U) {
		bl_puts("ER:slot range\n");
		return;
	}
	meta_load(&m);
	if (!m.slot[slot].valid) {
		bl_puts("ER:slot empty\n");     /* 槽里没有校验通过的固件 */
		return;
	}

	s_xfer_slot = slot;
	s_xfer_base = W25Q_SLOT_ADDR(slot);
	app_len     = m.slot[slot].size;
	crc32       = m.slot[slot].crc;
	s_xfer_ver  = m.slot[slot].version;
	s_last_ckpt = 0xFFFFFFFFUL;         /* 激活不走收数据，不记断点 */

	bl_puts("OK\n");
	now_status = bl_backup;
}


/* ================================================================
 * 蜂鸣器（PB8，低电平响 —— 模块带三极管驱动）
 *
 * 【bootloader 里也用它】刷机的人眼睛不一定盯着串口，
 * 蜂鸣器把"开始传 / 传得动 / 传完了 / 传失败了"变成听得见的信号：
 *   O 开始      短响 1 声
 *   每 4KB 断点  极短一响（滴滴滴的进度音）
 *   升级成功    两声长响 → 跳 app
 *   失败        三声急促短响
 *
 * 【为什么在这里手工初始化 GPIO 而不是改 .ioc】bootloader 的 .ioc
 * 是 CubeMX 工程属性，改了要重新生成代码才落到 main.c；
 * 手工 init 就几行，不碰生成代码，体积也可控。
 * ================================================================ */
#define BUZZER_PORT  GPIOB
#define BUZZER_PIN   GPIO_PIN_8
#define BUZZER_ON()  HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET)
#define BUZZER_OFF() HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET)

static void buzzer_init(void)
{
	GPIO_InitTypeDef g = {0};

	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* 【必须先写高，再配成输出】F1 的 GPIO 复位后 ODR=0。
	 * 蜂鸣器是低电平触发 —— 如果先 HAL_GPIO_Init 把脚变成推挽输出，
	 * 输出级立刻按 ODR=0 驱动，就在配置的那一瞬间给了蜂鸣器一个"响"的电平，
	 * 要等到下一行 BUZZER_OFF() 才关掉。APP 侧（main.c 的 MX_GPIO_Init）
	 * 就是先 WritePin 再 Init 的顺序，这里保持一致。 */
	BUZZER_OFF();

	g.Pin   = BUZZER_PIN;
	g.Mode  = GPIO_MODE_OUTPUT_PP;
	g.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BUZZER_PORT, &g);
	BUZZER_OFF();                       /* 配完再确认一次空闲不响 */
}

/* 响 times 次：on_ms 响、off_ms 停。
 * 【阻塞多久要心里有数】115200 下 4KB 约 356ms，进度音 15ms 只占 4%；
 * 成功/失败的提示音反正那时数据已经传完了，慢一点无所谓 */
static void buzzer_beep(uint32_t on_ms, uint32_t off_ms, uint8_t times)
{
	while (times--) {
		BUZZER_ON();
		HAL_Delay(on_ms);
		BUZZER_OFF();
		if (times) {
			HAL_Delay(off_ms);
		}
	}
}


/* ================================================================
 * OTA 会话：新传 / 续传 的唯一判定点
 *
 * 【续传条件】元数据里记着的断点，和这次 O 命令描述的是"同一个文件"：
 *   同槽 + 同大小 + 同 CRC32（大小和 CRC 一起构成文件指纹），
 *   且断点位置 written 满足 0 < written <= len。
 * 指纹对得上 → 槽里那截数据就是同一个文件的前缀，直接接着收；
 * 对不上（换了文件/换了槽/上次根本没传出数据）→ 全新传输，
 *   先把新断点记进元数据（written=0），bl_run 再去擦槽。
 *
 * 【mode 参数 —— "收包 0 表示续传，否则新传"的落点】
 *   O <slot> <len> <crc32> <ver> [chunk] [mode]
 *     mode 省略 → 自动：指纹一致就续，否则新传（老上位机的行为）
 *     mode = 0  → **显式续传**：指纹必须一致，否则回 ER:no resume，
 *                 并且**不擦槽**（保住已有断点，让上位机能重试）
 *     mode ≠ 0  → **显式新传**：忽略已有断点，擦掉重收
 *   上位机先发 Q 拿进度，读到 RESUME=1 就带 mode=0 续传；
 *   想从头再来就带 mode=1。
 *
 * 【为什么 written == len 也走续传】那说明上次数据已经收完了、
 * 只是还没来得及校验/激活就断线了。槽里是完整的 len 字节，
 * 直接跳到校验态省一遍重传；内容真坏了 CRC 校验自然会拦住。
 * ================================================================ */
#define OTA_MODE_AUTO   0xFFFFFFFFUL   /* mode 省略 */
#define OTA_MODE_RESUME 0U             /* 显式续传 */
#define OTA_MODE_NEW    1U             /* 显式新传 */

static void ota_begin(uint32_t slot, uint32_t len, uint32_t fcrc, uint32_t ver,
                      uint32_t mode)
{
	w25q_meta_t m;

	if (len == 0) {
		bl_puts("ER:len is 0\n");
		return;
	}
	if (len > APP_MAX_SIZE) {
		printf("ER:len > %u\n", (unsigned)APP_MAX_SIZE);
		return;
	}

	meta_load(&m);

	/* 指纹一致 = 同一个文件 */
	if (m.xfer.slot == slot && m.xfer.size == len && m.xfer.crc == fcrc &&
	    m.xfer.written > 0U && m.xfer.written <= len) {

		if (mode == OTA_MODE_NEW) {
			/* 显式新传：不管有没有断点都重收。
			 * 落到下面的"全新传输"分支去 */
			printf("NEW\n");
		} else {
			/* ---- 续传：沿用断点时的 chunk（对端在 RESUME 行里收到它） ---- */
			s_xfer_slot = slot;
			s_xfer_base = W25Q_SLOT_ADDR(slot);
			app_len     = len;
			crc32       = fcrc;
			s_xfer_ver  = m.xfer.version;   /* 版本以断点记录为准（可能是续传老文件） */
			s_chunk     = clamp_chunk(m.xfer.chunk);
			flash_write_offset = m.xfer.written;
			s_last_ckpt = flash_write_offset & XFER_CKPT_MASK;

			if (flash_write_offset == len) {
				now_status = bl_checkdata;  /* 收完了还没校验 —— 直接去校验 */
			} else {
				now_status = bl_recdata;
			}
			printf("RESUME %u %u\n", (unsigned)flash_write_offset, (unsigned)s_chunk);
			return;
		}
	} else if (mode == OTA_MODE_RESUME) {
		/* 显式续传但指纹对不上（或压根没有断点）：
		 * 不能自作主张擦槽重来 —— 那会把上位机已经传好的东西扔掉。
		 * 回错误，让上位机改用 mode=1 重新传。 */
		bl_puts("ER:no resume\n");
		return;
	}

	/* ---- 全新传输：先把断点（written=0）写进元数据再擦槽 ----
	 * 顺序不能反：擦到一半掉电，元数据里至少有"谁在传什么"的记录 */
	m.xfer.slot    = slot;
	m.xfer.size    = len;
	m.xfer.crc     = fcrc;
	m.xfer.version = ver;
	m.xfer.written = 0;
	m.xfer.chunk   = s_chunk;
	if (!meta_save(&m)) {
		bl_puts("ER:meta\n");
		return;
	}

	s_xfer_slot = slot;
	s_xfer_base = W25Q_SLOT_ADDR(slot);
	app_len     = len;
	crc32       = fcrc;
	s_xfer_ver  = ver;
	flash_write_offset = 0;
	s_last_ckpt = 0;

	now_status = bl_run;
}

/* 把当前进度记进元数据（断点）。
 * 只在"元数据里的记录还是这次传输"时更新 —— 防止状态错乱时
 * 把别的记录覆盖掉。失败静默：进度停留在上一个断点，
 * 代价只是重连后多传一段，不值得为它打断数据流 */
static void xfer_ckpt(void)
{
	w25q_meta_t m;

	meta_load(&m);
	if (m.xfer.slot == s_xfer_slot && m.xfer.size == app_len && m.xfer.crc == crc32) {
		m.xfer.written = flash_write_offset;
		m.xfer.chunk   = s_chunk;
		(void)meta_save(&m);
	}
}

/* ---- P 读分区表 ---- */
static void cmd_part(void)
{
	bl_puts("OK\n");
	/* 格式：PART=<名字>,<起始地址>,<大小>,<权限> */
	printf("PART=bootloader,0x%08X,%u,RO\n", (unsigned)FLASH_BASE, (unsigned)bl_size);
	printf("PART=app,0x%08X,%u,RW\n",        (unsigned)app_bl_addr, (unsigned)APP_MAX_SIZE);
	bl_puts("END\n");
}

/* ---- E 擦除 ---- */
static void cmd_erase(const char *p)
{
	uint32_t addr, len;
	int      r;

	if (!parse_addr_len(p, &addr, &len)) {
		bl_puts("ER:need addr len\n");
		return;
	}
	if (!range_ok(addr, len)) {
		bl_puts("ER:range\n");
		return;
	}

	HAL_FLASH_Unlock();
	r = erase_page(addr, addr + len);
	HAL_FLASH_Lock();

	if (r == 0) {
		bl_puts("OK\n");
	} else {
		bl_puts("ER:erase\n");
	}
}

/* ---- 内部 Flash 顺序写：半字编程 + 奇数补 0xFF ---- */
static void flash_wr_reset(uint32_t addr)
{
	s_wr_addr    = addr;
	s_pend_valid = 0;
}

static int flash_wr_chunk(const uint8_t *data, uint32_t len)
{
	uint32_t i = 0;

	HAL_FLASH_Unlock();

	/* 上一包落单的那个字节，跟本包第一个字节凑成一个半字 */
	if (s_pend_valid) {
		uint16_t hw = (uint16_t)s_pend_byte | ((uint16_t)data[0] << 8);
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, s_wr_addr, hw) != HAL_OK) {
			HAL_FLASH_Lock();
			return -1;
		}
		s_wr_addr += 2;
		s_pend_valid = 0;
		i = 1;
	}

	/* 剩下的两两配对写 */
	for (; i + 1 < len; i += 2) {
		uint16_t hw = (uint16_t)data[i] | ((uint16_t)data[i + 1] << 8);
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, s_wr_addr, hw) != HAL_OK) {
			HAL_FLASH_Lock();
			return -1;
		}
		s_wr_addr += 2;
	}

	/* 还剩一个孤零零的字节，存起来等下一包 */
	if (i < len) {
		s_pend_byte  = data[i];
		s_pend_valid = 1;
	}

	HAL_FLASH_Lock();
	return 0;
}

/* 全部收完，把最后那个落单字节补 0xFF 写掉 */
static int flash_wr_finish(void)
{
	HAL_StatusTypeDef r;

	if (!s_pend_valid) {
		return 0;
	}

	HAL_FLASH_Unlock();
	r = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, s_wr_addr,
	                      (uint16_t)s_pend_byte | ((uint16_t)0xFF << 8));
	HAL_FLASH_Lock();

	s_pend_valid = 0;
	return (r == HAL_OK) ? 0 : -1;
}

/* ---- W 写入 ----
 * 时序：
 *   上位机  W <addr> <len> <chunk>\n
 *   MCU     擦除该范围 → OK\n          ← 上位机等到这个 OK 才能发数据
 *   上位机  <chunk 字节>
 *   MCU     OK\n
 *   ...直到 len 字节发完
 *
 * 【关于 ota flag：失败时**故意不还原**】
 * 时序是"先 ota_flag_begin() 置升级中 → 再 erase_page() 擦掉 app 区"，
 * 所以一旦走到擦除这一步，**旧固件就已经没了**。
 * 这时候失败，把标志还原成"可信"不会救回任何东西，
 * 只会让 bootloader 跳进一个写了一半的固件 —— 跑飞、串口没输出、
 * 看起来像死机（HardFault）。所以失败时把标志**留在"升级中"**，
 * 让 bootloader 拒绝跳转，老老实实等你重刷。
 *
 * 【那会不会被锁死？不会】
 *   - 复位后 10 秒窗口照常打开，随时可以重新刷
 *   - 成功的 W（走到最后 ota_flag_commit）会把标志置回"可信"
 *   - 万一标志因为别的原因卡住，用 F 命令手工置回"可信"
 */
static void cmd_write(const char *p)
{
	uint32_t addr, len, got = 0, chunk;
	int      r;

	if (!parse_addr_len(p, &addr, &len)) {
		bl_puts("ER:need addr len\n");
		return;
	}
	/* 第三个字段（可选）是每包字节数 */
	if (!bl_parse_u32(&p, &chunk)) {
		chunk = 0;
	}
	chunk = clamp_chunk(chunk);

	if (!range_ok(addr, len)) {
		bl_puts("ER:range\n");
		return;
	}

	/* 先把标志置成"升级中"：从这一刻起掉电，app 都算不可信，
	 * 上电后 bootloader 会拒绝跳转等你重刷，而不是跳进残缺固件 */
	if (!ota_flag_begin()) {
		bl_puts("ER:flag\n");
		return;
	}

	/* W 直接覆盖内部 flash —— 原来那份固件没了，
	 * active_slot 指向的槽记录不再对应运行中的固件，作废。
	 * 不作废的话 ? 会把槽里的版本当成正在跑的版本，误导对端 */
	{
		w25q_meta_t m;

		meta_load(&m);
		if (m.active_slot <= 1U) {
			m.active_slot = 0xFFU;
			(void)meta_save(&m);
		}
	}

	HAL_FLASH_Unlock();
	r = erase_page(addr, addr + len);
	HAL_FLASH_Lock();
	if (r != 0) {
		/* 擦到一半失败 —— app 区可能已经是半空的，标志必须留在"升级中" */
		bl_puts("ER:erase\n");
		return;
	}

	/* 重新打开 IT 接收，准备收数据 */
	init_bootloader();
	flash_wr_reset(addr);

	bl_puts("OK\n");                     /* 擦好了，可以发数据了 */

	/* 【契约】上位机是"发一包 → 等一个 OK"，所以这里必须
	 * "收够一包（chunk 字节）才回一个 OK"。
	 * 老实现是"一个空闲事件回一个 OK"，一包被串口切成几段就会多回几个 OK，
	 * 上位机提前跑到下一包，后面全部错位。 */
	while (got < len) {
		uint32_t want = len - got;
		uint32_t n;

		if (want > chunk) {
			want = chunk;
		}

		n = bl_read_exact(io_buf, want, RX_TIMEOUT_MS);
		if (n != want) {
			/* 唯一能定位"数据为什么没到"的线索，必须留。
			 * 标志留在"升级中"：app 区已经被擦掉一部分了 */
			printf("ER:short %u/%u\n", (unsigned)n, (unsigned)want);
			return;
		}
		if (flash_wr_chunk(io_buf, n) != 0) {
			bl_puts("ER:flash\n");
			return;
		}
		got += n;
		bl_puts("OK\n");
	}

	if (flash_wr_finish() != 0) {
		bl_puts("ER:flash\n");
		return;
	}

	/* 全部写完，标志置成"可信"。
	 * 注意这里**没有**做回读校验（那是 V 命令的事），所以这一步只是
	 * "MCU 认为自己写完了"，不代表内容一定对。上位机发完会跟一次 V。 */
	if (!ota_flag_commit()) {
		bl_puts("ER:flag\n");
	}
}

/* ---- R 读出 ----
 * 时序：
 *   上位机  R <addr> <len>\n
 *   MCU     OK\n
 *   MCU     <len 字节裸数据>
 */
static void cmd_read(const char *p)
{
	uint32_t addr, len;

	if (!parse_addr_len(p, &addr, &len)) {
		bl_puts("ER:need addr len\n");
		return;
	}
	if (!range_ok(addr, len)) {
		bl_puts("ER:range\n");
		return;
	}

	bl_puts("OK\n");                     /* 告诉上位机：准备收裸数据 */

	while (len > 0) {
		uint32_t n = (len > uart_rec_len) ? uart_rec_len : len;

		/* 内部 Flash 是内存映射的，读它就像读内存 */
		memcpy(io_buf, (const void *)(uintptr_t)addr, n);
		bl_write(io_buf, n);           /* 原样吐字节，内部走 DMA */

		addr += n;
		len  -= n;
	}
}

/* ---- V 校验 ---- */
static void cmd_verify(const char *p)
{
	uint32_t addr, len, want;
	uint32_t crc = 0xFFFFFFFFU;

	if (!parse_addr_len(p, &addr, &len)) {
		bl_puts("ER:need addr len\n");
		return;
	}
	if (!bl_parse_u32(&p, &want)) {
		bl_puts("ER:need crc32\n");
		return;
	}
	if (!range_ok(addr, len)) {
		bl_puts("ER:range\n");
		return;
	}

	while (len > 0) {
		uint32_t n = (len > uart_rec_len) ? uart_rec_len : len;

		memcpy(io_buf, (const void *)(uintptr_t)addr, n);
		crc = crc32_update(crc, io_buf, n);

		addr += n;
		len  -= n;
	}
	crc ^= 0xFFFFFFFFU;

	if (crc == want) {
		bl_puts("OK\n");
	} else {
		/* 把算出来的值带回去，方便立刻判断是"传错"还是"算错" */
		printf("ER:crc =0x%08X\n", crc);
	}
}

/* ---- F 强制信任当前 app 并跳转 ----
 *
 * 【为什么需要这个命令】
 * ota flag 一旦停在"升级中"（0x00000000），bootloader 就会拒绝跳转。
 * 正常的恢复路径是重新刷一次固件（成功的 W / O 会把标志置回"可信"），
 * 但有两种情况刷不了、或者不想刷：
 *
 *   1. **app 是用 ST-Link 烧的**。ST-Link 只写内部 Flash，完全不碰
 *      W25Q 里的标志，所以上次失败留下的"升级中"会一直挂着 ——
 *      表现是"我明明烧进去了，怎么就是不跑"。这是最容易踩的一个坑。
 *   2. 固件其实完好，只是上次升级流程在**擦除之前**就中断了
 *      （比如 ota_flag_begin 之后掉电），内部 Flash 一个字节都没动。
 *
 * 这时用 F 把标志置回"可信"，然后跳转。
 *
 * 【风险要说清楚】
 * F 绕过了"app 是否完整"这层保护。如果 app 真的只写了一半，
 * 跳过去会跑飞（HardFault），表现是串口没输出、板子像死了一样。
 * **但不会变砖**：按一下复位，10 秒刷机窗口照常打开，重新刷一次即可。
 * 所以这个命令是"我知道我在干什么"的逃生口，不是日常操作。
 */
static void cmd_force(void)
{
	if (!ota_flag_commit()) {
		bl_puts("ER:flag\n");       /* W25Q 写不进去，没法置位 */
		return;
	}
	bl_puts("OK\n");
	now_status = bl_jump;
}


/* ================================================================
 * 四、命令分发
 * ================================================================ */

/* 兼容旧格式：start:<十进制长度>:<十六进制CRC32>
 * 【坑】CRC32 那段没有 0x 前缀，用通用的 bl_parse_u32 会被当十进制
 *       在遇到 'A' 时截断，所以这里单独按十六进制解析 */
static void parse_legacy_start(const char *p)
{
	const char *q = bl_strstr(p, "start:");
	uint32_t    len = 0;
	uint32_t    crc = 0;

	if (q == NULL) {
		bl_puts("ER:bad start\n");
		return;
	}
	q += 6;

	while (*q >= '0' && *q <= '9') {
		len = len * 10 + (uint32_t)(*q - '0');
		q++;
	}
	if (*q == ':') {
		q++;
	}
	for (;; q++) {
		char    c = *q;
		uint8_t d;

		if (c >= '0' && c <= '9')      { d = (uint8_t)(c - '0'); }
		else if (c >= 'a' && c <= 'f') { d = (uint8_t)(c - 'a' + 10); }
		else if (c >= 'A' && c <= 'F') { d = (uint8_t)(c - 'A' + 10); }
		else { break; }
		crc = (crc << 4) | d;
	}

	app_len = len;
	crc32   = crc;

	if (app_len == 0) {
		bl_puts("ER:len is 0\n");
		return;
	}
	if (app_len > APP_MAX_SIZE) {
		printf("ER:len > %u\n", (unsigned)APP_MAX_SIZE);
		return;
	}

	/* 老格式没有槽/版本的概念：槽选"非激活槽"，版本记 0.0.0。
	 * 统一走 ota_begin，让它拿到和 O 命令一样的断点记录/擦除流程 */
	{
		w25q_meta_t m;
		uint32_t slot;

		meta_load(&m);
		slot = (m.active_slot == 0U) ? 1U : 0U;
		s_chunk = clamp_chunk(0);
		ota_begin(slot, len, crc, 0U, OTA_MODE_AUTO);
	}
}

/* ---- 命令分发 ----
 * 返回 1 = 该跳 app 了（收到无法识别的命令）
 * 返回 0 = 继续留在命令行（真命令，窗口重置）
 * 返回 -1 = 这一行被忽略：留在命令行，但窗口【不】重置
 *          （不重置的东西：8266 的问候行、纯空白行、线路噪声字节。
 *            只要窗口能被它们续上，8266 每隔几秒说句话
 *            10 秒刷机窗口就永远关不上，APP 永远进不去 —— 实际踩过）
 *
 * 【只有"真的执行了一条命令"才配重置窗口】
 *   窗口重置的含义是"确认有人在刷机"，判据必须是认识的命令本身。
 *   噪声/空白/问候行都证明不了这一点。
 *
 * 【为什么未知命令要跳 app】上电后那 10 秒（BOOT_WAIT_MS）是"刷机窗口"。
 * 收到的命令全都认识 → 说明你确实在刷机，留下来陪你；
 * 收到一个完全不认识的命令 → 说明串口对面不是刷机工具（可能只是
 * 某些上位机在探测、或者线接错了），没必要一直占着，去跑 app。
 *
 * 【注意：参数写错不算"未知命令"】比如 W 的地址越界、E 没给参数，
 * 都只回一句 ER 然后继续等 —— 那种情况下你明显是在用 bootloader，
 * 只是敲错了，把你踢走反而添乱。 */
/* 这一行是不是 8266 的问候行（NET/TL/TH/THA/EVT/OTA? 开头）。
 * 问候行：不跳转、不重置刷机窗口 —— 否则窗口永远关不上。 */
static uint8_t line_is_chatter(const char *p)
{
	static const char *const chatter[] = {
		"NET ", "TL ", "TH ", "THA", "EVT ", "OTA?"
	};
	for (unsigned k = 0; k < sizeof(chatter) / sizeof(chatter[0]); k++) {
		if (bl_strstr(p, chatter[k]) == p) {
			return 1;
		}
	}
	return 0;
}

static int dispatch(void)
{
	const char *p = (const char *)cmd_buf;
	char c;

	/* 跳过前导空白和换行 */
	while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') {
		p++;
	}

	/* 空行/纯空白行直接忽略。上位机握手时会反复发换行，不该每次都回一句 ER；
	 * 更关键的是它不能续窗口 —— 回 -1（忽略但不重置） */
	if (*p == '\0') {
		return -1;
	}

	/* ---- 8266 的问候行不是命令 ----
	 * ESP8266 联网后会往这条线上发 NET 1 x.x.x.x / NET 0（联网状态）、
	 * TH ...（阈值回执）、TL/EVT 等行。bootloader 的 10 秒窗口恰好
	 * 撞上它们时，以前被当"未知命令"直接踢去跳 APP —— 上电 512ms 就跳，
	 * 刷机窗口名存实亡（实际踩过：8266 一联网 STM32 就"秒跳"）。 */
	if (line_is_chatter(p)) {
		return -1;      /* 静默忽略：不跳转，但也【不重置窗口】（见 app_bl_init） */
	}

	/* ---- 线路噪声不算命令 ----
	 * 打开/关闭串口、插拔 USB 时，CH340 的 TX 线会有一次跳变，
	 * MCU 的 UART 可能把它当成一个帧错字节（DR 里通常是 0x00 或 0xFF）。
	 * 这类不可打印字节如果按"未知命令"处理，一上电就会被噪声踢去跑 app，
	 * 刷机窗口等于白给。所以只有可打印 ASCII 才当"有人在敲命令"。
	 * 但它同样不是命令 —— 回 -1：忽略，且【不】续窗口。
	 * （PA10 被 USB-TTL 的 TX 和 8266 侧 CP2102 的 TX 双重驱动，
	 *   这条线上持续冒噪声字节是常态；让它续窗口的后果就是 APP 永远进不去） */
	if ((uint8_t)*p < 0x20U || (uint8_t)*p > 0x7EU) {
		return -1;
	}

	/* 命令大小写不敏感。
	 * 【为什么要加】"未知命令就跳 app"之后，敲错一个字母的代价变成了
	 * "被踢出 bootloader"，得重新上电才能回来。把大小写归一化能挡掉
	 * 最常见的一类手误（敲了 'w' 而不是 'W'） */
	c = *p;
	if (c >= 'a' && c <= 'z') {
		c = (char)(c - 'a' + 'A');
	}

	/* 旧格式优先判断，它和单字母命令不会冲突 */
	if (bl_strstr(p, "start:")) {
		parse_legacy_start(p);
		return 0;
	}

	switch (c) {
	case '?':
		cmd_info();
		break;

	case 'P':
		cmd_part();
		break;

	case 'E':
		cmd_erase(p + 1);
		break;

	case 'W':
		cmd_write(p + 1);
		break;

	case 'R':
		cmd_read(p + 1);
		break;

	case 'V':
		cmd_verify(p + 1);
		break;

	case 'O': {
		/* O <slot> <len> <crc32> <ver> [chunk] [mode] —— 下载到 A/B 槽（支持续传）
		 * mode: 省略=自动（指纹一致就续，否则新传）
		 *       0   =显式续传（指纹不一致回 ER:no resume，不擦槽）
		 *       ≠0  =显式新传（忽略断点，擦掉重收）
		 * 旧格式 O <len> <crc32> [chunk] 兼容：第一个字段 > 1 就当长度，
		 * 槽自动选"非激活槽"（不动正在运行的版本） */
		const char *q = p + 1;
		uint32_t first = 0, l = 0, c2 = 0, k = 0, v = 0, slot;
		uint32_t mode = OTA_MODE_AUTO;
		w25q_meta_t m;

		if (!bl_parse_u32(&q, &first)) {
			bl_puts("ER:need args\n");
			break;
		}
		if (first <= 1U) {
			slot = first;
			if (!bl_parse_u32(&q, &l) || !bl_parse_u32(&q, &c2)) {
				bl_puts("ER:need len crc32\n");
				break;
			}
			/* 版本号可省（= 0.0.0）。【坑】parse_version 失败时也可能
			 * 已经把数字读走了（比如把 chunk 当成了版本号），
			 * 失败必须把指针退回去，否则 chunk 解析就空了 */
			const char *save = q;
			if (!parse_version(&q, &v)) {
				v = 0;
				q = save;
			}
			if (!bl_parse_u32(&q, &k)) {
				k = 0;
			}
			/* 第 6 个字段 = mode（可省） */
			if (!bl_parse_u32(&q, &mode)) {
				mode = OTA_MODE_AUTO;
			}
		} else {
			l = first;
			if (!bl_parse_u32(&q, &c2)) {
				bl_puts("ER:need len crc32\n");
				break;
			}
			if (!bl_parse_u32(&q, &k)) {
				k = 0;
			}
			meta_load(&m);
			slot = (m.active_slot == 0U) ? 1U : 0U;
		}
		s_chunk = clamp_chunk(k);

		ota_begin(slot, l, c2, v, mode);
		if (now_status == bl_run) {
			buzzer_beep(60, 0, 1);      /* 新传输开始：短响一声 */
		}
		break;
	}

	case 'C': {
		/* C <slot> [len] —— 从 W25Q 槽里读回固件并算 CRC32
		 *
		 * 【为什么要有这条】上位机下载完固件、还没激活时，
		 * 要能确认"W25Q 里那份到底是不是我要的那个文件"——
		 * 不能只看元数据里记的 crc（那是我们自己写的）。
		 * 真正从 flash 里读回来重算一遍，才算数。
		 *
		 * 回复格式固定：
		 *   OK
		 *   CRC=0x%08X,SIZE=%u,MATCH=<0|1>   (MATCH=1 表示与元数据记录的 CRC 一致)
		 *   END
		 * len 省略 = 用元数据里记的该槽大小；给 0 或超界也退回元数据大小。 */
		const char *q = p + 1;
		uint32_t slot = 0, len = 0, off = 0, left;
		uint32_t c = 0xFFFFFFFFU;
		w25q_meta_t m;

		if (!bl_parse_u32(&q, &slot) || slot > 1U) {
			bl_puts("ER:need slot\n");
			break;
		}
		meta_load(&m);
		len = m.slot[slot].size;
		if (bl_parse_u32(&q, &len) && (len == 0U || len > APP_MAX_SIZE)) {
			len = m.slot[slot].size;
		}
		if (len == 0U) {
			printf("CRC=0x00000000,SIZE=0,MATCH=0\n");
			break;
		}
		left = len;
		while (left > 0) {
			uint32_t n = (left > uart_rec_len) ? uart_rec_len : left;
			if (!w25qd_read(W25Q_SLOT_ADDR(slot) + off, io_buf, n)) {
				bl_puts("ER:w25q read\n");
				break;
			}
			c = crc32_update(c, io_buf, n);
			off += n;
			left -= n;
		}
		c ^= 0xFFFFFFFFU;
		printf("CRC=0x%08X,SIZE=%u,MATCH=%u\n",
		       c, (unsigned)len,
		       (unsigned)(m.slot[slot].valid && m.slot[slot].crc == c ? 1U : 0U));
		break;
	}

	case 'S':
		cmd_slots();
		break;

	case 'Q':
		cmd_prog();
		break;

	case 'H':
		cmd_menu();
		break;

	case 'A':
		cmd_activate(p + 1);
		break;

	case 'G':
		now_status = bl_jump;
		break;

	case 'B':
		/* 回滚到 W25Q 里的旧 app 备份。
		 * 备份存在的意义就在这一条命令：新固件烧进去跑不起来时，
		 * 不用拆板子拿 ST-Link 重刷，一条 B 就能把上一版救回来。
		 * 真正的动作放在状态机里做（bl_restore），这里只应答，
		 * 免得主机以为板子死了。 */
		if (backup_state() != 1) {
			bl_puts("ER:no backup\n");
			break;
		}
		bl_puts("OK\n");
		now_status = bl_restore;
		break;

	case 'F':
		/* 强制信任当前 app 并跳转。给"ST-Link 烧完但标志卡在升级中"
		 * 这类情况留的逃生口，详见 cmd_force 上面的说明 */
		cmd_force();
		break;

	default:
		/* 先把话说清楚再走，免得用户以为是板子自己跑掉了 */
		bl_puts("ER:unknown cmd, boot app\n");
		return 1;
	}

	return 0;
}


/* ================================================================
 * 五、状态机各步
 * ================================================================ */

/* 上电后是否已经尝试过跳转。
 *
 * 【为什么要这个标志】跳不过去（空片 / 固件坏了 / ota flag 不可信）时，
 * 如果不记住，主循环每转一圈就会再试一次、再打一行 ER，
 * 日志会被瞬间刷爆，反而看不到真正有用的那第一行。
 * 所以"跳转"这件事整个上电周期只做一次；失败就老实留在命令行等刷机。 */
static uint8_t s_jump_tried = 0;

/* OTA 常驻的兜底超时时刻（见 OTA_HOLD_TIMEOUT_MS） */
static uint32_t s_hold_deadline = 0;

/* ================================================================
 * OTA 常驻模式（网络升级的关键）
 *
 * 【为什么需要】升级链路是"网页/后端 → ESP8266 联网拉固件 → 串口推送"。
 * 复位后 ESP8266 要连 WiFi、查版本、下载固件，10 秒窗口根本不够用。
 *
 * 【约定】APP 收到升级命令（S3 的 RF_CMD_OTA_START）后，先往
 * BKP->DR1 写入 OTA_BKP_MAGIC 再 NVIC_SystemReset()。
 * bootloader 上电发现魔数 → 这次进入常驻等命令模式，不自动跳 app；
 * 等到第一条合法命令（或未知命令）后魔数作废，行为恢复正常。
 * 常驻模式下 120 秒没有真命令会自动跳 app，避免 8266 掉线时永远卡死。
 * 备份寄存器在 VBAT 域，掉电重启（拔电池）也会保留 —— 所以
 * 必须在进命令行前就清掉，否则一块写坏的板子会永远卡在 bootloader。
 *
 * 【校验值】0x4F54 = 'OT'。APP（STM32_MQTT_OTA_Node 工程）里用同一个值。
 * ================================================================ */
#define OTA_BKP_MAGIC   0x4F54U

/* OTA 常驻不是无限等：拿魔数进来后，每来一条真命令就把窗口续到 120 秒；
 * 120 秒内没有真命令就跳回 app。原来是无条件无限等，一旦 8266 在下载途中
 * 复位/断网，STM32 就永远停在 bootloader，整机看起来"死机"。
 * 120 秒足够 8266 连 WiFi + 下载 + 推完一包固件。 */
#define OTA_HOLD_TIMEOUT_MS  120000U

/* OTA 常驻标志（1 = 停在命令行等刷机，不跳 app）。
 * 原来是 app_bl_init 里的函数级 static；现在 wait_command 的按键监听也要改它，
 * 提到文件作用域。 */
static uint8_t ota_hold = 0;

/* 按键被按住 → 进 OTA 常驻。
 * 【为什么要响两声】按下这一刻屏幕很可能还停在 APP 的画面（BL 平时不画屏），
 * 只有声音能让用户立刻确认"这一下被认到了"，而不是继续等 10 秒去猜。 */
static void key_takeover(void)
{
	s_key_watch     = 0U;
	s_key_low_ms    = 0U;
	ota_hold        = 1U;
	s_hold_deadline = HAL_GetTick() + OTA_HOLD_TIMEOUT_MS;
	bl_puts("BL v2.1 OTA hold\n");      /* 8266 看到这行就会去后台查更新 */
	BUZZER_ON();  HAL_Delay(80U);
	BUZZER_OFF(); HAL_Delay(80U);
	BUZZER_ON();  HAL_Delay(80U);
	BUZZER_OFF();
	ota_ui_begin();                     /* 屏幕切"升级中"，从此归 BL 管 */
}

/* ================================================================
 * 上电自检：内部 flash 里的 APP 到底能不能跳
 *
 * 三样一起看，缺一不可：
 *   +0  栈顶（MSP 初值）必须落在 RAM 里
 *   +4  复位向量必须落在 app 区内且 bit0=1（Thumb）
 *   +0x110 版本标签魔数必须对（擦除后是 0xFFFFFFFF，写坏了也过不了）
 *
 * 配合 app_bl_flash 的**倒序写入**（最后才写向量页），
 * 升级途中断电时这里必然判 APP 无效 → 自动从 W25Q 找回能跑的固件。
 * ================================================================ */
static uint8_t app_image_sane(void)
{
	uint32_t sp  = *(volatile uint32_t *)(uintptr_t)app_bl_addr;
	uint32_t rv  = *(volatile uint32_t *)(uintptr_t)(app_bl_addr + 4U);
	uint32_t tag = *(volatile uint32_t *)(uintptr_t)APP_TAG_ADDR;

	if ((sp < ram_start) || (sp > ram_end) || ((sp & 7U) != 0U)) {
		return 0;
	}
	if ((rv < app_bl_addr) || (rv > flash_end) || ((rv & 1U) == 0U)) {
		return 0;
	}
	if (tag != APP_TAG_MAGIC) {
		return 0;
	}
	return 1;
}

/* 槽里那份固件从 W25Q 读回来重算 CRC，和元数据记的一致才算可用。
 * 只信元数据的 valid 位不够：槽可能被后来的擦写踩坏。 */
static uint8_t slot_crc_ok(const w25q_meta_t *m, uint32_t slot)
{
	uint32_t off  = 0;
	uint32_t left = m->slot[slot].size;
	uint32_t c    = 0xFFFFFFFFU;

	if (left == 0U || left > APP_MAX_SIZE) {
		return 0;
	}
	while (left > 0U) {
		uint32_t n = (left > uart_rec_len) ? uart_rec_len : left;

		if (!w25qd_read(W25Q_SLOT_ADDR(slot) + off, io_buf, n)) {
			return 0;
		}
		c = crc32_update(c, io_buf, n);
		off  += n;
		left -= n;
	}
	c ^= 0xFFFFFFFFU;
	return (c == m->slot[slot].crc) ? 1U : 0U;
}

/* 1. 等命令 */
void app_bl_init(void)
{
	static uint8_t banner_done  = 0;
	static uint8_t recover_done = 0;

	/* 【接收必须一直开着】
	 * 命令阶段靠中断 + 环形缓冲收字节（见 wait_command 的说明）。
	 * 一进来就打开：这样哪怕屏幕/串口在长时间打印，对端发来的字节
	 * 也不会因为"没人读 DR"而溢出丢失。 */
	init_bootloader();

	if (!banner_done) {
		banner_done = 1;
		buzzer_init();

		/* ---- 查 OTA 常驻魔数（读一次，用完即清） ---- */
		PWR->CR |= PWR_CR_DBP;      /* F1 的 BKP 默认写保护，必须解除才能清 DR1 */
		__HAL_RCC_BKP_CLK_ENABLE();
		if (BKP->DR1 == OTA_BKP_MAGIC) {
			BKP->DR1  = 0U;             /* 必须清：见上面的"掉电重启也保留" */
			ota_hold  = 1;
			s_hold_deadline = HAL_GetTick() + OTA_HOLD_TIMEOUT_MS;
		}

		/* ---- 上电按住空闲按键（PA12）→ 进 OTA 常驻 ----
		 * 【为什么 bootloader 要自己读这个脚】
		 *   OTA 的软件入口在 APP 里（收到触发串写 BKP 再复位）。APP 一旦被刷成
		 *   "只会点灯、连串口都不解析"的极简版本，软件入口就彻底没了，只能 ST-Link。
		 *   PA12 在 APP 里只用来"响一声"（buttom.c），等于闲着的脚：
		 *   按键按下接地、内部上拉 —— 按住就进常驻等刷机，
		 *   完全不看 APP 一眼，APP 刷成什么样都能救回来。
		 * 【这里只把脚配好、把监听打开】
		 *   真正的"按住"判断在 wait_command 的等待循环里（见 s_key_watch 的说明）：
		 *   整个 10 秒窗口期间按一下都算数，而不是只在开机那一瞬间采一次。
		 * 【与 APP 不冲突】按键在 APP 里也是上拉输入，这里配完的脚
		 *   跳过去以后由 APP 的 MX_GPIO_Init 重新配置。 */
		{
			GPIO_InitTypeDef gb = {0};

			__HAL_RCC_GPIOA_CLK_ENABLE();
			gb.Pin  = GPIO_PIN_12;
			gb.Mode = GPIO_MODE_INPUT;
			gb.Pull = GPIO_PULLUP;
			HAL_GPIO_Init(GPIOA, &gb);

			s_key_watch = 1U;
		}

		if (ota_hold) {
			bl_puts("BL v2.1 OTA hold\n");  /* 告诉 ESP8266：我常驻等你 */
			/* 屏幕立刻切到"升级中"：此时 APP 已经停了，屏幕归 bootloader 管 */
			ota_ui_begin();
		} else {
			/* 一行，把"还剩多久"告诉用户 —— 10 秒的窗口不写出来，
			 * 用串口助手手动敲命令的人会不知道该抢时间还是该等。
			 * 秒数从 BOOT_WAIT_MS 直接算出来，改了宏这里自动跟着变 */
			printf("BL v2.1  wait %us\n", (unsigned)(BOOT_WAIT_MS / 1000U));
		}
		/* 定位用：只往 USART2 写一行，用来确认 USB-TTL 接的是哪个口 */
		(void)HAL_UART_Transmit(&huart2, (const uint8_t *)"U2ONLY\r\n", 8U, 200U);
	}

	/* ---- 上电自恢复（2026-11-03）----
	 * 升级途中断电：APP 可能被擦了 / 写了一半。上电先判 APP 能不能跳；
	 * 不能跳就自动从 W25Q 找回能跑的固件，而不是永远停在命令行等人刷：
	 *   1) 备份区（升级前原样备份的旧 APP，CRC 已验证）优先；
	 *   2) 没有备份，再看 A/B 槽里有没有 CRC 校验通过的固件。
	 * 恢复成功由 bl_restore / bl_flash 自己跳 app；都找不到才留在命令行。 */
	if (!recover_done) {
		recover_done = 1;
		if (!app_image_sane()) {
			bl_puts("BL:app invalid, auto recover\n");
			if (backup_state() == 1) {
				now_status = bl_restore;
				return;
			}
			{
				w25q_meta_t m;
				uint32_t i;

				meta_load(&m);
				for (i = 0; i < 2U; i++) {
					if (slot_bootable(&m, i) && slot_crc_ok(&m, i)) {
						s_xfer_slot = i;
						s_xfer_base = W25Q_SLOT_ADDR(i);
						s_xfer_ver  = m.slot[i].version;
						app_len     = m.slot[i].size;
						crc32       = m.slot[i].crc;
						bl_puts("BL:recover from slot\n");
						now_status = bl_flash;
						return;
					}
				}
			}
			bl_puts("ER:no recover image\n");
		}
	}

	/* ---- 上电自检（2026-11-02）----
	 *
	 * 【为什么需要它】
	 *   这块板子上 USB-TTL 的 TX 接 PA10，而板上并联着 8266 的 CP2102
	 *   （它的 TX 也驱动 PA10）—— 两个推挽输出顶在一起，
	 *   所以**人工往 bootloader 发命令这条路物理上不通**。
	 *   但新增的查询协议（S / Q / C / RUNVER）必须能验证，
	 *   于是把「本来要等上位机来问的内容」在上电时自己打一遍。
	 *
	 * 【打印什么】
	 *   只打**查询类**的输出（? / S / Q），不动任何状态：
	 *   不擦 flash、不写 W25Q、不改元数据。
	 *   上位机（8266）连上来时看到这些行会当成噪声跳过，
	 *   不影响协议 —— 它认的是 "OK ... END" 和 "READY/RESUME" 这几个关键字。
	 *
	 * 【怎么关掉】
	 *   把下面的 BL_SELFTEST 改成 0 即可（量产固件建议关，省得刷屏）。
	 *   ⚠ 2026-11-02：自检已验证过（RUNVER=1.0.0 / SLOT BOOTABLE / PROG=none
	 *   全部正确），现在**默认关掉** —— 开机横幅干净一点，
	 *   也不给上位机多灌无关的行。需要复验时把它改回 1 重编即可。 */
#define BL_SELFTEST 0
	if (BL_SELFTEST) {
		bl_puts("--- BL SELFTEST ---\n");
		cmd_info();
		cmd_slots();
		cmd_prog();
		bl_puts("--- SELFTEST END ---\n");
	}

	/* ---- 上电后的刷机窗口（沉默 10 秒）----
	 * 窗口语义【2026-11-02 重写】：
	 *   · 真命令（?/P/S/O/...）          → 留下，窗口重置
	 *   · 8266 的问候行（NET/TL/TH/...） → 静默忽略，窗口【继续倒数】
	 *   · 纯空白行 / 线路噪声字节         → 静默忽略，窗口【继续倒数】
	 *     （以前这三类都算"真命令"，谁都能给窗口续命：问候行、噪声行
	 *       一茬接一茬，窗口永远关不上，APP 永远进不去，实际踩过）
	 *   · 10 秒内没有任何"真命令"        → 跳 app
	 *   · OTA 常驻模式                    → 无限等（分段 10s 轮询） */

	/* ---- 上一轮的跳转已经被闸拦下过：不要再重开一个 10 秒计时窗口 ----
	 * app_bl_jump() 失败时把状态退回 bl_init，于是每次回到这里都会重新算一个
	 * deadline，外观上就是"刷机窗口永远关不上、APP 永远不跑、VTOR 一直是 0"，
	 * 和"超时失效"完全分不开（要的就是这一行把它区分开）。
	 * 跳转被拒说明内部 flash 里那份 app 确实不可用（空片 / 栈顶或复位向量不对 /
	 * 只写了一半），板子该待的地方就是命令行 —— 和 OTA 常驻同一种行为：
	 * 无限等真命令，问候行和噪声不续命。刷好或按复位就恢复正常。 */
	if (s_jump_tried) {
		ota_hold = 1;
		s_hold_deadline = HAL_GetTick() + OTA_HOLD_TIMEOUT_MS;
		bl_puts("BL:hold jump-refused\n");
	}

	bl_puts("BL:wait\n");
	{
		uint32_t deadline = HAL_GetTick() + BOOT_WAIT_MS;

		for (;;) {
			uint32_t now    = HAL_GetTick();
			uint32_t remain;

			if (ota_hold) {
				/* 常驻模式：最多再等 120 秒；每来一条真命令就续期 */
				if ((int32_t)(s_hold_deadline - now) <= 0) {
					bl_puts("BL:hold timeout\n");
					remain = 0U;
				} else {
					uint32_t left = s_hold_deadline - now;
					remain = (left > 10000U) ? 10000U : left;
				}
			} else {
				remain = (deadline > now) ? (deadline - now) : 0U;
			}

			if (remain == 0U) {
				break;                      /* 沉默期满 / 常驻超时，去跑 app */
			}

			if (!wait_command(remain)) {
				/* 常驻模式只是"分段 10 秒轮询"，没到 120 秒超时不能走 */
				if (ota_hold && (int32_t)(s_hold_deadline - HAL_GetTick()) > 0) {
					continue;
				}
				break;                      /* 沉默期满 */
			}

			int r = dispatch();
			if (r < 0) {
				bl_puts("BL:chatter\n");
				continue;                   /* 问候行：不重置窗口，接着倒数 */
			}
			if (r > 0) {
				s_jump_tried = 1;
				now_status   = bl_jump;
				return;
			}
			/* 真命令：留在命令行，窗口重置 */
			bl_puts("BL:cmd\n");
			deadline = HAL_GetTick() + BOOT_WAIT_MS;
			if (ota_hold) {
				s_hold_deadline = HAL_GetTick() + OTA_HOLD_TIMEOUT_MS;
			}

			/* ---- 【必须立刻把控制权交回状态机】----
			 * O / A / B / F / G 这些命令不只是"回一句就完事"：它们在 dispatch()
			 * 里直接把 now_status 推进到 bl_run / bl_flash / bl_jump。
			 * 这个 for(;;) 只该负责"等命令"，一旦状态变了就要退出，
			 * 否则状态机永远轮不到 —— 实际踩过：8266 发完 O 等 READY，
			 * 这里继续等命令，app_bl_run() 一步都没跑（READY 永远发不出去），
			 * 8266 等到超时，STM32 停在 bootloader 无限等，整机看着像死机。
			 * （2026-11-03 定位） */
			if (now_status != bl_init) {
				return;
			}
		}
	}

	/* ---- 窗口内没人理（或只剩问候行和噪声），或 OTA 常驻 120 秒无命令 ----
	 * 认为只是正常上电，去跑 app。失败原因 bl_jump_app 自己会打印。 */
	if (!s_jump_tried) {
		s_jump_tried = 1;
		now_status   = bl_jump;
	}
}

/* 2. 擦目标槽，回 READY */
void app_bl_run(void)
{
	uint32_t sectors;

	uart_rec_full_len  = 0;
	flash_write_offset = 0;

	/* 【关键】先把串口中断接收打开，再去擦 W25Q。
	 * 擦除要几百毫秒到几秒，这段时间接收必须是开着的，
	 * 否则对端发来的字节会因为没人取而溢出丢失 */
	init_bootloader();

	sectors = (app_len + 4095UL) / 4096UL;
	if (!w25qd_erase(s_xfer_base, sectors)) {
		bl_puts("ER:w25q erase\n");
		now_status = bl_init;       /* 失败要回退状态，否则主循环会死转 */
		return;
	}

	/* 告诉对端"我准备好了，可以发数据了"。
	 * 对端必须等到这句 READY 才能开始发 bin，
	 * 否则擦除期间发来的字节全丢 */
	bl_puts("READY\n");

	now_status = bl_recdata;
}

/* 3. 收数据：攒够一包 → 写槽 → 回 OK；每 4KB 记一次断点 + 进度音
 *
 * 【超时放哪了】以前这里是"自己数 uart_rec_full_len 有没有变"，现在
 * 交给 bootloader_process 内部的 bl_read_exact：连续 RX_TIMEOUT_MS
 * 都没攒够一整个包就算失败。这样"分帧"和"超时"是同一处逻辑，不会打架。
 *
 * 【失败了为什么不回 bl_init 就完了】状态回 init 等对端下一次 O；
 * 断点已经落在元数据里（xfer_ckpt），对端重连后重发 O 就能续传，
 * 这就是"断线重连"在 MCU 侧的全部支持 */
void app_bl_recbuf(void)
{
	if (flash_write_offset >= app_len) {
		now_status = bl_checkdata;
		return;
	}

	if (bootloader_process(s_xfer_base, app_len, s_chunk, RX_TIMEOUT_MS) != 0) {
		now_status = bl_init;       /* 失败原因 bootloader_process 里已经打过 */
		return;
	}

	/* 进度界面：下载占 0~80%。每收一包（512B≈1%）就推一次，
	 * I2C 只补新长出来的那几列，所以能跟上而不拖慢串口。
	 * 放在处理之后：入口那次永远慢一包，看起来就是一跳一跳的 */
	if (app_len > 0U) {
		ota_ui_set_pct((uint8_t)((flash_write_offset * 80UL) / app_len));
	}

	if (flash_write_offset >= app_len) {
		now_status = bl_checkdata;
		return;
	}

	/* 断点：第一包就记一次（覆盖"刚连上就断"），
	 * 之后每跨过一个 4KB 边界记一次 */
	if (flash_write_offset == XFER_CKPT_FIRST ||
	    (flash_write_offset & XFER_CKPT_MASK) != s_last_ckpt) {
		s_last_ckpt = flash_write_offset & XFER_CKPT_MASK;
		xfer_ckpt();
		buzzer_beep(15, 0, 1);      /* 滴 —— 进度音 */
	}
}

/* 4. 校验：从槽读回来算 CRC32，跟命令里给的比；通过就给槽登记"有效" */
void app_bl_check(void)
{
	uint32_t remain = app_len;
	uint32_t offset = 0;
	uint32_t crc    = 0xFFFFFFFFU;
	w25q_meta_t m;

	ota_ui_ramp(82U);               /* 下载完了，平滑走进校验/备份/搬运段 */

	while (remain > 0) {
		uint32_t n = (remain > uart_rec_len) ? uart_rec_len : remain;

		if (!w25qd_read(s_xfer_base + offset, io_buf, n)) {
			bl_puts("ER:w25q read\n");
			now_status = bl_init;
			return;
		}
		crc = crc32_update(crc, io_buf, n);

		offset += n;
		remain -= n;
	}
	crc ^= 0xFFFFFFFFU;

	if (crc != crc32) {
		/* 对端靠 'fail'/'error' 两个词判失败，这行必须带上 fail */
		buzzer_beep(60, 60, 3);     /* 急促三短响：失败了 */
		printf("ER:crc fail calc=0x%08X recv=0x%08X\n", crc, crc32);
		/*
		 * 【必须清掉断点记录 —— 否则会死循环】
		 *   槽里的数据已经证明是坏的（CRC 对不上）。
		 *   可断点记录还写着「这个槽里传了 len 字节、指纹是这份文件」，
		 *   对端重连后重发同样的 O → 指纹匹配 → 又从断点续传 →
		 *   续的是**同一份坏数据** → 又是 CRC fail → 无限重试。
		 *   所以这里把断点清零：下次 O 就是全新传输，擦掉重收。
		 *   （内部 Flash 一个字节都没动，设备照常能跑旧固件）
		 */
		{
			w25q_meta_t mc;

			meta_load(&mc);
			if (mc.xfer.slot == s_xfer_slot && mc.xfer.crc == crc32) {
				mc.xfer.written = 0;
				mc.xfer.slot    = 0xFFFFFFFFUL;
				(void)meta_save(&mc);
			}
		}
		now_status = bl_init;       /* 回开头重来，此时内部 Flash 还没被碰过 */
		return;
	}

	/* 【先登记槽，再动内部 flash】把"这槽里是什么、版本多少、CRC 多少"
	 * 写进元数据并清掉断点。这样就算后面备份/搬运失败，
	 * 槽里的固件也已经在案，随时可以单独用 A 命令激活，不用重传 */
	meta_load(&m);
	m.slot[s_xfer_slot].valid   = 1;
	m.slot[s_xfer_slot].version = s_xfer_ver;
	m.slot[s_xfer_slot].size    = app_len;
	m.slot[s_xfer_slot].crc     = crc32;
	/* 往"激活槽"里传了新内容 —— 内部 flash 里跑的还是旧版，
	 * 槽记录已经不对应运行中的固件了，active 记录作废 */
	if (m.active_slot == s_xfer_slot) {
		m.active_slot = 0xFFU;
	}
	m.xfer.slot    = 0xFFFFFFFFUL;  /* 断点使命完成 */
	m.xfer.written = 0;
	if (!meta_save(&m)) {
		bl_puts("ER:meta\n");
		now_status = bl_init;
		return;
	}

	/* 对端就等这一行，不能改 */
	printf("verify ok\n");
	ota_ui_ramp(85U);
	now_status = bl_backup;         /* 校验通过 → 先备份旧 app，再去动内部 Flash */
}

/* 5. 备份：把当前 app 搬到 W25Q 备份区 + 回读校验
 *
 * 【为什么要在擦内部 flash 之前做】
 * "擦掉内部 app 再写新固件"中间存在一个不可逆的窗口：
 * 写一半掉电，app 区就是半空的，设备直接变砖，只能拿 ST-Link 重刷。
 * 先把当前 app 原样备份到 W25Q，之后一条 B 命令就能回滚。
 *
 * 【备份不完整就中止升级】
 * 备份写完必须读回来重算一遍 CRC，和刚写进去的比。
 * 不一致说明 W25Q 写入有问题（掉电、坏块、供电不稳）——
 * 这时如果继续擦内部 flash，设备就是砖。
 * 所以直接中止：内部 flash 一个字节都没动，设备照常跑旧固件。
 */
void app_bl_backup(void)
{
	uint32_t len = APP_MAX_SIZE;     /* 备份整个 app 区（含尾部 0xFF） */
	uint32_t sectors;
	uint32_t off = 0, remain = len;
	uint32_t crc = 0xFFFFFFFFU;
	backup_hdr_t h;

	printf("backing up app...\n");
	ota_ui_ramp(86U);

	/* 1. 算当前 app 的 CRC（顺带确认内部 flash 读得到） */
	while (remain > 0) {
		uint32_t n = (remain > uart_rec_len) ? uart_rec_len : remain;
		memcpy(io_buf, (const void *)(uintptr_t)(app_bl_addr + off), n);
		crc = crc32_update(crc, io_buf, n);
		off += n; remain -= n;
	}
	crc ^= 0xFFFFFFFFU;

	/* 2. 擦备份区（备份区独立于新固件暂存区，互不覆盖） */
	sectors = (W25Q_BACKUP_HDR_LEN + len + 4095UL) / 4096UL;
	if (!w25qd_erase(W25Q_BACKUP_ADDR, sectors)) {
		bl_puts("ER:backup erase\n");
		now_status = bl_init;
		return;
	}

	/* 3. 写备份头（magic / len / crc），回滚前先看它 */
	h.magic    = W25Q_BACKUP_MAGIC;
	h.len      = len;
	h.crc      = crc;
	h.reserved = 0;
	if (!w25qd_write(W25Q_BACKUP_ADDR, (uint8_t *)&h, sizeof(h))) {
		bl_puts("ER:backup hdr\n");
		now_status = bl_init;
		return;
	}

	/* 4. 写内容 */
	off = 0; remain = len;
	while (remain > 0) {
		uint32_t n = (remain > uart_rec_len) ? uart_rec_len : remain;
		memcpy(io_buf, (const void *)(uintptr_t)(app_bl_addr + off), n);
		if (!w25qd_write(W25Q_BACKUP_ADDR + W25Q_BACKUP_HDR_LEN + off, io_buf, n)) {
			bl_puts("ER:backup write\n");
			now_status = bl_init;
			return;
		}
		off += n; remain -= n;
	}

	ota_ui_ramp(88U);               /* 备份内容写完，开始回读校验 */

	/* 5. 读回校验 —— 这一步不能省 */
	if (backup_crc_check(&h) != 0) {
		bl_puts("ER:backup crc\n");
		now_status = bl_init;      /* 内部 flash 还没动，设备照常跑 */
		return;
	}

	printf("backup ok\n");
	ota_ui_ramp(90U);
	now_status = bl_flash;         /* 备份好了，才允许动内部 flash */
}

/* 6. 回滚：把 W25Q 备份区里的旧 app 写回内部 flash
 *
 * 【什么时候用】新固件烧进去了却跑不起来（板子不启动、后台无响应），
 * 这时用 B 命令把上一版固件救回来，不用拆板子。
 */
void app_bl_restore(void)
{
	backup_hdr_t h;
	uint32_t off = 0, left;
	uint32_t crc = 0xFFFFFFFFU;

	if (backup_state() != 1) {
		bl_puts("ER:no backup\n");
		now_status = bl_init;
		return;
	}
	if (!w25qd_read(W25Q_BACKUP_ADDR, (uint8_t *)&h, sizeof(h))) {
		bl_puts("ER:backup read\n");
		now_status = bl_init;
		return;
	}
	left = h.len;

	printf("restoring backup...\n");

	/* 回滚过程中掉电，app 区就不可信 —— 标志停在"升级中"，
	 * 上电后 bootloader 会拒绝跳转，等你重刷，而不是跳进半截固件 */
	if (!ota_flag_begin()) {
		bl_puts("ER:flag\n");
		now_status = bl_init;
		return;
	}

	HAL_FLASH_Unlock();
	if (erase_page(app_bl_addr, app_bl_addr + h.len) != 0) {
		HAL_FLASH_Lock();
		bl_puts("ER:flash erase\n");
		now_status = bl_init;
		return;
	}
	HAL_FLASH_Lock();

	flash_wr_reset(app_bl_addr);
	while (left > 0) {
		uint32_t n = (left > uart_rec_len) ? uart_rec_len : left;

		if (!w25qd_read(W25Q_BACKUP_ADDR + W25Q_BACKUP_HDR_LEN + off, io_buf, n)) {
			bl_puts("ER:w25q read\n");
			now_status = bl_init;
			return;
		}
		if (flash_wr_chunk(io_buf, n) != 0) {
			bl_puts("ER:flash program\n");
			now_status = bl_init;
			return;
		}
		off += n; left -= n;
	}
	if (flash_wr_finish() != 0) {
		bl_puts("ER:flash program\n");
		now_status = bl_init;
		return;
	}

	/* 回读校验：写回去的内容必须和备份头里记的 CRC 一致 */
	{
		uint32_t roff  = 0;
		uint32_t rleft = h.len;
		crc = 0xFFFFFFFFU;
		while (rleft > 0) {
			uint32_t n = (rleft > uart_rec_len) ? uart_rec_len : rleft;
			memcpy(io_buf, (const void *)(uintptr_t)(app_bl_addr + roff), n);
			crc = crc32_update(crc, io_buf, n);
			roff += n; rleft -= n;
		}
		crc ^= 0xFFFFFFFFU;
	}

	if (crc != h.crc) {
		printf("ER:verify fail calc=0x%08X recv=0x%08X\n", crc, h.crc);
		now_status = bl_init;
		return;
	}

	if (!ota_flag_commit()) {
		bl_puts("ER:flag\n");
	}

	printf("restore ok\n");
	now_status = bl_jump;
}

/* 升级失败统一出口：备份可用就自动回滚，别把设备留在半截固件上。
 * 先打具体的 ER 行 —— ESP8266 的 settleAndCheck 认这个前缀，会如实上报失败。 */
static void flash_fail_rollback(const char *why)
{
	bl_puts(why);
	if (backup_state() == 1) {
		bl_puts("ER:rollback\n");
		now_status = bl_restore;
	} else {
		now_status = bl_init;
	}
}

/* 5. 搬运：擦内部 Flash，把 W25Q 里的固件写进去 */
void app_bl_flash(void)
{
	uint32_t remain = app_len;
	uint32_t crc;
	uint32_t off;
	uint32_t left;
	uint8_t  beep_pct = 0xFFU;      /* 上次"滴"过的百分比，搬运阶段也保持按节奏响 */

	printf("erasing internal flash...\n");
	ota_ui_ramp(92U);               /* 搬运阶段：92~99，写完校验通过才 100 */
	buzzer_beep(40, 0, 1);          /* 长一点：开始写内部 Flash（擦除要 1 秒左右） */

	/* 1. 先把标志置成"升级中"。
	 * 必须放在擦内部 Flash 之前 —— 从这里开始掉电，app 就不可信了，
	 * 上电后 bootloader 会拒绝跳转，等你重刷，而不是跳进残缺固件 */
	if (!ota_flag_begin()) {
		bl_puts("ER:flag\n");
		now_status = bl_init;
		return;
	}

	/* 2. 擦内部 app 区（到这里才擦，前面全程没碰过） */
	HAL_FLASH_Unlock();
	if (erase_page(app_bl_addr, app_bl_addr + app_len) != 0) {
		HAL_FLASH_Lock();
		/* 标志留在"升级中"：app 区可能已经是半空的，
		 * 放行跳转等于跳进废墟。备份可用就自动回滚旧固件。 */
		flash_fail_rollback("ER:flash erase\n");
		return;
	}
	HAL_FLASH_Lock();

	/* 3. 从 W25Q 目标槽**倒序**搬进内部 Flash：从最后一包往前写。
	 * 【为什么倒序】升级途中断电时，向量页（第一个 1KB 页）还没被写，
	 * 上电 app_image_sane() 必然判 APP 无效 → 自动从备份恢复。
	 * 正序写的话，掉电会留下"向量正确、代码半截"的固件，
	 * 跳进去直接跑飞，而且从外部根本判不出来。 */
	while (remain > 0U) {
		uint32_t n = (remain > uart_rec_len) ? uart_rec_len : remain;

		remain -= n;
		if (!w25qd_read(s_xfer_base + remain, io_buf, n)) {
			flash_fail_rollback("ER:w25q read\n");
			return;
		}
		flash_wr_reset(app_bl_addr + remain);
		if (flash_wr_chunk(io_buf, n) != 0) {
			flash_fail_rollback("ER:flash program\n");
			return;
		}
		if (flash_wr_finish() != 0) {
			flash_fail_rollback("ER:flash program\n");
			return;
		}
		/* 倒序写：remain 是"还没写的字节数"，剩下的越少进度越满。
		 * 每推进 1% 短响一声 —— 下载阶段（每 4KB）才有"滴"，
		 * 搬运阶段要是完全没声音，会让人以为升级卡住了 */
		if (app_len > 0U) {
			uint8_t pct = (uint8_t)(92U + ((app_len - remain) * 8UL) / app_len);

			if (pct != beep_pct) {
				beep_pct = pct;
				buzzer_beep(10, 0, 1);
			}
			ota_ui_set_pct(pct);
		}
	}

	/* 4. 搬完再从内部 Flash 读回来算一遍 CRC32，确认真的搬对了 */
	crc  = 0xFFFFFFFFU;
	off  = 0;
	left = app_len;

	while (left > 0) {
		uint32_t n = (left > uart_rec_len) ? uart_rec_len : left;

		memcpy(io_buf, (const void *)(uintptr_t)(app_bl_addr + off), n);
		crc = crc32_update(crc, io_buf, n);

		off  += n;
		left -= n;
	}
	crc ^= 0xFFFFFFFFU;

	if (crc != crc32) {
		/* 搬运后回读校验失败 —— Flash 里内容不可信，绝不能跳。
		 * 备份区可用就自动回滚到升级前那份，别让设备停在半截固件上 */
		buzzer_beep(60, 60, 3);
		printf("ER:verify fail calc=0x%08X recv=0x%08X\n", crc, crc32);
		if (backup_state() == 1) {
			bl_puts("ER:rollback\n");
			now_status = bl_restore;
		} else {
			now_status = bl_init;
		}
		return;
	}

	/* 5. 全部写完且回读 CRC 通过，才把标志置成"可信"。
	 * 这一步之后掉电也没事了 */
	if (!ota_flag_commit()) {
		bl_puts("ER:flag\n");
		/* 标志写失败不阻断流程：固件本身已经校验过了，
		 * 只是下次上电会停在 bootloader，需要重刷一次 */
	}

	/* 6. 记账：这份固件来自哪个槽 —— 版本查询/切换都靠它 */
	{
		w25q_meta_t m;

		meta_load(&m);
		m.active_slot = (uint8_t)s_xfer_slot;
		(void)meta_save(&m);
	}

	buzzer_beep(150, 120, 2);       /* 两声长响：升级成功，准备跳 app */
	ota_ui_set_pct(100U);
	now_status = bl_jump;
}

/* 6. 跳转 */
void app_bl_jump(void)
{
	/* 【先还原屏幕再跳】进度界面把 SSD1306 的窗口缩成了进度条那一块，
	 * u8g2 又从不重设窗口；不还原的话 APP 只会写进那几列，
	 * 主页右侧会一直残留着进度条。普通上电没有这步，只有 OTA 之后才会犯 */
	ota_ui_restore();

	/* 跳转失败时 bl_jump_app 自己会把原因打出来（坏栈顶 / 向量不对 /
	 * flag 不可信），这里不重复打 */
	(void)bl_jump_app();
	now_status = bl_init;
}

/* 7. 状态机 */
void app_bl_status(void)
{
	switch (now_status) {
	case bl_init:      app_bl_init();     break;
	case bl_run:       app_bl_run();      break;
	case bl_recdata:   app_bl_recbuf();   break;
	case bl_checkdata: app_bl_check();    break;
	case bl_backup:    app_bl_backup();   break;
	case bl_restore:   app_bl_restore();  break;
	case bl_flash:     app_bl_flash();    break;
	case bl_jump:      app_bl_jump();     break;
	default:           now_status = bl_init; break;
	}
}
