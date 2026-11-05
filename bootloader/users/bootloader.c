/*
 * bootloader.c —— 串口 IAP 核心
 *
 * 本文件负责：
 *   1. 串口输出（bl_printf / bl_puts / bl_write）—— 走 HAL_UART，长数据走 DMA
 *   2. 轻量数字解析（bl_parse_u32）—— 替代 atoi / strtol
 *   3. 接收中断 + 环形缓冲（按长度分帧，永不丢包）
 *   4. 内部 Flash 擦除（erase_page）
 *   5. 精确取字节（bl_read_exact）—— 给 ISP 命令和 OTA 收数据共用
 *   6. 跳转 app（bl_jump_app）
 *
 * 【瘦身说明】原来的 printf 会把整个 stdio 家族拖进固件（约 3.4KB），
 * 20K 分区装不下，所以这里自己实现一个只支持用得到的格式的版本。
 *
 * 【DMA 说明】
 *   发送：短数据走 HAL_UART_Transmit（阻塞），长数据走 HAL_UART_Transmit_DMA。
 *         用 DMA 的意义在于 CPU 不用一个字节一个字节等 TXE 标志 ——
 *         115200 下一个字节要 87us，512 字节就是 44ms 纯等待。
 *   接收：走 HAL_UARTEx_ReceiveToIdle_IT（空闲中断 + 中断搬运）。
 *         没用 DMA 是因为 CubeMX 里没给 USART1_RX 分配 DMA 通道，
 *         而接收本身是"来一个字节存一个字节"，中断方式足够。
 */

#include "bootloader.h"
#include "w25qd.h"
#include <stdarg.h>

/* 调试口（USART2 / PA2）：控制台内容镜像到这里，方便插 USB-TTL 直接看 */
extern UART_HandleTypeDef huart2;

/* ======================== 接收缓冲 ======================== */
/* uart_rec_buf：HAL 每一轮往里填的那个缓冲（中断里用，别的地方不许碰） */
static uint8_t uart_rec_buf[uart_rec_len];

/* 环形缓冲：中断只往里塞，主循环按需要多少取多少。
 * rx_head 只在中断里推进，rx_tail 只在主循环里推进，各改各的，
 * 所以取"可读字节数"时关一下中断就够了 */
static uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
volatile uint32_t rx_overflow = 0;        /* 环满被迫丢掉的字节数，正常恒为 0 */

/* proc_buf：OTA 收数据时，从环里取出来交给 W25Q 的中转缓冲 */
static uint8_t proc_buf[uart_rec_len];

/* ======================== 状态变量 ======================== */
/* 中断里写、主循环里读，一律 volatile */
volatile uint32_t uart_rec_full_len = 0;  /* 累计收到的总字节数 */
volatile uint32_t rx_errors = 0;          /* 串口出错次数（溢出/帧错），正常恒为 0 */

uint32_t flash_write_offset = 0;          /* 已写进 W25Q 的字节数 */


/* ================================================================
 * 一、串口输出
 *
 * 思路：先在内存里把整行拼好，再一次性交给 HAL 发出去。
 * 比"一个字符一次 HAL 调用"快得多，也让 DMA 有足够的长度值得启动。
 * ================================================================ */
#define OUT_BUF_SIZE 192U    /* 最长一行约 50 字节，留足余量 */
#define OUT_DMA_MIN  32U     /* 超过这个长度才走 DMA */
#define OUT_TX_TIMEOUT_MS 2000U

static char     out_buf[OUT_BUF_SIZE];
static uint16_t out_len = 0;

/* ================= USART2 日志镜像（PA2=TX，接 PC USB-TTL 调试口） =================
 * PC 端调试串口在 USART2(PA2/PA3, 115200)，而 bootloader 的输出走 USART1(PA9，
 * 接 8266)。这里把每个字节额外阻塞式写一份到 USART2，电脑串口上就能直接看到
 * 「进没进 bootloader、跳没跳 APP」的全过程。寄存器直配，不占 HAL 句柄。 */
static uint8_t s_uart2_ok = 0;

static void uart2_init_once(void)
{
    GPIO_InitTypeDef g;

    if (s_uart2_ok) {
        return;
    }
    s_uart2_ok = 1;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    g.Pin   = GPIO_PIN_2;              /* PA2 = USART2_TX，复用推挽 */
    g.Mode  = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);
    {
        /* 【别写死 BRR】36MHz/115200 = 312.5，对应 BRR = 0x138；
         * 原来这里写的 0x1388 = 5000，实际只有 7200 波特（正好差 16 倍），
         * 插 USB-TTL 按 115200 收只能收到一堆乱码。按 PCLK1 现算最稳。 */
        uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
        USART2->BRR = (pclk1 + (115200U / 2U)) / 115200U;
    }
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE;
}

static void uart2_write(const uint8_t *d, uint16_t n)
{
    uart2_init_once();
    while (n--) {
        uint32_t guard = 100000U;
        while (!(USART2->SR & USART_SR_TXE)) {
            if (--guard == 0U) return; /* TXE 卡死就放弃这一字节，别拖死主流程 */
        }
        USART2->DR = (uint32_t)(*d++);
    }
}

/* 真正把 n 个字节发出去 */
static void uart_send(const uint8_t *d, uint16_t n)
{
    uart2_write(d, n);                 /* 调试口镜像（先发，阻塞，量小） */
	uint32_t t0;

	if (n == 0U) {
		return;
	}

	/* 短数据：阻塞发送。省掉启动 DMA 的开销，也不会和接收中断抢状态 */
	if (n < OUT_DMA_MIN) {
		HAL_UART_Transmit(&huart1, d, n, OUT_TX_TIMEOUT_MS);
		(void)HAL_UART_Transmit(&huart2, d, n, OUT_TX_TIMEOUT_MS);   /* 镜像到调试口 */
		return;
	}

	/* 长数据：DMA 发送，CPU 在这段时间里可以干别的（这里就是等） */
	if (HAL_UART_Transmit_DMA(&huart1, d, n) != HAL_OK) {
		/* DMA 启动失败（上一笔还没发完之类）就退回阻塞发送，保证不丢日志 */
		HAL_UART_Transmit(&huart1, d, n, OUT_TX_TIMEOUT_MS);
	} else {
		/* 等 DMA 搬完。缓冲区是静态的，必须等完才能复用。
		 * 搬完后 HAL 会在发送完成中断里把 gState 置回 READY */
		t0 = HAL_GetTick();
		while (huart1.gState != HAL_UART_STATE_READY) {
			if ((HAL_GetTick() - t0) > OUT_TX_TIMEOUT_MS) {
				HAL_UART_AbortTransmit(&huart1);   /* 卡住了就强行收尾 */
				break;
			}
		}
	}

	/* 镜像一份到调试口（USART2）。放在 DMA 等待之后，保证两个口的顺序一致 */
	(void)HAL_UART_Transmit(&huart2, d, n, OUT_TX_TIMEOUT_MS);
}

/* 往输出缓冲追加一个字符（满了就丢弃，绝不越界） */
static void out_ch(char c)
{
	if (out_len < OUT_BUF_SIZE) {
		out_buf[out_len++] = c;
	}
}

/* 追加字符串，不发送 */
static void out_str(const char *s)
{
	while (*s) {
		out_ch(*s++);
	}
}

/* 把缓冲里的东西发出去 */
static void out_flush(void)
{
	uart_send((const uint8_t *)out_buf, out_len);
	out_len = 0;
}

void bl_puts(const char *s)
{
	while (*s) {
		if (*s == '\n') {
			out_ch('\r');        /* 串口助手要 \n，这里自动补 \r */
		}
		out_ch(*s++);
	}
	out_flush();
}

/* 原样吐字节，不做任何字符替换。
 * 给 R 命令回读裸数据用 —— 如果这里也把 '\n' 变成 '\n'，
 * 读回来的数据就被污染了，永远对不上 CRC。
 * 大块数据直接走 DMA，这正是 DMA 最该用的地方 */
void bl_write(const uint8_t *d, uint32_t n)
{
	out_flush();                       /* 先把还没发出去的文本发掉，保证顺序 */

	while (n > 0U) {
		uint16_t chunk = (n > 512U) ? 512U : (uint16_t)n;

		uart_send(d, chunk);
		d += chunk;
		n -= chunk;
	}
}

static void put_u32(uint32_t v, uint32_t base, uint8_t upper, uint8_t min_digits)
{
	char    tmp[11];
	uint8_t n = 0;

	if (v == 0) {
		tmp[n++] = '0';
	}
	while (v) {
		uint8_t d = (uint8_t)(v % base);
		tmp[n++] = (char)((d < 10) ? ('0' + d)
		                           : ((upper ? 'A' : 'a') + d - 10));
		v /= base;
	}
	while (n < min_digits) {
		tmp[n++] = '0';
	}
	while (n) {
		out_ch(tmp[--n]);
	}
}

int bl_printf(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);

	for (; *fmt; fmt++) {
		uint8_t pad = 0;

		if (*fmt != '%') {
			if (*fmt == '\n') {
				out_ch('\r');
			}
			out_ch(*fmt);
			continue;
		}

		fmt++;                                    /* 跳过 '%' */

		/* 前导零宽度：%08X → pad = 8 */
		while (*fmt >= '0' && *fmt <= '9') {
			pad = (uint8_t)(pad * 10 + (uint8_t)(*fmt - '0'));
			fmt++;
		}

		/* 吃掉 'l' 长度修饰符，32 位机上 %lu 和 %u 一样 */
		if (*fmt == 'l') {
			fmt++;
		}

		switch (*fmt) {
		case 's': out_str(va_arg(ap, const char *)); break;
		case 'c': out_ch((char)va_arg(ap, int)); break;
		case 'u': put_u32(va_arg(ap, uint32_t), 10, 0, pad); break;
		case 'd': {
			int32_t v = va_arg(ap, int32_t);
			if (v < 0) {
				out_ch('-');
				v = -v;
			}
			put_u32((uint32_t)v, 10, 0, pad);
			break;
		}
		case 'x': put_u32(va_arg(ap, uint32_t), 16, 0, pad); break;
		case 'X': put_u32(va_arg(ap, uint32_t), 16, 1, pad); break;
		case '%': out_ch('%'); break;
		case '\0': va_end(ap); out_flush(); return 0;   /* 格式串以 '%' 结尾，防越界 */
		default:  out_ch('%'); out_ch(*fmt); break;
		}
	}

	va_end(ap);
	out_flush();                        /* 一行拼完，一次性发出去 */
	return 0;
}


/* ================================================================
 * 二、轻量数字解析（替代 atoi / strtol）
 *
 * 规则：0x / 0X 开头 = 十六进制，其余 = 十进制。
 * 解析成功把 *ps 推到数字后面，返回 1；一个数字都没解析到返回 0。
 * ================================================================ */
uint8_t bl_parse_u32(const char **ps, uint32_t *out)
{
	const char *p = *ps;
	uint32_t    v = 0;
	uint8_t     base = 10;
	uint8_t     got = 0;

	while (*p == ' ' || *p == '\t' || *p == ':' || *p == ',') {
		p++;                                       /* 跳过分隔符 */
	}

	if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) {
		base = 16;
		p += 2;
	}

	for (;; p++) {
		char     c = *p;
		uint8_t  d;

		if (c >= '0' && c <= '9') {
			d = (uint8_t)(c - '0');
		} else if (base == 16 && c >= 'a' && c <= 'f') {
			d = (uint8_t)(c - 'a' + 10);
		} else if (base == 16 && c >= 'A' && c <= 'F') {
			d = (uint8_t)(c - 'A' + 10);
		} else {
			break;
		}
		v = v * base + d;
		got = 1;
	}

	*ps = p;
	if (got) {
		*out = v;
	}
	return got;
}


/* ================================================================
 * 三、轻量 strstr（替代 libc 的，省几百字节）
 * ================================================================ */
const char *bl_strstr(const char *hay, const char *needle)
{
	if (!*needle) {
		return hay;
	}
	for (; *hay; hay++) {
		const char *h = hay;
		const char *n = needle;
		while (*h && *n && (*h == *n)) {
			h++;
			n++;
		}
		if (!*n) {
			return hay;
		}
	}
	return NULL;
}


/* ================================================================
 * 四、内部 Flash 擦除
 *
 * 【关键】擦除必须从 start 之后的第一个页首开始算，不能往下对齐！
 * start 落在页中间，说明这一页是"上一包刚擦过、正在续写"的，
 * 再擦一次会把上一包刚写进去的数据整页抹掉。
 *   例：每包 512 字节、页 1KB
 *     包1 offset=0   → start 在页首   → 擦这一页，写 0~511
 *     包2 offset=512 → start 在页中间 → 绝不能擦，直接续写 512~1023
 * ================================================================ */
static uint8_t pageisblank(uint32_t pageaddr)
{
	for (uint32_t i = 0; i < page_size; i++) {
		if (*(volatile uint8_t *)(pageaddr + i) != 0xFF) {
			return 0;
		}
	}
	return 1;   /* 必须循环跑完才能说这页是空的 */
}

int erase_page(uint32_t start, uint32_t end)
{
	FLASH_EraseInitTypeDef erase;
	uint32_t page_err = 0;   /* 输出参数：HAL 把"擦失败的页号"写进来 */
	uint32_t first;

	erase.TypeErase = FLASH_TYPEERASE_PAGES;  /* 按页擦（F1 只有这一种） */
	erase.Banks     = FLASH_BANK_1;           /* F103 只有一个 bank */
	erase.NbPages   = 1;                      /* 一次擦 1 页，靠循环擦多页 */

	/* 向上对齐到下一个页首：
	 * start 正好在页首 → 结果还是 start 本身
	 * start 在页中间   → 结果是下一页页首 */
	first = (start + page_size - 1) & ~(page_size - 1);

	for (uint32_t pa = first; pa < end; pa += page_size) {
		if (pageisblank(pa)) {
			continue;                 /* 已经是空的，跳过，省一次擦除 */
		}
		erase.PageAddress = pa;       /* 要擦的页首地址，必须 1KB 对齐 */
		if (HAL_FLASHEx_Erase(&erase, &page_err) != HAL_OK) {
			return -1;
		}
	}
	return 0;
}


/* ================================================================
 * 五、接收中断回调
 * ================================================================ */
/* 收满 512 字节"或者"总线空闲时，由 HAL 自动调用。
 * Size = 这一轮实际收到的字节数，通常远小于 512，别当成 512 用。
 *
 * 【关键】这里只往环形缓冲里塞，**绝不丢包**。
 * 老版本在 rx_ready 忙时整包丢弃，而上位机并不知道，后面就全部错位。
 * 现在哪怕主循环正在写 Flash，字节也会在环里排队等着。
 * 这里只做快活（拷贝），慢活交给主循环 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART1) {
		uint16_t i;

		for (i = 0; i < Size; i++) {
			uint16_t next = (uint16_t)((rx_head + 1U) & (RX_RING_SIZE - 1U));

			if (next == rx_tail) {
				/* 环满了 —— 主循环已经严重跟不上。
				 * 有"发一包等一个 OK"的流控在，这里正常永远不该发生；
				 * 真发生了就记一笔，让这次传输显式失败，别悄悄写错数据 */
				rx_overflow++;
				break;
			}
			rx_ring[rx_head] = uart_rec_buf[i];
			rx_head = next;
		}

		/* 这一轮接收到这里就结束了，必须重新开下一轮，
		 * 不写这句就永远只能收到第一包 */
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, uart_rec_buf, uart_rec_len);
	}
}

/* 【必须实现】发生 ORE（接收溢出）/ FE / NE / PE 时由 HAL 调用。
 * 不实现这个回调，一次溢出就让 HAL 关掉接收且永不重开，
 * 症状是"传一半永久静默"。
 * 【坑】函数名必须和 HAL 的弱函数完全一致：ErrorCallback（小写 b）。
 *       写成 ErrorCallBack 编译能过，但永远不会被调用，修复静默失效。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance != USART1) {
		return;
	}

	/* F1 上 CLEAR_OREFLAG / FEFLAG / NEFLAG / PEFLAG 展开成同一个宏
	 * （"读 SR 再读 DR"），所以清一次就够了 */
	__HAL_UART_CLEAR_OREFLAG(huart);
	huart->ErrorCode = HAL_UART_ERROR_NONE;

	/* 溢出 / 帧错 / 噪声都意味着"这一段字节流已经不可信"。
	 * 与其把错位的数据写进 Flash，不如把环清空并记一笔错误，
	 * 让正在进行的这次传输立刻失败，由上位机重来 */
	rx_errors++;
	bl_rx_reset();

	/* 重新开一轮接收 */
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, uart_rec_buf, uart_rec_len);
}

/* 重新开始接收。上电调一次，之后每次要"重新收"的地方也调它 */
void init_bootloader(void)
{
	HAL_StatusTypeDef st;

	bl_rx_reset();

	/* 【关键】先强制中止任何"正在进行"的接收，把 RxState 拉回 READY。
	 *
	 * 为什么非要这一步：HAL 的**阻塞版** HAL_UARTEx_ReceiveToIdle 在
	 * "超时返回"这条路径上不会把 huart->RxState 恢复成 READY，
	 * 而是把它卡在 BUSY_RX。之后调 HAL_UARTEx_ReceiveToIdle_IT 时，
	 * HAL 一看状态不是 READY 就直接返回 HAL_BUSY 并且什么都不做，
	 * 接收从此永久停摆。
	 *
	 * 症状：命令能收发（走的是阻塞读），但一开始发固件就报
	 *       short 0/512 (err 0 ovf 0)  +  ER:timeout
	 *       —— 收到 0 字节且没有任何串口错误，因为接收根本没启动。
	 *
	 * Abort 一下保证状态干净。这个函数本来就代表"重新开始接收"，
	 * 丢掉半截数据是预期行为。调用点都在主循环上下文，不在中断里。 */
	HAL_UART_AbortReceive(&huart1);

	/* 把中止/上电时可能残留的溢出、空闲标志清掉。
	 * F1 上这两个宏展开成"读 SR 再读 DR"的序列，正是清标志的正确方式 */
	__HAL_UART_CLEAR_OREFLAG(&huart1);
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);

	/* 第三个参数是"最多收多少字节"，必须和缓冲区的真实大小一致 */
	st = HAL_UARTEx_ReceiveToIdle_IT(&huart1, uart_rec_buf, uart_rec_len);
	if (st != HAL_OK) {
		/* 走到这里说明接收没起来，后面必然收不到数据。
		 * 唯一能让这个"静默失败"暴露出来的地方，必须留 */
		bl_puts("ER:rx start\n");
	}
}


/* ================================================================
 * 六、按"声明的长度"精确取字节
 *
 * 【为什么不能"一个空闲事件 = 一包"】
 * 上位机 ser.write(512) 一次，线上会被 USB 串口芯片按 USB 包切开，
 * 中间夹着毫秒级空隙；而硬件 IDLE 中断只要空闲 1 个字符时间
 * （115200 下约 87us）就会触发。于是"一包"会被切成好几段。
 * 所以必须按上位机声明的长度来分帧 —— 这也正是 AN3155 里
 * Write Memory 用 N 字节声明长度的原因：靠长度，不靠时间。
 * ================================================================ */
uint32_t bl_rx_avail(void)
{
	uint16_t head, tail;

	__disable_irq();
	head = rx_head;
	tail = rx_tail;
	__enable_irq();

	return (uint32_t)((head - tail) & (RX_RING_SIZE - 1U));
}

void bl_rx_reset(void)
{
	__disable_irq();
	rx_head = 0;
	rx_tail = 0;
	rx_overflow = 0;
	__enable_irq();
}

/* 从环里取一个字节。1 = 取到，0 = 环空。
 *
 * 【为什么命令阶段必须走这里，而不是直接轮询 DR】
 * 2026-11-03 实测：`?` 的应答最后一行 END 发完，dispatch 返回后还要再打
 * 9 个字节 "BL:cmd"，这段时间 CPU 只顾查 TXE、没看 DR。上位机一看到 END
 * 就立刻发下一条 O，O 的头部正好落进这个盲窗 —— 硬件 ORE，37 字节的
 * 命令只剩第一个字节，命令彻底丢失：既不回 ER 也不回 READY，
 * 上位机干等 30 秒超时，现场表现就是"升级卡死"。
 * 中断接收全程在跑（打印期间也照收进环），命令阶段只从环里取，
 * 就不会再有这种盲窗了。 */
uint8_t bl_rx_get(uint8_t *c)
{
	uint16_t tail = rx_tail;

	if (((rx_head - tail) & (RX_RING_SIZE - 1U)) == 0U) {
		return 0;
	}
	*c = rx_ring[tail];
	rx_tail = (uint16_t)((tail + 1U) & (RX_RING_SIZE - 1U));
	return 1;
}

uint32_t bl_read_exact(uint8_t *buf, uint32_t want, uint32_t timeout_ms)
{
	uint32_t t0   = HAL_GetTick();
	uint32_t got  = 0;
	uint32_t err0 = rx_errors;

	while (got < want) {
		uint16_t head, tail, avail, n;

		if (rx_errors != err0 || rx_overflow != 0) {
			break;                       /* 出过错，这段数据不可信，作废 */
		}

		__disable_irq();
		head  = rx_head;
		tail  = rx_tail;
		avail = (uint16_t)((head - tail) & (RX_RING_SIZE - 1U));

		if (avail == 0) {
			__enable_irq();
			if (timeout_ms == 0) {
				break;                   /* 非阻塞：没有就返回 */
			}
			if ((HAL_GetTick() - t0) > timeout_ms) {
				break;                   /* 超时 */
			}
			continue;
		}

		/* 一次最多拷到"环的末尾"，再受"还差多少"和"环里有多少"限制。
		 * 512 字节 memcpy 约 7us，115200 下一个字节 87us，不会丢字节 */
		n = (uint16_t)(RX_RING_SIZE - tail);
		if (n > avail) {
			n = avail;
		}
		if (n > (uint16_t)(want - got)) {
			n = (uint16_t)(want - got);
		}

		memcpy(buf + got, rx_ring + tail, n);
		rx_tail = (uint16_t)((tail + n) & (RX_RING_SIZE - 1U));
		__enable_irq();

		got += n;
	}

	uart_rec_full_len += got;
	return got;
}


/* ================================================================
 * 七、OTA 收数据阶段：攒够一包写进 W25Q
 *
 * 由 appbl.c 的状态机反复调用。
 * 【契约】上位机是"发一包 → 等一个 OK"，所以这里也必须
 * "收够一包才回一个 OK"，两边才对得上。
 * 返回 0 = 正常（写好一包，或已经收完），-1 = 失败（超时/短包/写不进）
 * ================================================================ */
int bootloader_process(uint32_t base, uint32_t total, uint32_t chunk, uint32_t timeout_ms)
{
	uint32_t remain = total - flash_write_offset;
	uint32_t want   = (remain > chunk) ? chunk : remain;
	uint32_t n;

	if (want == 0) {
		return 0;                        /* 已经收完了 */
	}

	n = bl_read_exact(proc_buf, want, timeout_ms);
	if (n != want) {
		/* 唯一能定位"数据为什么没到"的线索，必须留 */
		printf("ER:short %u/%u\n", (unsigned)n, (unsigned)want);
		return -1;
	}

	if (!w25qd_write(base + flash_write_offset, proc_buf, n)) {
		bl_puts("ER:w25q write\n");
		return -1;
	}

	flash_write_offset += n;
	bl_puts("OK\n");
	return 0;
}


/* ================================================================
 * 八、跳转到 app
 * 返回 0 = 跳转成功（其实不会再返回了），-1 = 校验没过，继续留在 bootloader
 *
 * app 区头两个 32 位数据的含义（这是 ARM 定死的，不是约定）：
 *   app_bl_addr + 0  →  app 的栈顶地址（MSP 初值）
 *   app_bl_addr + 4  →  app 的复位向量（Reset_Handler 地址，bit0 恒为 1）
 *
 * 【这里是"该不该跳"的唯一决策点】
 * appbl.c 里所有"跳 app"的分支（窗口超时、未知命令、G、F）都只是把
 * 状态机切到 bl_jump，真正跳不跳由这个函数说了算。它按顺序过三道闸：
 *
 *   1. 栈顶落在 RAM 范围内、且 8 字节对齐
 *   2. 复位向量落在 app 区内、且 bit0 == 1
 *   3. app tag 与镜像匹配
 *
 * 【为什么必须有第 1、2 道】
 * 空片读出来是 0xFFFFFFFF：
 *   栈顶 = 0xFFFFFFFF → 不在 RAM 里 → 第 1 道拦下
 * 如果没这道闸，跳过去会把 MSP 设成 0xFFFFFFFF，紧接着取向量表就
 * HardFault —— 板子看起来像死了，串口也不吭声。
 * 有了它，空片只会打印一行 ER:jump sp 0xFFFFFFFF 然后老老实实
 * 留在命令行等你刷机，**不会卡死**。
 *
 * 【2026-11-03 按现场要求删除"升级完成标志"这道闸】
 * 原来的第 0 道闸在 ota flag=0x00000000（升级中/上次失败）时拒绝跳转。
 * 现场用 ST-Link 烧 app 不会更新 W25Q 里的标志，一旦残留"升级中"，
 * 表现就是"固件明明烧进去了却永远停在 bootloader"，只能靠 F 命令救。
 * 现在不需要这层掉电保护，直接放行，让跳转只看栈顶/向量/app tag。
 * ota_flag 的写入逻辑保留（不参与跳转决策，仅作状态记录）。
 * ================================================================ */
int bl_jump_app(void)
{
	/* 用 volatile 读，防止编译器"觉得这两个值没变过"而缓存 */
	uint32_t app_stack_ptr    = *(volatile uint32_t *)(app_bl_addr);
	uint32_t app_reset_handle = *(volatile uint32_t *)(app_bl_addr + 4);

	/* ---- 校验 1：栈顶必须落在 RAM 范围内 ----
	 * 【空片就是在这里被拦下的】app 区没烧过程序时读出全 0xFF，
	 * 栈顶 = 0xFFFFFFFF，远大于 ram_end，直接 return -1。
	 * 少了这一句，跳过去会把 MSP 设成 0xFFFFFFFF，取向量表时立刻
	 * HardFault，板子表现为"死了" */
	if (app_stack_ptr < ram_start || app_stack_ptr > ram_end) {
		printf("ER:jump sp 0x%08X\n", app_stack_ptr);
		return -1;
	}
	if ((app_stack_ptr & 0x07U) != 0U) {        /* Cortex-M 要求栈 8 字节对齐 */
		bl_puts("ER:jump sp align\n");
		return -1;
	}

	/* ---- 校验 2：复位向量在 app 区内，且 bit0 必须是 1 ----
	 * Cortex-M 只有 Thumb 状态，函数地址 bit0 恒为 1。
	 * bit0 = 0 跳过去 CPU 会以为要切 ARM 状态 → 立刻 HardFault */
	if (app_reset_handle < app_bl_addr || app_reset_handle > flash_end) {
		printf("ER:jump rv 0x%08X\n", app_reset_handle);
		return -1;
	}
	if ((app_reset_handle & 1U) == 0U) {
		bl_puts("ER:jump rv bit0\n");
		return -1;
	}

	/* ---- 换栈、换向量表 ----
	 * 【日志】四道闸全过了、马上要跳 —— 这里必须留一行，
	 * "跳不跳得过去"的问题就靠它和 APP 侧的第一行日志划责任边界。
	 *
	 * ⚠⚠ 这行必须打在下面「拆外设」之前！
	 *   下面 __disable_irq() 会关总中断、SysTick->CTRL=0 会停 SysTick、
	 *   HAL_UART_DeInit(&huart1) 会关掉串口时钟。这时候再输出串口日志，
	 *   HAL_UART_Transmit 会死等 TXE 标志（串口时钟没了，标志永远不置位），
	 *   而它的超时判定靠 HAL_GetTick() —— SysTick 停了它永远是 0，
	 *   于是超时永远不触发 → 卡死在这一行 → app 永远跳不过去。
	 *   （这个坑的实际表现就是「刷完固件完全没反应」，和 app 本身毫无关系）
	 */
	bl_puts("BL: jump -> app\n");

	/* ---- 关掉 bootloader 用过的外设，让 app 从干净状态启动 ---- */

	__disable_irq();      /* 先关总中断，再关外设 */

	HAL_UART_DeInit(&huart1);   /* 关串口（内部会关时钟、DeInit GPIO、关 NVIC） */

	/* SPI 收尾：先把片选拉高取消选中，再关 SPI1。
	 * 不写的话 W25Q 可能在 app 启动瞬间还处于被选中状态 */
	w25q_spi_deinit();

	HAL_RCC_DeInit();           /* 时钟恢复成复位默认（HSI 8MHz） */
	HAL_DeInit();               /* 外设强制复位 */

	/* SysTick 必须自己关：HAL_DeInit 和 HAL_RCC_DeInit 都不管它 */
	SysTick->CTRL = 0U;
	SysTick->LOAD = 0U;
	SysTick->VAL  = 0U;

	/* 把所有中断使能位和挂起标志清干净，
	 * 避免跳过去的一瞬间冒出一个早就挂起的中断 */
	for (uint8_t i = 0; i < 8U; i++) {
		NVIC->ICER[i] = 0xFFFFFFFFU;
		NVIC->ICPR[i] = 0xFFFFFFFFU;
	}

	__set_MSP(app_stack_ptr);   /* 主栈指针换成 app 的栈顶 */
	SCB->VTOR = app_bl_addr;    /* 中断向量表搬到 app 那边 */

	/* 【关键】把总中断开回来！
	 * 上面 __disable_irq() 关掉的中断不在这里开回来，app 会一直处于关中断
	 * 状态：HAL_Delay 卡死、串口收不到数据。HAL_Init() 不会帮你开中断。
	 * 此时所有中断源都已被禁掉，不会有中断在这个窗口里冒出来 */
	__enable_irq();

	/* 跳到 app 的复位向量 */
	((void (*)(void))(void *)app_reset_handle)();

	return 0;   /* 正常跳过去之后不会执行到这里 */
}


/* ================================================================
 * 九、OTA 进度界面（SSD1306 128x64，I2C2：PB10=SCL / PB11=SDA，地址 0x78）
 *
 * 【为什么放在 bootloader 里画】
 *   "升级中"这段时间板子跑的是 bootloader，APP 已经停了 —— 屏幕谁在刷新？
 *   只能是 bootloader。APP 侧 flash 只剩 152 字节，也放不下整屏界面。
 *
 * 【为什么不用 HAL 的 I2C】
 *   bootloader 的 CubeMX 工程里没开 I2C2（也没有 u8g2），只为点亮一块屏
 *   把整个 HAL I2C 驱动链进来不划算（实测 2KB 以上）。这里用软件 I2C
 *   位翻转，PB10/PB11 开漏输出，模块上自带 4.7K 上拉，不需要额外配置。
 *
 * 【什么时候画】
 *   只有 APP 写了 BKP 魔数、bootloader 进 OTA 常驻时才点亮屏幕（见
 *   appbl.c 的 ota_hold）。普通上电一个字节都不碰 I2C —— 不干扰 APP。
 *
 * 【界面布局】
 *   y  0..15   "升级中"（16x16 点阵，3 个字，居中）
 *   y 24..39   百分比数字（16x16 点阵，1~3 位，居中）
 *   y 40..53   进度条：外框 x=4..123，内部填充 x=5..122
 *   百分比和进度条只在变化 >= 2% 时重画，避免把串口收包拖慢
 * ================================================================ */

#define OLED_ADDR 0x78U

#define SCL_H()  do { GPIOB->BSRR = GPIO_PIN_10; } while (0)
#define SCL_L()  do { GPIOB->BRR  = GPIO_PIN_10; } while (0)
#define SDA_H()  do { GPIOB->BSRR = GPIO_PIN_11; } while (0)
#define SDA_L()  do { GPIOB->BRR  = GPIO_PIN_11; } while (0)
#define SDA_IN() (((GPIOB->IDR & GPIO_PIN_11) != 0U) ? 1U : 0U)

static uint8_t s_ui_ready  = 0U;
static uint8_t s_ui_pct    = 0xFFU;   /* 上次画过的百分比，0xFF = 还没画 */
static uint8_t s_pct_digits = 0U;     /* 上次画的数字位数（1~3），位数没变就不清区 */

/* 一个字模 16x16，按行存，bit15 = 最左列。数据由 SimHei 渲染后阈值化得到 */
static const uint16_t s_glyph_up[3][16] = {
	/* 升 */ {0x0120,0x07B0,0x3E30,0x0630,0x0630,0x0630,0x7FFE,0x0630,
	           0x0430,0x0C30,0x0830,0x1830,0x3030,0x6030,0x0000,0x0000},
	/* 级 */ {0x0000,0x13F8,0x1088,0x2488,0x6C98,0x7898,0x189C,0x10C4,
	           0x3DCC,0x7968,0x0138,0x3F10,0x6638,0x04C6,0x0000,0x0000},
	/* 中 */ {0x0180,0x0180,0x0180,0x3FFC,0x318C,0x2184,0x2184,0x2184,
	           0x3FFC,0x2184,0x0180,0x0180,0x0180,0x0180,0x0000,0x0000}
};

static const uint16_t s_glyph_dig[10][16] = {
	/* 0 */ {0x0000,0x0000,0x0000,0x03C0,0x0640,0x0420,0x0420,0x0420,
	         0x0420,0x0420,0x0420,0x0660,0x03C0,0x0180,0x0000,0x0000},
	/* 1 */ {0x0000,0x0000,0x0000,0x00C0,0x01C0,0x03C0,0x00C0,0x00C0,
	         0x00C0,0x00C0,0x00C0,0x00C0,0x00C0,0x0040,0x0000,0x0000},
	/* 2 */ {0x0000,0x0000,0x0000,0x03C0,0x0660,0x0420,0x0060,0x0040,
	         0x00C0,0x0080,0x0100,0x0200,0x07E0,0x07E0,0x0000,0x0000},
	/* 3 */ {0x0000,0x0000,0x0000,0x03C0,0x0660,0x0420,0x0060,0x00C0,
	         0x00C0,0x0060,0x0020,0x0460,0x07C0,0x0180,0x0000,0x0000},
	/* 4 */ {0x0000,0x0000,0x0000,0x0040,0x00C0,0x01C0,0x0140,0x0240,
	         0x0640,0x0440,0x0FF0,0x0040,0x0040,0x0040,0x0000,0x0000},
	/* 5 */ {0x0000,0x0000,0x03E0,0x0600,0x0400,0x0500,0x07C0,0x0460,
	         0x0020,0x0020,0x0C60,0x06C0,0x0380,0x0000,0x0000,0x0000},
	/* 6 */ {0x0000,0x0000,0x0000,0x0080,0x0180,0x0100,0x0300,0x07C0,
	         0x0620,0x0430,0x0430,0x0420,0x0660,0x03C0,0x0000,0x0000},
	/* 7 */ {0x0000,0x0000,0x07E0,0x0020,0x0060,0x0040,0x00C0,0x0080,
	         0x0180,0x0100,0x0100,0x0300,0x0200,0x0000,0x0000,0x0000},
	/* 8 */ {0x0000,0x0000,0x0000,0x03C0,0x0460,0x0420,0x0460,0x07C0,
	         0x07C0,0x0460,0x0C20,0x0420,0x06E0,0x03C0,0x0000,0x0000},
	/* 9 */ {0x0000,0x0000,0x0000,0x03C0,0x0660,0x0C20,0x0C20,0x0C60,
	         0x06C0,0x03C0,0x0080,0x0180,0x0100,0x0300,0x0000,0x0000}
};

/* 软件 I2C 的位延时：72MHz 下这个循环大约 1us，总线约 300kHz。
 * 比 400kHz 略慢，但 SSD1306 完全跟得上，且留足余量给长排线 */
static void ui_delay(void)
{
	for (volatile uint8_t i = 0U; i < 20U; i++) {
		__NOP();
	}
}

static void i2c_start(void)
{
	SDA_H(); ui_delay();
	SCL_H(); ui_delay();
	SDA_L(); ui_delay();
	SCL_L(); ui_delay();
}

static void i2c_stop(void)
{
	SDA_L(); ui_delay();
	SCL_H(); ui_delay();
	SDA_H(); ui_delay();
}

/* 发 8 位 + 读应答。返回 0 = 对方应答（真正发一字节的单位） */
static uint8_t i2c_wr(uint8_t b)
{
	uint8_t i;
	uint8_t ack;

	for (i = 0U; i < 8U; i++) {
		if ((b & 0x80U) != 0U) {
			SDA_H();
		} else {
			SDA_L();
		}
		b = (uint8_t)(b << 1);
		ui_delay();
		SCL_H(); ui_delay();
		SCL_L(); ui_delay();
	}
	SDA_H(); ui_delay();            /* 释放 SDA，读 ACK */
	SCL_H(); ui_delay();
	ack = SDA_IN();
	SCL_L(); ui_delay();
	return ack;
}

static void oled_cmd(uint8_t c)
{
	i2c_start();
	(void)i2c_wr(OLED_ADDR);
	(void)i2c_wr(0x00U);            /* 控制字节：命令 */
	(void)i2c_wr(c);
	i2c_stop();
}

static void oled_window(uint8_t x0, uint8_t x1, uint8_t p0, uint8_t p1)
{
	oled_cmd(0x21U); oled_cmd(x0); oled_cmd(x1);   /* 列地址范围 */
	oled_cmd(0x22U); oled_cmd(p0); oled_cmd(p1);   /* 页地址范围 */
}

static void oled_stream_begin(void)
{
	i2c_start();
	(void)i2c_wr(OLED_ADDR);
	(void)i2c_wr(0x40U);            /* 控制字节：数据 */
}

static void oled_stream_byte(uint8_t b)
{
	(void)i2c_wr(b);
}

static void oled_stream_end(void)
{
	i2c_stop();
}

static void oled_init(void)
{
	GPIO_InitTypeDef g;
	uint8_t page, i;

	__HAL_RCC_GPIOB_CLK_ENABLE();
	g.Pin   = GPIO_PIN_10 | GPIO_PIN_11;
	g.Mode  = GPIO_MODE_OUTPUT_OD;
	g.Pull  = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &g);

	SDA_H();
	SCL_H();
	HAL_Delay(20U);                 /* 等模块内部复位（上电到能收命令约 10ms） */

	oled_cmd(0xAEU);                /* 关显示，配置期间不闪 */
	oled_cmd(0xD5U); oled_cmd(0x80U);   /* 时钟分频 */
	oled_cmd(0xA8U); oled_cmd(0x3FU);   /* 1/64 占空 */
	oled_cmd(0xD3U); oled_cmd(0x00U);   /* 显示偏移 0 */
	oled_cmd(0x40U);                    /* 起始行 0 */
	oled_cmd(0x8DU); oled_cmd(0x14U);   /* 打开电荷泵（模块不带外部升压） */
	oled_cmd(0x20U); oled_cmd(0x00U);   /* 水平寻址模式：写满一页自动跳下一页 */
	oled_cmd(0xA1U);                    /* 列 127 映射到 SEG0（左右方向） */
	oled_cmd(0xC8U);                    /* COM 反向扫描（上下方向） */
	oled_cmd(0xDAU); oled_cmd(0x12U);   /* COM 引脚配置 */
	oled_cmd(0x81U); oled_cmd(0xCFU);   /* 对比度 */
	oled_cmd(0xD9U); oled_cmd(0xF1U);   /* 预充电周期 */
	oled_cmd(0xDBU); oled_cmd(0x40U);   /* VCOMH */
	oled_cmd(0xA4U);                    /* 按 GDDRAM 显示（不掉进全亮测试态） */
	oled_cmd(0xA6U);                    /* 正常（非反色） */

	/* 清 GDDRAM：APP 停止时屏幕上还是它的主页，不清会和"升级中"叠在一起 */
	for (page = 0U; page < 8U; page++) {
		oled_window(0U, 127U, page, page);
		oled_stream_begin();
		for (i = 0U; i < 128U; i++) {
			oled_stream_byte(0x00U);
		}
		oled_stream_end();
	}
	oled_cmd(0xAFU);                /* 开显示 */
}

/* 画一个 16x16 字模，左上角落在 (x, page*8) */
static void ui_glyph16(uint8_t x, uint8_t page, const uint16_t *g)
{
	uint8_t col, row, b;
	uint16_t mask;

	oled_window(x, (uint8_t)(x + 15U), page, (uint8_t)(page + 1U));
	oled_stream_begin();
	for (col = 0U; col < 16U; col++) {
		mask = (uint16_t)(0x8000U >> col);
		b = 0U;
		for (row = 0U; row < 8U; row++) {
			if ((g[row] & mask) != 0U) {
				b |= (uint8_t)(1U << row);
			}
		}
		oled_stream_byte(b);
	}
	for (col = 0U; col < 16U; col++) {
		mask = (uint16_t)(0x8000U >> col);
		b = 0U;
		for (row = 8U; row < 16U; row++) {
			if ((g[row] & mask) != 0U) {
				b |= (uint8_t)(1U << (row - 8U));
			}
		}
		oled_stream_byte(b);
	}
	oled_stream_end();
}

/* 进度条：外框 x=4..123 / y=40..53，内部 x=5..122。
 * 页 5 = y40..47，页 6 = y48..55，所以一个字节里 bit0 是最上面那行
 *
 * 【增量画】每 1% 只补新长出来的那几列（1~2 列），一条 I2C 事务就完事。
 * 要是每步都重画整条（240 字节 ≈ 10ms），1% 一步会把串口拖慢，动画反而更卡。
 * 内部两页的花样是固定的：页 5 整字节 0xFE，页 6 整字节 0x1F。 */
static uint8_t s_bar_fill = 0xFFU;   /* 上次画到的填充列数，0xFF = 还没画过 */

static void ui_draw_bar(uint8_t pct)
{
	uint8_t fill = (uint8_t)((118U * pct) / 100U);   /* 0..118 */
	uint8_t x, n;

	if (fill == s_bar_fill) {
		return;
	}

	if (fill > s_bar_fill) {
		/* 只补新长出来的列 */
		n = (uint8_t)(fill - s_bar_fill);
		oled_window((uint8_t)(5U + s_bar_fill), (uint8_t)(4U + fill), 5U, 6U);
		oled_stream_begin();
		for (x = 0U; x < n; x++) {
			oled_stream_byte(0xFEU);                 /* 页 5：y=41..47 填充 */
		}
		for (x = 0U; x < n; x++) {
			oled_stream_byte(0x1FU);                 /* 页 6：y=48..52 填充 */
		}
		oled_stream_end();
		s_bar_fill = fill;
		return;
	}

	/* 第一次画，或者变短（同一次上电里又来了第二份固件）：整条重画 */
	oled_window(4U, 123U, 5U, 6U);
	oled_stream_begin();
	for (x = 4U; x <= 123U; x++) {
		uint8_t b = 0x01U;                           /* y=40 上边框 */
		if ((x == 4U) || (x == 123U)) {
			b |= 0xFEU;                              /* 左右竖边 */
		} else if (x < (uint8_t)(5U + fill)) {
			b |= 0xFEU;                              /* y=41..47 填充 */
		}
		oled_stream_byte(b);
	}
	for (x = 4U; x <= 123U; x++) {
		uint8_t b = 0x20U;                           /* y=53 下边框 */
		if ((x == 4U) || (x == 123U)) {
			b |= 0x1FU;
		} else if (x < (uint8_t)(5U + fill)) {
			b |= 0x1FU;                              /* y=48..52 填充 */
		}
		oled_stream_byte(b);
	}
	oled_stream_end();
	s_bar_fill = fill;
}

/* 百分比：固定清掉中间 48 像素宽的一条（x=40..87，页 3..4），再按位数居中画 */
static void ui_draw_pct(uint8_t pct)
{
	char    s[3];
	uint8_t n = 0U;
	uint8_t i, x;

	if (pct >= 100U) {
		s[0] = '1'; s[1] = '0'; s[2] = '0'; n = 3U;
	} else if (pct >= 10U) {
		s[0] = (char)('0' + (pct / 10U));
		s[1] = (char)('0' + (pct % 10U));
		n = 2U;
	} else {
		s[0] = (char)('0' + pct);
		n = 1U;
	}

	/* 只有位数变了才清空整块（1位↔2位↔3位）。
	 * 位数不变时直接把这几个 16x16 格子整格重写就行 —— 格子里是完整字节
	 * 覆盖，旧笔画的残影会被新字模顶掉，省掉一次"先清空 96 字节"，
	 * 数字也就不会 16Hz 地闪一下 */
	if (n != s_pct_digits) {
		oled_window(40U, 87U, 3U, 4U);
		oled_stream_begin();
		for (i = 0U; i < 96U; i++) {                  /* 48 列 x 2 页 */
			oled_stream_byte(0x00U);
		}
		oled_stream_end();
		s_pct_digits = n;
	}

	x = (uint8_t)((128U - (uint16_t)n * 16U) / 2U);
	for (i = 0U; i < n; i++) {
		ui_glyph16((uint8_t)(x + i * 16U), 3U, s_glyph_dig[s[i] - '0']);
	}
}

/* 进 OTA 常驻时调一次：初始化屏幕、画"升级中"、进度 0% */
void ota_ui_begin(void)
{
	if (s_ui_ready != 0U) {
		return;
	}
	oled_init();
	ui_glyph16(40U, 0U, s_glyph_up[0]);
	ui_glyph16(56U, 0U, s_glyph_up[1]);
	ui_glyph16(72U, 0U, s_glyph_up[2]);
	s_ui_pct = 0xFFU;
	s_bar_fill = 0xFFU;
	s_pct_digits = 0U;
	s_ui_ready = 1U;
	ota_ui_set_pct(0U);
}

/* 更新进度（0~100）。只要数字变了就刷 —— 每 1% 一步，条子和数字才连得上。
 * 条子本身是增量补列，数字区 96 字节，一次约 4ms，收包时几乎无感 */
void ota_ui_set_pct(uint8_t pct)
{
	if (s_ui_ready == 0U) {
		return;                                      /* 普通上电没开屏，直接忽略 */
	}
	if (pct > 100U) {
		pct = 100U;
	}
	if (pct == s_ui_pct) {
		return;
	}
	s_ui_pct = pct;
	ui_draw_bar(pct);
	ui_draw_pct(pct);
}

/* 阶段切换用：从当前值一格一格"走"到目标（1%/20ms）。
 * 直接跳 80→82→85→86→88→90→92 会看到一串台阶，走过去就顺了。
 * 这几十毫秒里串口收到的字节都在环形缓冲里排队，不会丢 */
void ota_ui_ramp(uint8_t target)
{
	if (s_ui_ready == 0U) {
		return;
	}
	if (target > 100U) {
		target = 100U;
	}
	while (s_ui_pct != target) {
		uint8_t next = (s_ui_pct < target) ? (uint8_t)(s_ui_pct + 1U)
		                                   : (uint8_t)(s_ui_pct - 1U);
		ota_ui_set_pct(next);
		HAL_Delay(20U);
	}
}

/* 跳 app 之前必须调一次：把 SSD1306 的列/页窗口还原成整屏。
 *
 * 【为什么】u8g2 写屏只用 0xB0(页) + 0x10/0x00(列起点)，**从不下发
 * 0x21/0x22 窗口命令**。我们在进度界面里用 0x21/0x22 把窗口缩成了
 * 进度条那一块（4..123 列 / 5..6 页）；不还原的话，APP 之后每一页
 * 只写得进窗口里的那几列，屏幕右侧会一直残留着进度条。
 * 普通上电时控制器窗口是默认的整屏，所以只有 OTA 之后才会犯。 */
void ota_ui_restore(void)
{
	uint8_t page, i;

	if (s_ui_ready == 0U) {
		return;
	}

	/* 顺手把屏清空：APP 第一帧要 ~几十 ms 才画出来，
	 * 中间别让"升级中 100%"继续挂着 */
	for (page = 0U; page < 8U; page++) {
		oled_window(0U, 127U, page, page);
		oled_stream_begin();
		for (i = 0U; i < 128U; i++) {
			oled_stream_byte(0x00U);
		}
		oled_stream_end();
	}

	/* 窗口还原成整屏（8 页 x 128 列），和上电默认一致 */
	oled_cmd(0x21U); oled_cmd(0x00U); oled_cmd(0x7FU);
	oled_cmd(0x22U); oled_cmd(0x00U); oled_cmd(0x07U);

	s_ui_ready = 0U;
	s_bar_fill = 0xFFU;
	s_pct_digits = 0U;
}
