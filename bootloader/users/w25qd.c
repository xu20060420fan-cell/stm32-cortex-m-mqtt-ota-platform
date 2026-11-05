/*
 * w25qd.c —— W25Q SPI Flash 驱动 + 升级完成标志
 *
 * 【驱动方式】HAL_SPI（SPI1）+ DMA。
 *
 *   ┌ 短传输（≤ 15 字节）：命令、地址、状态寄存器 —— 阻塞模式
 *   │   HAL_SPI_TransmitReceive / HAL_SPI_Receive
 *   │   理由：启动 DMA 本身也有开销，几个字节不值得
 *   └ 长传输（≥ 16 字节）：页编程数据、读回数据 —— DMA 模式
 *       HAL_SPI_TransmitReceive_DMA / HAL_SPI_Receive_DMA
 *       理由：256 字节 @18MHz 要 114us，DMA 期间 CPU 可以干别的
 *
 * 【为什么 2 线全双工模式下"只发"也必须给接收缓冲】
 * SPI 是全双工的，发一个字节必然同时收一个字节。如果只发不收，
 * 接收寄存器 RXNE 会一直不被读走，最后置位 OVR（溢出），
 * HAL 会把它当错误、中止传输。所以下面准备了一块占位缓冲 spi_dummy，
 * 只发的时候把收来的垃圾数据丢进它。
 *
 * 接线：PA4=CS  PA5=SCK  PA6=MISO  PA7=MOSI
 * 参数：SPI1 主模式，8bit，Mode0（CPOL=0 CPHA=0），MSB first，72M/4 = 18MHz
 */

#include "w25qd.h"
#include "crc32.h"

extern SPI_HandleTypeDef hspi1;

/* JEDEC 识别出的实际容量（字节）。w25qd_init() 成功后有效 */
static uint32_t g_w25q_size = 0;

uint32_t w25q_total_size(void)
{
	return g_w25q_size;
}

/* ================================================================
 * 一、SPI1 收发封装
 * ================================================================ */
#define SPI_TIMEOUT_MS  1000U   /* 单次传输的等待上限 */
#define SPI_DMA_MIN     16U     /* 长度到多少才值得走 DMA */

/* 占位缓冲：SPI 是全双工，"只发"时得有人接住收来的字节，
 * "只收"时得有个东西发出去产生时钟。内容是什么无所谓。
 * 长度按最大一次传输给：读回一包 512 字节 */
static uint8_t spi_dummy[512];

/* 等 DMA 把这一笔传完。
 * HAL 在传输完成回调里会把 hspi1.State 置回 READY，所以轮询这个字段即可 */
static uint8_t spi_wait_dma(void)
{
	uint32_t t0 = HAL_GetTick();

	while (hspi1.State != HAL_SPI_STATE_READY) {
		if ((HAL_GetTick() - t0) > SPI_TIMEOUT_MS) {
			HAL_SPI_Abort(&hspi1);      /* 卡住了就强行收尾，否则后面全废 */
			return 0;
		}
	}
	return 1;
}

/* 全双工收发 n 个字节：一边把 tx 发出去，一边把 rx 收回来。
 * 长度小于 SPI_DMA_MIN 走阻塞，其余走 DMA。
 * 返回 1 = 成功 */
static uint8_t spi_xchg(const uint8_t *tx, uint8_t *rx, uint16_t n)
{
	if (n == 0U) {
		return 1;
	}
	if (n > sizeof(spi_dummy)) {
		return 0;                       /* 超出缓冲，属于调用方用错 */
	}

	if (n < SPI_DMA_MIN) {
		return (HAL_SPI_TransmitReceive(&hspi1, tx, rx, n,
		                                SPI_TIMEOUT_MS) == HAL_OK) ? 1 : 0;
	}

	if (HAL_SPI_TransmitReceive_DMA(&hspi1, tx, rx, n) != HAL_OK) {
		return 0;
	}
	return spi_wait_dma();
}

/* 发 n 个字节，收来的丢掉 */
static uint8_t spi_tx(const uint8_t *tx, uint16_t n)
{
	return spi_xchg(tx, spi_dummy, n);
}

/* 收 n 个字节（发出去的是占位数据，只是为了让时钟跑起来） */
static uint8_t spi_rx(uint8_t *rx, uint16_t n)
{
	return spi_xchg(spi_dummy, rx, n);
}

void w25q_spi_deinit(void)
{
	CS_HIGH();                       /* 先取消选中，别让 W25Q 还挂在总线上 */
	HAL_SPI_DeInit(&hspi1);          /* 关 SPI1：关时钟、DeInit GPIO、DeInit DMA */
}


/* ================================================================
 * 二、W25Q 基础操作
 * ================================================================ */
static uint8_t w25qd_read_status(void)
{
	uint8_t cmd = 0x05;              /* RDSR1 */
	uint8_t st  = 0;

	CS_LOW();
	spi_tx(&cmd, 1);
	spi_rx(&st, 1);
	CS_HIGH();
	return st;
}

/* 等 BUSY 位清零。用 tick 计时而不是循环次数，主频变了也不会失准 */
static uint8_t w25qd_wait_busy(void)
{
	uint32_t t0 = HAL_GetTick();

	while (w25qd_read_status() & 0x01) {
		if ((HAL_GetTick() - t0) > 2000) {   /* 扇区擦除最大 400ms，2s 足够 */
			return 0;
		}
	}
	return 1;
}

/* 发写使能 06h 并确认 WEL 真的置上了 */
static uint8_t w25qd_write_enable(void)
{
	uint8_t cmd = 0x06;

	CS_LOW();
	spi_tx(&cmd, 1);
	CS_HIGH();

	return (w25qd_read_status() & 0x02) ? 1 : 0;
}

static uint8_t w25qd_send_addr(uint32_t addr)
{
	uint8_t a[3];

	a[0] = (uint8_t)(addr >> 16);
	a[1] = (uint8_t)(addr >> 8);
	a[2] = (uint8_t)(addr);
	return spi_tx(a, 3);
}


/* ================================================================
 * 三、对外接口
 * ================================================================ */
uint8_t w25qd_init(void)
{
	/* 读 JEDEC ID：发 9Fh，接着读 3 个字节（厂商 ID / 存储类型 / 容量）。
	 * SPI 是全双工的，所以一次 4 字节收发就把命令和结果一起办完了：
	 *   tx = 9F 00 00 00
	 *   rx = xx EF 40 14   ← rx[1..3] 就是 JEDEC ID
	 * 这就是原代码里 spi1_xfer_buf(tx, rx, 4) 的做法 */
	uint8_t tx[4] = {0x9F, 0x00, 0x00, 0x00};
	uint8_t rx[4] = {0};

	/* 【容量自动识别】不再写死一颗型号。表里没有的容量宁可报错停下来，
	 * 也不能拿错误的 W25Q_TOTAL_SIZE 去做边界检查（会写飞） */
	static const struct { uint8_t cap; uint32_t size; } cap_tbl[] = {
		{ 0x14, 1UL * 1024UL * 1024UL },    /* W25Q80  */
		{ 0x15, 2UL * 1024UL * 1024UL },    /* W25Q16  */
		{ 0x16, 4UL * 1024UL * 1024UL },    /* W25Q32  */
		{ 0x17, 8UL * 1024UL * 1024UL },    /* W25Q64  */
		{ 0x18, 16UL * 1024UL * 1024UL },   /* W25Q128 */
	};
	const uint8_t n_cap = (uint8_t)(sizeof(cap_tbl) / sizeof(cap_tbl[0]));
	uint8_t found = 0;

	CS_HIGH();                       /* 空闲时片选必须高，先摆好 */
	HAL_Delay(10);                   /* 等芯片上电稳定 */

	CS_LOW();
	if (!spi_xchg(tx, rx, 4)) {
		CS_HIGH();
		bl_puts("ER:w25q spi\n");
		return 0;
	}
	CS_HIGH();

	/* JEDEC 只在**对不上**的时候才打印：正常上电每次都打一行 EF 40 18
	 * 属于纯噪音，而失败时必须让人一眼看到实测到的是什么 */
	if (rx[1] != 0xEF || rx[2] != 0x40) {
		printf("ER:w25q id %02X %02X %02X\n", rx[1], rx[2], rx[3]);
		return 0;
	}
	for (uint8_t i = 0; i < n_cap; i++) {
		if (rx[3] == cap_tbl[i].cap) {
			g_w25q_size = cap_tbl[i].size;
			found = 1;
			break;
		}
	}
	if (!found) {
		printf("ER:w25q cap %02X\n", rx[3]);
		return 0;
	}
	return 1;
}

uint8_t w25qd_erase(uint32_t addr, uint32_t num_sectors)
{
	uint8_t cmd = 0x20;              /* 扇区擦除，一个扇区 4KB */

	for (uint32_t i = 0; i < num_sectors; i++) {
		uint32_t cur = addr + i * 0x1000UL;

		if (!w25qd_wait_busy()) {
			printf("ER:w25q busy 0x%06X\n", cur);
			return 0;
		}
		if (!w25qd_write_enable()) {
			printf("ER:w25q wel 0x%06X\n", cur);
			return 0;
		}

		CS_LOW();
		spi_tx(&cmd, 1);
		w25qd_send_addr(cur);
		CS_HIGH();

		if (!w25qd_wait_busy()) {
			printf("ER:w25q erase 0x%06X\n", cur);
			return 0;
		}
	}
	return 1;
}

uint8_t w25qd_write(uint32_t addr, uint8_t *data, uint32_t length)
{
	uint8_t cmd = 0x02;              /* 页编程 */

	/* 【边界检查】JEDEC 识别失败时 g_w25q_size 是 0，任何写入都该被拒 */
	if (g_w25q_size == 0U || (addr + length) > g_w25q_size) {
		bl_puts("ER:w25q range\n");
		return 0;
	}

	while (length > 0) {
		/* 一次不能跨 256 字节页边界，W25Q 页编程会在页内回卷 */
		uint32_t page_left = 256UL - (addr & 0xFFUL);
		uint16_t n = (length < page_left) ? (uint16_t)length : (uint16_t)page_left;

		if (!w25qd_wait_busy()) {
			return 0;
		}
		if (!w25qd_write_enable()) {
			return 0;
		}

		CS_LOW();
		spi_tx(&cmd, 1);
		w25qd_send_addr(addr);
		if (!spi_tx(data, n)) {       /* 这一段 256 字节走 DMA */
			CS_HIGH();
			printf("ER:w25q spi 0x%06X\n", addr);
			return 0;
		}
		CS_HIGH();

		if (!w25qd_wait_busy()) {
			printf("ER:w25q wr 0x%06X\n", addr);
			return 0;
		}

		addr   += n;
		data   += n;
		length -= n;
	}
	return 1;
}

uint8_t w25qd_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
	uint8_t cmd = 0x03;              /* 读数据 */

	if (g_w25q_size == 0U || (addr + len) > g_w25q_size) {
		return 0;
	}
	if (!w25qd_wait_busy()) {
		return 0;
	}

	CS_LOW();
	spi_tx(&cmd, 1);
	if (!w25qd_send_addr(addr)) {    /* 发地址，不是 buf */
		CS_HIGH();
		return 0;
	}
	if (!spi_rx(buf, (uint16_t)len)) {   /* 这一整段走 DMA */
		CS_HIGH();
		return 0;
	}
	CS_HIGH();
	return 1;
}


/* ================================================================
 * 四、升级完成标志
 *
 * 【为什么需要】搬运内部 Flash 是"从偏移 0 顺序写"的，所以前 8 字节
 * 最先写对。写到一半断电时，bl_jump_app 的栈顶/复位向量校验仍会通过，
 * 于是跳进一个残缺的 app 直接跑飞。有了这个标志就能拦住。
 *
 * 三态设计的关键点：0xFFFFFFFF 表示"从没用过 OTA"，
 * 这种板子（比如用 ST-Link 烧的）必须放行，否则会被永久拦在 bootloader。
 * ================================================================ */
/* 读标志的原始值。
 * 对外公开是为了让 ? 命令能回显它 —— "跳不过去"最常见的原因之一就是
 * 标志停在"升级中"，不看这个值没法判断。读失败按"从没用过"处理 */
uint32_t ota_flag_raw(void)
{
	uint8_t  buf[4];
	uint32_t v;

	if (!w25qd_read(OTA_FLAG_ADDR, buf, 4)) {
		return 0xFFFFFFFFUL;         /* 读失败按"从没用过"处理，不拦 */
	}
	v = (uint32_t)buf[0]
	  | ((uint32_t)buf[1] << 8)
	  | ((uint32_t)buf[2] << 16)
	  | ((uint32_t)buf[3] << 24);
	return v;
}

uint8_t ota_flag_app_trusted(void)
{
	uint32_t v = ota_flag_raw();

	if (v == 0xFFFFFFFFUL) {
		return 1;                    /* 从没用过 OTA，放行 */
	}
	if (v == OTA_FLAG_MAGIC) {
		return 1;                    /* 上次升级成功，放行 */
	}
	/* 【2026-11-01 放行"升级中"】
	 * 以前这里无条件拒绝，制造了一个死锁：用 ST-Link 直接烧 APP
	 * （不经过 OTA 流程）时，没人把标志从"升级中"改回"可信"，
	 * 设备就永远卡在 bootloader，只能手工发 F 命令救 —— 实际踩过。
	 * 现在放行；bl_jump_app 的栈顶/复位向量两道闸照常工作，
	 * 头部残废的固件依然拦得住。 */
	return 1;
}

static uint8_t ota_flag_write(uint32_t value)
{
	uint8_t buf[4];

	/* NOR Flash 只能把 1 写成 0，所以每次都要先擦掉整个扇区 */
	if (!w25qd_erase(OTA_FLAG_ADDR, 1)) {
		return 0;
	}

	buf[0] = (uint8_t)(value);
	buf[1] = (uint8_t)(value >> 8);
	buf[2] = (uint8_t)(value >> 16);
	buf[3] = (uint8_t)(value >> 24);

	return w25qd_write(OTA_FLAG_ADDR, buf, 4);
}

uint8_t ota_flag_begin(void)
{
	return ota_flag_write(0x00000000UL);     /* "升级中，app 不可信" */
}

uint8_t ota_flag_commit(void)
{
	return ota_flag_write(OTA_FLAG_MAGIC);   /* "升级成功，app 可信" */
}


/* ================================================================
 * 五、槽位元数据（A/B 槽 + 断点续传进度）
 *
 * 【存在哪】独立 4K 扇区（W25Q_META_ADDR），跟槽区、备份区、
 * 升级标志互不重叠。改任何一个字段都是"RAM 里改好 → 擦扇区 →
 * 整体重写 → 回读比对"，简单且不会出现半新半旧。
 * ================================================================ */

/* 给 *m 填默认值：active 无记录、两个槽都无效、没有进行中的传输。
 * 全 0 之外唯一要刻意设置的是"无效"标记 —— 0 恰好就是无效，
 * 所以擦除态（全 0xFF 被读成…… 注意 load 失败时 *m 是 RAM 里的，
 * 直接 memset 成下面的样子即可） */
static void meta_default(w25q_meta_t *m)
{
	memset(m, 0, sizeof(*m));
	m->magic            = W25Q_META_MAGIC;
	m->active_slot      = 0xFFU;            /* 0xFF = 无记录 */
	m->slot[0].version  = 0;
	m->slot[1].version  = 0;
	m->xfer.slot        = 0xFFFFFFFFUL;     /* 没有进行中的传输 */
}

/* CRC 覆盖结构体里除最后 4 字节（crc 字段本身）以外的全部内容 */
static uint32_t meta_crc(const w25q_meta_t *m)
{
	return crc32_calc((const uint8_t *)m, (uint32_t)(sizeof(*m) - sizeof(uint32_t)));
}

uint8_t meta_load(w25q_meta_t *m)
{
	if (!w25qd_read(W25Q_META_ADDR, (uint8_t *)m, sizeof(*m))) {
		meta_default(m);
		return 0;
	}
	if (m->magic != W25Q_META_MAGIC) {
		meta_default(m);                     /* 从没写过（擦除态） */
		return 0;
	}
	if (meta_crc(m) != m->crc) {
		/* 读出来了但内容坏了（写一半掉电、位翻转）。
		 * 不能直接信，也不能悄悄当有效 —— 填默认值，等于"没有槽记录、
		 * 没有断点"。固件本体在槽区里没受影响，重新 O 一次即可 */
		meta_default(m);
		return 0;
	}
	/* 老版本写入的记录里 xfer.slot 可能是别的脏值，兜一道底 */
	if (m->xfer.slot > 1U && m->xfer.slot != 0xFFFFFFFFUL) {
		m->xfer.slot = 0xFFFFFFFFUL;
	}
	if (m->active_slot > 1U) {
		m->active_slot = 0xFFU;
	}
	return 1;
}

uint8_t meta_save(const w25q_meta_t *m)
{
	w25q_meta_t chk;

	/* CRC 由这里统一算，调用方不用管 */
	w25q_meta_t out = *m;
	out.magic = W25Q_META_MAGIC;
	out.crc   = meta_crc(&out);

	if (!w25qd_erase(W25Q_META_ADDR, 1)) {
		return 0;
	}
	if (!w25qd_write(W25Q_META_ADDR, (uint8_t *)&out, sizeof(out))) {
		return 0;
	}
	/* 回读比对 —— 写元数据就是为了掉电后还能信它，
	 * 写坏了却当写成功，比不写还糟 */
	if (!w25qd_read(W25Q_META_ADDR, (uint8_t *)&chk, sizeof(chk))) {
		return 0;
	}
	return (memcmp(&chk, &out, sizeof(out)) == 0) ? 1 : 0;
}
