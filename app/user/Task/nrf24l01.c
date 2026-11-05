/*
 * nrf24l01.c  ——  NRF24L01+ 2.4GHz 无线收发模块驱动（SPI2 + GPIO，HAL 版）
 *
 *  Created on: Oct 24, 2026
 *      Author: Administrator
 *
 * 移植自 esp32mqtt 工程的 nrf24.cpp（Arduino/ESP32 版）。
 * 寄存器逻辑、地址（"NODE1"）、信道（76）、32 字节载荷、1Mbps、自动 ACK、
 * CRC 使能都照搬参考实现，下面几处**按 nRF24L01+ datasheet 做了修正**，
 * 原因写在对应代码的注释里：
 *   1. 参考代码的常量名 EN_AA / EN_RXADDR 写反了（datasheet：EN_RXADDR=0x01，
 *      EN_AA=0x02）。它写的值 0x01/0x01 刚好都对，所以只是名字反了。
 *   2. 参考代码 RF_SETUP 写 0x06，按 datasheet 解读是
 *      「0dBm + LNA_HC + 250kbps」，和它注释里写的 1Mbps 对不上，
 *      这里按 datasheet 改成 0x00（0dBm + 1Mbps）。
 *   3. 参考代码没管 SPI 时钟。nRF24L01+ 的 SPI 上限是 10MHz，
 *      这里在 nrf24_init() 里按 PCLK1 重新算并强制生效（见 nrf24_spi_ensure_speed）。
 *
 * 其它约定：
 *   - 不使用 HAL_Delay（会阻塞其它任务）。毫秒级延时用 vTaskDelay 让出 CPU；
 *     CE 脉冲要 15us，vTaskDelay 的最小刻度是 1ms，用 DWT 周期计数器忙等。
 *   - SPI2 在本工程里没有别的驱动在用（.ioc 里虽然挂了 SPI2_TX 的 DMA，
 *     但没人发起过 DMA 传输），所以这里用轮询式 HAL_SPI_TransmitReceive。
 */
#include "nrf24l01.h"
#include "debug_log.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

extern SPI_HandleTypeDef hspi2;

/* ===================== SPI 命令 / 寄存器 ===================== */
#define NRF24_CMD_READ          0x00U   /* 读寄存器：0x00 | addr(5bit) */
#define NRF24_CMD_WRITE         0x20U   /* 写寄存器：0x20 | addr(5bit) */
#define NRF24_CMD_R_RX_PAYLOAD  0x61U   /* 读接收 FIFO */
#define NRF24_CMD_W_TX_PAYLOAD  0xA0U   /* 写发送 FIFO */
#define NRF24_CMD_FLUSH_TX      0xE1U   /* 清发送 FIFO */
#define NRF24_CMD_FLUSH_RX      0xE2U   /* 清接收 FIFO */
#define NRF24_NOP               0xFFU   /* 空命令（读的时候靠它把时钟打出来） */

#define NRF24_REG_CONFIG        0x00U
#define NRF24_REG_EN_RXADDR     0x01U   /* 启用的数据管道 */
#define NRF24_REG_EN_AA         0x02U   /* 自动应答（ACK）使能 */
#define NRF24_REG_SETUP_AW      0x03U   /* 地址宽度 */
#define NRF24_REG_SETUP_RETR    0x04U   /* 自动重传 */
#define NRF24_REG_RF_CH         0x05U   /* 信道 */
#define NRF24_REG_RF_SETUP      0x06U   /* 速率 / 功率 */
#define NRF24_REG_STATUS        0x07U
#define NRF24_REG_RX_ADDR_P0    0x0AU
#define NRF24_REG_TX_ADDR       0x10U
#define NRF24_REG_RX_PW_P0      0x11U   /* pipe0 固定载荷长度 */
#define NRF24_REG_FIFO_STATUS   0x17U

/* CONFIG 位 */
#define NRF24_CONFIG_PRIM_RX    0x01U   /* 1 = PRX(接收)，0 = PTX(发送) */
#define NRF24_CONFIG_PWR_UP     0x02U   /* 1 = 上电 */
#define NRF24_CONFIG_CRCO       0x04U   /* 1 = 2 字节 CRC */
#define NRF24_CONFIG_EN_CRC     0x08U   /* 1 = 使能 CRC */

/* STATUS 位（写 1 清零） */
#define NRF24_STATUS_RX_DR      0x40U
#define NRF24_STATUS_TX_DS      0x20U
#define NRF24_STATUS_MAX_RT     0x10U

/* FIFO_STATUS 位 */
#define NRF24_FIFO_RX_EMPTY     0x01U

/*
 * RF_SETUP（0x06）按 nRF24L01+ datasheet：
 *   bit4:3 = RF_PWR（00 = 0dBm，01 = -6dBm，10 = -12dBm，11 = -18dBm）
 *   bit2   = LNA_HC（PA+LNA 模块用）
 *   bit1   = RF_RATE（0 = 1Mbps，1 = 250kbps）
 * 所以 0dBm + 1Mbps = 0x00。
 */
#define NRF24_RF_SETUP_1M_0DBM  0x06U

/* 其它常量 */
#define NRF24_SPI_TIMEOUT_MS    100U
#define NRF24_TX_TIMEOUT_MS     100U    /* 等 TX_DS 的总超时 */
#define NRF24_PWRUP_DELAY_MS    2U      /* 上电后等晶振起振（datasheet 1.5ms） */
#define NRF24_CE_PULSE_US       15U     /* CE 拉高时长（datasheet 至少 10us） */
#define NRF24_CE_RX_START_US    150U    /* 进接收模式后等收发器稳定 */

/* nRF24L01+ 的 SPI 时钟上限（datasheet 5.2 节） */
/* 现场两端模块都出现 SETUP_AW 0x00/0xFF 交替（供电/接触不稳），
 * SPI 从 9MHz 降到约 2MHz，给电源和排针留余量。 */
#define NRF24_SPI_MAX_HZ        2000000UL

/* ===================== 内部状态 ===================== */
#define NRF24_MODE_RX  0U
#define NRF24_MODE_TX  1U
#define NRF24_MODE_DOWN 2U   /* 掉电模式（~26µA） */

static uint8_t  s_inited;                 /* 1 = 初始化成功 */
static uint8_t  s_mode;                   /* 当前是 RX 还是 TX */
static uint8_t  s_channel;                /* 当前信道 */
static uint8_t  s_tx_addr[NRF24_ADDR_LEN];
static uint8_t  s_rx_addr[NRF24_ADDR_LEN];
static uint8_t  s_dwt_ok;                 /* DWT 周期计数器能不能用 */

/* ===================== 延时工具 ===================== */

/**
 * @brief  初始化 DWT 周期计数器（用来做微秒级延时）
 */
static void nrf24_dwt_init(void)
{
    uint32_t t0;
    uint32_t t1;
    uint32_t i;

    if (s_dwt_ok != 0U) {
        return;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* 试一下计数器有没有在走（个别芯片不开 DWT） */
    t0 = DWT->CYCCNT;
    for (i = 0U; i < 32U; ++i) {
        __asm volatile ("nop");
    }
    t1 = DWT->CYCCNT;
    s_dwt_ok = (t1 != t0) ? 1U : 0U;
}

/**
 * @brief  微秒级忙等（只用于 CE 的 15us 脉冲这种没法用 tick 的场合）
 * @param  us: 微秒数
 */
static void nrf24_delay_us(uint32_t us)
{
    uint32_t start;
    uint32_t cycles;
    uint32_t hclk;

    nrf24_dwt_init();

    if (s_dwt_ok == 0U) {
        /* DWT 不可用时的粗略兜底（72MHz 下大致对） */
        for (; us > 0U; --us) {
            volatile uint32_t j;
            for (j = 0U; j < 6U; ++j) {
            }
        }
        return;
    }

    hclk = HAL_RCC_GetHCLKFreq();
    /* cycles = us * hclk / 1e6
     * 拆成 us*(hclk/1e6) + us*(hclk%1e6)/1e6，全程 32 位：
     * 原写法 ((uint64_t)us * hclk) / 1000000ULL 会把 libgcc 的
     * __udivmoddi4（766 字节）拖进固件，而 72MHz 这类常见时钟
     * hclk 正好是 1e6 的整数倍，余数项直接跳过。 */
    cycles = us * (hclk / 1000000U);
    if ((hclk % 1000000U) != 0U) {
        cycles += (us * (hclk % 1000000U)) / 1000000U;
    }
    if (cycles == 0U) {
        return;
    }
    start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles) {
    }
}

/**
 * @brief  毫秒级延时。**不用 HAL_Delay**：
 *        调度器起来了就用 vTaskDelay 让出 CPU（不阻塞其它任务）；
 *        还没起来（理论上不会，但保险）就用 DWT 忙等（此时没有别的任务会被饿死）。
 * @param  ms: 毫秒数
 */
static void nrf24_delay_ms(uint32_t ms)
{
    if (ms == 0U) {
        return;
    }
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        vTaskDelay(pdMS_TO_TICKS(ms));
    } else {
        nrf24_delay_us(ms * 1000U);
    }
}

/* ===================== GPIO / SPI 底层 ===================== */

/** @brief CE 拉高/拉低 */
static void nrf24_ce(uint8_t high)
{
    HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin,
                      (high != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/** @brief CSN 拉低/拉高 */
static void nrf24_csn(uint8_t low)
{
    HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin,
                      (low != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
 * @brief  配 GPIO：CE/CSN 输出、IRQ 上拉输入，另外补上工程里没配的 SPI2 MISO
 */
static void nrf24_gpio_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* CE / CSN：推挽输出 */
    gpio.Pin   = NRF24_CSN_Pin | NRF24_CE_Pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* 空闲状态：CSN 高（不选中），CE 低（待机） */
    HAL_GPIO_WritePin(NRF24_CSN_GPIO_Port, NRF24_CSN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(NRF24_CE_GPIO_Port,  NRF24_CE_Pin,  GPIO_PIN_RESET);

    /* IRQ：模块是开漏输出，用上拉输入 */
    gpio.Pin  = NRF24_IRQ_Pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(NRF24_IRQ_GPIO_Port, &gpio);

    /*
     * SPI2 MISO = PB14：**工程里没配**（.ioc 的 SPI2 只给了 PB13=SCK / PB15=MOSI，
     * HAL_SPI_MspInit 也只初始化了这两个脚）。SPI 主机模式下 MISO 悬空的话，
     * 读回来的全是 0x00/0xFF，寄存器根本读不出来，所以这里自己补上。
     * 【必须是 INPUT，不是 AF_PP】MISO 是"从片驱动、主机接收"的方向，
     * 主机配成推挽输出就和 nRF 的 MISO 输出对推（数据错 + 输出级发烧）。
     * F1 参考手册里 SPI 主机模式的 MISO 就是浮空输入。
     */
    gpio.Pin   = NRF24_SPI_MISO_Pin;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(NRF24_SPI_MISO_GPIO_Port, &gpio);
}

/**
 * @brief  强制把 SPI2 的 SCK 限制在 10MHz 以内
 *
 * 【怎么保证 ≤ 10MHz】
 *  nRF24L01+ 的 SPI 时钟上限是 10MHz（datasheet 5.2 节），超了会读错寄存器。
 *  main.c 里 MX_SPI2_Init() 写死的是 SPI_BAUDRATEPRESCALER_256，
 *  APB1 = 36MHz -> SCK = 36MHz/256 = 140.6kHz，本身已经远低于 10MHz。
 *  但 .ioc 可以被 CubeIDE 重新生成改掉分频系数，所以驱动**不依赖那个初值**：
 *  init() 里按 HAL_RCC_GetPCLK1Freq() 自己挑一个「PCLK1/分频 ≤ 10MHz」的分频，
 *  然后 F1 上改波特率必须先关 SPI：
 *      __HAL_SPI_DISABLE() -> 改 CR1 的 BR 位 -> __HAL_SPI_ENABLE()
 *  （SPI_CR1.SPE = 1 的时候不允许改 BR 位）
 *  36MHz / 4 = 9MHz ≤ 10MHz，所以本工程实际跑 9MHz。
 *
 * @retval 1:成功  0:失败
 */
static uint8_t nrf24_spi_ensure_speed(void)
{
    static const struct {
        uint32_t hal;    /* HAL 的分频常量（就是 CR1 的 BR 位） */
        uint32_t div;    /* 实际分频倍数 */
    } tbl[] = {
        { SPI_BAUDRATEPRESCALER_2,   2UL },
        { SPI_BAUDRATEPRESCALER_4,   4UL },
        { SPI_BAUDRATEPRESCALER_8,   8UL },
        { SPI_BAUDRATEPRESCALER_16,  16UL },
        { SPI_BAUDRATEPRESCALER_32,  32UL },
        { SPI_BAUDRATEPRESCALER_64,  64UL },
        { SPI_BAUDRATEPRESCALER_128, 128UL },
        { SPI_BAUDRATEPRESCALER_256, 256UL }
    };
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t hal = SPI_BAUDRATEPRESCALER_256;
    uint32_t div = 256UL;
    uint32_t i;

    for (i = 0U; i < (sizeof(tbl) / sizeof(tbl[0])); ++i) {
        if ((pclk1 / tbl[i].div) <= NRF24_SPI_MAX_HZ) {
            hal = tbl[i].hal;
            div = tbl[i].div;
            break;
        }
    }

    /* F1：SPE=1 时不能改波特率分频位，必须先关 SPI */
    __HAL_SPI_DISABLE(&hspi2);
    MODIFY_REG(hspi2.Instance->CR1, SPI_CR1_BR_Msk, hal);
    __HAL_SPI_ENABLE(&hspi2);
    /* 句柄里的 Init 同步改掉，免得以后谁再调 HAL_SPI_Init 又被改回去 */
    hspi2.Init.BaudRatePrescaler = hal;

    dbg_printf("[NRF24] SPI2 SCK=%lu (PCLK1=%lu/%lu, max 10MHz)\r\n",
               (unsigned long)(pclk1 / div), (unsigned long)pclk1, (unsigned long)div);
    return 1U;
}

/**
 * @brief  一帧 SPI 收发（命令字节和数据在同一个 CSN 低电平里）
 * @param  tx: 发送缓冲
 * @param  rx: 接收缓冲
 * @param  len: 字节数
 * @retval 1:成功  0:失败
 */
static uint8_t nrf24_spi_xfer(uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if ((tx == NULL) || (rx == NULL) || (len == 0U)) {
        return 0U;
    }
    if (hspi2.State != HAL_SPI_STATE_READY) {
        return 0U;
    }
    if (HAL_SPI_TransmitReceive(&hspi2, tx, rx, len, NRF24_SPI_TIMEOUT_MS) != HAL_OK) {
        return 0U;
    }
    return 1U;
}

/**
 * @brief  读一个寄存器
 * @retval 1:成功  0:失败
 */
uint8_t nrf24_read_reg(uint8_t reg, uint8_t *val)
{
    uint8_t tx[2];
    uint8_t rx[2];

    if (val == NULL) {
        return 0U;
    }
    tx[0] = (uint8_t)(NRF24_CMD_READ | (reg & 0x1FU));   /* 寄存器地址只有 5 位 */
    tx[1] = NRF24_NOP;

    nrf24_csn(1U);
    if (nrf24_spi_xfer(tx, rx, 2U) == 0U) {
        nrf24_csn(0U);
        return 0U;
    }
    nrf24_csn(0U);

    *val = rx[1];
    return 1U;
}

/**
 * @brief  写一个寄存器
 * @retval 1:成功  0:失败
 */
static uint8_t nrf24_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = (uint8_t)(NRF24_CMD_WRITE | (reg & 0x1FU));
    tx[1] = val;

    nrf24_csn(1U);
    if (nrf24_spi_xfer(tx, rx, 2U) == 0U) {
        nrf24_csn(0U);
        return 0U;
    }
    nrf24_csn(0U);
    return 1U;
}

/**
 * @brief  写多个字节到寄存器
 * @retval 1:成功  0:失败
 */
static uint8_t nrf24_write_reg_bytes(uint8_t reg, const uint8_t *data, uint8_t len)
{
    uint8_t tx[1U + NRF24_ADDR_LEN];
    uint8_t rx[1U + NRF24_ADDR_LEN];
    uint8_t i;

    if ((data == NULL) || (len == 0U) || (len > NRF24_ADDR_LEN)) {
        return 0U;
    }
    tx[0] = (uint8_t)(NRF24_CMD_WRITE | (reg & 0x1FU));
    for (i = 0U; i < len; ++i) {
        tx[1U + i] = data[i];
    }

    nrf24_csn(1U);
    if (nrf24_spi_xfer(tx, rx, (uint16_t)(1U + len)) == 0U) {
        nrf24_csn(0U);
        return 0U;
    }
    nrf24_csn(0U);
    return 1U;
}

/**
 * @brief  读多个字节（寄存器）
 * @retval 1:成功  0:失败
 */
static uint8_t nrf24_read_reg_bytes(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t tx[1U + NRF24_ADDR_LEN];
    uint8_t rx[1U + NRF24_ADDR_LEN];
    uint8_t i;

    if ((data == NULL) || (len == 0U) || (len > NRF24_ADDR_LEN)) {
        return 0U;
    }
    tx[0] = (uint8_t)(NRF24_CMD_READ | (reg & 0x1FU));
    for (i = 0U; i < len; ++i) {
        tx[1U + i] = NRF24_NOP;
    }

    nrf24_csn(1U);
    if (nrf24_spi_xfer(tx, rx, (uint16_t)(1U + len)) == 0U) {
        nrf24_csn(0U);
        return 0U;
    }
    nrf24_csn(0U);

    for (i = 0U; i < len; ++i) {
        data[i] = rx[1U + i];
    }
    return 1U;
}

/**
 * @brief  发一条 SPI 命令（FLUSH_TX / FLUSH_RX 等）
 * @retval 1:成功  0:失败
 */
static uint8_t nrf24_cmd(uint8_t cmd)
{
    uint8_t tx[1];
    uint8_t rx[1];

    tx[0] = cmd;

    nrf24_csn(1U);
    if (nrf24_spi_xfer(tx, rx, 1U) == 0U) {
        nrf24_csn(0U);
        return 0U;
    }
    nrf24_csn(0U);
    return 1U;
}

/**
 * @brief  切到接收模式（PRX）：CE 拉高开始监听
 * @retval 1:成功  0:失败
 */
static uint8_t nrf24_enter_rx(void)
{
    nrf24_ce(0U);
    /* 掉电状态下换地址，再上电 */
    if (nrf24_write_reg(NRF24_REG_CONFIG, NRF24_CONFIG_EN_CRC | NRF24_CONFIG_CRCO) == 0U) {
        return 0U;
    }
    if (nrf24_write_reg_bytes(NRF24_REG_RX_ADDR_P0, s_rx_addr, NRF24_ADDR_LEN) == 0U) {
        return 0U;
    }
    if (nrf24_write_reg(NRF24_REG_STATUS, 0x70U) == 0U) {   /* 清 RX_DR/TX_DS/MAX_RT */
        return 0U;
    }
    (void)nrf24_cmd(NRF24_CMD_FLUSH_RX);
    (void)nrf24_cmd(NRF24_CMD_FLUSH_TX);

    if (nrf24_write_reg(NRF24_REG_CONFIG,
                         NRF24_CONFIG_EN_CRC | NRF24_CONFIG_CRCO |
                         NRF24_CONFIG_PWR_UP | NRF24_CONFIG_PRIM_RX) == 0U) {
        return 0U;
    }
    nrf24_delay_ms(NRF24_PWRUP_DELAY_MS);      /* 等晶振起振 1.5ms */

    nrf24_ce(1U);                              /* 开始监听 */
    nrf24_delay_us(NRF24_CE_RX_START_US);

    s_mode = NRF24_MODE_RX;
    return 1U;
}

/* ===================== 对外 API ===================== */

/**
 * @brief  初始化 nRF24L01+
 * @retval 1:成功  0:失败（原因已打到串口）
 */
uint8_t nrf24_init(void)
{
    static const uint8_t def_addr[NRF24_ADDR_LEN] = NRF24_DEFAULT_ADDR;
    uint8_t status = 0U;
    uint8_t cfg = 0U;
    uint8_t ch = 0U;
    uint8_t rf = 0U;
    uint8_t pw = 0U;
    uint8_t addr_rb[NRF24_ADDR_LEN];
    uint8_t raw_e9 = 0U;

    s_inited = 0U;
    s_mode = NRF24_MODE_RX;

    memcpy(s_rx_addr, def_addr, NRF24_ADDR_LEN);
    memcpy(s_tx_addr, def_addr, NRF24_ADDR_LEN);
    s_channel = NRF24_DEFAULT_CHANNEL;

    nrf24_gpio_init();
    if (nrf24_spi_ensure_speed() == 0U) {
        dbg_printf("[NRF24] init fail: spi busy\r\n");
        return 0U;
    }

    /* ---- 先确认芯片在不应答之外 ----
     * 【为什么单列这一步】板② 2026-11-02 实测：开机第一次读寄存器永远是 0x00，
     * 同一个模块稍后再读就正常 —— 模块刚上电时 3V3 还在爬、晶振还没起稳，
     * 这时候一次性判死会把好芯片说成坏的。
     * SETUP_AW 复位值就是 3（5 字节地址），写 3 再读回 3 是代价最小的"活着吗"探针。
     * 读不回来就直接失败退出，让上层按周期重试，而不是带着一个假初始化的状态继续跑
     * （后面每次收发都会静默失败，比报错更难查）。 */
    {
        uint8_t aw = 0U;
        uint8_t k;
        for (k = 0U; k < 5U; k++) {
            (void)nrf24_write_reg(NRF24_REG_SETUP_AW, 0x03U);
            if (nrf24_read_reg(NRF24_REG_SETUP_AW, &aw) == 0U) {
                aw = 0U;
            }
            if (aw == 0x03U) {
                break;
            }
            nrf24_delay_ms(20U);
        }
        if (aw != 0x03U) {
            /* 提示语保持短：app 区只剩几百字节，中文一条 3 字节。
             * 该查什么写在上面的注释里，日志只报数值。 */
            dbg_printf("[NRF24] no reply AW=0x%02X want 0x03\r\n", (unsigned)aw);
            return 0U;
        }
    }

    /* ---- 掉电状态下把所有寄存器配好 ---- */
    (void)nrf24_write_reg(NRF24_REG_CONFIG, NRF24_CONFIG_EN_CRC | NRF24_CONFIG_CRCO);
    (void)nrf24_write_reg(NRF24_REG_EN_RXADDR, 0x01U);      /* 只用 pipe 0 */
    (void)nrf24_write_reg(NRF24_REG_EN_AA, 0x01U);          /* pipe 0 自动应答 */
    (void)nrf24_write_reg(NRF24_REG_SETUP_AW, 0x03U);      /* 5 字节地址 */
    (void)nrf24_write_reg(NRF24_REG_SETUP_RETR, 0x15U);     /* ARD=500us, ARC=5 */
    (void)nrf24_write_reg(NRF24_REG_RF_CH, NRF24_DEFAULT_CHANNEL);
    (void)nrf24_write_reg(NRF24_REG_RF_SETUP, NRF24_RF_SETUP_1M_0DBM);
    (void)nrf24_write_reg_bytes(NRF24_REG_RX_ADDR_P0, s_rx_addr, NRF24_ADDR_LEN);
    (void)nrf24_write_reg_bytes(NRF24_REG_TX_ADDR, s_tx_addr, NRF24_ADDR_LEN);
    (void)nrf24_write_reg(NRF24_REG_RX_PW_P0, NRF24_PAYLOAD_SIZE);
    (void)nrf24_write_reg(NRF24_REG_STATUS, 0x70U);         /* 清 RX_DR/TX_DS/MAX_RT */
    (void)nrf24_cmd(NRF24_CMD_FLUSH_RX);
    (void)nrf24_cmd(NRF24_CMD_FLUSH_TX);

    /* ---- 读回校验：SPI 接线/供电/芯片有问题的话这里就会对不上 ---- */
    if ((nrf24_read_reg(NRF24_REG_CONFIG, &cfg) == 0U) ||
        (nrf24_read_reg(NRF24_REG_RF_CH, &ch) == 0U) ||
        (nrf24_read_reg(NRF24_REG_RF_SETUP, &rf) == 0U) ||
        (nrf24_read_reg(NRF24_REG_RX_PW_P0, &pw) == 0U) ||
        (nrf24_read_reg(NRF24_REG_STATUS, &status) == 0U)) {
        dbg_printf("[NRF24] init fail: spi read error (CSN/CE/MISO 接线?)\r\n");
        return 0U;
    }

    if ((cfg & 0x0FU) != (NRF24_CONFIG_EN_CRC | NRF24_CONFIG_CRCO)) {
        dbg_printf("[NRF24] init fail: CONFIG readback=0x%02X (want 0x0C)\r\n", (unsigned)cfg);
        return 0U;
    }
    if (ch != NRF24_DEFAULT_CHANNEL) {
        dbg_printf("[NRF24] init fail: RF_CH readback=%u (want %u)\r\n",
                   (unsigned)ch, (unsigned)NRF24_DEFAULT_CHANNEL);
        return 0U;
    }
    if (rf != NRF24_RF_SETUP_1M_0DBM) {
        dbg_printf("[NRF24] init fail: RF_SETUP readback=0x%02X (want 0x%02X)\r\n",
                   (unsigned)rf, (unsigned)NRF24_RF_SETUP_1M_0DBM);
        return 0U;
    }
    if (pw != NRF24_PAYLOAD_SIZE) {
        dbg_printf("[NRF24] init fail: RX_PW_P0 readback=%u (want %u)\r\n",
                   (unsigned)pw, (unsigned)NRF24_PAYLOAD_SIZE);
        return 0U;
    }

    /* 读回接收地址 */
    if (nrf24_read_reg_bytes(NRF24_REG_RX_ADDR_P0, addr_rb, NRF24_ADDR_LEN) == 0U) {
        dbg_printf("[NRF24] init fail: read RX_ADDR_P0 error\r\n");
        return 0U;
    }

    /*
     * 关于「读 0xE9」：nRF24L01+ 的寄存器地址字段只有 5 位（0x00~0x1F），
     * 0xE9 & 0x1F = 0x09 = CURRENT（当前频段/模式），不是一个独立寄存器；
     * 0xE9 这个字节值本身是 SPI 命令 R_ACK_PAYLOAD。
     * 真正的芯片自检靠上面「把刚写的寄存器读回来对比」。
     */
    (void)nrf24_read_reg(0xE9U, &raw_e9);

    dbg_printf("[NRF24] ST=0x%02X CFG=0x%02X CH=%u RF=0x%02X PW=%u raw=0x%02X\r\n",
               (unsigned)status, (unsigned)cfg, (unsigned)ch, (unsigned)rf, (unsigned)pw, (unsigned)raw_e9);
    dbg_printf("[NRF24] RX_ADDR = %c%c%c%c%c\r\n",
               addr_rb[0], addr_rb[1], addr_rb[2], addr_rb[3], addr_rb[4]);

    if (nrf24_enter_rx() == 0U) {
        dbg_printf("[NRF24] init fail: enter rx mode\r\n");
        return 0U;
    }

    s_inited = 1U;
    dbg_printf("[NRF24] init ok, payload=%u bytes, addr=NODE1\r\n", (unsigned)NRF24_PAYLOAD_SIZE);
    return 1U;
}

/**
 * @brief  射频是否已经初始化成功（供上层决定要不要周期性重试）
 * @retval 1:已就绪  0:还没就绪
 */
uint8_t nrf24_inited(void)
{
    return s_inited;
}

/**
 * @brief  切到发送模式（PTX）
 * @retval 1:成功  0:失败
 */
uint8_t nrf24_begin_tx(void)
{
    if (s_inited == 0U) {
        return 0U;
    }

    nrf24_ce(0U);
    /* 先掉电再改寄存器（datasheet：PRIM_RX 只能在掉电时改） */
    (void)nrf24_write_reg(NRF24_REG_CONFIG, NRF24_CONFIG_EN_CRC | NRF24_CONFIG_CRCO);
    (void)nrf24_write_reg(NRF24_REG_SETUP_AW, 0x03U);       /* 5 字节地址 */
    (void)nrf24_write_reg(NRF24_REG_EN_AA, 0x01U);          /* pipe 0 自动应答 */
    (void)nrf24_write_reg(NRF24_REG_EN_RXADDR, 0x01U);
    (void)nrf24_write_reg(NRF24_REG_SETUP_RETR, 0x15U);      /* ARD=500us, ARC=5 */
    (void)nrf24_write_reg(NRF24_REG_RF_CH, s_channel);
    (void)nrf24_write_reg(NRF24_REG_RF_SETUP, NRF24_RF_SETUP_1M_0DBM);

    /* 写 TX_ADDR */
    (void)nrf24_write_reg_bytes(NRF24_REG_TX_ADDR, s_tx_addr, NRF24_ADDR_LEN);
    /* 自动应答要从 pipe 0 收 ACK，所以 pipe 0 的地址必须和发送地址一致 */
    (void)nrf24_write_reg_bytes(NRF24_REG_RX_ADDR_P0, s_tx_addr, NRF24_ADDR_LEN);

    (void)nrf24_write_reg(NRF24_REG_STATUS, 0x70U);
    (void)nrf24_cmd(NRF24_CMD_FLUSH_RX);
    (void)nrf24_cmd(NRF24_CMD_FLUSH_TX);

    /* 上电（PRIM_RX = 0 -> PTX），等晶振起振 */
    (void)nrf24_write_reg(NRF24_REG_CONFIG,
                          NRF24_CONFIG_EN_CRC | NRF24_CONFIG_CRCO | NRF24_CONFIG_PWR_UP);
    nrf24_delay_ms(NRF24_PWRUP_DELAY_MS);

    s_mode = NRF24_MODE_TX;
    return 1U;
}

/**
 * @brief  切回接收模式（PRX）
 * @retval 1:成功  0:失败
 */
uint8_t nrf24_end_tx(void)
{
    if (s_inited == 0U) {
        return 0U;
    }
    return nrf24_enter_rx();
}

/**
 * @brief  进入掉电模式（Power-Down）
 * @note   CONFIG 的 POWER_UP 位清零就是掉电，~26µA（RX 模式 ~12mA）。
 *         nrf24_begin_tx() 本来就是「先掉电改寄存器再上电」，所以随时能掉。
 */
uint8_t nrf24_power_down(void)
{
    if (s_inited == 0U) {
        return 0U;
    }
    if (nrf24_write_reg(NRF24_REG_CONFIG, NRF24_CONFIG_EN_CRC | NRF24_CONFIG_CRCO) == 0U) {
        return 0U;
    }
    s_mode = NRF24_MODE_DOWN;
    return 1U;
}

/**
 * @brief  从掉电唤醒并进 RX
 * @note   掉电前收到的包会留在 FIFO 里，唤醒后先冲掉，只听新的。
 */
uint8_t nrf24_power_up_rx(void)
{
    if (s_inited == 0U) {
        return 0U;
    }
    if (s_mode == NRF24_MODE_RX) {
        return 1U;      /* 已经在 RX，别冲 FIFO（否则会丢掉刚收到的包） */
    }
    (void)nrf24_cmd(NRF24_CMD_FLUSH_RX);
    (void)nrf24_write_reg(NRF24_REG_STATUS, 0x70U);   /* 清 RX_DR/TX_DS/MAX_RT */
    return nrf24_enter_rx();
}

/**
 * @brief  发送一包数据
 * @retval 1:成功  0:失败
 */
uint8_t nrf24_write(const uint8_t *buf, uint8_t len)
{
    uint8_t tx[1U + NRF24_PAYLOAD_SIZE];
    uint8_t rx[1U + NRF24_PAYLOAD_SIZE];
    uint8_t status = 0U;
    uint32_t t0;

    if ((buf == NULL) || (s_inited == 0U) || (s_mode != NRF24_MODE_TX)) {
        return 0U;
    }
    if ((len == 0U) || (len > NRF24_PAYLOAD_SIZE)) {
        return 0U;
    }

    /* 清掉上一次的 TX_DS / MAX_RT（STATUS 写 1 清零） */
    (void)nrf24_write_reg(NRF24_REG_STATUS, NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);

    /* 固定载荷模式：接收端 RX_PW_P0 = 32，所以这里必须写满 32 字节，不足补 0 */
    tx[0] = NRF24_CMD_W_TX_PAYLOAD;
    memset(&tx[1U], 0, NRF24_PAYLOAD_SIZE);
    memcpy(&tx[1U], buf, len);

    nrf24_csn(1U);
    if (nrf24_spi_xfer(tx, rx, (uint16_t)(1U + NRF24_PAYLOAD_SIZE)) == 0U) {
        nrf24_csn(0U);
        return 0U;
    }
    nrf24_csn(0U);

    /* CE 拉高 >= 10us（datasheet）触发一次发送，之后 CE 可以拉低，射频自己会发完并等 ACK */
    nrf24_ce(1U);
    nrf24_delay_us(NRF24_CE_PULSE_US);
    nrf24_ce(0U);

    /* 等 TX_DS（发成功）或者 MAX_RT（重传耗尽） */
    t0 = HAL_GetTick();
    for (;;) {
        if (nrf24_read_reg(NRF24_REG_STATUS, &status) == 0U) {
            return 0U;
        }
        if ((status & NRF24_STATUS_TX_DS) != 0U) {
            (void)nrf24_write_reg(NRF24_REG_STATUS, NRF24_STATUS_TX_DS);
            return 1U;
        }
        if ((status & NRF24_STATUS_MAX_RT) != 0U) {
            (void)nrf24_write_reg(NRF24_REG_STATUS, NRF24_STATUS_MAX_RT);
            (void)nrf24_cmd(NRF24_CMD_FLUSH_TX);
            return 0U;
        }
        if ((HAL_GetTick() - t0) > NRF24_TX_TIMEOUT_MS) {
            (void)nrf24_write_reg(NRF24_REG_STATUS, NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);
            (void)nrf24_cmd(NRF24_CMD_FLUSH_TX);
            return 0U;
        }
        nrf24_delay_ms(1U);      /* 让出 CPU，别忙等 */
    }
}

/**
 * @brief  接收缓冲里有没有数据
 * @retval 1:有  0:没有
 */
uint8_t nrf24_available(void)
{
    uint8_t fifo = 0U;

    if ((s_inited == 0U) || (s_mode != NRF24_MODE_RX)) {
        return 0U;
    }
    if (nrf24_read_reg(NRF24_REG_FIFO_STATUS, &fifo) == 0U) {
        return 0U;
    }
    return ((fifo & NRF24_FIFO_RX_EMPTY) == 0U) ? 1U : 0U;
}

/**
 * @brief  取一包数据
 * @retval 32:取到  0:没有
 */
uint8_t nrf24_read(uint8_t *buf)
{
    uint8_t tx[1U + NRF24_PAYLOAD_SIZE];
    uint8_t rx[1U + NRF24_PAYLOAD_SIZE];
    uint8_t i;

    if ((buf == NULL) || (s_inited == 0U) || (s_mode != NRF24_MODE_RX)) {
        return 0U;
    }
    if (nrf24_available() == 0U) {
        return 0U;
    }

    tx[0] = NRF24_CMD_R_RX_PAYLOAD;
    for (i = 1U; i <= NRF24_PAYLOAD_SIZE; ++i) {
        tx[i] = NRF24_NOP;
    }

    nrf24_csn(1U);
    if (nrf24_spi_xfer(tx, rx, (uint16_t)(1U + NRF24_PAYLOAD_SIZE)) == 0U) {
        nrf24_csn(0U);
        return 0U;
    }
    nrf24_csn(0U);

    memcpy(buf, &rx[1U], NRF24_PAYLOAD_SIZE);
    /* 按 datasheet：读完 payload 要清 RX_DR */
    (void)nrf24_write_reg(NRF24_REG_STATUS, NRF24_STATUS_RX_DR);

    return NRF24_PAYLOAD_SIZE;
}

/**
 * @brief  设置信道
 * @retval 1:成功  0:失败
 */
uint8_t nrf24_set_channel(uint8_t ch)
{
    if (ch > NRF24_MAX_CHANNEL) {
        return 0U;
    }
    if (nrf24_write_reg(NRF24_REG_RF_CH, ch) == 0U) {
        return 0U;
    }
    s_channel = ch;
    return 1U;
}

/**
 * @brief  设置发送地址
 * @retval 1:成功  0:失败
 */
uint8_t nrf24_set_tx_addr(const uint8_t *addr)
{
    uint8_t rb[NRF24_ADDR_LEN];

    if (addr == NULL) {
        return 0U;
    }
    if (nrf24_write_reg_bytes(NRF24_REG_TX_ADDR, addr, NRF24_ADDR_LEN) == 0U) {
        return 0U;
    }
    if (nrf24_read_reg_bytes(NRF24_REG_TX_ADDR, rb, NRF24_ADDR_LEN) == 0U) {
        return 0U;
    }
    if (memcmp(rb, addr, NRF24_ADDR_LEN) != 0) {
        return 0U;
    }
    memcpy(s_tx_addr, addr, NRF24_ADDR_LEN);
    return 1U;
}

/**
 * @brief  设置接收地址（pipe 0）
 * @retval 1:成功  0:失败
 */
uint8_t nrf24_set_rx_addr(const uint8_t *addr)
{
    uint8_t rb[NRF24_ADDR_LEN];

    if (addr == NULL) {
        return 0U;
    }
    if (nrf24_write_reg_bytes(NRF24_REG_RX_ADDR_P0, addr, NRF24_ADDR_LEN) == 0U) {
        return 0U;
    }
    if (nrf24_read_reg_bytes(NRF24_REG_RX_ADDR_P0, rb, NRF24_ADDR_LEN) == 0U) {
        return 0U;
    }
    if (memcmp(rb, addr, NRF24_ADDR_LEN) != 0) {
        return 0U;
    }
    memcpy(s_rx_addr, addr, NRF24_ADDR_LEN);
    return 1U;
}

/**
 * @brief  读回接收地址
 * @retval 1:成功  0:失败
 */
uint8_t nrf24_get_rx_addr(uint8_t *addr)
{
    if (addr == NULL) {
        return 0U;
    }
    if (nrf24_read_reg_bytes(NRF24_REG_RX_ADDR_P0, addr, NRF24_ADDR_LEN) == 0U) {
        return 0U;
    }
    return 1U;
}

/**
 * @brief  读 STATUS
 * @retval 1:成功  0:失败
 */
uint8_t nrf24_get_status(uint8_t *status)
{
    return nrf24_read_reg(NRF24_REG_STATUS, status);
}

/**
 * @brief  IRQ 引脚是否为低
 * @retval 1:有事件  0:无事件
 */
uint8_t nrf24_irq_pending(void)
{
    return (HAL_GPIO_ReadPin(NRF24_IRQ_GPIO_Port, NRF24_IRQ_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
}
