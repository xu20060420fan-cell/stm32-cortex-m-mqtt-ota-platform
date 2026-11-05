/*
 * sensor_task.c
 *
 * AHT20 + BMP280 采集任务。两个传感器都挂在 I2C1（PB6=SCL、PB7=SDA）上。
 * 采样周期 1 秒；aht20_cluate 内部要等约 85ms（vTaskDelay，会让出 CPU），
 * 所以任务优先级给低一点。
 *
 * 传感器初始化失败不会让任务退出：两个驱动都会在运行期自动重试
 * （aht20_cluate / bmp280_sample 内部各有一段"没就绪就先初始化"的逻辑），
 * 这样传感器供电比 MCU 慢、或者热插拔，都能自己恢复。
 */
#include "sensor_task.h"

#include "aht20.h"
#include "bmp280.h"
#include "debug_log.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

extern I2C_HandleTypeDef hi2c1;

volatile sensor_data_t g_sensor;

#define SENSOR_PERIOD_MS 1000U   /* 1 秒采一次；数值变化由显示任务平滑滚动出来 */

/* ================================================================
 * I2C1 总线恢复（热插拔必需）
 *
 * 传感器从板上拔掉的瞬间，如果正好处在一次传输中途，从机会一直拉住 SDA
 * 不放；再加上 HAL 的 I2C 状态机可能停在 BUSY，之后每次调用都直接返回
 * HAL_BUSY —— 驱动层再怎么重试 init 也读不到数，屏幕就一直 ERR，
 * 插回去也不会自己好（用户实测的 bug）。
 *
 * 这里做标准的 9 时钟恢复：把 PB6/PB7 切成开漏 GPIO，打 9 个 SCL 时钟
 * 让从机把没发完的字节移出去，再补一个 STOP，最后重新 HAL_I2C_Init
 * （MspInit 会把引脚切回 AF_OD）。
 *
 * 只在连续多轮两个传感器都失败时才调用，正常工作一次都不做。
 * ================================================================ */
#define I2C_RECOVER_AFTER 5U   /* 连续 5 轮（约 5 秒）都失败才动总线 */

static void i2c1_bus_recover(void)
{
    GPIO_InitTypeDef g = {0};
    volatile uint32_t n;
    uint8_t i;

    HAL_I2C_DeInit(&hi2c1);

    __HAL_RCC_GPIOB_CLK_ENABLE();
    g.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
    for (n = 0U; n < 40U; n++) { }        /* 线上电平稳定（几十 us） */

    for (i = 0U; i < 9U; ++i) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        for (n = 0U; n < 40U; n++) { }
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        for (n = 0U; n < 40U; n++) { }
    }

    /* STOP：SCL 保持高电平期间 SDA 由低变高 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    for (n = 0U; n < 40U; n++) { }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    for (n = 0U; n < 40U; n++) { }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    for (n = 0U; n < 40U; n++) { }

    HAL_I2C_Init(&hi2c1);
    dbg_printf("[SEN] i2c1 bus recovered\r\n");
}

/* ================================================================
 * 电池电压（PA0 = ADC12_IN0，接线表 ①-10 的「BAT_ADC」）
 *
 * 【为什么这一段是裸寄存器而不是 HAL】
 *   这个工程只 vendor 了 .ioc 里启用的 HAL 模块 —— Drivers 下面根本没有
 *   stm32f1xx_hal_adc.c/.h，hal_conf.h 里的 HAL_ADC_MODULE_ENABLED 也是注释掉的。
 *   要补 ADC 就得在 CubeMX 里重开模块重生成一大片代码，而这个工程已经被
 *   "重生成把用户改动冲掉"咬过两次。F1 读一路 ADC 就那几个寄存器，
 *   所以直接写寄存器：不新增文件、不动 .ioc、重生成也冲不掉。
 *
 * 【接线表说"需分压（建议 1/2），确认分压比"】—— 分压比就是下面这一个数，
 *   上电后日志会打 [BAT] mv=…，拿万用表对一下电池实际电压，差多少就按比例
 *   改 BAT_DIV_X100，别改别的。
 * ================================================================ */
#define BAT_DIV_X100      200U            /* 分压比 ×100：1/2 分压 → 200 */
#define BAT_ADC_CH        0U              /* PA0 = 通道 0 */
#define BAT_VREF_MV       3300U           /* ADC 参考电压 = 3V3 */
#define BAT_SR_EOC        (1UL << 1)      /* RM0008 的 ADC_SR.EOC；本版 CMSIS 里叫 ADC_SR_EOS，同一个位 */
#define BAT_SAMPLE_EVERY  20U             /* 每 20 轮（≈20 秒）测一次，电压变得慢 */

/* 锂电池电压 → 电量 %。分段线性，比"从 3.3V 到 4.2V 拉一条直线"接近得多
 *（锂电曲线中间平、两头陡）。两张表都递增，最后一项是端点。 */
static const uint16_t s_lipo_mv[]  = {3300U, 3450U, 3600U, 3750U, 3900U, 4050U, 4200U};
static const uint8_t  s_lipo_pct[] = {  0U,   10U,   25U,   55U,   80U,   95U,  100U};
#define LIPO_POINTS ((uint8_t)(sizeof(s_lipo_mv) / sizeof(s_lipo_mv[0])))

static uint8_t s_batt_ready = 0U;

void battery_adc_init(void)
{
    GPIO_InitTypeDef g = {0};
    uint32_t guard;

    /* PA0 必须是模拟模式。留着数字输入（上电默认就是浮空输入）的话，
     * 读回来永远是 0 或满量程 —— 这是"ADC 数值不动"最常见的原因。 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    g.Pin  = GPIO_PIN_0;
    g.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &g);

    /* ADC 时钟 = PCLK2/8 = 72/8 = 9MHz（F1 的 ADC 上限 14MHz） */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_DIV8;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* 采样时间给最长（239.5 个 ADC 时钟 ≈ 26.6us @9MHz）：
     * 分压电阻源阻抗高，采样电容充不满读数就偏低。 */
    ADC1->SMPR2 = (ADC1->SMPR2 & ~ADC_SMPR2_SMP0) | (7UL << ADC_SMPR2_SMP0_Pos);
    ADC1->SQR1  = 0UL;                                   /* L=0 → 规则组只转 1 次 */
    ADC1->SQR3  = (BAT_ADC_CH << ADC_SQR3_SQ1_Pos);      /* 第 1 位 = 通道 0 */
    ADC1->CR1   = 0UL;                                   /* 不扫描、不开中断 */
    /* EXTSEL=111 → 外部触发关闭，只有 SWSTART 能启动转换。ALIGN=0 = 右对齐。 */
    ADC1->CR2   = (7UL << ADC_CR2_EXTSEL_Pos);

    ADC1->CR2 |= ADC_CR2_ADON;
    /* ADON 置起后要等稳定时间（tSTAB ≥1us）。这里在调度器启动前调用，
     * 还不能用 vTaskDelay，用空转顶过去。 */
    for (guard = 0; guard < 8000UL; guard++) { }

    /* 自校准（F1 特有）。CAL 位做完由硬件自己清零，所以等它清零 = 校准完成。 */
    ADC1->CR2 |= ADC_CR2_CAL;
    for (guard = 0; guard < 300000UL; guard++) {
        if ((ADC1->CR2 & ADC_CR2_CAL) == 0UL) {
            break;
        }
    }
    s_batt_ready = ((ADC1->CR2 & ADC_CR2_CAL) == 0UL) ? 1U : 0U;
}

/* 一次转换。返回 12 位原始值；超时返回 0 并就地放弃。 */
static uint16_t battery_raw_once(void)
{
    uint32_t guard;

    ADC1->CR2 |= ADC_CR2_SWSTART;
    for (guard = 0; guard < 300000UL; guard++) {
        if (ADC1->SR & BAT_SR_EOC) {
            (void)ADC1->DR;      /* 读 DR 同时清掉 EOC */
            return (uint16_t)(ADC1->DR & 0x0FFFU);
        }
    }
    return 0xFFFFU;               /* 转换没完成：ADC 没起来或时钟不对 */
}

uint16_t battery_mv(void)
{
    uint32_t sum = 0U;
    uint32_t pin_mv;
    uint8_t  k;
    uint16_t raw;

    if (s_batt_ready == 0U) {
        return 0U;
    }

    /* 连采 5 次取平均：ADC 噪声 + 电源纹波都靠这个压掉，比加滤波简单 */
    for (k = 0; k < 5U; k++) {
        raw = battery_raw_once();
        if (raw == 0xFFFFU) {
            s_batt_ready = 0U;
            return 0U;
        }
        sum += raw;
    }

    pin_mv = (sum / 5UL) * BAT_VREF_MV / 4095UL;
    return (uint16_t)((pin_mv * BAT_DIV_X100) / 100UL);
}

static uint8_t battery_percent(uint16_t mv)
{
    uint8_t i;

    if (mv <= s_lipo_mv[0]) {
        return 0U;
    }
    if (mv >= s_lipo_mv[LIPO_POINTS - 1U]) {
        return 100U;
    }
    for (i = 1U; i < LIPO_POINTS; i++) {
        if (mv <= s_lipo_mv[i]) {
            uint32_t span = (uint32_t)s_lipo_mv[i] - s_lipo_mv[i - 1U];
            uint32_t step = (uint32_t)s_lipo_pct[i] - s_lipo_pct[i - 1U];
            return (uint8_t)((uint32_t)s_lipo_pct[i - 1U] +
                             (uint32_t)(mv - s_lipo_mv[i - 1U]) * step / span);
        }
    }
    return 100U;
}

void sensor_task(void *pvParameters)
{
    float t_aht = 0.0f;
    float hum   = 0.0f;
    float t_bmp = 0.0f;
    float press = 0.0f;

    (void)pvParameters;

    /* ---- 上电初始化：各自独立，一个失败不影响另一个 ---- */
    if (aht20_init() != 0U) {
        dbg_printf("[SEN] AHT20 ready\r\n");
    } else {
        dbg_printf("[SEN] AHT20 not ready, will retry in loop\r\n");
    }

    if (bmp280_init() != 0U) {
        if (bmp280corect() != 0U) {
            dbg_printf("[SEN] BMP280 ready\r\n");
        } else {
            dbg_printf("[SEN] BMP280 calib failed, will retry in loop\r\n");
        }
    } else {
        dbg_printf("[SEN] BMP280 not found, will retry in loop\r\n");
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t    batt_divider  = 0U;
    uint8_t    batt_logged   = 0U;
    uint8_t    fail_rounds   = 0U;   /* 连续「两颗都读不到」的轮数 */

    for (;;) {
        uint8_t aht_now = 0U;
        uint8_t bmp_now = 0U;

        if (aht20_cluate(&t_aht, &hum) != 0U) {
            g_sensor.temperature = t_aht;
            g_sensor.humidity    = hum;
            g_sensor.aht_ok      = 1U;
            aht_now = 1U;
        } else {
            g_sensor.aht_ok      = 0U;
        }

        if (bmp280_sample(&t_bmp, &press) != 0U) {
            g_sensor.pressure = press;
            g_sensor.bmp_ok   = 1U;
            bmp_now = 1U;
        } else {
            g_sensor.bmp_ok   = 0U;
        }

        /* 两颗都连续读不到 → 大概率是总线被拉死（拔插的瞬间），
         * 做一次 9 时钟恢复；只要有一颗在出数就认为总线正常。 */
        if ((aht_now == 0U) && (bmp_now == 0U)) {
            if (++fail_rounds >= I2C_RECOVER_AFTER) {
                fail_rounds = 0U;
                i2c1_bus_recover();
            }
        } else {
            fail_rounds = 0U;
        }

        /* ---- 电池电压 ----
         * 电压变得慢，20 秒测一次就够（省 ADC 功耗，也省得分压电阻一直耗电）。
         * 上一次转换超时会把 s_batt_ready 清掉，这里顺带重新配一遍 ——
         * 和 AHT20/BMP280 一样，热插拔或上电顺序问题都能自己恢复。 */
        if ((batt_divider++) % BAT_SAMPLE_EVERY == 0U) {
            uint16_t mv;

            if (s_batt_ready == 0U) {
                battery_adc_init();
            }
            mv = battery_mv();
            if (mv != 0U) {
                g_sensor.batt    = battery_percent(mv);
                g_sensor.batt_ok = 1U;
            } else {
                g_sensor.batt_ok = 0U;
            }
            if (batt_logged == 0U && mv != 0U) {
                /* 第一次出数打一行，方便用万用表核对分压比 */
                dbg_printf("[BAT] %u mV -> %u%%\r\n",
                           (unsigned)mv, (unsigned)g_sensor.batt);
                batt_logged = 1U;
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}
