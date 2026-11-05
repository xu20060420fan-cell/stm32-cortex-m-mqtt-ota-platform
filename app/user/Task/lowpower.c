/*
 * lowpower.c —— STM32F103 低功耗：外设分时上电 + STOP 模式(可选)
 * ============================================================================
 * 设计依据：docs/00-总体方案与接线.md §7.1「低功耗与屏幕刷新」
 *
 * 【这个文件分两部分】
 *
 *   A. 外设分时上电（**始终编译**，省电的大头，零时序风险）
 *      · nRF24 收听窗口：只在窗口里开 RX，其余时间掉电（12mA → 26µA）
 *      · MPU6050 采样后进低功耗（3.5mA → 60µA）
 *      · OLED 空闲关屏（20mA → 10µA）
 *      → 详见 app_tasks.c / oled_home.c / nrf24l01.c / mpu6050.c
 *
 *   B. STOP 模式（**受 LP_ENABLE_STOP 开关控制，默认关**）
 *      · LSE + RTC 闹钟（1 秒周期）
 *      · 唤醒源：RTC 闹钟 / EXTI0(PC0 MPU INT) / EXTI1(PB1 nRF24 IRQ) / USART1 RX
 *      · STOP 期间丢失的 tick 怎么补 —— 见下面「tick 补偿」一节
 *      · 打开的代价与前提 —— 见 lowpower.h
 *
 * 【为什么 F103 的 STOP 只能按 1 秒粒度】
 *   F1 的 HAL **没有** 唤醒定时器 API（HAL_RTCEx_SetWakeupTimer 之类不存在），
 *   只有 RTC 闹钟，而闹钟只能在整秒边界触发。所以最小睡眠粒度是 1 秒。
 *
 * 【STOP 期间丢失的 tick 怎么补】—— 这一节是 B 部分最关键的地方
 *   STOP 里 SysTick 停摆，HAL 的 uwTick 和 FreeRTOS 的 xTickCount 都不再走。
 *   本工程的做法是：
 *
 *     · 每次都从 RTC 闹钟醒来（闹钟在整秒边界触发，周期精确 1 秒）；
 *     · 醒来时 **一次性把 tick 补 1000ms**：
 *         - HAL 的 uwTick：循环调 HAL_IncTick()（F1 的 HAL_IncTick 每次 +1ms）
 *         - FreeRTOS 的 tick：xTaskCatchUpTicks(100)  （100Hz → 100 tick）
 *     · 因为「一次闹钟周期 = 真实 1000ms」，所以这个补偿是 **精确** 的，
 *       不累积误差，也不受 sleep 实际长短影响。
 *     · EXTI / USART 唤醒（周期中间的事件）**不动 tick** —— 它们只提前干活，
 *       不改变闹钟节拍，tick 仍然跟着闹钟走。
 *
 *   注：FreeRTOS 还有个 vTaskStepTick()，但它带 configASSERT(新 tick <=
 *   下一个任务解锁时刻)，一次跳 1000ms 会直接断言失败，所以用
 *   xTaskCatchUpTicks()（它就是为「补过期的 tick」设计的）。
 *
 * 【为什么时钟要恢复】
 *   STOP 醒来后 SYSCLK 退回 HSI 8MHz，PLL 是关的。要恢复 72MHz 必须重新
 *   起 HSE + PLL。这里做了 **HSI 兜底**：万一 HSE 起不来（比如晶振没焊好），
 *   退到 HSI×2 = 8MHz 继续跑，而不是死在 Error_Handler() 里让节点变砖。
 *
 * 【RTC 驱动的来历】
 *   本工程的 HAL 包里只收了 CubeMX 用到的模块，没有 RTC。
 *   HAL_RTC 驱动来自 stm32ota 工程的同一份实现
 *   （同一套 STM32CubeF1 V1.8.x；F103 的 RTC 就是 F2/F4 那套
 *   PRLH/PRLL/DIV 寄存器，完全兼容），已放进 Drivers/STM32F1xx_HAL_Driver/。
 *
 * ============================================================================
 */
#include "lowpower.h"

#include "main.h"
#include "debug_log.h"

#include "FreeRTOS.h"
#include "task.h"

/* ==========================================================================
 * A. EXTI 唤醒源（始终编译）
 *    PC0 = MPU6050 的 INT 脚；PB1 = nRF24L01 的 IRQ 脚
 *    这两个脚在 STOP 模式下是唤醒源，平时也是普通中断源。
 * ========================================================================== */

static volatile uint8_t s_exti_flag;
static volatile uint8_t s_uart_flag;
static volatile uint32_t s_block;
static volatile uint32_t s_sleep_cnt;
static volatile uint32_t s_wake_cnt[4];

/**
 * 配置两个 EXTI 唤醒源。
 * @note  F1 的 EXTI 源选择是在 **AFIO->EXTICR** 里（不是 F2/F4 的 SYSCFG）
 */
void lp_init_exti(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_AFIO_CLK_ENABLE();

    /* PA15 → EXTI15：跌落/碰撞/数据就绪 */
    gpio.Pin  = MPU_INT_Pin;
    gpio.Mode = GPIO_MODE_IT_FALLING;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(MPU_INT_GPIO_Port, &gpio);

    /* PB1 → EXTI1：下降沿触发 */
    gpio.Pin  = NRF24_IRQ_Pin;
    gpio.Mode = GPIO_MODE_IT_FALLING;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(NRF24_IRQ_GPIO_Port, &gpio);

    /* EXTI 源映射：EXTI15=PA15（在 EXTICR4），EXTI1=PB1（在 EXTICR1）
     * F1 的 EXTI 源选择器在 AFIO->EXTICR，不是 F2/F4 的 SYSCFG */
    AFIO->EXTICR[3] &= ~AFIO_EXTICR4_EXTI15_Msk;
    AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI15_PA;
    AFIO->EXTICR[0] &= ~AFIO_EXTICR1_EXTI1_Msk;
    AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PB;

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2U, 0U);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 2U, 0U);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

void lp_block(void)
{
    s_block++;
}

void lp_unblock(void)
{
    if (s_block > 0U) {
        s_block--;
    }
}

uint8_t lp_can_sleep(void)
{
    return (s_block == 0U) ? 1U : 0U;
}

uint32_t lp_sleep_count(void)
{
    return s_sleep_cnt;
}

uint32_t lp_wake_count(uint8_t reason)
{
    if (reason < 4U) {
        return s_wake_cnt[reason];
    }
    return 0U;
}

uint8_t lp_exti_pending(void)
{
    return s_exti_flag;
}

void lp_exti_clear(void)
{
    s_exti_flag = 0U;
}

/* ==========================================================================
 * B. STOP 模式（LP_ENABLE_STOP 控制）
 * ========================================================================== */

#if LP_ENABLE_STOP

/* RTC 句柄：中断处理要用，放全局（stm32f1xx_it.c 里引用） */
RTC_HandleTypeDef hrtc;

static volatile uint8_t s_alarm_flag;
static volatile uint8_t s_wake_reason;

/**
 * 从 STOP 醒来后把系统时钟拉回 72MHz。
 * HSE 起不来时退到 HSI×2（8MHz）继续跑，绝不把节点卡死。
 */
static void lp_restore_clock(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    /* ⚠ STOP 唤醒后有两个坑：
     *   ① PLL 硬件是关着的，但 RCC->CFGR 里还写着「系统时钟 = PLLCLK」。
     *      HAL_RCC_OscConfig 看到「配置没变」会直接返回 OK，**根本不会重启 PLL**。
     *      所以先把系统时钟切回 HSI，逼它走「PLL 不是当前系统时钟」的分支，
     *      那条分支会老老实实 disable → 重配 → enable → 等 PLLRDY。
     *   ② F1 的 PLL 段是靠 osc.PLL.PLLState 判断的，不是靠 OscillatorType 掩码，
     *      所以 OscillatorType 里不用（也没有）RCC_OSCILLATORTYPE_PLL。
     */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_SYSCLKSOURCE_HSI);
    while (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_HSI) {
        /* 等切到 HSI */
    }

    osc.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState        = RCC_HSE_ON;
    osc.HSEPredivValue  = RCC_HSE_PREDIV_DIV1;
    osc.PLL.PLLState    = RCC_PLL_ON;
    osc.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL      = RCC_PLL_MUL9;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        /* HSE 没起来（晶振没焊/起振失败）→ 退到 HSI×2 = 8MHz */
        dbg_puts("[LP] HSE fail, fallback HSI 8MHz\r\n");

        osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
        osc.HSEState       = RCC_HSE_OFF;
        osc.HSIState       = RCC_HSI_ON;
        osc.PLL.PLLState   = RCC_PLL_ON;
        osc.PLL.PLLSource  = RCC_PLLSOURCE_HSI_DIV2;
        osc.PLL.PLLMUL     = RCC_PLL_MUL2;

        if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
            dbg_puts("[LP] HSI fail!\r\n");
            return;
        }
    }

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;

    (void)HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2);
}

/**
 * 把 STOP 期间丢掉的 tick 补回来。
 * @param ms 丢失的毫秒数（一个闹钟周期固定 1000）
 */
static void lp_catch_up_ticks(uint32_t ms)
{
    uint32_t i;
    TickType_t ticks;

    /* HAL 的 uwTick：F1 的 HAL_IncTick() 每次 +1ms（uwTickFreq 默认 1kHz） */
    for (i = 0U; i < ms; ++i) {
        HAL_IncTick();
    }

    /* FreeRTOS 的 tick：100Hz → ms/10 个 tick */
    ticks = pdMS_TO_TICKS(ms);
    if (ticks != 0U) {
        (void)xTaskCatchUpTicks(ticks);
    }
}

/** 读当前 RTC 时间，并算出「下一秒」的闹钟时刻 */
static void lp_arm_next_alarm(void)
{
    RTC_TimeTypeDef t = {0};
    RTC_AlarmTypeDef a = {0};
    uint32_t sec;

    if (HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN) != HAL_OK) {
        return;
    }

    sec = (uint32_t)t.Seconds + 1U;
    t.Seconds = (uint8_t)(sec % 60U);
    t.Minutes = (uint8_t)(t.Minutes + (sec / 60U));
    if (t.Minutes >= 60U) {
        t.Minutes = 0U;
        t.Hours = (uint8_t)((t.Hours + 1U) % 24U);
    }

    a.AlarmTime.Hours   = t.Hours;
    a.AlarmTime.Minutes = t.Minutes;
    a.AlarmTime.Seconds = t.Seconds;
    a.Alarm             = RTC_ALARM_A;

    (void)HAL_RTC_SetAlarm_IT(&hrtc, &a, RTC_FORMAT_BIN);
}

void lp_init(void)
{
    RTC_TimeTypeDef t = {0};
    RTC_DateTypeDef d = {0};
    uint32_t timeout;

    /* --- EXTI 唤醒源（一直都要） --- */
    lp_init_exti();

    /* --- 1. 备份域 + LSE + RTC 时钟 ---
     * ⚠ 这里 **不能** 用 HAL_RCC_OscConfig()：F1 的 HAL 会把「没点名」的
     *   振荡器和 PLL 全部关掉（只传 LSE → HSE/HSI/PLL 全灭 → 72MHz 变 8MHz）。
     *   所以 LSE 直接写 RCC->BDCR，位定义用 CMSIS 设备头里的官方掩码。
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    /* F1 没有独立的 RTC 时钟使能位，RTC 的时钟由 BDCR.RTCSEL + RTCEN 决定 */
    HAL_PWR_EnableBkUpAccess();

    RCC->BDCR |= RCC_BDCR_LSEON;                 /* 开 LSE */
    timeout = 1000000U;
    while (((RCC->BDCR & RCC_BDCR_LSERDY) == 0U) && (timeout-- != 0U)) {
        /* 等 LSE 稳定 */
    }
    if ((RCC->BDCR & RCC_BDCR_LSERDY) == 0U) {
        dbg_puts("[LP] LSE not ready\r\n");
        return;
    }

    /* RTC 选 LSE 并使能 RTC */
    RCC->BDCR &= ~RCC_BDCR_RTCSEL_Msk;
    RCC->BDCR |= RCC_BDCR_RTCSEL_1;              /* 01 = LSE */
    RCC->BDCR |= RCC_BDCR_RTCEN;

    /* --- 2. RTC 初始化 --- */
    hrtc.Instance          = RTC;
    hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;   /* 1Hz 时基 */
    hrtc.Init.OutPut       = RTC_OUTPUTSOURCE_NONE;
    if (HAL_RTC_Init(&hrtc) != HAL_OK) {
        dbg_puts("[LP] RTC init failed\r\n");
        return;
    }

    /* 设一个基准时间（日历用不到，但 RTC 的影子寄存器必须有合法值） */
    t.Hours = 0U; t.Minutes = 0U; t.Seconds = 0U;
    d.Year = 24U; d.Month = RTC_MONTH_JANUARY; d.Date = 1U;
    (void)HAL_RTC_SetTime(&hrtc, &t, RTC_FORMAT_BIN);
    (void)HAL_RTC_SetDate(&hrtc, &d, RTC_FORMAT_BIN);

    /* --- 3. RTC 闹钟中断 --- */
    HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);

    dbg_puts("[LP] init ok: RTC(LSE 1Hz) + EXTI0(PC0) + EXTI1(PB1)\r\n");
}

void lp_enter(void)
{
    uint8_t reason;

    /* 武装下一秒的闹钟 */
    lp_arm_next_alarm();

    s_alarm_flag = 0U;

    /* 进 STOP。醒来后 SYSCLK = HSI 8MHz，PLL 是关的 */
    HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

    /* 恢复 72MHz（失败则退 HSI 8MHz） */
    lp_restore_clock();

    /* 判断是谁把咱们叫醒的 */
    if (s_alarm_flag != 0U) {
        reason = LP_WAKE_ALARM;
    } else if (s_exti_flag != 0U) {
        reason = LP_WAKE_EXTI;
    } else if (s_uart_flag != 0U) {
        reason = LP_WAKE_UART;
    } else {
        reason = LP_WAKE_NONE;
    }

    s_alarm_flag = 0U;
    s_exti_flag  = 0U;
    s_uart_flag  = 0U;
    s_wake_reason = reason;

    if (reason < 4U) {
        s_wake_cnt[reason]++;
    }
    s_sleep_cnt++;

    /*
     * 只有「闹钟周期」才补 tick —— 一个闹钟周期精确等于 1000ms，
     * 所以补 1000ms 是精确的。EXTI/USART 是周期中间的事件，
     * 提前干活但不动 tick，节拍仍然跟闹钟走，不会漂。
     */
    if (reason == LP_WAKE_ALARM) {
        lp_catch_up_ticks(LP_CYCLE_MS);
    }
}

uint8_t lp_wake_reason(void)
{
    return s_wake_reason;
}

/** RTC 闹钟到点 —— HAL 的回调 */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *h)
{
    (void)h;
    s_alarm_flag = 1U;
}

#else /* LP_ENABLE_STOP == 0 —— 空实现，保证调用点不用改 */

/* RTC 句柄仍要存在：stm32f1xx_it.c 的 RTC_Alarm_IRQHandler 会引用它 */
RTC_HandleTypeDef hrtc;

void lp_init(void)
{
    /* STOP 模式没开，只配 EXTI（PC0 MPU INT / PB1 nRF24 IRQ） */
    lp_init_exti();
}

void lp_enter(void)
{
    /* STOP 模式没开 */
}

uint8_t lp_wake_reason(void)
{
    return LP_WAKE_NONE;
}

#endif /* LP_ENABLE_STOP */

/* ==========================================================================
 * EXTI 回调（始终编译）
 * ========================================================================== */

/** EXTI 事件（PC0 = MPU INT，PB1 = nRF24 IRQ）—— HAL 的弱回调 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if ((GPIO_Pin == MPU_INT_Pin) || (GPIO_Pin == NRF24_IRQ_Pin)) {
        s_exti_flag = 1U;
    }
}
