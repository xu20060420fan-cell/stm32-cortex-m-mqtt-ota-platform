/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <ssd1306.h>
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"buzzer.h"
#include"buttom.h"
#include"main.h"
#include"oled_home.h"
#include"sensor_task.h"
#include"debug_log.h"
#include"task_monitor.h"
/* APP 版本号 + 给 bootloader 读的固定地址版本标签 */
#include "app_version.h"
/* 三节点方案的业务任务入口（由业务层实现，这里只负责调用） */
#include "app_tasks.h"
/* 低功耗：LSE + RTC 闹钟 + EXTI 唤醒源（STOP 模式） */
#include "lowpower.h"
/* 最小验证模式要直接驱动 AHT20 / BMP280 */
#include "aht20.h"
#include "bmp280.h"
/* 8266↔APP 的串口命令枢纽（TH 阈值 / OTA 触发 / TL 遥测行） */
#include "uart_cmd.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 最小验证模式：1 = 只留「USART2 日志 + OLED 显示 + AHT20/BMP280」，
 * 其它任务（nRF24 / 8266 / IMU / 告警 / 低功耗 …）一律不建。
 * 目的：把「固件能不能启动 / 屏幕亮不亮 / 传感器读不读得到」三件事一次验清，
 * 排除「加的东西太多把启动搞挂」这种可能。
 * 日志走 USART2（PA2），也就是板子上那个「调试UART 4P」口，插 USB-TTL 直接看。 */
#define MINIMAL_TEST 0
/* USER CODE END PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;   /* 传感器总线：AHT20 + BMP280 */
I2C_HandleTypeDef hi2c2;   /* OLED：SSD1306 */

SPI_HandleTypeDef hspi1;   /* W25Q128 外置 Flash：与 bootloader 同一路 SPI1/同一组引脚 */
SPI_HandleTypeDef hspi2;   /* NRF24L01+ 无线模块 */
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_usart1_tx;  /* 日志串口 DMA */

UART_HandleTypeDef huart1; /* 运行期日志 + bootloader ISP */
UART_HandleTypeDef huart2; /* ESP8266 下载器串口 */

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if MINIMAL_TEST
/* ================================================================
 * 最小验证模式（MINIMAL_TEST = 1）
 *
 * 目标：一次把三件事验清楚 —— ①固件能不能启动 ②屏幕亮不亮 ③传感器读不读得到。
 * 做法：只建一个任务（读传感器 + 刷屏），其它全部不初始化；
 *       日志走 USART2（PA2 / 板上「调试UART 4P」），插 USB-TTL 就能看，
 *       完全不碰 USART1（PA9/PA10 = ESP8266 的 OTA 通道）。
 * ================================================================ */

/* ---- 串口输出（阻塞，115200 8N1）----
 * 两个口都发：USART2（PA2，板上「调试UART 4P」口）+ USART1（PA9/PA10，接 ESP8266）。
 * 这样不管 USB-TTL 插在哪个头上都能抓到日志。 */
static void min_out(const char *s)
{
    uint16_t n;

    if (s == NULL) {
        return;
    }
    n = (uint16_t)strlen(s);
    (void)HAL_UART_Transmit(&huart2, (const uint8_t *)s, n, 500U);
    (void)HAL_UART_Transmit(&huart1, (const uint8_t *)s, n, 500U);
}

static void min_out_u32(uint32_t v)
{
    char     b[12];
    uint8_t  i = 11U;

    b[11] = '\0';
    if (v == 0U) {
        b[10] = '0';
        min_out(&b[10]);
        return;
    }
    while (v != 0U) {
        b[--i] = (char)('0' + (v % 10U));
        v /= 10U;
    }
    min_out(&b[i]);
}

/* ---- 字符串拼装（给 OLED 用）---- */
static char *ap_str(char *p, char *end, const char *s)
{
    while ((*s != '\0') && (p < (end - 1))) {
        *p++ = *s++;
    }
    *p = '\0';
    return p;
}

static char *ap_int(char *p, char *end, int32_t v)
{
    char     b[12];
    uint8_t  i = 11U;

    if (v < 0) {
        if (p < (end - 1)) {
            *p++ = '-';
        }
        v = -v;
    }
    b[11] = '\0';
    if (v == 0) {
        b[10] = '0';
    } else {
        while (v != 0) {
            b[--i] = (char)('0' + ((uint32_t)v % 10U));
            v /= 10;
        }
    }
    while ((i < 11U) && (p < (end - 1))) {
        *p++ = b[i++];
    }
    *p = '\0';
    return p;
}

/* 带一位小数的浮点（温度/湿度） */
static char *ap_f1(char *p, char *end, float f)
{
    int32_t t10 = (int32_t)(f * 10.0f + ((f >= 0.0f) ? 0.5f : -0.5f));
    int32_t ip;

    if (t10 < 0) {
        if (p < (end - 1)) {
            *p++ = '-';
        }
        t10 = -t10;
    }
    ip = t10 / 10;
    p = ap_int(p, end, ip);
    if (p < (end - 1)) {
        *p++ = '.';
        *p++ = (char)('0' + (t10 % 10));
        *p = '\0';
    }
    return p;
}

/* 扫一条 I2C 总线，把有应答的地址打到 USART2 */
static void min_i2c_scan(I2C_HandleTypeDef *hi2c, const char *name)
{
    min_out("[MIN] scan ");
    min_out(name);
    min_out(" -> ");
    for (uint8_t a = 1U; a < 127U; a++) {
        if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)((uint16_t)a << 1), 1U, 20U) == HAL_OK) {
            min_out_u32((uint32_t)((uint16_t)a << 1));
            min_out(" ");
        }
    }
    min_out("\r\n");
}

/* 唯一的一个任务：读 AHT20 + BMP280，串口打一行，屏幕刷两行 */
static void min_task(void *pvParameters)
{
    float    t = 0.0f, h = 0.0f, tb = 0.0f, p = 0.0f;
    uint8_t  a_ok, b_ok;
    char     l0[32];
    char     l1[32];
    char    *e;

    (void)pvParameters;

    if (aht20_init() != 0U) {
        min_out("[MIN] AHT20 init OK\r\n");
    } else {
        min_out("[MIN] AHT20 init FAIL\r\n");
    }
    if (bmp280_init() != 0U) {
        if (bmp280corect() != 0U) {
            min_out("[MIN] BMP280 init+calib OK\r\n");
        } else {
            min_out("[MIN] BMP280 calib FAIL\r\n");
        }
    } else {
        min_out("[MIN] BMP280 init FAIL\r\n");
    }

    for (;;) {
        a_ok = aht20_cluate(&t, &h);
        b_ok = bmp280_sample(&tb, &p);

        /* ---- 串口 ---- */
        min_out("[MIN] T=");
        min_out_u32((uint32_t)(int32_t)(t * 10.0f + ((t >= 0.0f) ? 0.5f : -0.5f)));
        min_out("0 H=");
        min_out_u32((uint32_t)(int32_t)(h * 10.0f + ((h >= 0.0f) ? 0.5f : -0.5f)));
        min_out("0 P=");
        min_out_u32((uint32_t)(int32_t)p);
        min_out(" aht=");
        min_out_u32(a_ok);
        min_out(" bmp=");
        min_out_u32(b_ok);
        min_out("\r\n");

        /* ---- 屏幕 ---- */
        e = ap_str(l0, &l0[sizeof(l0)], "T:");
        e = ap_f1(e, &l0[sizeof(l0)], t);
        e = ap_str(e, &l0[sizeof(l0)], "C H:");
        e = ap_f1(e, &l0[sizeof(l0)], h);
        e = ap_str(e, &l0[sizeof(l0)], "%");

        e = ap_str(l1, &l1[sizeof(l1)], "P:");
        e = ap_int(e, &l1[sizeof(l1)], (int32_t)p);
        e = ap_str(e, &l1[sizeof(l1)], "Pa ");
        e = ap_str(e, &l1[sizeof(l1)], (a_ok != 0U) ? "AHT:OK " : "AHT:NG ");
        e = ap_str(e, &l1[sizeof(l1)], (b_ok != 0U) ? "BMP:OK" : "BMP:NG");

        oled_clear();
        oled_print_line(0, "MINIMAL TEST");
        oled_print_line(1, l0);
        oled_print_line(2, l1);
        oled_print_line(3, "uptime(s):");
        oled_flush();

        vTaskDelay(pdMS_TO_TICKS(1000U));
    }
}

/* ================================================================
 * W25Q 单测：读 JEDEC ID（确认型号）→ 擦扇区 → 写特征数据 → 读回比对
 * 全部用阻塞 SPI，不走 DMA（先把链路本身验干净）
 * ================================================================ */
#define W25Q_CMD_JEDEC 0x9FU
#define W25Q_CMD_RDSR1 0x05U
#define W25Q_CMD_WREN  0x06U
#define W25Q_CMD_SE    0x20U   /* 扇区擦除 4KB */
#define W25Q_CMD_PP    0x02U   /* 页编程 */
#define W25Q_CMD_RD    0x03U   /* 读数据 */
#define W25Q_TEST_ADDR 0x000F0000UL   /* 实测芯片是 W25Q80 = 1MB，所以用 960KB 处，避开槽区/元数据/标志 */
#define W25Q_TEST_LEN  64U

static void w25q_cs(uint8_t select)
{
    HAL_GPIO_WritePin(W25Q_CS_GPIO_Port, W25Q_CS_Pin,
                      (select != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* 全双工交换 n 字节。返回 1 = SPI 层成功（不代表芯片有应答） */
static uint8_t w25q_xchg(const uint8_t *tx, uint8_t *rx, uint16_t n)
{
    uint8_t ok;

    if (n == 0U) {
        return 1;
    }
    w25q_cs(1U);
    HAL_Delay(1);
    ok = (HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx, rx, n, 500U) == HAL_OK) ? 1U : 0U;
    w25q_cs(0U);
    HAL_Delay(1);
    return ok;
}

static void min_out_hex8(uint8_t v)
{
    static const char hexd[] = "0123456789ABCDEF";
    char b[3];
    b[0] = hexd[(v >> 4) & 0x0FU];
    b[1] = hexd[v & 0x0FU];
    b[2] = '\0';
    min_out(b);
}

/* 读状态寄存器（0x05），等 BUSY 清零。返回读到的状态值 */
static uint8_t w25q_status_wait(void)
{
    uint8_t tx[2];
    uint8_t rx[2];
    uint32_t t0 = HAL_GetTick();

    tx[0] = W25Q_CMD_RDSR1;
    tx[1] = 0x00U;
    for (;;) {
        if (w25q_xchg(tx, rx, 2U) == 0U) {
            return 0xFFU;
        }
        if ((rx[1] & 0x01U) == 0U) {        /* BUSY = bit0 */
            return rx[1];
        }
        if ((HAL_GetTick() - t0) > 5000U) {
            return 0xFFU;                    /* 超时 */
        }
    }
}

static void w25q_test(void)
{
    uint8_t  tx[80];
    uint8_t  rx[80];
    uint8_t  id[3];
    uint8_t  st;
    uint32_t i;
    uint32_t bad = 0;
    uint8_t  spi_ok;

    min_out("\r\n===== W25Q test =====\r\n");

    /* ---- 1. JEDEC ID ---- */
    for (i = 0; i < 4U; i++) {
        tx[i] = (i == 0U) ? W25Q_CMD_JEDEC : 0x00U;
    }
    spi_ok = w25q_xchg(tx, rx, 4U);
    if (spi_ok == 0U) {
        min_out("[W25Q] SPI transfer FAILED (HAL timeout)\r\n");
        min_out("[W25Q] -> SPI1 链路或 hspi1 句柄有问题\r\n");
        return;
    }
    id[0] = rx[1];
    id[1] = rx[2];
    id[2] = rx[3];
    min_out("[W25Q] JEDEC ID = ");
    min_out_hex8(id[0]);
    min_out(" ");
    min_out_hex8(id[1]);
    min_out(" ");
    min_out_hex8(id[2]);
    min_out("\r\n");

    if ((id[0] == 0xFFU) || (id[0] == 0x00U)) {
        min_out("[W25Q] -> 无应答：CS#/MISO/MOSI/SCK 接错、芯片没焊、或上电顺序不对\r\n");
    } else if ((id[0] == 0xEFU) && (id[1] == 0x40U)) {
        min_out("[W25Q] -> Winbond 系列，容量码 0x");
        min_out_hex8(id[2]);
        min_out(" (16=32Mbit 17=64Mbit 18=128Mbit)\r\n");
    } else if ((id[0] == 0xC8U) || (id[0] == 0x85U)) {
        min_out("[W25Q] -> 兼容 Winbond 的第三方型号\r\n");
    } else {
        min_out("[W25Q] -> 未知厂商 ID，先确认贴片型号\r\n");
    }

    /* ---- 2. 状态寄存器 ---- */
    tx[0] = W25Q_CMD_RDSR1;
    tx[1] = 0x00U;
    if (w25q_xchg(tx, rx, 2U) != 0U) {
        min_out("[W25Q] SR1 = ");
        min_out_hex8(rx[1]);
        min_out("\r\n");
    }

    /* ---- 3. 写使能 ---- */
    tx[0] = W25Q_CMD_WREN;
    (void)w25q_xchg(tx, rx, 1U);
    tx[0] = W25Q_CMD_RDSR1;
    tx[1] = 0x00U;
    if (w25q_xchg(tx, rx, 2U) != 0U) {
        st = rx[1];
        min_out("[W25Q] after WREN SR1 = ");
        min_out_hex8(st);
        min_out((st & 0x02U) != 0U ? "  (WEL=1 OK)\r\n" : "  (WEL=0 FAIL)\r\n");
        if ((st & 0x02U) == 0U) {
            min_out("[W25Q] -> 写使能没成功，后面的擦写不用试了\r\n");
            return;
        }
    }

    /* ---- 4. 扇区擦除 ---- */
    tx[0] = W25Q_CMD_SE;
    tx[1] = (uint8_t)((W25Q_TEST_ADDR >> 16) & 0xFFU);
    tx[2] = (uint8_t)((W25Q_TEST_ADDR >> 8) & 0xFFU);
    tx[3] = (uint8_t)(W25Q_TEST_ADDR & 0xFFU);
    (void)w25q_xchg(tx, rx, 4U);
    st = w25q_status_wait();
    min_out("[W25Q] sector erase done, SR1 = ");
    min_out_hex8(st);
    min_out("\r\n");

    /* ---- 5. 写 64 字节特征数据 ---- */
    /* 关键：擦除命令用掉了一次 WREN，WEL 会自动清零，
     * 所以每一条擦/写命令前面都必须重新写使能，否则命令被直接忽略（读回全 FF）。 */
    tx[0] = W25Q_CMD_WREN;
    (void)w25q_xchg(tx, rx, 1U);
    tx[0] = W25Q_CMD_PP;
    tx[1] = (uint8_t)((W25Q_TEST_ADDR >> 16) & 0xFFU);
    tx[2] = (uint8_t)((W25Q_TEST_ADDR >> 8) & 0xFFU);
    tx[3] = (uint8_t)(W25Q_TEST_ADDR & 0xFFU);
    for (i = 0; i < W25Q_TEST_LEN; i++) {
        tx[4U + i] = (uint8_t)(0xA0U + i);
    }
    (void)w25q_xchg(tx, rx, (uint16_t)(4U + W25Q_TEST_LEN));
    st = w25q_status_wait();
    min_out("[W25Q] page program done, SR1 = ");
    min_out_hex8(st);
    min_out("\r\n");

    /* ---- 6. 读回比对 ---- */
    tx[0] = W25Q_CMD_RD;
    tx[1] = (uint8_t)((W25Q_TEST_ADDR >> 16) & 0xFFU);
    tx[2] = (uint8_t)((W25Q_TEST_ADDR >> 8) & 0xFFU);
    tx[3] = (uint8_t)(W25Q_TEST_ADDR & 0xFFU);
    for (i = 0; i < W25Q_TEST_LEN; i++) {
        tx[4U + i] = 0x00U;
    }
    if (w25q_xchg(tx, rx, (uint16_t)(4U + W25Q_TEST_LEN)) == 0U) {
        min_out("[W25Q] read back SPI FAILED\r\n");
        return;
    }
    for (i = 0; i < W25Q_TEST_LEN; i++) {
        uint8_t got = rx[4U + i];
        uint8_t exp = (uint8_t)(0xA0U + i);
        if (got != exp) {
            if (bad < 5U) {
                min_out("[W25Q] mismatch @");
                min_out_u32(i);
                min_out(": got ");
                min_out_hex8(got);
                min_out(" expect ");
                min_out_hex8(exp);
                min_out("\r\n");
            }
            bad++;
        }
    }
    if (bad == 0U) {
        min_out("[W25Q] READ-BACK OK, 64/64 bytes match -> 写读都正常\r\n");
    } else {
        min_out("[W25Q] READ-BACK MISMATCH: ");
        min_out_u32(bad);
        min_out(" / 64 bytes wrong\r\n");
    }

    /* ---- 7. 擦回去，别留垃圾 ---- */
    tx[0] = W25Q_CMD_WREN;
    (void)w25q_xchg(tx, rx, 1U);
    tx[0] = W25Q_CMD_SE;
    tx[1] = (uint8_t)((W25Q_TEST_ADDR >> 16) & 0xFFU);
    tx[2] = (uint8_t)((W25Q_TEST_ADDR >> 8) & 0xFFU);
    tx[3] = (uint8_t)(W25Q_TEST_ADDR & 0xFFU);
    (void)w25q_xchg(tx, rx, 4U);
    (void)w25q_status_wait();
    min_out("[W25Q] test sector erased again\r\n");
}
#endif /* MINIMAL_TEST */

static void ledtask(void *pvParameters) {
    (void)pvParameters;          // 避免未使用参数警告

    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);    // 直接翻转，不关心极性
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(400)); // 固定周期 400 ms
    }
}

/* 建任务并把结果打出来：堆不够时 xTaskCreate 会失败，
 * 以前忽略返回值的话任务静默不跑，屏幕就一直不亮，很难查 */
static void create_task_checked(TaskFunction_t fn, const char *name,
                                uint16_t stack_words, UBaseType_t priority)
{
    TaskHandle_t handle = NULL;

    if (xTaskCreate(fn, name, stack_words, NULL, priority, &handle) == pdPASS) {
        dbg_printf("[BOOT] %s ok\r\n", name);
        task_monitor_add(name, handle);
    } else {
        dbg_printf("[BOOT] %s FAIL heap=%u\r\n", name, (unsigned)xPortGetFreeHeapSize());
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* 【向量表必须由 APP 自己搬到自己的基址，不能只指望 bootloader】
   * APP 不在 0x08000000，而 CubeMX 生成的 SystemInit() 没有开
   * USER_VECT_TAB_ADDRESS —— 以前只有 bootloader 在跳转前设了 SCB->VTOR。
   * 于是用 ST-Link 单独烧 APP（不经过 bootloader）时 VTOR 一直停在 0，
   * HAL_Init() 一开 SysTick 就去 0x08000000 取处理函数：
   * 那里要么是全 0xFF（空片，第一个中断即 HardFault），
   * 要么是对面 bootloader 的表（处理函数改的是 bootloader 的 RAM）。
   * 症状就是"上电即死、屏幕不亮、VTOR 读出来是 0"。
   * 【这里取链接符号而不是写死地址】改了 .ld 的 ORIGIN 它自动跟着走，
   *   不会再出现"启动地址和向量表地址两个地方各写一份、然后漂移"。 */
  extern const uint32_t g_pfnVectors[];   /* startup_stm32f103c8tx.s 的向量表标号 */
  SCB->VTOR = (uint32_t)&g_pfnVectors[0];

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* 顺序有讲究：DMA 必须排在 SPI 前面（SPI2 的 DMA 句柄是在 HAL_SPI_MspInit 里挂上去的，
   * 那时 DMA1 的时钟必须已经打开）；所有外设 Init 都必须在 HAL_Init()/SystemClock_Config() 之后 */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
#if MINIMAL_TEST
  /* ============================================================
   * 最小验证模式：只做「启动 → 屏幕 → 传感器」，别的都不建。
   * 日志出口是 USART2（PA2，板上「调试UART 4P」口）。
   * ============================================================ */
  min_out("\r\n\r\n===== MINIMAL TEST boot (USART2 115200) =====\r\n");
  w25q_test();
  min_i2c_scan(&hi2c2, "I2C2/OLED");
  min_i2c_scan(&hi2c1, "I2C1/SEN");

  ssd1306_init();
  min_out("[MIN] OLED present=");
  min_out_u32(oled_device_present());
  min_out(" i2c_err=");
  min_out_u32(oled_i2c_errors());
  min_out("\r\n");
  /* ssd1306_selftest();  \u2014\u2014 \u5f00\u673a\u6574\u5c4f\u70b9\u4eae 1 \u79d2\u7684\u81ea\u68c0\uff0c\u7528\u6237\u89c6\u89d2\u662f\u300c\u767d\u4e00\u4e0b\u5c31\u9ed1\u300d\uff0c\u53bb\u6389 */

  {
    TaskHandle_t h_min = NULL;
    if (xTaskCreate(min_task, "min", 512U, NULL, 2U, &h_min) == pdPASS) {
      min_out("[MIN] task create OK\r\n");
    } else {
      min_out("[MIN] task create FAILED\r\n");
    }
  }
  vTaskStartScheduler();
  min_out("[MIN] scheduler returned!\r\n");
  while (1) {
    HAL_Delay(1000U);
  }
#else
  /* 低功耗初始化：LSE + RTC(1Hz 闹钟) + EXTI0(PC0 MPU INT) + EXTI1(PB1 nRF24 IRQ)
   * 必须在调度器启动前做完，否则任务一起来就可能进 STOP，来不及报日志 */
  /* lp_init(); \u2014\u2014 \u4f4e\u529f\u8017\u6682\u65f6\u505c\u7528\uff08\u7528\u6237\u8981\u6c42\uff09 */

  dbg_printf("\r\n\r\n===== STM32_MQTT_OTA_Node boot (uart1 115200) =====\r\n");
  dbg_printf("[APP] version=%s\r\n", app_version_str());
  dbg_report_reset_cause();   /* 为什么重启：复位脚/上电/看门狗/软件复位 */

  buzzer_init();

  /* 电池 ADC（PA0 = ADC12_IN0）：必须在调度器启动前配好，
   * 采集本身由 sensor_task 按 20 秒一次去做。
   * 【.ioc 里没有 ADC，也不会有】这一段是运行时自己配寄存器的，
   *   CubeMX 重生成冲不掉（见 sensor_task.c 文件头的说明）。 */
  battery_adc_init();

  /* 屏幕初始化 + 自检：整屏点亮 1 秒，亮说明面板和通信都通 */
  ssd1306_init();
  dbg_printf("[OLED] device present=%u, i2c errors=%lu\r\n",
             (unsigned)oled_device_present(), (unsigned long)oled_i2c_errors());
  /* ssd1306_selftest();  \u2014\u2014 \u5f00\u673a\u6574\u5c4f\u70b9\u4eae 1 \u79d2\u7684\u81ea\u68c0\uff0c\u7528\u6237\u89c6\u89d2\u662f\u300c\u767d\u4e00\u4e0b\u5c31\u9ed1\u300d\uff0c\u53bb\u6389 */

  /* 建任务。
   * 蜂鸣器原来单独建一个任务等信号量，那条链 2026-11-03 现场静默失效
   * （告警任务明明调了 buzzer_beep，声音就是不出来）。现在蜂鸣器由调用者
   * 任务直接写脚，这个任务不再需要，删掉也省回 512 字节堆。 */
  /* 栈是堆里最贵的部分（1 word = 4 字节，13 个任务加起来要 10KB+），
   * 这里的数字是「够用」的下限，别再往上加 —— 堆只有 12KB。 */
  create_task_checked(button,      "2",    160U, 2U);
  create_task_checked(ledtask,     "led",  128U, 1U);
  /* 优先级不能比 ucmd 低：ucmd 是阻塞收包的，低于它的任务会被饿死
   *（表现：传感器数值永远 0、OLED 全黑）。这两个抬到 2 跟 ucmd 同级。 */
  create_task_checked(oled_task,   "oled", 320U, 2U);
  create_task_checked(sensor_task, "sen",  320U, 2U);

  /* 三节点方案的业务任务入口（建 W25Q / NRF24 等任务）：
   * 必须放在调度器启动之前，这样它建的任务和上面这些老任务一起被调度，
   * 不会漏掉开机阶段 */
  app_tasks_init();

  vTaskStartScheduler();

  /* 正常不会走到这里；走到了说明调度器启动失败（多半是堆不够） */
  dbg_printf("[FATAL] vTaskStartScheduler returned! heap free=%u B\r\n",
             (unsigned)xPortGetFreeHeapSize());
#endif /* MINIMAL_TEST */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  * 传感器总线：PB6=SCL、PB7=SDA，挂 AHT20(0x38) 和 BMP280(0x77)
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  * @note W25Q128 外置 Flash：PA5=SCK / PA6=MISO / PA7=MOSI，片选 PA4。
  *       引脚必须和 bootloader 的 bootloaer.ioc 完全一致，否则应用能读写、
  *       bootloader 却读不到同一颗 Flash。
  *       SPI1 挂在 APB2（72MHz），/2/16 = 2.25MHz，W25Q 根本用不着更快。
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  * @note NRF24L01+：PB13=SCK / PB14=MISO / PB15=MOSI。
  *       nRF24L01+ 的 SPI 上限是 10MHz，SPI2 在 APB1（36MHz），
  *       /4 = 9MHz 刚好在范围内（/2 = 18MHz 会读错寄存器）。
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  /* 与 8266 的 STM32_UART_BAUD 一致；降档时两边同时 -DSTM32_LINK_BAUD=57600。
   * 默认值保持 115200，不影响现有部署。 */
#ifndef STM32_LINK_BAUD
#define STM32_LINK_BAUD 115200
#endif
  huart1.Init.BaudRate = STM32_LINK_BAUD;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  * @note ESP8266 下载器串口：PA2=TX / PA3=RX，115200 8N1，收发都开。
  *       PA2/PA3 原本是 LED1/KEY2，已分别迁到 PA8/PA12。
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  /* USART2 parameter configuration*/
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();


  /* 日志走 DMA1_Channel4（USART1_TX）：串口打印不再阻塞任务 */
  hdma_usart1_tx.Instance = DMA1_Channel4;
  hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart1_tx.Init.Mode = DMA_NORMAL;
  hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
  {
    Error_Handler();
  }

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  /* W25Q 的 CS# 低有效，空闲保持高；LED1 初始为灭 */
  HAL_GPIO_WritePin(GPIOA, W25Q_CS_Pin|LED1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  /* NRF24 的 CE/CSN 初值都为低（不发射、不选中），LED2 初始为灭 */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin|NRF24_CE_Pin|NRF24_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  /* 蜂鸣器低电平触发，空闲必须写高（不响）；NC_RES/NC_DC 是已下线 TFT 屏的占位脚 */
  HAL_GPIO_WritePin(GPIOB, BUZZER_Pin|NC_RES_Pin|NC_DC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : KEY1_Pin KEY2_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin|KEY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : W25Q_CS_Pin LED1_Pin */
  GPIO_InitStruct.Pin = W25Q_CS_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF24_IRQ_Pin（模块拉低表示有事件，MCU 侧上拉输入） */
  GPIO_InitStruct.Pin = NRF24_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY3_Pin（原 LCD 片选，现为第三个按键，上拉输入） */
  GPIO_InitStruct.Pin = KEY3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin NRF24_CE_Pin NRF24_CSN_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|NRF24_CE_Pin|NRF24_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NC_RES_Pin NC_DC_Pin */
  GPIO_InitStruct.Pin = NC_RES_Pin|NC_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* 【OTA 演示脚 PA11】推挽输出，开机即置高。
   * 为什么不是 PA7：PA7 是 SPI1_MOSI（接 W25Q128），已被占用。
   * 这块板能用的空脚只剩 PA0（电池分压 ADC）、PA11（原本保留给 USART CTS）、
   * PA15（MPU6050 INT），所以挑 PA11。
   * 这里直接写寄存器而不是 HAL_GPIO_Init：APP 区只剩几十字节 Flash，
   * 省一点是一点。CRH 里 pin11 占 bit12~15：MODE=11(50MHz)、CNF=00(推挽)。
   *
   * 【两套固件靠这个开关区分】
   *   DEMO_PA11 = 0 → 基线固件（PA11 不动，演示前板子上跑的就是它）
   *   DEMO_PA11 = 1 → 演示固件（PA11 置高，后台 OTA 刷完这版 LED 就亮） */
#ifndef DEMO_PA11
#define DEMO_PA11 1
#endif
#if DEMO_PA11
  GPIOA->CRH  = (GPIOA->CRH & ~(0xFU << 12)) | (0x3U << 12);
  GPIOA->BSRR = GPIO_PIN_11;              /* 输出高 = 点亮对面那颗 LED */
#endif

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 【原来这里是 __disable_irq(); while(1){} —— 一句话不说地冻死】
   * HSE 起不来、任何一个 MX_*_Init 返回非 HAL_OK 都会走进来，本工程有 9 个入口。
   * 现场没有调试器时现象就是"上电即死"，和芯片坏了根本分不开。
   *
   * 现在：先把调用点地址打出来（拿它去 STM32_MQTT_OTA_Node.list / .map 里查是哪一个
   * MX_xxx_Init 失败的，不用猜），空转一会儿让日志发完，然后自复位。
   * 宁可反复重启、每一轮都留下一行原因，也不要永久冻住。
   *
   * 【为什么这里不能用 HAL_Delay 或任何带超时的 HAL 调用】
   *   __disable_irq() 之后 TIM4/SysTick 不再进中断，HAL_GetTick() 停在原地，
   *   超时判断永远不成立 —— 那就是"卡死在打印之后"的第二个坑。
   *   所以只用带计数守卫的 dbg_fatal_* 输出 + 空转循环。 */
  uint32_t from = (uint32_t)__builtin_return_address(0);

  __disable_irq();

  dbg_fatal_puts("\r\n[FATAL] Error_Handler\r\n");
  dbg_fatal_hex("  called from ", from);

  for (volatile uint32_t i = 0; i < 4000000UL; i++) { }

  NVIC_SystemReset();
  for ( ; ; )
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
