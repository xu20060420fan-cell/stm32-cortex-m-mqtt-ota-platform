/*
 * u8g2.c
 *
 *  Created on: Sep 20, 2026
 *      Author: Administrator
 */
#include <ssd1306.h>
#include"main.h"
#include"debug_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdarg.h>
#include <stdio.h>
extern I2C_HandleTypeDef hi2c2;
u8g2_t ssd1306;

/* 通信统计：屏幕不亮时先看这两个数 */
volatile uint32_t oled_i2c_ok_count;
volatile uint32_t oled_i2c_err_count;
volatile uint8_t  oled_probe_ok;      /* 初始化时 0x78 是否有应答 */

uint32_t oled_i2c_errors(void)
{
	return oled_i2c_err_count;
}

uint8_t oled_device_present(void)
{
	return oled_probe_ok;
}

/* 屏幕帧数据用阻塞方式发送（HAL_I2C_Master_Transmit）。
 *
 * 为什么不用中断式（HAL_I2C_Master_Transmit_IT）：
 *   中断式确实能在传输期间让出 CPU，但代价是整套 I2C 中断路径被链接进固件 ——
 *   HAL_I2C_EV_IRQHandler 单独就 2224 字节，加上错误路径、回调、信号量，
 *   实测砍掉能省 2.5KB 以上。在 40K 的 app 分区里这是实打实的一大块。
 *
 * 代价评估：I2C2 现在是 400kHz，1KB 一帧约 23ms。u8g2 本来就是整页刷，
 * 一屏本来就要 20~50ms，阻塞 23ms 完全扛得住。
 *
 * ⚠ 前提：stm32f1xx_it.c 里的 I2C2_EV/ER 中断处理也必须一起去掉，
 *   否则中断向量表仍然引用它，链接器还是会把这 2.2KB 留下。 */
static uint32_t s_i2c2_speed = 400000U;   /* 当前 I2C2 速率，恢复时要用 */

static uint8_t oled_write(const uint8_t *buf, uint16_t len)
{
	if (HAL_I2C_Master_Transmit(&hi2c2, 0x78, (uint8_t *)buf, len, 200) != HAL_OK) {
		oled_i2c_err_count++;
		/* 关键：失败后 I2C2 会停在 BUSY，不恢复的话后面每帧都会失败 */
		(void)HAL_I2C_DeInit(&hi2c2);
		hi2c2.Init.ClockSpeed = s_i2c2_speed;
		(void)HAL_I2C_Init(&hi2c2);
		return 0U;
	}
	return 1U;
}

uint8_t u8x8_byte_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    /* 一页 128 字节数据 + u8g2 加的 1 个控制字节(0x40)，所以缓冲要 > 128 */
    static uint8_t buffer[132];
    static uint8_t buf_idx;

    switch(msg)
    {
    case U8X8_MSG_BYTE_INIT:
        /* I2C 已经在 MX_I2C1_Init 里初始化过了，这里什么都不用做 */
        break;

    case U8X8_MSG_BYTE_START_TRANSFER:
        buf_idx = 0;   // 开始一次传输，清缓冲索引
        break;

    case U8X8_MSG_BYTE_SEND:
        /* arg_int 是本轮要发的字节数，arg_ptr 是数据指针 */
        /* 把数据攒到 buffer 里，等 END_TRANSFER 时一次性发出去 */
        while(arg_int > 0){
            buffer[buf_idx++] = *(uint8_t *)arg_ptr;
            arg_ptr = (void *)((uint8_t *)arg_ptr + 1);
            arg_int--;
        }
        break;

    case U8X8_MSG_BYTE_END_TRANSFER:
        /* buffer[0] 是 u8g2 加的控制字节：0x00=命令，0x40=数据 */
        /* 后面跟着的就是真正要发的内容 */
        /* 一帧数据在这里发出去；失败次数是判断"屏幕不亮"的关键指标 */
        if (oled_write(buffer, buf_idx) == 0U) {
            oled_i2c_err_count++;
            if (oled_i2c_err_count <= 3U) {
                dbg_printf("[OLED] i2c write FAIL #%lu len=%u\r\n",
                           (unsigned long)oled_i2c_err_count, (unsigned)buf_idx);
            }
        } else {
            oled_i2c_ok_count++;
        }
        break;

    case U8X8_MSG_BYTE_SET_DC:
        /* I2C 模式下不用 DC 引脚，忽略 */
        break;

    default:
        return 0;   // 未知 msg，返回失败
    }
    return 1;       // 成功
}
uint8_t gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch(msg)
    {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
        /* 如果用复位引脚，在这里初始化；I2C 模式一般不用 */
        break;

    case U8X8_MSG_DELAY_MILLI:
        HAL_Delay(arg_int);
        break;

    case U8X8_MSG_DELAY_10MICRO:
        /* 10 微秒延时，用简单循环凑 */
        for(volatile int i = 0; i < arg_int * 10; i++);
        break;

    case U8X8_MSG_DELAY_100NANO:
        /* 100 纳秒延时，用 __NOP() */
        for(volatile int i = 0; i < arg_int; i++) __NOP();
        break;

    case U8X8_MSG_GPIO_RESET:
        /* 如果用复位引脚，在这里拉低/拉高；I2C 模式一般不用 */
        break;

    default:
        return 0;
    }
    return 1;
}
/** 重新配置 I2C2 速率（屏幕点不亮时先怀疑 400kHz 太快，自动退到 100kHz 再试） */
static void i2c2_set_clock(uint32_t speed)
{
	HAL_I2C_DeInit(&hi2c2);
	hi2c2.Init.ClockSpeed = speed;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		dbg_printf("[OLED] re-init I2C2 @%lu failed\r\n", (unsigned long)speed);
	} else {
		s_i2c2_speed = speed;   /* 记下来，超时恢复时用同一个速率 */
	}
}

/** 探测 0x78 是否应答 */
static uint8_t oled_probe(void)
{
	return (HAL_I2C_IsDeviceReady(&hi2c2, 0x78, 3U, 50U) == HAL_OK) ? 1U : 0U;
}

void ssd1306_init(){
	oled_probe_ok = oled_probe();

	if (oled_probe_ok == 0U) {
		/* 没应答先降速重试：接线长/上拉弱时 400kHz 很容易失败 */
		i2c2_set_clock(100000U);
		oled_probe_ok = oled_probe();
	}

	u8g2_Setup_ssd1306_i2c_128x64_noname_f (&ssd1306, U8G2_R0, u8x8_byte_hw_i2c,gpio_and_delay_cb);
	 u8g2_InitDisplay(&ssd1306);      // 初始化显示屏
	 u8g2_SetPowerSave(&ssd1306, 0);  // 唤醒
	 u8g2_ClearBuffer(&ssd1306);      // 清缓冲
	 u8g2_SendBuffer(&ssd1306);       // 立刻推一帧空屏，验证通信
}

/* 屏幕自检：整屏点亮 1 秒再清掉。
 * 亮 = 面板/初始化/接线都没问题；不亮 = 通信或硬件问题 */
void ssd1306_selftest(void)
{
	u8g2_ClearBuffer(&ssd1306);
	u8g2_DrawBox(&ssd1306, 0, 0, 128, 64);
	u8g2_SendBuffer(&ssd1306);
	HAL_Delay(1000);

	u8g2_ClearBuffer(&ssd1306);
	u8g2_SendBuffer(&ssd1306);
}
static uint8_t  oled_cx = 0;      /* 光标式接口用 */
static uint8_t  oled_cy = 0;

/* ---------------- 仿 Arduino 风格的显示接口 ---------------- */

void oled_clear(void)
{
	u8g2_ClearBuffer(&ssd1306);
}

void oled_flush(void)
{
	u8g2_SendBuffer(&ssd1306);    // 必须发送缓冲才会真正显示
}

void oled_set_font(const uint8_t *font)
{
	u8g2_SetFont(&ssd1306, font);
	/* 让 ascent 覆盖中文字形（否则用默认的 ASCII 高度 8，
	 * 中文会往上顶出 2 像素，y=顶部 的语义就不准了） */
	u8g2_SetFontRefHeightAll(&ssd1306);
}

/* y 是文字「顶部」：内部加上 ascent 换算成 u8g2 需要的基线 */
void oled_print(uint8_t x, uint8_t y, const char *s)
{
	u8g2_DrawUTF8(&ssd1306, x, (u8g2_uint_t)(y + u8g2_GetAscent(&ssd1306)), s);
}

void oled_print_line(uint8_t row, const char *s)
{
	oled_print(0, (uint8_t)(row * OLED_LINE_H), s);
}

void oled_print_center(uint8_t y, const char *s)
{
	u8g2_uint_t w = u8g2_GetUTF8Width(&ssd1306, s);

	oled_print((uint8_t)((w < OLED_W) ? (OLED_W - w) / 2 : 0), y, s);
}

void oled_locate(uint8_t x, uint8_t y)
{
	oled_cx = x;
	oled_cy = y;
}

void oled_puts(const char *s)
{
	oled_print(oled_cx, oled_cy, s);
	oled_cx = (uint8_t)(oled_cx + u8g2_GetUTF8Width(&ssd1306, s));   // 光标自动右移
}

void oled_printf(uint8_t x, uint8_t y, const char *fmt, ...)
{
	char buf[32];
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	oled_print(x, y, buf);
}

/* 量一段文字在当前字体下的像素宽度（右对齐、居中都要用） */
uint16_t oled_text_width(const char *s)
{
	return u8g2_GetUTF8Width(&ssd1306, s);
}

/* ---------------------------------------------------------------------------
 * 低功耗：关屏 / 开屏
 * ------------------------------------------------------------------------- */

/* OLED 点亮时 ~20mA，是整块板上最大的耗电项。
 * u8g2_SetPowerSave → u8x8_SetPowerSave → SSD1306 的
 * U8X8_MSG_DISPLAY_SET_POWER_SAVE（发 0xAE / 0xAF）。 */
void oled_display_off(void)
{
	u8g2_SetPowerSave(&ssd1306, 1);
}

void oled_display_on(void)
{
	u8g2_SetPowerSave(&ssd1306, 0);
}

/* 主页内容在 oled_home.c 里画，这里只提供显示原语 */
