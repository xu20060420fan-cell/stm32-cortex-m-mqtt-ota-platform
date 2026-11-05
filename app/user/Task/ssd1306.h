/*
 * u8g2.h
 *
 *  Created on: Sep 20, 2026
 *      Author: Administrator
 */

#ifndef TASK_SSD1306_H_
#define TASK_SSD1306_H_

#include"u8g2.h"

extern u8g2_t ssd1306;

/* ---------- 屏幕参数 ---------- */
#define OLED_W      128
#define OLED_H      64
#define OLED_LINE_H 13      /* 12 号字一行占 13 像素，64 高正好 5 行 */

/* ---------- 仿 Arduino 风格的显示接口 ----------
 * 说明：u8g2 原生的 u8g2_DrawUTF8(&ssd1306, x, y, s) 里 y 是「基线」，
 *       也就是文字底部，第一次用很容易画到屏幕外。
 *       下面这些函数里 y 统一是「文字顶部」，和 Arduino 的 setCursor 习惯一致。
 */
void oled_clear(void);                                   /* 清缓冲（还没显示） */
void oled_flush(void);                                   /* 把缓冲推到屏幕 */
void oled_set_font(const uint8_t *font);                 /* 换字体 */
void oled_print(uint8_t x, uint8_t y, const char *s);    /* (x, y=顶部) 显示 UTF-8 文字 */
void oled_print_line(uint8_t row, const char *s);        /* 按行显示，row = 0~4 */
void oled_print_center(uint8_t y, const char *s);        /* 水平居中 */
void oled_printf(uint8_t x, uint8_t y, const char *fmt, ...);  /* 带格式化，同 printf */
void oled_locate(uint8_t x, uint8_t y);                  /* 设光标位置（类似 setCursor） */
void oled_puts(const char *s);                           /* 在光标处显示并自动右移 */
uint16_t oled_text_width(const char *s);                 /* 量文字宽度（像素） */
/* 精简中文字库（u8g2/u8g2_fonts_custom.c，约 4.5KB，含 118 个常用汉字）
 * 想改收录的字：改 .workbuddy-ai/mkfont.py 的 --text 重新生成即可 */
extern const uint8_t u8g2_font_custom12[];
void ssd1306_init(void);

/* ---- 诊断接口 ---- */
uint32_t oled_i2c_errors(void);        /* I2C 写失败次数 */
uint8_t  oled_device_present(void);    /* 初始化时 0x78 是否应答 */
void     ssd1306_selftest(void);       /* 整屏点亮 1 秒，用来判断面板是否工作 */

/* ---- 低功耗：OLED 是板子上最大的耗电项（点亮 ~20mA，熄灭 ~10µA） ---- */
void     oled_display_off(void);       /* 关屏 */
void     oled_display_on(void);        /* 开屏（会重画一帧，见 oled_home.c） */

#endif /* TASK_SSD1306_H_ */
