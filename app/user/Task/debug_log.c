/*
 * debug_log.c
 *
 * 日志实现：环形缓冲 + UART1 TX DMA。
 *
 * 为什么不直接 HAL_UART_Transmit：
 *   它是忙等式的，128 字节最坏要等几十毫秒，每秒打一条就会拖慢任务。
 *   改成 DMA 后任务只往环形缓冲里写，剩下交给 DMA 和中断。
 *
 * 缓冲满了怎么办：丢新数据并计数（诊断日志丢几条无所谓，绝不能因此卡住业务）。
 * 钩子（栈溢出/断言失败）里中断是关的，DMA 完成中断进不来，
 * 所以另给一条 dbg_fatal_puts：停掉 DMA 后直接轮询写 USART1->DR。
 */
#include "debug_log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;   /* 调试口镜像（main.c 里 MX_USART2 初始化） */
extern DMA_HandleTypeDef hdma_usart1_tx;

#define LOG_RING_SIZE 1024U

static uint8_t  s_ring[LOG_RING_SIZE];
static volatile uint16_t s_head;      /* 写入位置（任务侧） */
static volatile uint16_t s_tail;      /* 已发送位置（中断侧） */
static volatile uint16_t s_tx_len;    /* 本次 DMA 发送的字节数 */
static volatile uint8_t  s_tx_busy;   /* DMA 是否在发送 */
static volatile uint32_t s_dropped;   /* 因缓冲满被丢掉的字节数 */

/** 若空闲且有数据，取一段连续数据启动 DMA（中断安全，可在任意上下文调用） */
static void log_kick(void)
{
    uint32_t primask;
    uint16_t len;

    primask = __get_PRIMASK();
    __disable_irq();

    if (s_tx_busy == 0U && s_head != s_tail) {
        if (s_head > s_tail) {
            len = (uint16_t)(s_head - s_tail);
        } else {
            len = (uint16_t)(LOG_RING_SIZE - s_tail);   /* 先发到缓冲区末尾 */
        }
        s_tx_len = len;
        s_tx_busy = 1U;
        if (HAL_UART_Transmit_DMA(&huart1, &s_ring[s_tail], len) != HAL_OK) {
            /* 启动失败就当这段丢了，绝不能在这里死等 */
            s_tx_busy = 0U;
            s_tail = (uint16_t)((s_tail + len) % LOG_RING_SIZE);
            s_dropped += len;
        }
    }

    __set_PRIMASK(primask);
}

/** 一段 DMA 发完，推进 tail 并接着发下一段 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* 必须判断 s_tx_busy：不是我们发起的传输就不能推进 tail，
     * 否则 tail 会跑过头，导致日志丢失或重复 */
    if (huart->Instance == USART1 && s_tx_busy != 0U) {
        s_tail = (uint16_t)((s_tail + s_tx_len) % LOG_RING_SIZE);
        s_tx_busy = 0U;
        log_kick();
    }
}

/* ================================================================
 * 精简格式化器（顶替 newlib 的 snprintf / vsnprintf）
 *
 * 为什么要自己写：
 *   newlib-nano 的 snprintf 会把 _printf_i / _svfprintf_r / _printf_common /
 *   __ssprint_r / __sflush_r / _malloc_r / _realloc_r / sbrk 这一整套 stdio
 *   机器拖进固件，实测约 3.4KB flash，而本工程实际只用到
 *   %s %c %d %u %ld %lu %x %X %p 以及宽度/零填充/左对齐几种修饰。
 *
 * 原理：
 *   直接在 debug_log.c 里定义 snprintf / vsnprintf 这两个符号，
 *   链接器就再也不会从 libc.a 里把 newlib 的同名实现拉进来
 *   （静态库成员只在符号未定义时才被抽取）。
 *   调用点一个都不用改，diag.c / diag_probes.c / oled_home.c / ssd1306.c
 *   里的 snprintf(...) 会自动落到这里。
 *
 * 支持：%% %c %s %d %i %u %x %X %o %p %ld %lu %lld %llu
 *       以及 - 0 + 空格 # 标志、宽度、.精度
 * 不支持：%f %e %g（本工程已无浮点格式化调用，遇到了原样吐回）
 * ================================================================ */
#include <stdint.h>

typedef struct {
    char  *buf;
    size_t cap;   /* 缓冲区容量（含结尾 '\0'） */
    size_t len;   /* 已格式化出的字符数，可能 > cap，表示被截断 */
} mp_sink;

static void mp_putc(mp_sink *o, char c)
{
    if ((o->buf != NULL) && (o->cap > 1U) && (o->len < (o->cap - 1U))) {
        o->buf[o->len] = c;
    }
    o->len++;
}

static void mp_puts(mp_sink *o, const char *s, size_t n)
{
    size_t i;
    for (i = 0U; i < n; ++i) {
        mp_putc(o, s[i]);
    }
}

/** 无符号整数转字符串（不带符号），返回长度
 *
 *  只支持 32 位：故意用 uint32_t 而不是 uint64_t，
 *  因为 64 位取模/除法会调 libgcc 的 __udivmoddi4（766 字节）。
 *  本工程所有格式化出来的数字都远小于 4G，够用。 */
static size_t mp_utoa(uint32_t v, unsigned base, int upper, char *out)
{
    const char *digits = upper ? "0123456789ABCDEF" : "0123456789abcdef";
    char tmp[24];
    size_t n = 0U;
    size_t i;

    if (v == 0U) {
        tmp[n] = '0';
        n++;
    }
    while (v != 0U) {
        uint32_t d;

        if (base == 16U) {
            d = v & 0xFU;
            v >>= 4;
        } else if (base == 8U) {
            d = v & 0x7U;
            v >>= 3;
        } else {
            d = v % 10U;      /* 常量除法，编译器展开成乘移，不产生库调用 */
            v /= 10U;
        }
        tmp[n] = digits[d];
        n++;
    }
    for (i = 0U; i < n; ++i) {
        out[i] = tmp[n - 1U - i];
    }
    out[n] = '\0';
    return n;
}

/** 输出一个数字字段：符号 + 数字 + 宽度填充 */
static void mp_emit_num(mp_sink *o, char sign, const char *digits, size_t ndig,
                        int width, int left, int zero)
{
    int total = (int)ndig + ((sign != '\0') ? 1 : 0);
    int pad = width - total;
    int i;

    if ((pad > 0) && (left == 0) && (zero != 0)) {
        if (sign != '\0') {
            mp_putc(o, sign);
        }
        for (i = 0; i < pad; ++i) {
            mp_putc(o, '0');
        }
        mp_puts(o, digits, ndig);
        return;
    }
    if ((pad > 0) && (left == 0)) {
        for (i = 0; i < pad; ++i) {
            mp_putc(o, ' ');
        }
    }
    if (sign != '\0') {
        mp_putc(o, sign);
    }
    mp_puts(o, digits, ndig);
    if ((pad > 0) && (left != 0)) {
        for (i = 0; i < pad; ++i) {
            mp_putc(o, ' ');
        }
    }
}

static void mp_format(mp_sink *o, const char *fmt, va_list ap)
{
    while (*fmt != '\0') {
        char conv;
        char sign = '\0';
        char numbuf[32];
        int left = 0;
        int zero = 0;
        int plus = 0;
        int width = 0;
        int prec = -1;
        int lmod = 0;
        size_t n;
        int pad;
        int i;

        if (*fmt != '%') {
            mp_putc(o, *fmt);
            fmt++;
            continue;
        }
        fmt++;
        if (*fmt == '%') {
            mp_putc(o, '%');
            fmt++;
            continue;
        }

        /* 标志位 */
        for (;;) {
            if (*fmt == '-') {
                left = 1;
            } else if (*fmt == '0') {
                zero = 1;
            } else if (*fmt == '+') {
                plus = 1;
            } else if ((*fmt == ' ') || (*fmt == '#')) {
                /* 占位：本工程没用到 */
            } else {
                break;
            }
            fmt++;
        }
        /* 宽度 */
        while ((*fmt >= '0') && (*fmt <= '9')) {
            width = (width * 10) + (*fmt - '0');
            fmt++;
        }
        /* 精度（只对字符串有意义） */
        if (*fmt == '.') {
            fmt++;
            prec = 0;
            while ((*fmt >= '0') && (*fmt <= '9')) {
                prec = (prec * 10) + (*fmt - '0');
                fmt++;
            }
        }
        /* 长度修饰：l 的个数决定取 long / long long */
        for (;;) {
            if (*fmt == 'l') {
                lmod++;
            } else if (*fmt == 'h') {
                /* %hd / %hhd 的实参经过默认提升后就是 int，直接按 int 取 */
            } else if ((*fmt == 'z') || (*fmt == 'j') || (*fmt == 't')) {
                lmod = 1;
            } else {
                break;
            }
            fmt++;
        }

        conv = *fmt;
        if (conv == '\0') {
            break;
        }
        fmt++;

        switch (conv) {
        case 'd':
        case 'i': {
            long long v;
            unsigned long long uv;

            if (lmod >= 2) {
                v = va_arg(ap, long long);
            } else if (lmod == 1) {
                v = va_arg(ap, long);
            } else {
                v = va_arg(ap, int);
            }
            if (v < 0) {
                sign = '-';
                uv = (unsigned long long)(-(v + 1)) + 1ULL;   /* 避免 INT64_MIN 溢出 */
            } else {
                uv = (unsigned long long)v;
                if (plus != 0) {
                    sign = '+';
                }
            }
            n = mp_utoa((uint32_t)uv, 10U, 0, numbuf);
            mp_emit_num(o, sign, numbuf, n, width, left, zero);
            break;
        }
        case 'u': {
            unsigned long long uv;

            if (lmod >= 2) {
                uv = va_arg(ap, unsigned long long);
            } else if (lmod == 1) {
                uv = va_arg(ap, unsigned long);
            } else {
                uv = va_arg(ap, unsigned int);
            }
            n = mp_utoa((uint32_t)uv, 10U, 0, numbuf);
            mp_emit_num(o, '\0', numbuf, n, width, left, zero);
            break;
        }
        case 'x':
        case 'X': {
            unsigned long long uv;

            if (lmod >= 2) {
                uv = va_arg(ap, unsigned long long);
            } else if (lmod == 1) {
                uv = va_arg(ap, unsigned long);
            } else {
                uv = va_arg(ap, unsigned int);
            }
            n = mp_utoa((uint32_t)uv, 16U, (conv == 'X') ? 1 : 0, numbuf);
            mp_emit_num(o, '\0', numbuf, n, width, left, zero);
            break;
        }
        case 'o': {
            unsigned long long uv;

            if (lmod >= 2) {
                uv = va_arg(ap, unsigned long long);
            } else if (lmod == 1) {
                uv = va_arg(ap, unsigned long);
            } else {
                uv = va_arg(ap, unsigned int);
            }
            n = mp_utoa((uint32_t)uv, 8U, 0, numbuf);
            mp_emit_num(o, '\0', numbuf, n, width, left, zero);
            break;
        }
        case 'c': {
            int c = va_arg(ap, int);

            pad = width - 1;
            if ((pad > 0) && (left == 0)) {
                for (i = 0; i < pad; ++i) {
                    mp_putc(o, ' ');
                }
            }
            mp_putc(o, (char)c);
            if ((pad > 0) && (left != 0)) {
                for (i = 0; i < pad; ++i) {
                    mp_putc(o, ' ');
                }
            }
            break;
        }
        case 's': {
            const char *s = va_arg(ap, const char *);

            if (s == NULL) {
                s = "(null)";
            }
            n = strlen(s);
            if ((prec >= 0) && ((size_t)prec < n)) {
                n = (size_t)prec;
            }
            pad = width - (int)n;
            if ((pad > 0) && (left == 0)) {
                for (i = 0; i < pad; ++i) {
                    mp_putc(o, ' ');
                }
            }
            mp_puts(o, s, n);
            if ((pad > 0) && (left != 0)) {
                for (i = 0; i < pad; ++i) {
                    mp_putc(o, ' ');
                }
            }
            break;
        }
        case 'p': {
            void *ptr = va_arg(ap, void *);

            n = mp_utoa((uint32_t)(uintptr_t)ptr, 16U, 0, numbuf);
            mp_puts(o, "0x", 2U);
            mp_emit_num(o, '\0', numbuf, n, 8, 0, 1);
            break;
        }
        default:
            /* 不支持的转换（含 %f/%e/%g）：原样吐回去，避免吃掉后面的字符 */
            mp_putc(o, '%');
            mp_putc(o, conv);
            break;
        }
    }
}

/* ---- 顶替 newlib 的两个符号 ---- */
int vsnprintf(char *__restrict s, size_t n, const char *__restrict fmt, va_list ap)
{
    mp_sink o;

    o.buf = s;
    o.cap = n;
    o.len = 0U;
    mp_format(&o, fmt, ap);
    if ((s != NULL) && (n > 0U)) {
        s[(o.len < n) ? o.len : (n - 1U)] = '\0';
    }
    return (int)o.len;
}

int snprintf(char *__restrict s, size_t n, const char *__restrict fmt, ...)
{
    va_list ap;
    int r;

    va_start(ap, fmt);
    r = vsnprintf(s, n, fmt, ap);
    va_end(ap);
    return r;
}

/* ================================================================
 * 精简格式化器结束
 * ================================================================ */

void dbg_puts(const char *s)
{
    uint16_t len;
    uint16_t i;
    uint32_t primask;

    if (s == NULL) {
        return;
    }
    len = (uint16_t)strlen(s);

    /* 【必须关中断写环】这个环有多个任务同时写：ucmd 发 TL、alarm 发 EVT、
     * oled/sensor 打调试行…… 而 s_head 是"读-改-写"三步。
     * 一个任务刚读完 s_head 就被更高优先级的任务打断、对方也往同一个槽写，
     * 两边再各自把 s_head 往前推 —— 轻则两行文字互相穿插（8266 那边
     * 解析 TL 直接失败），重则 s_head 跨过 s_tail 把环搅乱。
     * 一行最长 128 字节，拷完也就几微秒，临界段这么短不影响节拍。
     * USART2 镜像是阻塞发送，必须留在临界段外面。 */
    primask = __get_PRIMASK();
    __disable_irq();

    for (i = 0U; i < len; ++i) {
        uint16_t next = (uint16_t)((s_head + 1U) % LOG_RING_SIZE);

        if (next == s_tail) {
            s_dropped += (uint32_t)(len - i);   /* 满了：丢新数据 */
            break;
        }
        s_ring[s_head] = (uint8_t)s[i];
        s_head = next;
    }
    log_kick();

    __set_PRIMASK(primask);

    /* 【USART2 镜像（2026-11-01）】PC 的 USB-TTL 接在调试口 USART2(PA2/PA3) 上，
     * 电脑上直接看日志用。阻塞发送、20ms 超时：日志行很短（<128B），
     * 115200 下最多 ~11ms，不影响任务节奏。huart2 没初始化（MX 未跑）时跳过。 */
    if (huart2.Instance == USART2) {
        (void)HAL_UART_Transmit(&huart2, (const uint8_t *)s, len, 20U);
    }
}

void dbg_printf(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    int len;

    va_start(ap, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (len <= 0) {
        return;
    }
    if (len > (int)sizeof(buf) - 1) {
        len = (int)sizeof(buf) - 1;
    }
    buf[len] = '\0';
    dbg_puts(buf);
}

void dbg_fatal_puts(const char *s)
{
    volatile uint32_t guard;

    if (s == NULL) {
        return;
    }

    /* 中断已关，DMA 完成中断进不来：先把在跑的 DMA 停掉，免得抢 DR 寄存器 */
    if (s_tx_busy != 0U) {
        HAL_DMA_Abort(&hdma_usart1_tx);
        USART1->CR3 &= ~USART_CR3_DMAT;
        s_tx_busy = 0U;
    }

    /* 纯轮询直发，不依赖任何中断。
     * 【两个口都发】USART1(PA9) 是 8266 那条协议线，USART2(PA2) 才是板上
     * 「调试UART 4P」/USB-TTL 那个头子。fatal 路径原来只喂 USART1 ——
     * 恰恰在最需要看日志的时候（栈爆了、HardFault 了），拿 USB-TTL 的人
     * 一个字都看不到，只能看见"屏幕黑了"。 */
    while (*s != '\0') {
        guard = 200000U;
        while (((USART1->SR & USART_SR_TXE) == 0U) && (guard > 0U)) {
            guard--;
        }
        USART1->DR = (uint16_t)(uint8_t)(*s);

        guard = 200000U;
        while (((USART2->SR & USART_SR_TXE) == 0U) && (guard > 0U)) {
            guard--;
        }
        USART2->DR = (uint16_t)(uint8_t)(*s);
        s++;
    }
    guard = 200000U;
    while (((USART1->SR & USART_SR_TC) == 0U) && (guard > 0U)) {
        guard--;
    }
    guard = 200000U;
    while (((USART2->SR & USART_SR_TC) == 0U) && (guard > 0U)) {
        guard--;
    }
}

void dbg_i2c_scan(I2C_HandleTypeDef *hi2c, const char *bus_name)
{    uint8_t addr;
    uint8_t found = 0U;

    dbg_printf("[I2C] scan %s ...\r\n", bus_name);
    for (addr = 0x08U; addr < 0x78U; ++addr) {
        if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)((uint16_t)addr << 1U),
                                  2U, 20U) == HAL_OK) {
            dbg_printf("      found: 7bit=0x%02X  (HAL addr=0x%02X)\r\n",
                       addr, (unsigned)((uint16_t)addr << 1U));
            found++;
        }
    }
    if (found == 0U) {
        dbg_printf("      NOTHING on %s (check wiring / pull-up / power)\r\n",
                   bus_name);
    } else {
        dbg_printf("      %u device(s) on %s\r\n", (unsigned)found, bus_name);
    }
}

/* ---------------- 异常诊断（可在中断/异常上下文使用） ---------------- */

/** 打一个 32 位十六进制数（不格式化、不占栈，给 fault 处理用） */
void dbg_fatal_hex(const char *tag, uint32_t value)
{
    static const char hexdigits[] = "0123456789ABCDEF";
    char buf[11];
    int i;

    dbg_fatal_puts(tag);
    buf[0] = '0';
    buf[1] = 'x';
    for (i = 0; i < 8; ++i) {
        buf[2 + i] = hexdigits[(value >> ((7 - i) * 4)) & 0xFU];
    }
    buf[10] = '\0';
    dbg_fatal_puts(buf);
    dbg_fatal_puts("\r\n");
}

/** 打印本次复位的原因，并清掉标志（这样每次上电都能看到为什么重启） */
void dbg_report_reset_cause(void)
{
    uint32_t csr = RCC->CSR;

    dbg_printf("[BOOT] reset cause:");
    if ((csr & RCC_CSR_PINRSTF) != 0U) {
        dbg_puts(" NRST(pin/debugger)");
    }
    if ((csr & RCC_CSR_PORRSTF) != 0U) {
        dbg_puts(" POR(power-on/brownout)");
    }
    if ((csr & RCC_CSR_SFTRSTF) != 0U) {
        dbg_puts(" SW(software reset)");
    }
    if ((csr & RCC_CSR_IWDGRSTF) != 0U) {
        dbg_puts(" IWDG(watchdog)");
    }
    if ((csr & RCC_CSR_WWDGRSTF) != 0U) {
        dbg_puts(" WWDG(window watchdog)");
    }
    if ((csr & RCC_CSR_LPWRRSTF) != 0U) {
        dbg_puts(" LOWPWR(low-power)");
    }
    dbg_puts("\r\n");

    __HAL_RCC_CLEAR_RESET_FLAGS();
}
