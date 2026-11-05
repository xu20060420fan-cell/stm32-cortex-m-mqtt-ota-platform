/*
 * diag_probes.c —— 各外设的自检探针
 * -----------------------------------------------------------------------------
 * diag.c 是框架，这里是"具体某个外设怎么证明自己活着"。
 * 两者分开的原因：框架不该认识任何驱动；
 * 换驱动、换传感器型号时只改这个文件。
 *
 * 每条探针的写法都是一样的：
 *   1. 尽量用**只读、不改状态**的寄存器访问去确认器件在；
 *   2. 把关键证据写进 item->detail（器件 ID / 读到的值），
 *      这样后台上一眼就能看出"它到底回了个什么"，
 *      而不是只看到一个 FAIL；
 *   3. 失败时给**具体**的错误码，不要笼统地报"不工作"——
 *      超时、NACK、ID 不对、总线错误，现场处理方式完全不同。
 * -----------------------------------------------------------------------------
 */
#include "diag.h"

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "aht20.h"
#include "bmp280.h"
#include "ssd1306.h"
#include "debug_log.h"

/* 各探针用到的 I2C 句柄（由 CubeMX 生成，extern 引用） */
extern I2C_HandleTypeDef hi2c1;   /* 传感器总线：AHT20 / BMP280 / MPU6050 */
extern I2C_HandleTypeDef hi2c2;   /* 显示总线：SSD1306 */

/* ================================================================
 * 板卡自身
 * ================================================================ */
static int probe_self(diag_item *item)
{
    /*
     * 复位原因：F1 没有 HAL_RCC_GetResetSource()（那是 F4 的 API），
     * F1 要直接读 RCC->CSR 的标志位：
     *   PINRSTF 上电/复位引脚   PORRSTF 上电
     *   SFTRSTF 软件复位        IWDGRSTF 独立看门狗   WWDGRSTF 窗口看门狗
     * ⚠ 这些标志读一次就被"认领"掉，硬件不再置位，
     *   所以它们反映的是**上一次复位**的原因 —— 正是后台想看的。
     */
    uint32_t csr   = RCC->CSR;
    const char *why = "unknown";

    if (csr & RCC_CSR_IWDGRSTF)      why = "iwdg";
    else if (csr & RCC_CSR_WWDGRSTF) why = "wwdg";
    else if (csr & RCC_CSR_SFTRSTF)  why = "soft";
    else if (csr & RCC_CSR_PINRSTF)  why = "pin";
    else if (csr & RCC_CSR_PORRSTF)  why = "por";

    item->err = DIAG_OK;
    snprintf(item->detail, sizeof(item->detail), "reset=%s csr=0x%08lX",
             why, (unsigned long)csr);
    return 1;
}

/* ================================================================
 * 定点格式化：把 float 输出成 "整.一" 的文本
 * ------------------------------------------------------------
 * 为什么不用 %f：newlib-nano 的浮点格式化会把整套 double 运算库拖进固件。
 * 实测 __adddf3 / __subdf3 / __muldf3 / __divdf3 各约 600 字节，
 * 加上各自的 __aeabi_* 包装和浮点格式化，一共 7KB 以上 ——
 * 而 app 分区只有 40K，这一项就直接决定链接能不能过。
 * 定点输出 0.1 单位，自检用完全够。
 * ================================================================ */
static void fd_f1(char *out, size_t n, float v)
{
    int v10 = (int)((v >= 0.0f) ? (v * 10.0f + 0.5f) : (v * 10.0f - 0.5f));
    int a   = (v10 < 0) ? -v10 : v10;
    snprintf(out, n, "%s%d.%d", (v10 < 0) ? "-" : "", a / 10, a % 10);
}

/* ================================================================
 * AHT20（温湿度）
 * ================================================================ */
static int probe_aht20(diag_item *item)
{
    float t = 0.0f, h = 0.0f;
    char tb[20], hb[20];

    if (!aht20_init()) {
        item->err = DIAG_ERR_NACK;
        snprintf(item->detail, sizeof(item->detail), "init failed");
        return 0;
    }
    if (!aht20_cluate(&t, &h)) {
        item->err = DIAG_ERR_TIMEOUT;
        snprintf(item->detail, sizeof(item->detail), "read failed");
        return 0;
    }
    fd_f1(tb, sizeof(tb), t);
    fd_f1(hb, sizeof(hb), h);
    /* 读数离谱多半是 CRC 校验没通过（驱动已重试），也算异常 */
    if (t < -40.0f || t > 85.0f || h < 0.0f || h > 100.0f) {
        item->err = DIAG_ERR_PARAM;
        snprintf(item->detail, sizeof(item->detail), "bad value t=%s h=%s", tb, hb);
        return 0;
    }
    item->err = DIAG_OK;
    snprintf(item->detail, sizeof(item->detail), "t=%sC h=%s%%", tb, hb);
    return 1;
}

/* ================================================================
 * BMP280（气压）
 * ================================================================ */
static int probe_bmp280(diag_item *item)
{
    float t = 0.0f, p = 0.0f;
    char tb[20], pb[20];

    if (!bmp280_init()) {
        item->err = DIAG_ERR_NACK;
        snprintf(item->detail, sizeof(item->detail), "init failed");
        return 0;
    }
    if (!bmp280_sample(&t, &p)) {
        item->err = DIAG_ERR_TIMEOUT;
        snprintf(item->detail, sizeof(item->detail), "read failed");
        return 0;
    }
    fd_f1(tb, sizeof(tb), t);
    /* 气压是 Pa，直接整数算成 0.1hPa，避免任何浮点除法 */
    {
        long p10 = (long)p / 10L;          /* 0.1 hPa */
        long pa  = (p10 < 0L) ? -p10 : p10;
        snprintf(pb, sizeof(pb), "%ld.%ld", pa / 1000L, pa % 1000L);
    }
    /* 气压合理范围：300~1200 hPa（驱动内部已换算成 Pa，这里按 hPa 判） */
    if (p < 30000.0f || p > 120000.0f) {
        item->err = DIAG_ERR_PARAM;
        snprintf(item->detail, sizeof(item->detail), "bad value p=%sPa", pb);
        return 0;
    }
    item->err = DIAG_OK;
    snprintf(item->detail, sizeof(item->detail), "p=%shPa t=%sC", pb, tb);
    return 1;
}

/* ================================================================
 * SSD1306（显示）
 * ================================================================ */
static int probe_ssd1306(diag_item *item)
{
    if (!oled_device_present()) {
        item->err = DIAG_ERR_NACK;
        snprintf(item->detail, sizeof(item->detail), "no ack @0x3C");
        return 0;
    }
    /* 整屏点亮一次：确认它不只是"应答了"，而是真的能显示 */
    ssd1306_selftest();
    item->err = DIAG_OK;
    snprintf(item->detail, sizeof(item->detail), "%dx%d ok", (int)OLED_W, (int)OLED_H);
    return 1;
}

/* ================================================================
 * I2C 总线扫描
 * ================================================================
 * 这个探针不针对某个器件，而是回答"这条总线上到底挂了几个人"。
 * 现场排障时最有用的一句话就是这个：哪个地址应答了。
 */
static int probe_i2c1_scan(diag_item *item)
{
    uint8_t found = 0;
    char buf[28];
    int  n = 0;

    buf[0] = '\0';
    for (uint8_t a = 0x08; a <= 0x77; a++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 1, 10) == HAL_OK) {
            if (n < 22) {
                n += snprintf(buf + n, sizeof(buf) - n, "%02X ", a);
            }
            found++;
        }
    }
    if (found == 0) {
        item->err = DIAG_ERR_NO_DEV;
        snprintf(item->detail, sizeof(item->detail), "no device on I2C1");
        return 0;
    }
    item->err = DIAG_OK;
    snprintf(item->detail, sizeof(item->detail), "%u dev: %s", (unsigned)found, buf);
    return 1;
}

/* ================================================================
 * 注册
 * ================================================================ */
void diag_probes_register(void)
{
    diag_register(DIAG_DEV_SELF,    "self",    probe_self);
    diag_register(DIAG_DEV_AHT20,   "aht20",   probe_aht20);
    diag_register(DIAG_DEV_BMP280,  "bmp280",  probe_bmp280);
    diag_register(DIAG_DEV_SSD1306, "ssd1306", probe_ssd1306);
    diag_register(DIAG_DEV_LINK,    "i2c1",    probe_i2c1_scan);
}
