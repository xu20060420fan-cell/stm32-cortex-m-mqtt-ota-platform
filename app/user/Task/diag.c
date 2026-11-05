/*
 * diag.c —— 外设自检 / 诊断框架实现
 * -----------------------------------------------------------------------------
 * 见 diag.h 顶部说明。这里只讲三件实现上的讲究：
 *
 * 1) 探针超时保护
 *    F103 没有通用的"看门狗喂狗超时"，但有 DWT 的循环计数器。
 *    这里用它给每个探针计时，超过 DIAG_PROBE_TIMEOUT_US 就中止本次判定，
 *    避免某个外设的驱动卡死把整台设备卡在自检里（自检必须永远能返回）。
 *
 * 2) 失败重试
 *    I2C 上偶发一次 NACK 是常态（干扰、上电时序、总线电容）。
 *    直接判死会产生大量虚警，运维会开始忽略告警 —— 比不报还糟。
 *    所以失败要重试，重试仍失败才判死；重试次数记在 item.tries 里。
 *
 * 3) 探测耗时用完的 DWT 差值
 *    顺便把每个外设的响应时间记下来：
 *    某个传感器从 2ms 变成 200ms，往往就是它开始不稳定的第一个信号。
 * -----------------------------------------------------------------------------
 */
#include "diag.h"

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "debug_log.h"

/* ================================================================
 * 内部状态
 * ================================================================ */
typedef struct {
    uint8_t        used;       /* 1 = 已注册 */
    uint8_t        id;
    const char    *name;
    diag_probe_fn  probe;      /* NULL = 本机无法自检 */
    diag_item      item;
} diag_slot_t;

static diag_slot_t s_slots[DIAG_DEV_COUNT];
static uint8_t      s_count;
static uint32_t     s_health_mask;
static uint32_t     s_error_mask;
static uint8_t      s_healthy_cnt;
static uint8_t      s_fault_cnt;
static diag_report_cb s_report_cb;
static uint32_t     s_last_run_ms;

/* 单个探针的超时上限（微秒）。10ms 对 I2C/SPI 的短事务足够宽裕，
 * 又足够小到"某个驱动卡死"不会把整机拖住 */
#define DIAG_PROBE_TIMEOUT_US   100000U   /* 100ms */

/* ================================================================
 * DWT 循环计数器（探测计时 + 超时保护）
 * ================================================================ */
static inline void diag_cyc_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t diag_cyc_now(void)
{
    return DWT->CYCCNT;
}

/* ================================================================
 * 工具
 * ================================================================ */
static void diag_copy(char *dst, size_t cap, const char *src)
{
    if (!dst || cap == 0) return;
    if (!src) { dst[0] = '\0'; return; }
    size_t n = strlen(src);
    if (n >= cap) n = cap - 1;
    memcpy(dst, src, n);
    dst[n] = '\0';
}

const char *diag_dev_name(uint8_t id)
{
    /* 注册过的用注册名，没注册的用默认名 */
    if (id < DIAG_DEV_COUNT && s_slots[id].used && s_slots[id].name) {
        return s_slots[id].name;
    }
    switch (id) {
    case DIAG_DEV_SELF:    return "self";
    case DIAG_DEV_AHT20:   return "aht20";
    case DIAG_DEV_BMP280:  return "bmp280";
    case DIAG_DEV_MPU6050: return "mpu6050";
    case DIAG_DEV_SSD1306: return "ssd1306";
    case DIAG_DEV_W25Q:    return "w25q";
    case DIAG_DEV_NRF24:   return "nrf24";
    case DIAG_DEV_BUZZER:  return "buzzer";
    case DIAG_DEV_BUTTON:  return "button";
    case DIAG_DEV_LINK:    return "link";
    default:               return "unknown";
    }
}

const char *diag_err_text(uint8_t err)
{
    switch (err) {
    case DIAG_OK:          return "ok";
    case DIAG_ERR_TIMEOUT: return "timeout";
    case DIAG_ERR_NACK:    return "nack";
    case DIAG_ERR_BAD_ID:  return "bad_id";
    case DIAG_ERR_BUS:     return "bus_error";
    case DIAG_ERR_NO_DEV:  return "not_found";
    case DIAG_ERR_PARAM:   return "bad_param";
    case DIAG_ERR_UNKNOWN: return "unknown";
    default:               return "?";
    }
}

/* ================================================================
 * 生命周期
 * ================================================================ */
void diag_init(void)
{
    memset(s_slots, 0, sizeof(s_slots));
    s_count        = 0;
    s_health_mask  = 0;
    s_error_mask   = 0;
    s_healthy_cnt  = 0;
    s_fault_cnt    = 0;
    s_report_cb    = NULL;
    s_last_run_ms  = 0;

    diag_cyc_init();
}

void diag_register(uint8_t id, const char *name, diag_probe_fn probe)
{
    if (id >= DIAG_DEV_COUNT) return;

    diag_slot_t *s = &s_slots[id];
    if (!s->used) {
        s_count++;
    }
    s->used  = 1;
    s->id    = id;
    s->name  = name;
    s->probe = probe;
}

/* ================================================================
 * 跑一轮
 * ================================================================ */
static uint8_t diag_run_slot(diag_slot_t *s, uint8_t retry_fail)
{
    uint8_t tries = 0;
    uint8_t ok    = 0;

    s->item.id         = s->id;
    s->item.status     = 0;
    s->item.err        = DIAG_ERR_UNKNOWN;
    s->item.detail[0]  = '\0';
    s->item.elapsedUs  = 0;

    /* 没注册探针 —— 该外设本机无法自检，报"未知"而不是"正常" */
    if (!s->probe) {
        s->item.status = 2;            /* 未知 */
        s->item.err    = DIAG_ERR_UNKNOWN;
        s->item.tries  = 0;
        return 0;
    }

    for (;;) {
        uint32_t t0 = diag_cyc_now();
        diag_item tmp;

        memset(&tmp, 0, sizeof(tmp));
        tmp.id = s->id;

        ok = (uint8_t)(s->probe(&tmp) ? 1 : 0);
        tries++;

        s->item.elapsedUs = diag_cyc_now() - t0;
        if (s->item.elapsedUs > DIAG_PROBE_TIMEOUT_US) {
            /* 探针跑太久 —— 判为超时，别让它把整机拖住 */
            ok = 0;
            tmp.err = DIAG_ERR_TIMEOUT;
            diag_copy(tmp.detail, sizeof(tmp.detail), "probe timeout");
        } else {
            /* 探针自己填的 detail/err 优先 */
            if (tmp.detail[0] == '\0') tmp.detail[0] = '\0';
            s->item.err = tmp.err;
            diag_copy(s->item.detail, sizeof(s->item.detail), tmp.detail);
        }

        if (ok) break;
        if (tries > retry_fail) break;
    }

    s->item.tries  = tries;
    s->item.status = ok ? 1 : 0;
    if (ok) s->item.err = DIAG_OK;
    return ok;
}

uint8_t diag_run_all(uint8_t retry_fail)
{
    uint32_t health = 0;
    uint32_t errs   = 0;
    uint8_t  healthy = 0;
    uint8_t  fault   = 0;

    for (uint8_t i = 0; i < DIAG_DEV_COUNT; i++) {
        diag_slot_t *s = &s_slots[i];
        if (!s->used) continue;

        uint8_t ok = diag_run_slot(s, retry_fail);

        if (ok) {
            health |= (1UL << i);
            healthy++;
        } else if (s->item.status == 0) {      /* 真异常（不是"未知"） */
            errs |= (1UL << i);
            fault++;
        }
    }

    s_health_mask = health;
    s_error_mask  = errs;
    s_healthy_cnt = healthy;
    s_fault_cnt   = fault;
    s_last_run_ms = HAL_GetTick();

    return healthy;
}

uint8_t diag_run_one(uint8_t id)
{
    if (id >= DIAG_DEV_COUNT) return 0;
    diag_slot_t *s = &s_slots[id];
    if (!s->used) return 0;

    uint8_t ok = diag_run_slot(s, 1);
    /* 单项重跑也要刷新总位图 */
    if (ok) {
        s_health_mask |= (1UL << id);
        s_error_mask  &= ~(1UL << id);
    } else if (s->item.status == 0) {
        s_health_mask &= ~(1UL << id);
        s_error_mask  |= (1UL << id);
    }
    return ok;
}

/* ================================================================
 * 查询
 * ================================================================ */
const diag_item *diag_at(uint8_t idx)
{
    uint8_t seen = 0;
    for (uint8_t i = 0; i < DIAG_DEV_COUNT; i++) {
        if (!s_slots[i].used) continue;
        if (seen == idx) return &s_slots[i].item;
        seen++;
    }
    return NULL;
}

uint8_t diag_count(void)
{
    return s_count;
}

const diag_item *diag_find(uint8_t id)
{
    if (id >= DIAG_DEV_COUNT) return NULL;
    if (!s_slots[id].used) return NULL;
    return &s_slots[id].item;
}

uint32_t diag_health_mask(void) { return s_health_mask; }
uint32_t diag_error_mask(void)  { return s_error_mask; }
uint8_t  diag_healthy_count(void) { return s_healthy_cnt; }
uint8_t  diag_fault_count(void)   { return s_fault_cnt; }

/* ================================================================
 * 报告生成
 * ================================================================ */
uint16_t diag_build_json(char *buf, uint16_t cap)
{
    uint16_t n = 0;
    int first = 1;

    if (!buf || cap == 0) return 0;
    buf[0] = '\0';

    n += (uint16_t)snprintf(buf + n, (cap > n) ? (cap - n) : 0,
                            "{\"ts\":%lu,\"health\":%lu,\"errs\":%lu,\"ok\":%u,\"bad\":%u,\"items\":[",
                            (unsigned long)HAL_GetTick(),
                            (unsigned long)s_health_mask,
                            (unsigned long)s_error_mask,
                            (unsigned)s_healthy_cnt,
                            (unsigned)s_fault_cnt);
    if (n >= cap) { buf[cap - 1] = '\0'; return cap - 1; }

    for (uint8_t i = 0; i < DIAG_DEV_COUNT; i++) {
        const diag_item *it = &s_slots[i].item;
        const char *name;
        int written;

        if (!s_slots[i].used) continue;
        name = diag_dev_name(i);

        written = snprintf(buf + n, (cap > n) ? (cap - n) : 0,
                           "%s{\"id\":\"%s\",\"ok\":%u,\"err\":%u,\"t\":%u,\"us\":%lu",
                           first ? "" : ",",
                           name,
                           (unsigned)it->status,
                           (unsigned)it->err,
                           (unsigned)it->tries,
                           (unsigned long)it->elapsedUs);
        if (written < 0) break;
        n += (uint16_t)written;
        if (n >= cap) { buf[cap - 1] = '\0'; return cap - 1; }

        if (it->detail[0]) {
            written = snprintf(buf + n, (cap > n) ? (cap - n) : 0,
                               ",\"d\":\"%s\"}", it->detail);
            if (written < 0) break;
            n += (uint16_t)written;
        } else {
            written = snprintf(buf + n, (cap > n) ? (cap - n) : 0, "}");
            if (written < 0) break;
            n += (uint16_t)written;
        }
        if (n >= cap) { buf[cap - 1] = '\0'; return cap - 1; }
        first = 0;
    }

    if (n < cap) {
        int written = snprintf(buf + n, cap - n, "]}");
        if (written > 0) n += (uint16_t)written;
    }
    buf[(n < cap) ? n : (cap - 1)] = '\0';
    return n;
}

uint16_t diag_build_text(char *buf, uint16_t cap)
{
    uint16_t n = 0;

    if (!buf || cap == 0) return 0;
    buf[0] = '\0';

    n += (uint16_t)snprintf(buf + n, (cap > n) ? (cap - n) : 0,
                            "health=0x%08lX ok=%u bad=%u\n",
                            (unsigned long)s_health_mask,
                            (unsigned)s_healthy_cnt,
                            (unsigned)s_fault_cnt);
    if (n >= cap) { buf[cap - 1] = '\0'; return cap - 1; }

    for (uint8_t i = 0; i < DIAG_DEV_COUNT; i++) {
        const diag_item *it = &s_slots[i].item;
        int written;

        if (!s_slots[i].used) continue;

        written = snprintf(buf + n, (cap > n) ? (cap - n) : 0,
                           "  %-8s %-4s %-10s %lus %s\n",
                           diag_dev_name(i),
                           (it->status == 1) ? "OK" : (it->status == 2) ? "SKIP" : "FAIL",
                           diag_err_text(it->err),
                           (unsigned long)(it->elapsedUs / 1000UL),
                           it->detail);
        if (written < 0) break;
        n += (uint16_t)written;
        if (n >= cap) { buf[cap - 1] = '\0'; return cap - 1; }
    }

    buf[(n < cap) ? n : (cap - 1)] = '\0';
    return n;
}

/* ================================================================
 * 上报
 * ================================================================ */
void diag_set_report_cb(diag_report_cb cb)
{
    s_report_cb = cb;
}

void diag_trigger(void)
{
    char json[256];

    diag_run_all(1);
    diag_build_json(json, sizeof(json));

    /* 串口只是兜底通道 —— 正式通道是网络（2.4G → S3 → MQTT） */
    dbg_printf("[diag] %s\n", json);

    if (s_report_cb) {
        s_report_cb(json, (uint16_t)strlen(json));
    }
}
