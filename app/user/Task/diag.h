/*
 * diag.h —— 外设自检 / 诊断框架
 * -----------------------------------------------------------------------------
 * 设计目标
 *   1. 上电后一次性把"板子上每个外设到底活没活"摸清楚；
 *   2. 结果**不依赖串口** —— 串口只是调试期的兜底通道，正式通道是
 *      「2.4G → ESP32-S3 → MQTT → 服务器」，因为产线/现场没有 USB 线；
 *   3. 每个外设自己注册自己的探针，框架不认识任何具体驱动，
 *      加一个外设只改一处（注册），不改框架；
 *   4. 探针可以失败、可重试、可"未知"（该外设本机无法自检），
 *      三态而不是二态 —— 蜂鸣器、按键这类没法自己证明自己活着的外设，
 *      报"未知"比报"正常"诚实。
 *
 * 使用
 *   diag_init();                       // 建表、准备缓冲
 *   diag_register(DIAG_DEV_AHT20, "aht20", probe_aht20);   // 驱动自己注册
 *   ...
 *   diag_run_all();                    // 跑一轮全量自检（阻塞几百 ms）
 *   diag_build_json(buf, sizeof buf);  // 机器可读，上报用
 *   diag_build_text(buf, sizeof buf);  // 人话，串口/日志用
 *
 * -----------------------------------------------------------------------------
 * 线程模型
 *   自检只在启动任务里跑一次（以及串口/后台手动触发时再跑）。
 *   运行期不再碰硬件总线，所以不需要和传感器任务抢 I2C —— 但仍建议
 *   在传感器任务尚未启动、或已挂起时调用。
 * -----------------------------------------------------------------------------
 */
#ifndef CAMPUS_DIAG_H
#define CAMPUS_DIAG_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * 一、外设清单
 * ================================================================
 * ⚠ 只增不改：下标一旦分配出去就不能再改，否则服务器上存的
 *   历史报告会错位。新增外设一律追加到末尾。
 */
typedef enum {
    DIAG_DEV_SELF    = 0,   /* 板卡自身：时钟/复位原因/供电 */
    DIAG_DEV_AHT20   = 1,   /* 温湿度 */
    DIAG_DEV_BMP280  = 2,   /* 气压/温度 */
    DIAG_DEV_MPU6050 = 3,   /* 六轴（姿态/跌落） */
    DIAG_DEV_SSD1306 = 4,   /* 128x160 OLED */
    DIAG_DEV_W25Q    = 5,   /* 外置 SPI Flash */
    DIAG_DEV_NRF24   = 6,   /* 2.4G 无线 */
    DIAG_DEV_BUZZER  = 7,   /* 蜂鸣器（低电平触发） */
    DIAG_DEV_BUTTON  = 8,   /* 按键 */
    DIAG_DEV_LINK    = 9,   /* 2.4G 对端（ESP32-S3）在线 */
    DIAG_DEV_COUNT   = 10
} diag_dev_id;

/* ================================================================
 * 二、探针结果
 * ================================================================ */
typedef enum {
    DIAG_OK          = 0,   /* 正常 */
    DIAG_ERR_TIMEOUT = 1,   /* 无应答 / 超时 */
    DIAG_ERR_NACK    = 2,   /* I2C 应答了但是 NACK（地址错/未上电） */
    DIAG_ERR_BAD_ID  = 3,   /* 应答了但 ID 不对（型号不对/总线串了） */
    DIAG_ERR_BUS     = 4,   /* SPI/串口 通信失败 */
    DIAG_ERR_NO_DEV  = 5,   /* 明确没有这个器件 */
    DIAG_ERR_PARAM   = 6,   /* 器件在但配置/参数不对 */
    DIAG_ERR_UNKNOWN = 7    /* 无法自检（蜂鸣器/按键这类） */
} diag_err;

/* 单个外设的自检结果 */
typedef struct {
    uint8_t  id;              /* diag_dev_id */
    uint8_t  status;          /* 1=正常 0=异常 2=未知（无法自检） */
    uint8_t  err;             /* diag_err，status=1 时为 0 */
    uint8_t  tries;           /* 实际试了几次 */
    uint32_t elapsedUs;       /* 本次探测耗时（微秒） */
    char     detail[28];      /* 补充信息："who_am_i=0x70" / "jedic=EF4015" */
} diag_item;

/* 探针函数：填 item->detail / item->err，返回 1=正常 0=异常。
 * 探针内部**不允许**长时间阻塞（I2C/SPI 的超时由驱动自己兜住）。 */
typedef int (*diag_probe_fn)(diag_item *item);

/* ================================================================
 * 三、API
 * ================================================================ */

/* 建表、准备缓冲。必须在任何 diag_register 之前调用 */
void diag_init(void);

/*
 * 注册本工程自带的探针（AHT20 / BMP280 / SSD1306 / I2C 总线扫描 / 板卡自身）。
 * 各外设的探针实现在 diag_probes.c 里，与框架分开。
 */
void diag_probes_register(void);

/*
 * 注册一个外设的探针。
 *   id     外设编号
 *   name   短名（进 JSON 用，如 "aht20"）
 *   probe  探针函数；传 NULL 表示"这个外设本机无法自检"，
 *           自检时它的状态会是"未知"而不是"正常"
 * 同一个 id 重复注册会覆盖前面的（用于热替换探针）。
 */
void diag_register(uint8_t id, const char *name, diag_probe_fn probe);

/*
 * 跑一轮全量自检。
 *   retry_fail  失败的探针额外重试几次（I2C 的偶发 NACK 很常见，
 *                重试一次能救回绝大多数虚警）
 * 返回"正常"的个数；任何一项异常都会把总的"健康位"置 0。
 */
uint8_t diag_run_all(uint8_t retry_fail);

/* 只重跑某一项（后台手动触发时用） */
uint8_t diag_run_one(uint8_t id);

/* 最近一轮的结果 */
const diag_item *diag_at(uint8_t idx);
uint8_t            diag_count(void);
const diag_item   *diag_find(uint8_t id);

/*
 * 健康位图：bit i = 1 表示 id=i 的外设正常。
 * 这是给"远端一眼看出这台设备哪坏了"用的 —— 上报一个 32 位整数即可。
 * "未知"状态（status=2）**不置位**，也**不影响**整体健康判定
 * （蜂鸣器坏了不该让整台设备被判死）。
 */
uint32_t diag_health_mask(void);
uint32_t diag_error_mask(void);   /* bit i = 1 表示 id=i 当前有错误 */
uint8_t  diag_healthy_count(void);
uint8_t  diag_fault_count(void);

/* 外设短名 / 错误码文案（给串口和上报用） */
const char *diag_dev_name(uint8_t id);
const char *diag_err_text(uint8_t err);

/*
 * 生成上报用的 JSON（机器可读）。
 *   {"ts":123456,"health":3,"errs":1,"items":[{"id":"aht20","ok":1,"err":0,"d":"..."}]}
 * 返回写入的字节数（不含结尾 '\0'）。buf 不够时截断并保证结尾 '\0'。
 */
uint16_t diag_build_json(char *buf, uint16_t cap);

/*
 * 生成人话报告（串口日志 / 调试用）。
 * 形如：
 *   [diag] aht20    OK      who_am_i=0x70
 *   [diag] bmp280   FAIL    nack (err=2, tries=2)
 */
uint16_t diag_build_text(char *buf, uint16_t cap);

/* 触发一次全量自检并把报告交给回调（供 2.4G/MQTT 通道发出去）。
 * 回调在诊断任务上下文里被调用，不要在里面长时间阻塞。 */
typedef void (*diag_report_cb)(const char *json, uint16_t len);
void diag_set_report_cb(diag_report_cb cb);

/* 手动触发一次自检 + 上报（后台/串口命令用） */
void diag_trigger(void);

#ifdef __cplusplus
}
#endif

#endif /* CAMPUS_DIAG_H */
