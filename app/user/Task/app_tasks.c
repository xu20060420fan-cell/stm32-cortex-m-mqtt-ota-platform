/*
 * app_tasks.c —— 业务任务编排实现
 * -----------------------------------------------------------------------------
 * 任务划分（栈单位是 word，4 字节一个）：
 *
 *   imu_task    50ms   读 MPU6050 → 跌落/碰撞判定 → 姿态角
 *   rf_tx_task  200ms  把传感器/状态打成 2.4G 帧发出去
 *   rf_rx_task  50ms   收 S3 的命令（自检/复位/OTA/配置）
 *   alarm_task  500ms  阈值判定 → 蜂鸣器
 *   diag_task   开机一次 跑自检 → 把报告发到 2.4G
 *
 * 优先级安排：imu > rf_rx > rf_tx > alarm > diag
 *   姿态和收命令要"准"，报警和上报可以延后；
 *   自检只在开机跑一次，优先级最低不影响别的。
 *
 * 栈不能给大：F103C8 只有 20K RAM，已经被 FreeRTOS 堆(8K)和
 * 既有任务占掉一大半。传感器任务 384 words、显示任务 384 words 是
 * 既有工程已经验证过的量级，这里照着给。
 * -----------------------------------------------------------------------------
 */
#include "app_tasks.h"

#include <string.h>

#include "app_version.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "main.h"
#include "debug_log.h"
#include "task_monitor.h"
#include "diag.h"
#include "fall_detect.h"
#include "mpu6050.h"
#include "nrf24l01.h"
#include "rf_protocol.h"
#include "sensor_task.h"
#include "lowpower.h"
#include "uart_cmd.h"
#include "ssd1306.h"

/* 既有工程的蜂鸣器接口（buzzer.c 里实现） */
void buzzer_beep(void);

/* ---------------------------------------------------------------------------
 * 低功耗参数（依据 docs/00-总体方案与接线.md §7.1）
 * ------------------------------------------------------------------------- */

/* 按变化上报的阈值：变化不到这个数就不发 2.4G 包，空口功耗降一个数量级 */
#define RF_REPORT_TEMP_EPS     0.2f    /* 0.2 ℃ */
#define RF_REPORT_HUM_EPS      0.5f    /* 0.5 %RH */
#define RF_REPORT_PRESS_EPS    100.0f  /* 100 Pa = 1 hPa */

/* 状态帧固定周期（保活 + 掉线检测） */
#define RF_STATUS_PERIOD_MS    5000U

/* nRF24 的「收听窗口」：每个周期只开这么一会儿，其余时间掉电
 * （掉电 ~26µA，常开 RX 是 ~12mA） */
/* 20ms：150ms 会把 bin 顶到 40960（分区满），STM32 无法正常启动，
 * 所以窗口不能再靠加大来解决，通信问题另找原因。 */
#define RF_RX_WINDOW_MS        20U

/* 深度睡眠（STOP 模式）开关：见 lowpower.c 的说明。
 * 默认关 —— F1 的 RTC 闹钟只有 1 秒粒度，而本工程的采样任务最短 50ms，
 * 直接进 STOP 会让 FreeRTOS 的节拍补偿误差过大。
 * 等把采样节奏改成 1Hz（跌落检测改用 MPU 的自由落体中断唤醒）后再打开。 */
static uint8_t s_deep_sleep_enabled = 0U;

/* --------------------------------------------------------------------------- */
/* 共享状态                                                                    */
/* --------------------------------------------------------------------------- */
static app_state_t     s_state;
static app_thresholds_t s_thr;
static SemaphoreHandle_t s_lock;

/* 【射频互斥】rf_tx_task 与 rf_rx_task 是两条任务，原来都能在任意时刻
 * 调 nrf24_begin_tx()/nrf24_read()/nrf24_power_down()：
 * 发送任务刚切到 PTX，接收任务又把 CONFIG 改成 PRX，
 * 结果就是"命令偶发收不到、上报偶发发不出"，且日志看不出规律。
 * 所有 nRF24 访问必须串行化（同一时刻只有一个任务碰射频）。 */
static SemaphoreHandle_t s_rf_lock;

static void state_lock(void)   { if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY); }
static void state_unlock(void) { if (s_lock) xSemaphoreGive(s_lock); }
/* 用宏而不是函数：LTO 下每多一个调用点就多一份栈帧，固件只剩几十字节余量。 */
#define rf_lock()    do { if (s_rf_lock) xSemaphoreTake(s_rf_lock, portMAX_DELAY); } while (0)
#define rf_unlock()  do { if (s_rf_lock) xSemaphoreGive(s_rf_lock); } while (0)

void app_state_get(app_state_t *out)
{
    if (!out) return;
    state_lock();
    *out = s_state;
    state_unlock();
}

void app_thresholds_get(app_thresholds_t *out)
{
    if (!out) return;
    state_lock();
    *out = s_thr;
    state_unlock();
}

void app_thresholds_set(const app_thresholds_t *t)
{
    if (!t) return;
    state_lock();
    s_thr = *t;
    state_unlock();
}

const char *app_state_name(uint8_t state)
{
    switch (state) {
    case RF_STATE_STEADY:   return "steady";
    case RF_STATE_MOVING:   return "moving";
    case RF_STATE_FREEFALL: return "freefall";
    case RF_STATE_IMPACT:   return "impact";
    case RF_STATE_ALARM:    return "alarm";
    default:                return "unknown";
    }
}

/* 建任务并把结果打出来：堆不够时 xTaskCreate 会失败，
 * 以前忽略返回值的话任务静默不跑，现象是「数据不上报但什么都不报错」。 */
static void create_task_checked(TaskFunction_t fn, const char *name,
                                uint16_t stack_words, UBaseType_t priority)
{
    TaskHandle_t handle = NULL;

    if (xTaskCreate(fn, name, stack_words, NULL, priority, &handle) == pdPASS) {
        dbg_printf("[BOOT] %s %uw ok\r\n", name, (unsigned)stack_words);
        /* 进监控表 —— 少了这一步，这个任务的栈水位就永远看不到。
         * main.c 那份 create_task_checked 是带的，这份原来没带，
         * 于是 alarm/rfrx/rftx/ucmd 恰好是栈最紧、又最没人看的一批。 */
        task_monitor_add(name, handle);
    } else {
        dbg_printf("[BOOT] %s %uw FAIL heap=%u\r\n", name,
                   (unsigned)stack_words, (unsigned)xPortGetFreeHeapSize());
    }
}

/* --------------------------------------------------------------------------- */
/* 2.4G 帧收发                                                                */
/* --------------------------------------------------------------------------- */
static uint8_t s_seq;

/* 把一帧发出去：magic + type + seq + len + payload */
static void rf_send(uint8_t type, const void *payload, uint8_t len)
{
    uint8_t buf[RF_HEADER_LEN + RF_MAX_PAYLOAD];

    if (len > RF_MAX_PAYLOAD) len = RF_MAX_PAYLOAD;

    /* 调用方负责持 rf 锁（rf_tx_task / rf_rx_task 都在锁内调用），
     * 这里不再加锁 —— 普通 mutex 不可重入，套两层会直接死锁。 */

    buf[0] = RF_MAGIC;
    buf[1] = type;
    buf[2] = s_seq++;
    buf[3] = len;
    if (len && payload) {
        memcpy(&buf[RF_HEADER_LEN], payload, len);
    }

    /* 发送前先清空 FIFO，避免把上一次没发完的残留当成这次的 ACK */
    nrf24_begin_tx();
    nrf24_write(buf, (uint8_t)(RF_HEADER_LEN + len));
    /*
     * 发射完 **立刻掉电**（~26µA），不要停在 RX（~12mA）。
     * 射频的自动应答是在发送过程中收的，掉电不影响 ACK；
     * 收下行命令靠 rf_rx_task 的「收听窗口」，见那里的注释。
     */
    nrf24_power_down();

}

/* 把自检结果打包成 DIAG_SUM + N 个 DIAG_DET 发出去 */
static void rf_send_diag(void)
{
    rf_diag_sum_t sum;
    uint8_t i;

    sum.health_mask = diag_health_mask();
    sum.error_mask  = diag_error_mask();
    sum.ok_count    = diag_healthy_count();
    sum.bad_count   = diag_fault_count();
    rf_send(RF_TYPE_DIAG_SUM, &sum, (uint8_t)sizeof(sum));

    /* 明细一帧一个器件：只发异常的，正常的不占空口 */
    for (i = 0; i < DIAG_DEV_COUNT; i++) {
        const diag_item *it = diag_find(i);
        uint8_t payload[RF_MAX_PAYLOAD];
        uint8_t dlen;

        if (!it) continue;
        if (it->status == 1) continue;          /* 正常的不发 */

        payload[0] = (uint8_t)i;
        payload[1] = it->err;
        dlen = (uint8_t)strlen(it->detail);
        if (dlen > (RF_MAX_PAYLOAD - 3)) dlen = (uint8_t)(RF_MAX_PAYLOAD - 3);
        payload[2] = dlen;
        if (dlen) memcpy(&payload[3], it->detail, dlen);
        rf_send(RF_TYPE_DIAG_DET, payload, (uint8_t)(3 + dlen));
    }
}

/* diag 模块的回调：自检跑完（或被手动触发）时把报告发到 2.4G */
static void diag_report_to_rf(const char *json, uint16_t len)
{
    (void)json;
    (void)len;
    rf_send_diag();
}

/* --------------------------------------------------------------------------- */
/* 任务                                                                        */
/* --------------------------------------------------------------------------- */

/* 姿态与跌落 */
static void imu_task(void *pv) __attribute__((unused));
static void imu_task(void *pv)
{
    (void)pv;
    int16_t ax, ay, az, gx, gy, gz;
    float    a_g[3], g_dps[3];
    uint32_t last = HAL_GetTick();

    for (;;) {
        if (mpu6050_read(&ax, &ay, &az, &gx, &gy, &gz)) {
            mpu6050_read_accel_gravity(&a_g[0], &a_g[1], &a_g[2]);
            mpu6050_read_gyro_dps(&g_dps[0], &g_dps[1], &g_dps[2]);

            fall_detect_update(HAL_GetTick(), a_g[0], a_g[1], a_g[2],
                               g_dps[0], g_dps[1], g_dps[2]);

            state_lock();
            s_state.pitch_deg = 0.0f;
            s_state.roll_deg  = 0.0f;
            fall_detect_get_attitude(&s_state.pitch_deg, &s_state.roll_deg);

            if (fall_detect_is_fall() || fall_detect_is_impact()) {
                s_state.state = fall_detect_is_fall() ? RF_STATE_FREEFALL
                                                      : RF_STATE_IMPACT;
                if (fall_detect_is_impact()) {
                    s_state.fall_count++;
                    fall_detect_reset();
                    /* 事件行：EVT <type> <detail>，type 里不能有空格。
                     * 8266 侧 uart_task 目前只认 TL 行，EVT 要等它加一行
                     * 前缀判断才会转发到云端；这里先发出来，串口调试能看见。 */
                    dbg_printf("EVT impact fall=%lu\r\n",
                               (unsigned long)s_state.fall_count);
                } else {
                    dbg_printf("EVT fall\r\n");
                }
            } else {
                /* 没在跌落过程里，且没有阈值告警 → 平稳/移动中 */
                s_state.state = (s_state.alarm_bits ? RF_STATE_ALARM
                                  : (fall_detect_get_state() == 0 ? RF_STATE_STEADY
                                                                : RF_STATE_MOVING));
            }
            state_unlock();
        }

        /* vTaskDelayUntil：固定节拍，不累积漂移 */
        vTaskDelayUntil(&last, pdMS_TO_TICKS(50));
    }
}

/* 2.4G 上行 */
static void rf_tx_task(void *pv)
{
    (void)pv;
    uint32_t last = HAL_GetTick();
    uint32_t last_status = 0U;
    float    last_temp = 0.0f;
    float    last_hum  = 0.0f;
    float    last_press = 0.0f;
    uint8_t  have_last = 0U;
    uint8_t  last_state = 0xFFU;

    for (;;) {
        app_state_t st;
        float d_temp, d_hum, d_press;
        uint8_t changed;

        /*
         * 温湿度/气压由既有的 sensor_task 采集，放在 g_sensor 里。
         * 这里顺手搬进自己的状态：上位机（告警判定、上报）都用同一份快照，
         * 不会出现"温度用 A 的、湿度用 B 的"这种半新半旧的数据。
         * ⚠ 采样失败时保留上一次的��，不要用 0 覆盖 —— 0℃/0%RH 是
         *   "没读到"，不是"真的是 0"，发上去会误触发阈值告警。
         */
        state_lock();
        if (g_sensor.aht_ok) {
            s_state.temp_c = g_sensor.temperature;
            s_state.hum    = g_sensor.humidity;
        }
        if (g_sensor.bmp_ok) {
            s_state.press = g_sensor.pressure;
        }
        /* 电池：采样失败时保留上一次的值（同上，0% 会被当成"电用光"） */
        if (g_sensor.batt_ok) {
            s_state.batt = g_sensor.batt;
        }
        state_unlock();

        app_state_get(&st);

        /* 与上一次上报的值比：变化够大才发包（省空口功耗，见 docs/00 §7.1） */
        d_temp = st.temp_c - last_temp;
        if (d_temp < 0.0f) { d_temp = -d_temp; }
        d_hum  = st.hum - last_hum;
        if (d_hum < 0.0f) { d_hum = -d_hum; }
        d_press = st.press - last_press;
        if (d_press < 0.0f) { d_press = -d_press; }

        changed = 0U;
        if (have_last == 0U) {
            changed = 1U;                        /* 第一次全发 */
        } else if ((d_temp >= RF_REPORT_TEMP_EPS) ||
                   (d_hum  >= RF_REPORT_HUM_EPS) ||
                   (d_press >= RF_REPORT_PRESS_EPS)) {
            changed = 1U;
        } else if (st.state != last_state) {
            changed = 1U;                        /* 状态变了（跌落/告警）必须立刻报 */
        }

        if (changed != 0U) {
            rf_lock();
            /* 传感器数据帧 */
            {
                rf_sensor_t s;
                s.temp_c_x100 = (int16_t)(st.temp_c * 100.0f);
                s.hum_x100    = (int16_t)(st.hum * 100.0f);
                s.press_pa    = (uint32_t)st.press;
                s.state       = st.state;
                s.batt        = st.batt;
                s.pitch_x100  = (int16_t)(st.pitch_deg * 100.0f);
                s.roll_x100   = (int16_t)(st.roll_deg * 100.0f);
                s.ts          = HAL_GetTick();
                rf_send(RF_TYPE_SENSOR, &s, (uint8_t)sizeof(s));
            }
            rf_unlock();
            last_temp  = st.temp_c;
            last_hum   = st.hum;
            last_press = st.press;
            last_state = st.state;
            have_last  = 1U;
        }

        /* 状态帧：固定 5 秒一帧（保活 + 掉线检测，不受温度变化影响） */
        if ((HAL_GetTick() - last_status) >= RF_STATUS_PERIOD_MS) {
            last_status = HAL_GetTick();
            rf_lock();
            {
                rf_status_t s;
                s.state       = st.state;
                s.fall_count  = st.fall_count;
                s.alarm_bits  = st.alarm_bits;
                s.reset_count = st.reset_count;
                /* 版本号随状态帧一起上报：S3 详情页要显示节点固件版本。
                 * APP_VERSION_STRING 是编译期字面量，memcpy 不引入 sprintf。 */
                memcpy(s.fw_ver, APP_VERSION_STRING, sizeof(s.fw_ver));
                rf_send(RF_TYPE_STATUS, &s, (uint8_t)sizeof(s));
            }
            rf_unlock();
            last_state = st.state;
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(200));
    }
}

/* 2.4G 下行：命令与配置 */
static void rf_rx_task(void *pv)
{
    (void)pv;
    uint8_t buf[32];
    uint32_t last_init_try = 0U;

    for (;;) {
        /* 【运行期重试初始化】
         * nrf24_init() 只在 app_tasks_init() 里调一次，失败就永久放弃。
         * 现场有两种必然遇到的情况需要它能在跑起来之后再认芯片：
         *   ① 这块 bench 只有一颗 nRF24，两块板轮流插 —— 上电时没插、
         *      后来插上，原来的代码永远不会再认；
         *   ② 模块刚上电时 3V3 还在爬（板② 2026-11-02 实测：开机第一次
         *      读回 0x00，同一个模块稍后读回正常），一次性判定会误杀。
         * 5 秒试一次，成功/失败都只在"状态变化"时打印，不刷屏。 */
        if (nrf24_inited() == 0U) {
            if ((HAL_GetTick() - last_init_try) >= 5000U) {
                last_init_try = HAL_GetTick();
                (void)nrf24_init();
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /*
         * 收听窗口：每个周期只把射频开 20ms，其余时间掉电。
         * nRF24 常开 RX 是 ~12mA，掉电是 ~26µA —— 这是板子上第二大耗电项。
         * ⚠ 前提：上位机（S3）发命令必须 **重发**，否则命令落在窗口外就丢了。
         *   STM32 这边每 100ms 开一次 20ms 的窗口，丢包率约 20%，
         *   所以 S3 侧要按 ~200ms 重发、直到收到 ACK。
         */
        rf_lock();      /* 整个收听窗口独占射频 */
        (void)nrf24_power_up_rx();
        vTaskDelay(pdMS_TO_TICKS(RF_RX_WINDOW_MS));

        while (nrf24_available()) {
            uint8_t n = nrf24_read(buf);
            if (n >= RF_HEADER_LEN && buf[0] == RF_MAGIC) {
                uint8_t type = buf[1];
                uint8_t len  = buf[3];
                const uint8_t *pl = &buf[RF_HEADER_LEN];

                if (len > (n - RF_HEADER_LEN)) len = (uint8_t)(n - RF_HEADER_LEN);

                if (type == RF_TYPE_CMD && len >= 1) {
                    uint8_t cmd = pl[0];
                    switch (cmd) {
                    case RF_CMD_PING:
                        /* rf_send 内部要拿同一把锁，这里先放开再发，
                         * 发完重新上锁并把射频切回收听态。 */
                        rf_send(RF_TYPE_ACK, &cmd, 1);
                        break;
                    case RF_CMD_SELFTEST:
                        /* 【先 ACK 再执行】网关没收到 ACK 会不停重发，
                         * 自检就会被执行多次；ACK 必须在动作之前发。 */
                        rf_send(RF_TYPE_ACK, &cmd, 1);
                        app_diag_trigger();
                        break;
                    case RF_CMD_GET_STATUS:
                        {
                            app_state_t st;
                            app_state_get(&st);
                            rf_send(RF_TYPE_ACK, &st.state, 1);
                        }
                        break;
                    case RF_CMD_REBOOT:
                        /*
                         * 普通复位：进 bootloader 的 10 秒刷机窗口，
                         * 串口助手/调试时手动抓住窗口刷机用。
                         * 复位前先把状态置好，让 app 复位后不丢跌落计数。
                         * 【先 ACK 再复位】否则网关收不到确认会重发，
                         * 重启后的设备又执行一次复位，形成"反复重启"。
                         */

                        rf_send(RF_TYPE_ACK, &cmd, 1);
                        vTaskDelay(pdMS_TO_TICKS(20));   /* 给 ACK 发出/网关收到的时间 */
                        NVIC_SystemReset();
                        break;
                    case RF_CMD_OTA_START:
                        /*
                         * 网络升级：复位前先在备份寄存器 DR1 里留下 OTA 魔数。
                         * bootloader 上电看到 0x4F54（'OT'）就进入"常驻等待"，
                         * 不受 10 秒窗口限制 —— ESP8266 连 WiFi、查版本、
                         * 下载固件都要时间，10 秒肯定不够。
                         * 魔数由 bootloader 在进命令行前清掉，只生效一次。
                         * 【两边约定】0x4F54 = 'OT'，bootloader appbl.c 同值。
                         */
                        /* 同样先 ACK 再进 OTA，避免网关重发导致多次复位/多次进刷机。 */

                        rf_send(RF_TYPE_ACK, &cmd, 1);
                        vTaskDelay(pdMS_TO_TICKS(20));
                        /* 【F1 备份域默认写保护】只开 BKP 时钟不够，
                         * 还要置 PWR_CR 的 DBP，否则 BKP->DR1 的写入被硬件
                         * 静默丢弃，复位后 bootloader 看不到魔数 → S3 点"升级"
                         * 只会普通重启。和 uart_cmd.c 的 enter_ota() 同一条规则。 */
                        PWR->CR |= PWR_CR_DBP;
                        __HAL_RCC_BKP_CLK_ENABLE();
                        BKP->DR1 = 0x4F54U;
                        NVIC_SystemReset();
                        break;
                    default:
                        rf_send(RF_TYPE_ACK, &cmd, 1);
                        break;
                    }
                } else if (type == RF_TYPE_CFG && len >= 1) {
                    /*
                     * 配置帧（阈值）：S3 从云端取到后转下来。
                     * 布局：见下（逐字节，不用结构体以免对齐差异）。
                     */
                    app_thresholds_t t;
                    app_thresholds_get(&t);
                    if (len >= 10) {
                        int16_t v;
                        memcpy(&v, &pl[0], 2); t.temp_max = v / 100.0f;
                        memcpy(&v, &pl[2], 2); t.temp_min = v / 100.0f;
                        memcpy(&v, &pl[4], 2); t.hum_max  = v / 100.0f;
                        memcpy(&v, &pl[6], 2); t.hum_min  = v / 100.0f;
                        memcpy(&v, &pl[8], 2); t.fall_g   = v / 100.0f;
                        app_thresholds_set(&t);
                        /* 【阈值回执】S3 没收到回执就会按重发策略反复下发 CFG，
                         * 节点每收一次就重写一次阈值（还会触发蜂鸣）。
                         * 回一个专用码，让网关知道"这次确实落盘了"。 */
                        {
                            uint8_t ack = 0x10U;
                            rf_send(RF_TYPE_ACK, &ack, 1);
                        }
                    }
                }
            }
        }

        /* 窗口结束，射频掉电 */
        (void)nrf24_power_down();
        rf_unlock();

        /*
         * 顺手做一次轻量低功耗：__WFI 让 CPU 停到下一个中断。
         * 不进 STOP 模式 —— F103 的 STOP 会把 HSI 也停掉，SysTick 随之停摆，
         * HAL 的 uwTick 会丢时间，FreeRTOS 的节拍就乱了（表现为任务周期漂移、
         * 偶发看门狗复位）。要真正省电得配 RTC 唤醒源并补偿 tick，
         * 那是下一步单独做、且必须上板验证的事。
         *
         * 【2026-10-25 更新】轻量低功耗现在由两处负责：
         *   ① FreeRTOS 的 tickless idle（configUSE_TICKLESS_IDLE=1）：
         *      空闲时 CPU 直接睡到下一个任务截止点，不用每 10ms 醒一次。
         *   ② nRF24 的收听窗口：只开 20ms，其余时间掉电（~26µA）。
         */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ---------------------------------------------------------------------------
 * 低功耗协调任务
 * ------------------------------------------------------------------------- */

/**
 * 优先级最低，只有在别的任务都阻塞时才会被调度到 —— 正好是「系统空闲」的时刻。
 * 它一上来就判断能不能睡，能睡就进 STOP（由 RTC 闹钟 / EXTI / USART 叫醒）。
 */
static void lp_task(void *pv) __attribute__((unused));
static void lp_task(void *pv)
{
    (void)pv;

    for (;;) {
        /* 让别的任务先把本周期的事干完 */
        vTaskDelay(pdMS_TO_TICKS(50));

        if ((s_deep_sleep_enabled != 0U) && (lp_can_sleep() != 0U)) {
            lp_enter();
        }
    }
}

/* 阈值告警 */
static void alarm_task(void *pv)
{
    uint32_t last = HAL_GetTick();
    uint8_t  alarming = 0;
    uint32_t last_beep = 0;
    uint8_t  last_bits = 0;

    (void)pv;

    for (;;) {
        app_state_t st;
        app_thresholds_t t;
        uint8_t bits = 0;

        /* 1) 把传感器采集结果搬进全局状态 —— TL 遥测、OLED 首页、阈值判定
         *    都从这一份快照取，不会出现"温度用 A 的、湿度用 B 的"。
         *    采样失败时保留上一次的值，不要用 0 覆盖：0℃/0%RH 是"没读到"，
         *    填 0 会误触发阈值告警。 */
        state_lock();
        if (g_sensor.aht_ok) {
            s_state.temp_c = g_sensor.temperature;
            s_state.hum    = g_sensor.humidity;
        }
        if (g_sensor.bmp_ok) {
            s_state.press = g_sensor.pressure;
        }
        /* 电池：采样失败时保留上一次的值（同上，0% 会被当成"电用光"） */
        if (g_sensor.batt_ok) {
            s_state.batt = g_sensor.batt;
        }
        state_unlock();

        app_state_get(&st);
        app_thresholds_get(&t);

        if (t.alarm_enable) {
            if (st.temp_c > t.temp_max) bits |= 0x01;
            if (st.temp_c < t.temp_min) bits |= 0x02;
            if (st.hum    > t.hum_max)  bits |= 0x04;
            if (st.hum    < t.hum_min)  bits |= 0x08;
            /* 气压：s_state.press 是 Pa，阈值是 hPa，先换算再比。
             * BMP280 没采到（bmp_ok=0）时 press 保留上一次的值，不会误报 0。 */
            if (st.press / 100.0f > t.press_max) bits |= 0x10;
            if (st.press / 100.0f < t.press_min) bits |= 0x20;
        }

        state_lock();
        s_state.alarm_bits = bits;
        state_unlock();

        /* 超限就周期性鸣叫：进入告警先响一声，之后每 2 秒再响一次，
         * 直到数值回到阈值内。原来的逻辑只在告警位上升沿响一次，
         * 现场容易以为"阈值没生效"。 */
        if (bits) {
            uint32_t now = HAL_GetTick();
            if (!alarming || (uint32_t)(now - last_beep) >= 2000U) {
                alarming = 1;
                last_beep = now;
                buzzer_beep();
            }
        } else {
            alarming = 0;
        }

        /* 事件行：EVT <type> <detail>（type 里不能有空格）。
         * 阀值是从云端下发的 TH 行里来的，所以阀值改了就算一次事件，
         * 也能在串口看到。 */
        if (bits != last_bits) {
            if (bits != 0U) {
                dbg_printf("EVT alarm threshold bits=0x%02X T=%ld H=%ld\r\n",
                           (unsigned)bits,
                           (long)(st.temp_c * 100.0f),
                           (long)(st.hum * 100.0f));
            } else {
                dbg_puts("EVT alarm_clear threshold back to normal\r\n");
            }
            last_bits = bits;
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(500));
    }
}

/* 开机自检 → 报告发到 2.4G */
static void diag_task(void *pv)
{
    (void)pv;

    /* 传感器任务还在起，先等它把总线占稳 */
    vTaskDelay(pdMS_TO_TICKS(500));

    diag_run_all(1);
    rf_send_diag();

    dbg_printf("[diag] selftest h=0x%08lX ok=%u bad=%u\r\n",
               (unsigned long)diag_health_mask(),
               (unsigned)diag_healthy_count(),
               (unsigned)diag_fault_count());

    /* 任务只跑一次，之后挂起（vTaskSuspend），省一个任务的栈 */
    vTaskSuspend(NULL);
}

void app_diag_trigger(void)
{
    diag_trigger();     /* 会调 diag_report_to_rf → 把报告发出去 */
}

/* --------------------------------------------------------------------------- */
/* 初始化                                                                      */
/* --------------------------------------------------------------------------- */
void app_tasks_init(void)
{
    /* 状态初值 */
    memset(&s_state, 0, sizeof(s_state));
    s_state.state = RF_STATE_STEADY;

    s_thr.temp_max     = 35.0f;
    s_thr.temp_min     = 5.0f;
    s_thr.hum_max      = 90.0f;
    s_thr.hum_min      = 20.0f;
    s_thr.press_max    = 1200.0f;
    s_thr.press_min    = 300.0f;
    s_thr.fall_g       = 2.5f;
    s_thr.alarm_enable = 1;
    s_thr.report_ms    = 2000;

    s_lock = xSemaphoreCreateMutex();
    s_rf_lock = xSemaphoreCreateMutex();

    /* --- 2.4G 初始化（保留：部分站的东方 S3 面板靠它取数据） --- */
    if (nrf24_init()) {
        static const uint8_t kAddr[5] = { 'N', 'O', 'D', 'E', '1' };
        /* 【必须跟 NRF24_DEFAULT_CHANNEL 走】这里原来写死 76，
         * 把头文件改到 125 后运行时又被覆盖回 76，
         * 导致 STM32 在 76、S3 在 125 —— 两端从头到尾就没同过信道。 */
        nrf24_set_channel(NRF24_DEFAULT_CHANNEL);
        nrf24_set_rx_addr(kAddr);
        nrf24_set_tx_addr(kAddr);
    } else {
        dbg_puts("[rf] nRF24 init fail\r\n");
    }

    /* --- 自检（on-demand，走 2.4G 的 SELFTEST 命令）---
     * 【这三行原来一句都没调】diag.c 的表是空的、报告回调是 NULL，
     * 于是 RF_CMD_SELFTEST → diag_trigger() → 打出一份"谁都没注册"的报告
     * 然后丢掉。app_tasks.c 里的 diag_report_to_rf 因为没人注册，
     * 编译器早就报了"defined but not used"——那就是在提示这条链断了。
     * 注意：周期性的 diag 任务仍然是去掉的状态（见下面注释），
     * 这里只把"按需自检"这条命令接通。 */
    diag_init();
    diag_set_report_cb(diag_report_to_rf);
    diag_probes_register();

    /* --- 任务 ---
     *   alarm_task → 把 AHT20/BMP280 的值搬进 s_state，做阀值判定，告警时发 EVT 事件行
     *   rf_tx/rx   → 2.4G 上行/下行（与 S3 面板通讯）
     *   uart_cmd   → 收 TH 阀值行 / 发 TL 遥测行（给 8266 → 云端）
     *
     * 【去掉姿态传感器】MPU6050 不再初始化、不再建 imu_task，
     * 不再读取姿态（原先它的读取会把整块板拉进 HardFault）。
     * 跌落检测依赖姿态，一并去掉；自检 diag 单独走 2.4G 上报，一并去掉。 */
    /* alarm_task 原来只有 128 字（512B）：它每轮要做 app_state/app_thresholds
     * 两份快照 + 阀值判定 + dbg_printf（自带 128B 栈缓冲）+ 发 EVT 事件行，
     * 是最容易顶到栈顶的一个；而栈溢出的下场是钩子里永久停机 = 整机卡死。
     * 128→192（多 256B 堆）。堆 12288B，九个任务栈合计约 8.6KB，加这 256B
     * 仍在预算内，而且现在所有任务都进了 task_monitor ——
     * 开机日志里的 [STACK] 水位线才是真数据，静态算不出来（本工程开了 LTO，
     * 函数被内联合并，.su 的帧长和最终 ELF 对不上）。 */
    create_task_checked(alarm_task, "alarm", 192, 2);
    create_task_checked(rf_rx_task, "rfrx",  192, 3);
    create_task_checked(rf_tx_task, "rftx",  192, 2);
    uart_cmd_init();        /* 8266→APP 串口命令（TH 阀值 / OTA 触发） */
}
