/*
 * sched.cpp —— 协作式调度器 + 环形消息队列
 */
#include "sched.h"
#include "config.h"
#include <string.h>

/* 四个任务。定义在各自的 .cpp 里，这里只负责轮流调用。 */
void wifiTask(void);
void mqttTask(void);
void uartTask(void);
void otaTask(void);

#define SCHED_TASK_COUNT 4

static void (*const s_tasks[SCHED_TASK_COUNT])(void) = {
    wifiTask, mqttTask, uartTask, otaTask
};

static const char* const s_taskNames[SCHED_TASK_COUNT] = {
    "wifi", "mqtt", "uart", "ota"
};

/* 单个任务一次最多该占多久。
 * 【为什么要盯着这个】调度器是协作式的：任何一个任务里有一处
 * 不喂软狗的长阻塞（同步扫描、WebSocket 重连、socket 忙等），
 * 顶上的 Soft WDT（~6.3 秒）就把整台 8266 复位，表现出来是
 * "过十几秒又活过来"。把超过 1500ms 的那一段当场打印出来，
 * 串口一接就能指认是哪个人在堵路，不用靠猜。 */
#define SCHED_SLOW_MS 1500UL

/* 当前正在跑哪个任务（-1 = 不在任务里） */
static int     s_curTask = -1;
/* 调度器是否正在跑（用来防重入） */
static bool    s_running = false;
/* 嵌套转动的层数，防止互相等待时无限递归 */
static uint8_t s_depth = 0;

static void schedCall(int i)
{
    uint32_t t0 = millis();

    s_curTask = i;
    s_tasks[i]();

    {
        uint32_t took = (uint32_t)(millis() - t0);
        if (took >= SCHED_SLOW_MS) {
            LOG_SERIAL.printf("[sched] 任务 %s 单次占用 %lu ms（堆 %u）\n",
                          s_taskNames[i], (unsigned long)took,
                          (unsigned)ESP.getFreeHeap());
        }
    }
    s_curTask = -1;
}

void schedRun(void)
{
    int i;

    if (s_running) {
        return;                 /* 已经在里面了，别重入 */
    }
    s_running = true;

    for (i = 0; i < SCHED_TASK_COUNT; i++) {
        schedCall(i);
    }

    s_running = false;
}

void schedPumpOthers(void)
{
    int saved;
    int i;

    if (!s_running) {
        /* 不在任务里（正常流程调用）→ 直接整体转一遍 */
        schedRun();
        return;
    }
    if (s_depth >= 2) {
        return;                 /* 嵌套太深了，再转会无限递归 */
    }

    saved = s_curTask;
    s_depth++;

    for (i = 0; i < SCHED_TASK_COUNT; i++) {
        if (i == saved) {
            continue;           /* 当前任务不能重入 */
        }
        schedCall(i);
    }

    s_depth--;
    s_curTask = saved;
}

/* ================================================================
 * 环形消息队列
 * ================================================================ */
MsgQueue::MsgQueue(void* storage, uint16_t capacity, uint16_t itemSize)
    : buf_((uint8_t*)storage),
      cap_(capacity),
      itemSize_(itemSize),
      head_(0),
      tail_(0),
      count_(0)
{
}

bool MsgQueue::push(const void* item)
{
    if (count_ >= cap_) {
        return false;
    }
    memcpy(buf_ + (size_t)head_ * itemSize_, item, itemSize_);
    head_ = (uint16_t)((head_ + 1) % cap_);
    count_++;
    return true;
}

bool MsgQueue::pop(void* item, uint32_t timeoutMs)
{
    uint32_t start = millis();

    for (;;) {
        if (count_ > 0) {
            memcpy(item, buf_ + (size_t)tail_ * itemSize_, itemSize_);
            tail_ = (uint16_t)((tail_ + 1) % cap_);
            count_--;
            return true;
        }
        if (timeoutMs == 0) {
            return false;
        }
        if ((uint32_t)(millis() - start) >= timeoutMs) {
            return false;
        }

        /* 【为什么要转调度器】
         * 队列空的时候如果只是干等，别的任务就停摆了 ——
         * 串口会漏收 MCU 的应答，MQTT 会掉线。
         * 所以等的时候把别的任务转一遍。 */
        schedPumpOthers();
        delay(1);
    }
}

void MsgQueue::clear(void)
{
    head_  = 0;
    tail_  = 0;
    count_ = 0;
}

uint16_t MsgQueue::count(void) const
{
    return count_;
}
