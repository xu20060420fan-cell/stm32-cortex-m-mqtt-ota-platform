/*
 * task_monitor.c
 *
 * 任务栈水位监控实现。只存句柄，不额外占堆。
 */
#include "task_monitor.h"

#include "debug_log.h"

#define TASK_MONITOR_MAX 8U

static TaskHandle_t s_handles[TASK_MONITOR_MAX];
static const char  *s_names[TASK_MONITOR_MAX];
static uint8_t      s_count;

void task_monitor_add(const char *name, TaskHandle_t handle)
{
    if (s_count < TASK_MONITOR_MAX) {
        s_names[s_count] = name;
        s_handles[s_count] = handle;
        s_count++;
    }
}

void task_monitor_print(void)
{
    uint8_t i;

    dbg_printf("[STACK] heap=%u B free |", (unsigned)xPortGetFreeHeapSize());
    for (i = 0U; i < s_count; ++i) {
        /* 单位是字；这是历史最小剩余，越接近 0 越危险 */
        dbg_printf(" %s=%uW", s_names[i],
                   (unsigned)uxTaskGetStackHighWaterMark(s_handles[i]));
    }
    dbg_printf("\r\n");
}
