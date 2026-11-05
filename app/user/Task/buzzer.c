/*
 * buzzer.c —— 蜂鸣器（低电平触发）
 *
 * 2026-10-24 修正两处老问题：
 *   1) 老代码驱动的其实是 **LED1**（PA2），不是蜂鸣器；
 *      而且 PA2 现在是 USART2_TX（给 ESP8266 下载器用），必须改到真正的蜂鸣器脚。
 *   2) 老代码两行都写 GPIO_PIN_RESET —— 低电平触发的蜂鸣器会**一直响**。
 *      正确逻辑：拉低=响，抬高=不响。
 *
 * 低电平触发的含义：输出低 → 蜂鸣器通电 → 响；输出高 → 断开 → 不响。
 *
 * 2026-11-03 现场：阈值超了、EVT alarm 也打到后台了，但一声不响。
 * bootloader 里同一根 PB8 直接写就能响 —— 硬件和引脚没问题，
 * 断在"给信号量 → buzzer_task 醒来 → 写脚"这条链上（任务/堆/调度任一处
 * 出问题都会静默失效，而且完全看不出来）。现在改成在**调用者任务里直接写脚**：
 * 链路上只剩一次 GPIO 写，不再依赖额外任务和信号量。
 * 代价是调用者会被阻塞约 0.6 秒 —— 调用方都是任务（告警/按键/阈值回执），
 * 这个时长可以接受，换来的是"一定会响"。
 */
#include "buzzer.h"

#include "FreeRTOS.h"
#include "task.h"

/* 一次"滴"的时长（毫秒）与一组里的次数 */
#define BUZZER_ON_MS   120U
#define BUZZER_TIMES   3U

void buzzer_init(void)
{
    /* 初始状态必须是不响（高电平），否则上电就嗡嗡嗡 */
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);

    /* 开机自检：两声短响。
     * 现场"阈值超了但没声音"时，这两声能把问题一刀切开：
     *   响 → GPIO/蜂鸣器硬件是好的，问题在阈值或触发链路；
     *   不响 → 接线/驱动方式/蜂鸣器本体有问题。
     * 此时调度器还没启动，用 HAL_Delay 阻塞是安全的。 */
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    HAL_Delay(120U);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    HAL_Delay(120U);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    HAL_Delay(120U);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
}

/* 响一组：3 声短响（"哔哔哔"），总时长约 0.6 秒。
 * 调度器已经在跑时用 vTaskDelay（不霸占 CPU）；还没跑起来（理论上只有
 * 初始化阶段）退回 HAL_Delay。 */
void buzzer_beep(void)
{
    uint8_t i;

    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) {
        for (i = 0U; i < BUZZER_TIMES; i++) {
            HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
            HAL_Delay(BUZZER_ON_MS);
            HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
            if (i < (BUZZER_TIMES - 1U)) {
                HAL_Delay(BUZZER_ON_MS);
            }
        }
        return;
    }

    for (i = 0U; i < BUZZER_TIMES; i++) {
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);  /* 低=响 */
        vTaskDelay(pdMS_TO_TICKS(BUZZER_ON_MS));
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);    /* 高=停 */
        if (i < (BUZZER_TIMES - 1U)) {
            vTaskDelay(pdMS_TO_TICKS(BUZZER_ON_MS));
        }
    }
}
