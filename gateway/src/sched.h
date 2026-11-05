/*
 * sched.h —— 协作式调度器 + 极简消息队列
 *
 * ================================================================
 * 一、为什么不是 FreeRTOS
 * ================================================================
 * PlatformIO 的 ESP8266 Arduino 框架（framework-arduinoespressif8266 3.1.2）
 * 自带的 SDK 是 **NONOS SDK**：
 *   - 链接库：tools/sdk/lib/NONOSDK22x_190703/ 里没有 libfreertos.a
 *   - 头文件：tools/sdk/include/ 里没有 freertos/FreeRTOS.h
 *   - 核心入口用的是 ets_task()（SDK 自己的任务），不是 xTaskCreate
 * 也就是说，在这套工具链下 xTaskCreate / xQueueCreate / vTaskDelay 全都用不了。
 * （有 FreeRTOS 的是另一套 framework = esp8266-rtos-sdk，但那是裸 SDK，
 *   没有 Arduino API，本工程用不上。）
 *
 * 所以这里改成 Arduino 核心本来就有的单线程模型：
 *   loop() 里轮流调用四个"任务"，谁干完一段就立刻返回。
 *   需要等待的地方（等 READY、等 OK、等 HTTP 数据）把调度器转起来，
 *   保证别的任务不会被饿死 —— 等价于"可抢占"，只是切换点在显式的等待点。
 *
 * ================================================================
 * 二、消息队列
 * ================================================================
 * 自己实现一个环形队列，接口跟 FreeRTOS 的 xQueueSend/xQueueReceive 很像。
 * 以后真换成 FreeRTOS，只要改这一层，上层代码不用动。
 * 存储区由调用方给（静态数组），不动态分配 —— ESP8266 的堆很金贵。
 */
#ifndef SCHED_H
#define SCHED_H

#include <Arduino.h>
#include <stdint.h>

class MsgQueue {
public:
    /* storage 必须是 cap_ * itemSize_ 字节，生命周期至少和队列一样长 */
    MsgQueue(void* storage, uint16_t capacity, uint16_t itemSize);

    /* 入队。满了返回 false（不阻塞，避免把接收路径卡住） */
    bool push(const void* item);

    /* 出队。timeoutMs == 0 表示不等，立即返回。
     * 等待期间会把调度器转起来（见 schedPumpOthers），别的任务不会饿死。 */
    bool pop(void* item, uint32_t timeoutMs);

    void   clear(void);
    uint16_t count(void) const;

private:
    uint8_t* buf_;
    uint16_t cap_;
    uint16_t itemSize_;
    uint16_t head_;
    uint16_t tail_;
    uint16_t count_;
};

/* loop() 里调用：把四个任务依次转一遍 */
void schedRun(void);

/* 在某个任务内部等待时调用：把**别的**任务转一遍（不动当前任务） */
void schedPumpOthers(void);

#endif /* SCHED_H */
