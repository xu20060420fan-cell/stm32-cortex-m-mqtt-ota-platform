/*
 * uart_bridge.h —— 把 SoftwareSerial 包成协议引擎要的 OtaSink
 *
 * 【为什么收发分两个模块】
 *   uart_task 只干一件事：把串口收到的字节拼成整行，塞进队列。
 *   ota_task 只干另一件事：从队列里取行、按协议发命令。
 *   这样"下载固件"这种耗时十几秒的事情不会堵住串口接收，
 *   MCU 回的每一行 OK 都能第一时间被取走。
 */
#ifndef UART_BRIDGE_H
#define UART_BRIDGE_H

#include <Arduino.h>

#include "ota_proto.h"

class SerialSink : public OtaSink {
public:
    /* 发字节。整段写完才返回（协议要求一次 write 发完整包）。 */
    size_t write(const uint8_t* data, size_t len) override;

    /* 从队列里取一行（uart_task 已经把 '\r'/'\n' 收尾的行放进来了）。
     * 超时返回 false。等待期间别的任务照常跑。 */
    bool readLine(char* out, size_t cap, uint32_t timeoutMs) override;

    /* 丢掉队列里还没被取走的行。发新命令前调用。 */
    void flushInput() override;
};

extern SerialSink g_serialSink;

#endif /* UART_BRIDGE_H */
