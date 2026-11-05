/*
 * uart_bridge.cpp —— SoftwareSerial ↔ 协议引擎的适配
 */
#include "uart_bridge.h"
#include "app_ctx.h"

#if !STM32_LINK_UART0
/* RX=GPIO13(D7)←PA9, TX=GPIO15(D8)→PA10 —— EspSoftwareSerial 构造参数顺序是 (rx, tx) */
SoftwareSerial g_stm32Serial(STM32_UART_RX_PIN, STM32_UART_TX_PIN);
#endif

SerialSink g_serialSink;

size_t SerialSink::write(const uint8_t* data, size_t len)
{
    if (len == 0) {
        return 0;
    }
    /* 【为什么没有互斥量】
     * 协作式调度是单线程的：uart_task 只读、ota_task 只写，
     * 两者不会同时执行。 */
    /* 【这里绝对不能再调 g_stm32Serial.flush()】
     * 本核心的 SoftwareSerial::flush() 是 UARTBase::flush()
     * （libraries/SoftwareSerial/src/SoftwareSerial.cpp:444），它干的事是
     * **把接收缓冲清空**，而不是"等发送排空"—— 软串口的发送本来就是中断里
     * 逐位打出去的，没有可排空的队列。
     * 原来每次写完都调它，等于每次向 STM32 开口之前先把对面已经发过来、
     * 还没取走的行全丢掉：READY / OK / 横幅 / ? 的 12 行应答都可能被吃掉，
     * 于是协议引擎只能等到超时（每包最长 OTA_T_CHUNK_MS），
     * 整轮 OTA 反复失败重试。 */
    return LINK_SERIAL.write(data, len);
}

bool SerialSink::readLine(char* out, size_t cap, uint32_t timeoutMs)
{
    if (cap == 0) {
        return false;
    }
    out[0] = '\0';

    /* 从队列里取一行。
     * 队列空着的时候，MsgQueue::pop 会把别的任务转起来
     * （wifi / mqtt / uart），所以等应答的这段时间里
     * 串口照样在收、MQTT 照样在跑。 */
    if (!g_uartLineQueue.pop(out, timeoutMs)) {
        return false;
    }
    return true;
}

void SerialSink::flushInput(void)
{
    /* 把队列里残留的行全部丢掉。
     * 不丢的话，上一条命令的尾巴会被当成这一条命令的应答。 */
    g_uartLineQueue.clear();
}
