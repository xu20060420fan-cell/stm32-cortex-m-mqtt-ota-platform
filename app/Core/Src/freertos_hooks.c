/* freertos_hooks.c
 *
 * FreeRTOS 应用层回调（钩子）的弱实现占位。
 *
 * 为什么需要这个文件：
 *   FreeRTOSConfig.h 里开启了 configCHECK_FOR_STACK_OVERFLOW = 2，
 *   该选项要求用户必须提供 vApplicationStackOverflowHook()，否则链接阶段
 *   会报 "undefined reference to vApplicationStackOverflowHook"。
 *
 * 注意：这些钩子是“应用提供”的函数，FreeRTOS 内核只声明、不实现，
 *   所以必须出现在某个被编译进工程的 .c 文件里。
 */

#include "FreeRTOS.h"
#include "task.h"
#include "debug_log.h"

/*
 * 这三个钩子可能跑在"栈已经坏了"或"中断被关"的上下文里，
 * 所以一律只用 dbg_puts（不格式化、几乎不占栈），绝不能用 dbg_printf 那种
 * 带 128 字节局部缓冲的写法，否则会顺着坏栈再往下踩。
 */

/* 栈溢出钩子：configCHECK_FOR_STACK_OVERFLOW = 1 或 2 时启用。
 * 原来这里是静默死循环，任务栈一爆整个系统就停住、屏幕再也不刷新，
 * 表现就是"屏幕点不亮"，非常难查。现在先把任务名打出来再停机。 */
void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                    char * pcTaskName )
{
    ( void ) xTask;

    taskDISABLE_INTERRUPTS();
    dbg_fatal_puts("\r\n[FATAL] STACK OVERFLOW, task name = ");
    dbg_fatal_puts((pcTaskName != NULL) ? pcTaskName : "(null)");
    dbg_fatal_puts("\r\n         -> increase that task's stack size in main.c\r\n");

    /* 【为什么不再永久停机】原来这里是 for(;;) 停机 —— 板子在现场是"没人接
     * 调试器的"，一停机就表现成整机卡死、屏幕一直不亮，而且再也看不到第二眼
     * 线索。现在：先把名字发出去，空转约 3 秒（接了调试器的人这段时间足够
     * halt 在这里细看），然后自复位，重启后日志会再打一遍，循环着也能查。 */
    for (volatile uint32_t i = 0; i < 6000000UL; i++) { }
    NVIC_SystemReset();
    for( ; ; )
    {
        /* 复位失败才会停在这里 */
    }
}

/* 堆耗尽钩子：configUSE_MALLOC_FAILED_HOOK = 1 时启用。
 * xTaskCreate 之类拿不到内存会走到这里，否则任务会静默地建不起来。 */
void vApplicationMallocFailedHook( void )
{
    dbg_fatal_puts("\r\n[FATAL] pvPortMalloc failed: FreeRTOS heap exhausted\r\n");
    /* 同上：发完日志、留 3 秒给调试器，然后自复位，不要永久冻住 */
    for (volatile uint32_t i = 0; i < 6000000UL; i++) { }
    NVIC_SystemReset();
    for( ; ; )
    {
        /* 复位失败才会停在这里 */
    }
}

/* 空闲钩子：configUSE_IDLE_HOOK = 1 时启用。
 *
 * 为什么重要：FreeRTOS 的 ARM_CM3 port 里，**空闲任务默认是空转**（没有 WFI），
 * 所以只要所有任务都在等延时，CPU 就一直在 72MHz 跑 —— 电流 ~24mA，
 * 这是整块板子上最大的一笔「固定」开销（OLED 20mA、nRF24 RX 12mA 都是「可变」的）。
 *
 * 这里在空闲钩子里下一条 WFI，让 CPU 进 Sleep 模式：
 *   - Sleep 模式只停 CPU，SysTick / 外设时钟照常 → FreeRTOS 的节拍完全不受影响，
 *     不需要任何 tick 补偿（这是它比 STOP 模式省事的地方）；
 *   - 下一个中断把它唤醒：SysTick(10ms)、TIM4(HAL 时基 1ms)、EXTI(PC0/PB1)；
 *   - 电流从 ~24mA 降到 ~4~6mA。
 *
 * 和 STOP 模式的区别见 user/Task/lowpower.c：STOP 连 HSI/PLL 都停，能到 ~20µA，
 * 但必须配 RTC 唤醒源并补偿 tick。
 */
void vApplicationIdleHook( void )
{
    __WFI();
}

/* configASSERT 失败入口：把文件和行号打出来，比原来的静默死循环好定位 */
void vAssertCalled( const char * file, unsigned long line )
{
    char linebuf[16];
    unsigned long v = line;
    int i = 15;

    taskDISABLE_INTERRUPTS();

    linebuf[i] = '\0';
    if (v == 0UL) {
        linebuf[--i] = '0';
    }
    while (v > 0UL && i > 0) {
        linebuf[--i] = (char)('0' + (v % 10UL));
        v /= 10UL;
    }

    dbg_fatal_puts("\r\n[FATAL] configASSERT failed at ");
    dbg_fatal_puts((file != NULL) ? file : "?");
    dbg_fatal_puts(":");
    dbg_fatal_puts(&linebuf[i]);
    dbg_fatal_puts("\r\n");
    /* 发完就自复位（同栈溢出钩子）—— 永久停机在现场就等于板子死了 */
    for (volatile uint32_t k = 0UL; k < 6000000UL; k++) { }
    NVIC_SystemReset();
    for( ; ; )
    {
        /* 复位没成功才会停在这里 */
    }
}
