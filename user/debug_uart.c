/**
  ******************************************************************************
  * @file    debug_uart.c
  * @brief   printf 重定向到 USART6（独立调试口）。
  *
  * 职责：把 newlib/picolibc 的 _write() 最终落到 USART6 的发送寄存器上。
  * 调用方：Chassis 任务（每 100 ms 一行）；初始化：App_Init()。
  * 周期：无（事件驱动）。
  *
  * 为什么直接用寄存器而不是 HAL_UART_Transmit()：
  *   1) HAL_UART_Transmit() 内部会锁 huart6 状态、操作 tx 计数与超时，返回
  *      前的状态判断对“逐字节 printf”是纯开销；这里只需要 TXE 标志。
  *   2) 不引入 huart6 的 extern 依赖，调试口与业务口的耦合降到最低。
  *
  * 重要：本函数**会阻塞**等待 TXE（整个 printf 期间任务被占住）。
  * 按本工程的任务表，Chassis(32) 优先级高于 Remote(26)，因此打印期间 Remote
  * 不会抢占 Chassis，最长约一行耗时的窗口内 SRM_Remote_Poll() 不被执行。
  * 该窗口对蓝牙链路是安全的：
  *   - 一行约 90 字节 @115200 ≈ 7.8 ms；100 Hz 的帧率下约 0.8 帧到达；
  *   - 256 字节环形缓冲 @9600 ≈ 266 ms 才写满，允许的停顿远大于 7.8 ms；
  *   - 即使真的丢字节，协议层会丢弃 CRC 失败的帧并自动重新同步，不会卡死。
  * 若以后把打印频率提到 100 Hz 或蓝牙升到高帧率，请把 Chassis 优先级降到
  * Remote 之下（只改 app_config.h 的 PRIO_CHASSIS），这是唯一需要改的地方。
  ******************************************************************************
  */

#include "debug_uart.h"

#include "FreeRTOS.h"
#include "task.h"

#include "main.h"

/* 阻塞等待 TXE 的循环上限，防止 USART6 未使能时死等。 */
#define DEBUG_UART_TXE_SPIN (DEBUG_UART_TXE_TIMEOUT)

int DebugUart_PutChar(int ch) {
    uint32_t spin = 0u;

    /* 等到发送数据寄存器空。USART6 由 CubeMX 初始化并常开，
     * 正常情况下第一个循环就能通过。 */
    while ((DEBUG_UART_INSTANCE->SR & USART_SR_TXE) == 0u) {
        if (++spin > DEBUG_UART_TXE_SPIN) {
            /* 调试口异常（未使能/时钟丢失）：直接丢弃该字节，绝不挂死任务。
             * 这里刻意不调用 HAL 或 Error_Handler()：一个调试输出不应该
             * 让整台机器人停机。 */
            return (int)(uint8_t)ch;
        }
    }
    DEBUG_UART_INSTANCE->DR = (uint16_t)((uint32_t)ch & 0x01FFu);
    return (int)(uint8_t)ch;
}

void DebugUart_Init(void) {
    /* 波特率统一到 DEBUG_UART_BAUD。写 BRR 必须在外设关闭（UE = 0）时进行，
     * 否则新值要到下一次 UE 上升沿才生效。
     * USART6 挂在 APB2：SystemClock_Config 下 HCLK = 168 MHz、APB2 = HCLK/2 = 84 MHz。
     * 下面这个分频公式与 HAL_UART_Init() 在 OVERSAMPLING_16 下使用的
     * UART_BRR_SAMPLING16 完全一致（BRR ≈ fCK / baud），所以结果与 CubeMX
     * 初始化 huart6 时算出来的 BRR 相同。 */
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();

    DEBUG_UART_INSTANCE->CR1 &= ~USART_CR1_UE;
    DEBUG_UART_INSTANCE->BRR =
        (uint16_t)((pclk2 + (uint32_t)DEBUG_UART_BAUD / 2u) / (uint32_t)DEBUG_UART_BAUD);
    DEBUG_UART_INSTANCE->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

    /* 清一次标志位，避免上一轮（例如调试器复位）残留的 ORE/NE 影响判断。 */
    (void)DEBUG_UART_INSTANCE->SR;
    (void)DEBUG_UART_INSTANCE->DR;
}

/*
 * ── 与 C 库的对接 ───────────────────────────────────────────────────────────
 * Core/Src/syscalls.c 里的 _write() 逐字节调用 __io_putchar()，而该符号在那里
 * 声明为 __attribute__((weak)) 且没有定义。这里提供强定义，于是
 * printf() → _write() → __io_putchar() → USART6。
 * 注意不能给函数体加 static/weak，否则链接时 _write 仍然找不到实现，
 * printf 会静默丢失所有输出。
 */
int __io_putchar(int ch);

int __io_putchar(int ch) {
    return DebugUart_PutChar(ch);
}
