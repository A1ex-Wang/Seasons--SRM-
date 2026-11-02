/** @file debug_uart.c
 * @brief printf 重定向到 USART6（独立调试口）。
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
    /* 波特率统一到 DEBUG_UART_BAUD。写 BRR 必须在外设关闭（UE = 0）时进行， */
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();

    DEBUG_UART_INSTANCE->CR1 &= ~USART_CR1_UE;
    DEBUG_UART_INSTANCE->BRR =
        (uint16_t)((pclk2 + (uint32_t)DEBUG_UART_BAUD / 2u) / (uint32_t)DEBUG_UART_BAUD);
    DEBUG_UART_INSTANCE->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

    /* 清一次标志位，避免上一轮（例如调试器复位）残留的 ORE/NE 影响判断。 */
    (void)DEBUG_UART_INSTANCE->SR;
    (void)DEBUG_UART_INSTANCE->DR;
}

/* 与 C 库的对接 */
int __io_putchar(int ch);

int __io_putchar(int ch) {
    return DebugUart_PutChar(ch);
}
