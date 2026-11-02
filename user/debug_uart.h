/** @file debug_uart.h
 * @brief 调试串口（USART6）重定向，供 printf 使用。
 */

#ifndef ROBO_DEBUG_UART_H
#define ROBO_DEBUG_UART_H

#include <stdint.h>

#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
  * @brief 初始化调试串口。
  *
  * USART6 的引脚、时钟与 8N1 由 CubeMX（MX_USART6_UART_Init）完成，
  * 这里只把波特率统一成 DEBUG_UART_BAUD 并清一次发送状态，
  * 保证 printf 输出的速度足够跟上 10 Hz 的打印频率。
 */
void DebugUart_Init(void);

/*
  * @brief 向调试串口写一个字节（printf 的底层实现）。
  * @param ch 待发送字符
  * @return 写入的字符
 */
int DebugUart_PutChar(int ch);

#ifdef __cplusplus
}
#endif

#endif /* ROBO_DEBUG_UART_H */
