/** @file remote.h
 * @brief 遥控链路接入层：SRM v4 协议 → remote_in_t → q_remote_cmd。
 */

#ifndef ROBO_REMOTE_H
#define ROBO_REMOTE_H

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "app_config.h"
#include "app_types.h"

/* 上游协议层头文件。它同时提供： */
#include "srm_remote.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CubeMX（Core/Src/usart.c）生成的蓝牙串口句柄：USART2，RX 走 DMA1_Stream5。
 * 这里显式 extern 而不是包含 usart.h，是为了让“蓝牙口是哪一个”这件事
 * 只在 app_config.h（BT_UART_HANDLE）与本文件出现一次。 */
extern UART_HandleTypeDef BT_UART_HANDLE;

/*
 * @brief 初始化遥控链路：复位解析器状态、启动 USART2 的 RX DMA。
 * @return pdPASS 表示协议层初始化成功（HAL_OK）；
 */
BaseType_t Remote_ProtocolInit(void);

/* 板级回调（覆盖上游 srm_remote.c 里的 __weak 空实现）： */

#ifdef __cplusplus
}
#endif

#endif /* ROBO_REMOTE_H */
