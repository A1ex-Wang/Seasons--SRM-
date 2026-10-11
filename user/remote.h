/**
  ******************************************************************************
  * @file    remote.h
  * @brief   遥控链路接入层：SRM v4 协议 → remote_in_t → q_remote_cmd。
  ******************************************************************************
  */

#ifndef ROBO_REMOTE_H
#define ROBO_REMOTE_H

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "app_config.h"
#include "app_types.h"

/* 上游协议层头文件。它同时提供：
 *   - HAL 类型（通过其内部的 stm32f4xx_hal.h 引入 UART_HandleTypeDef）；
 *   - 板级回调（SRM_BoardApplyControl / SRM_BoardApplyProControl）的原型声明，
 *     因此本文件不需要也不应该重复声明它们。
 * 包含顺序上必须放在 app_config.h / app_types.h 之后，因为本工程的头文件
 * 不使用 HAL 类型，只有这里才需要。 */
#include "srm_remote.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CubeMX（Core/Src/usart.c）生成的蓝牙串口句柄：USART2，RX 走 DMA1_Stream5。
 * 这里显式 extern 而不是包含 usart.h，是为了让“蓝牙口是哪一个”这件事
 * 只在 app_config.h（BT_UART_HANDLE）与本文件出现一次。 */
extern UART_HandleTypeDef BT_UART_HANDLE;

/**
  * @brief 初始化遥控链路：复位解析器状态、启动 USART2 的 RX DMA。
  *
  * 由 Remote 任务在进入主循环前调用一次。内部会调用 SRM_Remote_Init()，
  * 它会真的启动 HAL_UART_Receive_DMA() 并把 rx_buffer 交给 DMA，
  * 因此调用后 rx_buffer 必须一直有效（本工程的 g_remote 是文件作用域静态对象，
  * 生命周期 = 程序，满足要求）。
  *
  * @return pdPASS 表示协议层初始化成功（HAL_OK）；
  *         pdFAIL 表示失败（例如 USART2 的 RX DMA 不是 Circular 模式），
  *         此时已触发 configASSERT，队列仍可用，chassis 侧会显示 LOST。
  */
BaseType_t Remote_ProtocolInit(void);

/*
 * 板级回调（覆盖上游 srm_remote.c 里的 __weak 空实现）：
 *
 *   void SRM_BoardApplyControl(const srm_control_state_t *s);      // TYPE 0x0
 *   void SRM_BoardApplyProControl(const srm_pro_control_state_t *s); // TYPE 0x7
 *
 * 原型由 srm_remote.h 声明，实现见 remote.c。
 * 约束（来自任务书，实现时必须遵守）：只允许
 *   ① 把字段拷贝到一个静态结构体；
 *   ② 置一个 volatile 新帧标志。
 * 严禁在回调里投队列、调串口发送、调 printf、或做任何可能阻塞/耗时的操作 ——
 * 它是在 SRM_Remote_Poll() 的解析路径里**同步调用**的，会直接拖慢 1 kHz 的
 * Remote 任务并把 DMA 环形缓冲耗光。真正的投队列动作在 Remote 的任务循环里
 * 由标志位触发。
 */

#ifdef __cplusplus
}
#endif

#endif /* ROBO_REMOTE_H */
