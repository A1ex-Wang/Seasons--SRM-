/**
  ******************************************************************************
  * @file    chassiscontrol.h
  * @brief   底盘控制任务（Chassis）：消费 q_remote_cmd，本阶段只打印不驱动电机。
  ******************************************************************************
  */

#ifndef ROBO_CHASSISCONTROL_H
#define ROBO_CHASSISCONTROL_H

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief 取一次队列（非阻塞），取不到就沿用上一次的值。
  *
  * 供调试或其它任务读取“最近一次遥控输入”。本阶段电机侧不使用。
  * @param out 输出缓冲，允许为 NULL（此时只丢弃）。
  */
void Chassis_TakeLatest(remote_in_t *out);

/**
  * @brief 底盘任务本体（覆盖 Core/Src/freertos.c 里的 __weak StartChassisTask）。
  *
  * 周期 1 ms：每个周期非阻塞取队列并按“丢旧语义”覆盖本地副本；
  * 每 CHASSIS_PRINT_DIVIDER（100）个周期打印一行原始值，即 10 Hz。
  *
  * 本阶段**不做**轴→速度映射、不做运动学、不碰任何电机或 PWM。
  */
void StartChassisTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* ROBO_CHASSISCONTROL_H */
