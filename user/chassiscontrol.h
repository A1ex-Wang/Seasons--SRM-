/** @file chassiscontrol.h
 * @brief 底盘控制任务（Chassis）：消费 q_remote_cmd，接收遥控指令并执行底盘速度控制。
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

/* 复制当前遥控指令快照，out 为 NULL 时不操作。 */
void Chassis_TakeLatest(remote_in_t *out);

/* 底盘任务：周期接收遥控队列、执行速度环并按配置输出遥测。 */
void StartChassisTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* ROBO_CHASSISCONTROL_H */
