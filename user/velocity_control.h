/**
  ******************************************************************************
  * @file    velocity_control.h
  * @brief   底盘速度控制：摇杆映射、麦轮逆运动学、四路独立速度环。
  *
  * 对外接口只有三个，声明统一放在 app_types.h（Chassis 任务要用）：
  *
  *   void VelCtl_Init(void);                           // 初始化编码器/PID/PWM
  *   void VelCtl_Feed(const remote_in_t*, float dt_s); // 每 10 ms 调一次，dt_s 单位是【秒】
  *   void VelCtl_SafeState(void);                      // 链路失效时立即安全停机
  *
  * 本文件不重复声明它们，只作为“模块归属”的说明与将来扩展的落点；
  * 需要更多状态时（例如每轮实测速度的只读快照），在这里加 getter 即可。
  ******************************************************************************
  */

#ifndef ROBO_VELOCITY_CONTROL_H
#define ROBO_VELOCITY_CONTROL_H

#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief 读某一轮最近一个窗口的实测速度（mm/s，前进为正）。
  *
  * 供遥测打印使用；控制回路内部不经过这个函数。
  * @param id 轮标识
  * @return 该轮线速度，单位 mm/s
  */
float VelCtl_GetWheelSpeedMmps(WheelId_e id);

/**
  * @brief 读某一轮最近一次的速度环输出占空比（PWM 计数，带符号）。
  * @param id 轮标识
  * @return 带符号占空比，正 = 该轮前进
  */
int32_t VelCtl_GetWheelDuty(WheelId_e id);

/**
  * @brief 读某一轮最近一周期发给它的目标速度（mm/s，前进为正）。
  *
  * ⚠ 想看"目标 vs 实测"时用这个 + VelCtl_GetWheelSpeedMmps()，
  *   **不要**去读 PIDInstance 里的 Ref/Measure —— 那两个字段名与实际语义相反
  *   （`Ref` 装的是实测值、`Measure` 装的是目标值），见 controller.h 的警告。
  *
  * @param id 轮标识
  * @return 该轮目标线速度，单位 mm/s
  */
float VelCtl_GetWheelTargetMmps(WheelId_e id);

#ifdef __cplusplus
}
#endif

#endif /* ROBO_VELOCITY_CONTROL_H */
