/** @file velocity_control.h
 * @brief 底盘速度控制：摇杆映射、麦轮逆运动学、四路独立速度环。
 */

#ifndef ROBO_VELOCITY_CONTROL_H
#define ROBO_VELOCITY_CONTROL_H

#include "app_types.h"

typedef struct {
    float omega_ref_rads;
    float omega_measured_rads;
    float error_rads;
    float integral;
    float correction_rads;
    float omega_output_rads;
    uint8_t active;
} YawRateWatch_t;

extern volatile YawRateWatch_t g_yaw_rate_watch;

#ifdef __cplusplus
extern "C" {
#endif

/*
  * @brief 读某一轮最近一个窗口的实测速度（mm/s，前进为正）。
  *
  * 供遥测打印使用；控制回路内部不经过这个函数。
  * @param id 轮标识
  * @return 该轮线速度，单位 mm/s
 */
float VelCtl_GetWheelSpeedMmps(WheelId_e id);

/*
  * @brief 读某一轮最近一次的速度环输出占空比（PWM 计数，带符号）。
  * @param id 轮标识
  * @return 带符号占空比，正 = 该轮前进
 */
int32_t VelCtl_GetWheelDuty(WheelId_e id);

/**
 * @brief 读某一轮最近一周期发给它的目标速度（mm/s，前进为正）。
 * @param id 轮标识
 * @return 该轮目标线速度，单位 mm/s
 */
float VelCtl_GetWheelTargetMmps(WheelId_e id);

/* 遥测接口（只读，调试用，不参与控制） */

/** @brief 车身前进指令 vy [mm/s]（>0 = 前进）。 */
float VelCtl_TelemCmdVy(void);

/** @brief RL 轮（M3）目标轮速 [mm/s]，正数 = 该轮物理前进。 */
float VelCtl_TelemTargetRL(void);

/** @brief RL 轮（M3）实测轮速 [mm/s]，与目标值同量纲、同符号约定。 */
float VelCtl_TelemSpeedRL(void);

/** @brief RL 轮（M3）本周期有符号计数增量（满速 ±87）。 */
int32_t VelCtl_TelemDeltaRL(void);

/** @brief RL 轮（M3）累计位置计数，用来判断"真在转"还是"数错了"。 */
int32_t VelCtl_TelemCountRL(void);

/** @brief RL 轮（M3）当前带符号占空比（PWM 计数，满量程 8400）。 */
int32_t VelCtl_TelemDutyRL(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBO_VELOCITY_CONTROL_H */
