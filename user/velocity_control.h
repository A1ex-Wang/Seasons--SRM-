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

/*============================================================================*/
/* 遥测接口（只读，调试用，不参与控制）                                        */
/*============================================================================*/
/*
 * ★ 为什么要有这组函数 ★
 *
 * Ozone 在 watch 里写 `s_enc[2].speed_mmps` 时，会按**它自己缓存的**数组元素
 * 步长算地址、按**它自己缓存的**结构体布局算字段偏移。本工程反复改过 Encode_t
 * 的布局，一旦调试信息与固件不同步，就会算出错位地址、显示物理不可能的读数。
 *
 * 下面这组函数**参数是编译期常量、返回标量**，Ozone 直接写函数名求值，
 * 不经过数组下标计算，不受调试信息缓存影响。
 *
 * ★ 函数 ↔ 变量 对照（M3 = RL = 索引 2）
 *   VelCtl_TelemTargetRL() ↔ s_target_mmps[2]      （目标 = 运动学输出）
 *   VelCtl_TelemSpeedRL()  ↔ s_enc[2].speed_mmps   （实测 = PID 的 measure）
 *   VelCtl_TelemDeltaRL()  ↔ s_enc[2].last_delta   （本周期计数增量）
 *   VelCtl_TelemCountRL()  ↔ s_enc[2].total_count  （累计位置）
 *   VelCtl_TelemDutyRL()   ↔ s_cmd_snapshot.duty[2]（写入的带符号占空比）
 */

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
