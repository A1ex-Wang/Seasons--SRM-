/** @file motor_driver.h
 * @brief 底盘四路 TB6612FNG 通道的底层驱动（PWM + 方向脚 + 死区补偿）。
 */

#ifndef ROBO_MOTOR_DRIVER_H
#define ROBO_MOTOR_DRIVER_H

#include <stdint.h>

#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
  * @brief 初始化四路 PWM 与八个方向脚，并把四个轮子置为停止。
  *
  * 内部会启动 TIM8_CH3/CH4 与 TIM9_CH1/CH2 的 PWM 输出（Pulse = 0），
  * 并把八个方向脚写 0（TB6612 两输入同时为 0 = 滑行/coast）。
 */
void Motor_Init(void);

/*
  * @brief 写一个轮子的*物理占空比*（不含死区补偿，供标定用）。
  * @param id         轮标识
  * @param duty       带符号占空比，单位 PWM 计数，正 = 该轮物理前进
 */
void Motor_SetRawDuty(WheelId_e id, int32_t duty);

/*
  * @brief 写一个轮子的*期望占空比*，内部做死区补偿。
  *
  * 这是给速度环用的接口：输入的 duty 是“我想要多大的出力”，
  * 函数内部会把它线性映射到 [DEADBAND, FULL] 区间，抵消电机的启动死区。
  *
  * @param id         轮标识
  * @param duty       期望带符号占空比，单位 PWM 计数，正 = 该轮物理前进
 */
void Motor_SetDuty(WheelId_e id, int32_t duty);

/** @brief 让一个轮子滑行停止（TB6612 两输入同时为 0）。 */
void Motor_Stop(WheelId_e id);

/** @brief 让四个轮子全部滑行停止。 */
void Motor_StopAll(void);

/*
  * @brief 取某个轮子的启动死区（PWM 计数）。
  * @note 值来自 app_config.h 的 MOTOR_DEADBAND_*，P1 标定后替换。
 */
uint32_t Motor_GetDeadband(WheelId_e id);

/**
 * @brief 死区补偿的纯函数版本，便于单元观察与标定计算。
 * @param desired    期望占空比（带符号）
 * @param deadband   该轮的启动死区
 * @return 实际要写进 PWM 比较寄存器的值（带符号，绝对值 ≤ MOTOR_PWM_FULL）
 */
int32_t Motor_ApplyDeadbandComp(int32_t desired, uint32_t deadband);

#ifdef __cplusplus
}
#endif

#endif /* ROBO_MOTOR_DRIVER_H */
