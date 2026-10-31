/**
  ******************************************************************************
  * @file    safety.h
  * @brief   安全任务（Safety）：喂独立看门狗，保证系统卡死时能自动复位。
  ******************************************************************************
  */

#ifndef ROBO_SAFETY_H
#define ROBO_SAFETY_H

#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief 启动独立看门狗 IWDG（标称约 0.5 s 超时）。
  *
  * 必须在调度器起来之后、且即将进入喂狗循环之前调用一次，
  * 否则“启动完就没人喂”会在约 0.5 s 后被复位。
  * 直接操作寄存器，不依赖 HAL IWDG 模块（见 app_config.h 的说明）。
  */
void Safety_WatchdogStart(void);

/**
  * @brief 安全任务本体（覆盖 Core/Src/freertos.c 里的 __weak StartSafetyTask）。
  *
  * 周期 10 ms：仅调用 HAL_IWDG_Refresh()。
  * 优先级 34（全工程最高）：喂狗不允许被业务任务挤掉。
  */
void StartSafetyTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* ROBO_SAFETY_H */
