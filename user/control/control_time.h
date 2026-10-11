//
// Created by 18045 on 2026/9/2.
//
#ifndef USER_MOTOR_CONTROL_TIME_H
#define USER_MOTOR_CONTROL_TIME_H
#include <stdint.h>
void ControlTime_DWT_Init(uint32_t cpu_hz);
float ControlTime_DWT_GetDelta(uint32_t *last_count);
#endif
