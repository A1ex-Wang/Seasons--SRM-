//
// Created by 18045 on 2026/9/28.
//

#ifndef ROBO_GRAB_H
#define ROBO_GRAB_H

#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 五个机械臂舵机的逻辑编号。J1 步进旋转电机暂时保留，不在本模块驱动。 */
typedef enum {
    GRAB_J2_BIG_ARM = 0,
    GRAB_J3_FOREARM,
    GRAB_J4_PITCH,
    GRAB_J5_ROTATE,
    GRAB_J6_GRIPPER,
    GRAB_SERVO_COUNT
} GrabServoId_e;

typedef enum {
    GRAB_MODE_CHASSIS = 0,
    GRAB_MODE_COARSE,
    GRAB_MODE_FINE,
    GRAB_MODE_SAFE
} GrabMode_e;

/* 初始化五路舵机 PWM，设置初始姿态。 */
void Grab_Init(void);

/* 以固定速度处理一帧遥控输入，dt_s 单位为秒。 */
void Grab_Feed(const remote_in_t *cmd, float dt_s);

GrabMode_e Grab_GetMode(void);
float Grab_GetTargetAngle(GrabServoId_e id);

/* CubeMX 已创建 GrabTask；此强定义覆盖 freertos.c 中的弱空壳。 */
void StartGrabTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif //ROBO_GRAB_H
