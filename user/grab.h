

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

/* 舵机状态快照，不参与控制计算。 */
typedef struct {
    float target_angle_deg;
    uint32_t pulse_us;
} GrabServoWatch_t;

/* 机械臂输入与主导轴选择状态。 */
typedef struct {
    int16_t lx;
    int16_t ly;
    int8_t selected_lx;
    int8_t selected_ly;
    int8_t axis_lock;
    uint8_t online;
    uint8_t mode;
} GrabInputWatch_t;

/* 初始化五路舵机 PWM，设置初始姿态。 */
void Grab_Init(void);

/* 以固定速度处理一帧遥控输入，dt_s 单位为秒。 */
void Grab_Feed(const remote_in_t *cmd, float dt_s);

GrabMode_e Grab_GetMode(void);
float Grab_GetTargetAngle(GrabServoId_e id);

/* CubeMX 已创建 GrabTask；此强定义覆盖 freertos.c 中的弱空壳。 */
void StartGrabTask(void *argument);

/* 舵机目标角度监视量（度）。 */
extern volatile float g_grab_j2_target_angle_deg;
extern volatile float g_grab_j3_target_angle_deg;
extern volatile float g_grab_j4_target_angle_deg;
extern volatile float g_grab_j5_target_angle_deg;
extern volatile float g_grab_j6_target_angle_deg;

extern volatile GrabServoWatch_t g_grab_j2_watch;
extern volatile GrabServoWatch_t g_grab_j3_watch;
extern volatile GrabServoWatch_t g_grab_j4_watch;
extern volatile GrabServoWatch_t g_grab_j5_watch;
extern volatile GrabServoWatch_t g_grab_j6_watch;
extern volatile GrabInputWatch_t g_grab_input_watch;

#ifdef __cplusplus
}
#endif

#endif //ROBO_GRAB_H
