/** @file app_types.h
 * @brief 跨模块共享的类型定义与 extern 声明（无 .c 文件）。
 */

#ifndef ROBO_APP_TYPES_H
#define ROBO_APP_TYPES_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 遥控输入帧：近协议层结构 */

typedef struct {
    int16_t lx;        /**< 左摇杆 X，原始协议量纲 -512..511 */
    int16_t ly;        /**< 左摇杆 Y，原始协议量纲 -512..511（正 = 向下）*/
    int16_t rx;        /**< 右摇杆 X，原始协议量纲 -512..511 */
    int16_t ry;        /**< 右摇杆 Y，原始协议量纲 -512..511 */
    uint32_t buttons;  /**< 24 位 PRO_CONTROL 按键位图（bit0..bit16 有效）*/
    uint8_t lt;        /**< 左扳机 0..255（上游称 left_trigger）*/
    uint8_t rt;        /**< 右扳机 0..255（上游称 right_trigger）*/
    uint8_t online;    /**< 链路是否有效：1 = 收到合法控制帧，0 = 协议层判定失联 */
    uint8_t _align[3]; /**< 显式填充与对齐保留位 */
    uint32_t stamp;    /**< 生成该帧时的 FreeRTOS tick（ms），用于排查延迟 */
    uint8_t _pad[4];   /**< 尾部填充，保证 sizeof == 28 */
} remote_in_t;

/* 遥控帧尺寸及时间戳对齐校验。 */
APP_STATIC_ASSERT((sizeof(remote_in_t) % 4u) == 0u,
                  "remote_in_t size must be a multiple of 4 (FreeRTOS queue word copy)");
APP_STATIC_ASSERT(sizeof(remote_in_t) == 28u,
                  "remote_in_t must be 28 bytes; adjust the _pad[] field if fields changed");
APP_STATIC_ASSERT((__builtin_offsetof(remote_in_t, stamp) % 4u) == 0u,
                  "stamp must stay 4-byte aligned inside remote_in_t");
APP_STATIC_ASSERT((sizeof(uint32_t) == 4u), "uint32_t must be 4 bytes for the layout above");

/* 校验任务节拍与静态分配支持。 */
APP_STATIC_ASSERT((configTICK_RATE_HZ == APP_TICK_RATE_HZ),
                  "configTICK_RATE_HZ must be 1000 for APP_*_PERIOD_MS to be valid");
APP_STATIC_ASSERT((configSUPPORT_STATIC_ALLOCATION == 1),
                  "configSUPPORT_STATIC_ALLOCATION must be 1 (no FreeRTOS heap allowed)");

/* 底盘控制：四轮标识与每轮最终指令 */

/**
 * @brief 四个轮子的标识。顺序 = 数组下标，全工程统一。
 */
typedef enum {
    WHEEL_FL = 0,
    WHEEL_FR = 1,
    WHEEL_RL = 2,
    WHEEL_RR = 3,
    WHEEL_COUNT = 4
} WheelId_e;

/* 四轮最终 PWM 指令快照，包含 PID、前馈和输出限幅。 */
typedef struct {
    int16_t duty[WHEEL_COUNT];   /**< 带符号占空比，正 = 该轮物理前进 */
    uint8_t active[WHEEL_COUNT]; /**< 1 = 该轮处于闭环/输出状态，0 = 已停 */
} motor_cmd_t;

/** @brief 每轮最终指令快照（遥测/调试用）。 */
extern motor_cmd_t g_motor_cmd;

/*  底盘控制模块入口 */

/** @brief 初始化四路 PWM 与方向脚，并让四个轮子立即处于停止状态。 */
void Motor_Init(void);

/** @brief 初始化四路编码器、速度环整定状态与四个 PID 实例。 */
void VelCtl_Init(void);

/** @brief 链路失效时的安全动作：参考值清零、积分清零、PWM 立即归零。 */
void VelCtl_SafeState(void);

/* 速度环主入口，周期 dt_s 单位为秒；依次完成测速、指令映射、运动学及四轮闭环。 */
void VelCtl_Feed(const remote_in_t *cmd, float dt_s);

/* 队列句柄 */

/*
  * @brief 遥控帧队列（remote 任务 → chassis 任务）。
  *
  * 使用 xQueueCreateStatic 创建，缓冲区与队列控制块都是 static 数组，
  * 不占用 FreeRTOS heap（configSUPPORT_STATIC_ALLOCATION = 1）。
 */
extern QueueHandle_t q_remote_cmd;

/* 任务入口 */

/* 任务入口由 CubeMX 的 FreeRTOS 初始化统一创建。 */

/** @brief 安全任务：10 ms 周期，仅喂 IWDG。 */
void StartSafetyTask(void *argument);

/* 底盘任务：接收遥控队列并周期执行四轮速度环。 */
void StartChassisTask(void *argument);

/** @brief 遥控任务：1 ms 轮询 SRM_Remote_Poll()，检测新帧并投队列。 */
void StartRemoteTask(void *argument);

/* 应用初始化 */

/* 调度器启动前初始化外设与静态队列；任务由 CubeMX 创建。 */
void App_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBO_APP_TYPES_H */
