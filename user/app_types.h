/**
  ******************************************************************************
  * @file    app_types.h
  * @brief   跨模块共享的类型定义与 extern 声明（无 .c 文件）。
  *
  * 这里只放“接口”：数据结构、队列句柄、任务入口原型。
  * 具体实现分别在 remote.c / chassiscontrol.c / safety.c / debug_uart.c。
  ******************************************************************************
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

/*============================================================================*/
/* 遥控输入帧：近协议层结构                                                     */
/*============================================================================*/

/**
  * @brief 一帧遥控输入，忠实映射线上字段并保留原始量纲。
  *
  * 本阶段**不做**任何轴→速度映射，也不做去死区/滤波：
  * 收到什么就转发什么，等实测确认方向符号后再在底盘侧映射。
  *
  * 内存布局（28 字节，与下表严格一致）：
  *   偏移  0   lx / ly / rx / ry      4 × int16_t  =  8
  *   偏移  8   buttons                uint32_t     =  4
  *   偏移 12   lt / rt / online       3 × uint8_t  =  3
  *   偏移 15   _align[3]                           =  3   ← 显式填充
  *   偏移 20   stamp                  uint32_t     =  4
  *   偏移 24   _pad[4]                             =  4
  *   总计 28 字节，sizeof % 4 == 0，对齐要求 4。
  *
  * ⚠ 为什么必须有 _align[3] 这个“多余”的字段（踩过的坑，勿删）：
  *   如果写成 online 后面直接跟 stamp，那 1 字节的隐式填充 + 4 字节尾部填充
  *   全靠编译器生成。本工程使用的 arm-none-eabi-gcc 15.3.1 在这类结构上会算错
  *   sizeof：字段偏移全部正确（stamp@16、_pad@20），但 sizeof(remote_in_t)
  *   返回 24 而不是 28，也就是**隐式填充没有计入 sizeof**。
  *   后果很严重：xQueueCreateStatic() 用它做队列元素的“跨度”，
  *   uxItemSize 会偏小 4 字节，元素之间错位、越界，收到的数据是垃圾。
  *   把填充写成显式数组后，sizeof 立刻恢复正确（28），DWARF 与 offsetof 一致。
  *   结论：本工程的填充字节一律显式写出来，不依赖隐式填充。
  *   下面的 _Static_assert 会把这件事钉死在编译期。
  */
typedef struct {
    int16_t lx;        /**< 左摇杆 X，原始协议量纲 -512..511 */
    int16_t ly;        /**< 左摇杆 Y，原始协议量纲 -512..511（正 = 向下）*/
    int16_t rx;        /**< 右摇杆 X，原始协议量纲 -512..511 */
    int16_t ry;        /**< 右摇杆 Y，原始协议量纲 -512..511 */
    uint32_t buttons;  /**< 24 位 PRO_CONTROL 按键位图（bit0..bit16 有效）*/
    uint8_t lt;        /**< 左扳机 0..255（上游称 left_trigger）*/
    uint8_t rt;        /**< 右扳机 0..255（上游称 right_trigger）*/
    uint8_t online;    /**< 链路是否有效：1 = 收到合法控制帧，0 = 协议层判定失联 */
    uint8_t _align[3]; /**< 显式填充：把 stamp 顶到偏移 20，见上方说明 */
    uint32_t stamp;    /**< 生成该帧时的 FreeRTOS tick（ms），用于排查延迟 */
    uint8_t _pad[4];   /**< 尾部填充，保证 sizeof == 28 */
} remote_in_t;

/* ── 队列元素尺寸约束（硬性要求）─────────────────────────────────────────────
 * FreeRTOS 的 xQueueSend/xQueueReceive 以“字”为单位做 4 字节对齐拷贝。
 * 若 sizeof(remote_in_t) 不是 4 的倍数，队列里相邻元素会错位，
 * 收到的按键位图/轴值会是上一帧的碎片——而且不会有任何编译或运行时报错。
 * 用 _align[] / _pad[] 显式补齐后在此断言，把这类错误提前到编译期。 */
APP_STATIC_ASSERT((sizeof(remote_in_t) % 4u) == 0u,
                  "remote_in_t size must be a multiple of 4 (FreeRTOS queue word copy)");
APP_STATIC_ASSERT(sizeof(remote_in_t) == 28u,
                  "remote_in_t must be 28 bytes; adjust the _pad[] field if fields changed");
APP_STATIC_ASSERT((__builtin_offsetof(remote_in_t, stamp) % 4u) == 0u,
                  "stamp must stay 4-byte aligned inside remote_in_t");
APP_STATIC_ASSERT((sizeof(uint32_t) == 4u), "uint32_t must be 4 bytes for the layout above");

/* 节拍一致性：本工程所有“周期”都按 1 kHz 节拍写成毫秒数，
 * 一旦有人改了 configTICK_RATE_HZ，这里直接编译失败，而不是让任务周期悄悄跑偏。
 * （放在本文件是因为 FreeRTOS.h 已经在上面包含，configTICK_RATE_HZ 在此可见。）*/
APP_STATIC_ASSERT((configTICK_RATE_HZ == APP_TICK_RATE_HZ),
                  "configTICK_RATE_HZ must be 1000 for APP_*_PERIOD_MS to be valid");
APP_STATIC_ASSERT((configSUPPORT_STATIC_ALLOCATION == 1),
                  "configSUPPORT_STATIC_ALLOCATION must be 1 (no FreeRTOS heap allowed)");

/*============================================================================*/
/* 底盘控制：四轮标识与每轮最终指令                                            */
/*============================================================================*/

/**
  * @brief 四个轮子的标识。顺序 = 数组下标，全工程统一。
  *
  * 物理位置（你在实车上认定的，标定时请核对，与 CubeMX 的标签无关）：
  *   WHEEL_FL = 左前，WHEEL_FR = 右前，WHEEL_RL = 左后，WHEEL_RR = 右后
  * 与它绑定的硬件：
  *   FL: PWM=TIM9_CH1(PE5)  DIR=PE1/PE2   ENC=TIM2(PA0/PA1)
  *   FR: PWM=TIM9_CH2(PE6)  DIR=PE3/PE4   ENC=TIM1(PE9/PE11)
  *   RL: PWM=TIM8_CH3(PC8)  DIR=PG3/PG4   ENC=TIM4(PB6/PB7)
  *   RR: PWM=TIM8_CH4(PC9)  DIR=PG5/PG6   ENC=TIM3(PB4/PB5)
  */
typedef enum {
    WHEEL_FL = 0,
    WHEEL_FR = 1,
    WHEEL_RL = 2,
    WHEEL_RR = 3,
    WHEEL_COUNT = 4
} WheelId_e;

/**
  * @brief 速度环输出的“每轮最终指令”，单位 PWM 计数（±MOTOR_PWM_FULL）。
  *
  * 已经是 PID 输出 + 死区补偿 + 限幅之后的结果，可直接写硬件。
  * 本工程里 PWM 的写入就发生在速度环内部，所以这个全局主要供**遥测打印**与
  * 调试观察使用，不承担跨任务实时传值。
  */
typedef struct {
    int16_t duty[WHEEL_COUNT];   /**< 带符号占空比，正 = 该轮物理前进 */
    uint8_t active[WHEEL_COUNT]; /**< 1 = 该轮处于闭环/输出状态，0 = 已停 */
} motor_cmd_t;

/** @brief 每轮最终指令快照（遥测/调试用）。 */
extern motor_cmd_t g_motor_cmd;

/* ── 底盘控制模块入口 ──────────────────────────────────────────────────────── */

/** @brief 初始化四路 PWM 与方向脚，并让四个轮子立即处于停止状态。 */
void Motor_Init(void);

/** @brief 初始化四路编码器、速度环整定状态与四个 PID 实例。 */
void VelCtl_Init(void);

/** @brief 链路失效时的安全动作：参考值清零、积分清零、PWM 立即归零。 */
void VelCtl_SafeState(void);

/**
  * @brief 速度环主入口，由 Chassis 任务每 VEL_LOOP_PERIOD_MS 调用一次。
  *
  * 内部依次完成：读编码器 → 算反馈速度 → 摇杆映射（含死区）→ 限加速度 →
  * 麦轮逆运动学 → 车轮限幅 → 四路独立 PI + 死区补偿 → 写 PWM。
  *
  * @param cmd   最近一次遥控输入（Chassis 任务已从队列取出的最新帧）
  * @param dt_s  距上一次调用的实际时间间隔，单位【秒】（不是毫秒！）
  */
void VelCtl_Feed(const remote_in_t *cmd, float dt_s);

/*============================================================================*/
/* 队列句柄                                                                    */
/*============================================================================*/

/**
  * @brief 遥控帧队列（remote 任务 → chassis 任务）。
  *
  * 使用 xQueueCreateStatic 创建，缓冲区与队列控制块都是 static 数组，
  * 不占用 FreeRTOS heap（configSUPPORT_STATIC_ALLOCATION = 1）。
  */
extern QueueHandle_t q_remote_cmd;

/*============================================================================*/
/* 任务入口                                                                    */
/*============================================================================*/

/* 说明：下面这几个 Start* 在 Core/Src/freertos.c 中已经由 CubeMX 生成为 __weak
 * 空壳（函数体里只有 osDelay(1)）。本工程用同名的**强定义**覆盖其中三个，
 * 把它们改成真正的业务任务，从而完全不需要修改 CubeMX 生成的 freertos.c。
 *
 * 任务创建统一放在 App_Init() 里用 xTaskCreateStatic 完成，而不是用 CubeMX 的
 * osThreadNew()：后者无法表达“指定的 raw 优先级 + 全静态分配”这两个要求。 */

/** @brief 安全任务：10 ms 周期，仅喂 IWDG。 */
void StartSafetyTask(void *argument);

/** @brief 底盘任务：1 ms 收队列 + 每 100 ms 打印一次（本阶段不打电机）。 */
void StartChassisTask(void *argument);

/** @brief 遥控任务：1 ms 轮询 SRM_Remote_Poll()，检测新帧并投队列。 */
void StartRemoteTask(void *argument);

/*============================================================================*/
/* 应用初始化                                                                  */
/*============================================================================*/

/**
  * @brief 应用层唯一入口：建队列 → 建任务。
  *
  * 必须在 osKernelStart() 之前调用（main.c 的 USER CODE 区）。
  * 顺序要求：先建队列，再建任务；任务中先建消费者（chassis），后建生产者（remote），
  * 这样队列在两边都开始运行前就已经存在，永远不会出现“生产者先跑、队列还是 NULL”。
  */
void App_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBO_APP_TYPES_H */
