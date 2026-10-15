/**
  ******************************************************************************
  * @file    encode.h
  * @brief   四路增量编码器接口（STM32 定时器编码器模式，TI12 四倍频）。
  *
  * 设计要点（与原来单实例版本的差异）：
  *   1) **多实例**：四个轮子各持有一个 Encode_t，所有 API 都显式传入实例指针，
  *      不再使用文件级 static 单例（原版只能带一个编码器）；
  *   2) **符号可标定**：sign 取自 app_config.h 的 MOTOR_SIGN_ENC_*，
  *      用来把“编码器计数增减”统一成“该轮物理前进为正”；
  *   3) **测速窗口由调用方给出真实时间**：Encode_Update(enc, dt_ms) 必须传入
  *      距上次调用的实际毫秒数。本工程速度环是 10 ms 一次，所以每次传 10。
  *      （历史坑：这里曾按"调用一次=1 ms"写死累加 +1，导致窗口变成 10 倍长、
  *        速度读数放大 10 倍，进而让 PID 输出饱和、电机狂转。）
  *
  * 为什么不能用“输入捕获测周法（T 法）”：
  *   本工程的四路编码器占满了 TIM1/2/3/4 的 CH1 与 CH2（A 相进 CH1、B 相进 CH2），
  *   而输入捕获也需要 CH1/CH2 的同一个通道，两者互斥；且 PB6 在 F407 上只能作
  *   TIM4_CH1、没有 CH2 可用，无法腾出通道。所以只能走“定窗口计数（M 法）”，
  *   并且接受由此带来的量化噪声地板（见 app_config.h 第 7 节的详细推导）。
  ******************************************************************************
  */

#ifndef ROBO_ENCODE_H
#define ROBO_ENCODE_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "main.h"

#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 累计计数的测速窗口长度（毫秒）。取 10 ms → 速度环 100 Hz。 */
#define ENCODE_WINDOW_MS VEL_LOOP_PERIOD_MS

/**
  * @brief 一路编码器的运行时状态。必须由调用方提供存储（全静态分配）。
  */
typedef struct {
    TIM_HandleTypeDef *htim;   /**< 该轮对应的编码器定时器句柄 */
    int8_t sign;               /**< +1/-1：把计数方向统一成“该轮前进为正” */
    uint32_t window_ms;        /**< 测速窗口长度（毫秒） */

    uint16_t last_raw;         /**< 上一次读到的定时器计数值（16 位快照） */
    int32_t total_count;       /**< 累计有符号计数（位置，已乘 sign），可正可负 */

    volatile int32_t window_count;  /**< 当前测速窗口内累积的计数 */
    volatile int32_t speed_counts_s;/**< 最近一个窗口算出的速度 [counts/s] */
    volatile int32_t elapsed_ms;    /**< 当前窗口已累积的毫秒数 */

    uint8_t running;           /**< 1 = 定时器已启动并开始计翻转 */
} Encode_t;

/**
  * @brief 初始化一路编码器（不启动定时器）。
  * @param enc        调用方提供的实例存储
  * @param htim       编码器定时器句柄（CubeMX 已在 TI12 模式下初始化）
  * @param sign       +1 / -1，来自 app_config.h 的 MOTOR_SIGN_ENC_*
  * @param window_ms  测速窗口长度（毫秒），一般传 ENCODE_WINDOW_MS
  * @return 无
  */
void Encode_Init(Encode_t *enc, TIM_HandleTypeDef *htim, int8_t sign, uint32_t window_ms);

/** @brief 启动编码器接口（HAL_TIM_Encoder_Start，两个通道都要开）。 */
void Encode_Start(Encode_t *enc);

/** @brief 清零累计位置与速度（P1 标定或重连后调用）。 */
void Encode_Reset(Encode_t *enc);

/**
  * @brief 读取计数增量并推进测速窗口。
  *
  * ★ 必须传入**真实的**经过时间 dt_ms，不能假设"调用一次就是 1 ms" ★
  *   本工程的速度环（以及这里的调用）是**每 VEL_LOOP_PERIOD_MS = 10 ms**
  *   才执行一次，所以每次调用要让 elapsed_ms += 10。
  *   如果这里写死 += 1，编码器会以为过了 10 ms、实际过了 100 ms，
  *   窗口时长变成 10 倍、速度读数也**放大 10 倍**
  *   —— 这正是本工程踩过的坑（表现为 measure 比实际大一个数量级、
  *      PID 误判后输出饱和、电机狂转）。
  *
  * @param enc    编码器实例
  * @param dt_ms  距上一次调用的实际毫秒数（调用方用 tick 差值算出来）
  */
void Encode_Update(Encode_t *enc, uint32_t dt_ms);

/** @brief 取累计位置（有符号计数，已按 sign 统一方向）。 */
int32_t Encode_GetCount(const Encode_t *enc);

/** @brief 取最近一个窗口的速度，单位 counts/s（已按 sign 统一方向）。 */
int32_t Encode_GetSpeedCountsPerSec(const Encode_t *enc);

/**
  * @brief 取最近一个窗口的速度，单位 mm/s（该轮的线速度，前进为正）。
  *
  * 返回 float 是因为它要喂给 PID（PID 内部本来就是 float）。
  * 换算：mm/s = counts/s × WHEEL_PERIMETER_MM / counts_per_wheel_rev。
  */
float Encode_GetSpeedMmps(const Encode_t *enc);

/** @brief 取最近一个窗口的速度，单位 rpm（轮子转速，便于和手册对照）。 */
float Encode_GetSpeedRpm(const Encode_t *enc);

#ifdef __cplusplus
}
#endif

#endif /* ROBO_ENCODE_H */
