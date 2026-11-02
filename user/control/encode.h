/** @file encode.h
 * @brief 四路增量编码器接口（定时器编码器模式 TI12 四倍频 + 单周期测速）。
 */

/* 计数差按 16 位回绕处理，单周期增量应在 int16_t 范围内；dt_s 使用秒。 */
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

/*
  * @brief 一路编码器的运行时状态。必须由调用方提供存储（全静态分配）。
 */
typedef struct {
    TIM_HandleTypeDef *htim;   /**< 该轮对应的编码器定时器句柄 */
    int8_t sign;               /**< +1/-1：把计数方向统一成“该轮前进为正” */
    float speed_filter_rc_s;   /**< 测速一阶低通的时间常数 RC（秒），0 = 不滤波 */

    uint16_t last_raw;         /**< 上一次读到的定时器计数值（16 位快照） */
    int32_t total_count;       /**< 累计有符号计数（位置，已乘 sign） */
    int32_t last_delta;        /**< 上一周期的有符号计数增量 */
    float speed_mmps;          /**< 最近一次算出的轮速 [mm/s]，前进为正，已滤波 */

    uint8_t running;           /**< 1 = 定时器已启动并开始计翻转 */
} Encode_t;

/*
  * @brief 初始化一路编码器（不启动定时器）。
  * @param enc               调用方提供的实例存储
  * @param htim              编码器定时器句柄（CubeMX 已在 TI12 模式下初始化）
  * @param sign              +1 / -1，来自 app_config.h 的 MOTOR_SIGN_ENC_*
  * @param speed_filter_rc_s 测速一阶低通 RC（秒），传 VEL_FEEDBACK_LPF_RC_S
 */
void Encode_Init(Encode_t *enc, TIM_HandleTypeDef *htim, int8_t sign,
                 float speed_filter_rc_s);

/** @brief 启动编码器接口（HAL_TIM_Encoder_Start，两个通道都要开）。 */
void Encode_Start(Encode_t *enc);

/** @brief 清零累计位置与速度（标定或重连后调用）。 */
void Encode_Reset(Encode_t *enc);

/*
  * @brief 读计数增量、更新位置，并算出本周期的轮速。
  *
  * @param enc   编码器实例
  * @param dt_s  本周期经过的秒数（本工程固定用 VEL_LOOP_PERIOD_MS/1000 = 0.01）
 */
void Encode_Update(Encode_t *enc, float dt_s);

/** @brief 取累计位置（有符号计数，已按 sign 统一方向）。 */
int32_t Encode_GetCount(const Encode_t *enc);

/** @brief 取上一周期的有符号计数增量。 */
int32_t Encode_GetLastDelta(const Encode_t *enc);

/** @brief 取最近一周期的轮速 [mm/s]（前进为正，已滤波）。 */
float Encode_GetSpeedMmps(const Encode_t *enc);

/** @brief 取最近一周期的轮速 [counts/s]。 */
float Encode_GetSpeedCountsPerSec(const Encode_t *enc);

/** @brief 取最近一周期的速度 [rpm]（便于和电机手册对照）。 */
float Encode_GetSpeedRpm(const Encode_t *enc);

#ifdef __cplusplus
}
#endif

#endif /* ROBO_ENCODE_H */
