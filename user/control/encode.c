/**
  ******************************************************************************
  * @file    encode.c
  * @brief   四路增量编码器实现（单周期计数差 + 一级低通）。
  *
  * ★ 逻辑与作者已跑通的项目 D:\STM32\PID_Freertos\user\motor\encode.c 一致 ★
  *
  * Encode_Update(enc, dt_s) 每个控制周期被调用一次（本工程 10 ms = 100 Hz），
  * 内部只有两步：
  *
  *     1) now_raw = __HAL_TIM_GET_COUNTER(htim);
  *        delta   = (int16_t)(now_raw - last_raw);      // 自动处理 16 位回绕
  *        signed  = delta * sign;                        // 统一成"前进为正"
  *        total_count += signed;
  *
  *     2) raw_mmps  = signed * ENCODER_COUNTS_TO_MMPS / dt_s;
  *        speed_mmps += alpha * (raw_mmps - speed_mmps);   // 一级一阶低通
  *
  * 没有别的了。没有限幅、没有守卫、没有自检、没有窗口、没有累加器
  * —— 为什么必须这样，详见 encode.h 文件头（六次"加保护反而制造故障"的记录）。
  ******************************************************************************
  */

#include "encode.h"

#include <stddef.h>

void Encode_Init(Encode_t *enc, TIM_HandleTypeDef *htim, int8_t sign,
                 float speed_filter_rc_s) {
    if (enc == NULL) return;

    enc->htim = htim;
    enc->sign = (sign < 0) ? (int8_t)-1 : (int8_t)1;
    enc->speed_filter_rc_s = (speed_filter_rc_s > 0.0f) ? speed_filter_rc_s : 0.0f;

    enc->last_raw = 0u;
    enc->total_count = 0;
    enc->last_delta = 0;
    enc->speed_mmps = 0.0f;
    enc->running = 0u;
}

void Encode_Start(Encode_t *enc) {
    if (enc == NULL || enc->htim == NULL) return;

    /* 两个通道都要启动（TI12 模式下 CH1/CH2 都是输入，HAL 要求 CHANNEL_ALL）。 */
    (void)HAL_TIM_Encoder_Start(enc->htim, TIM_CHANNEL_ALL);

    /* 以当前计数值为基准，避免把“上电前残留的计数值”当成运动。 */
    enc->last_raw = (uint16_t)__HAL_TIM_GET_COUNTER(enc->htim);
    enc->total_count = 0;
    enc->last_delta = 0;
    enc->speed_mmps = 0.0f;
    enc->running = 1u;
}

void Encode_Reset(Encode_t *enc) {
    if (enc == NULL || enc->htim == NULL) return;

    __HAL_TIM_SET_COUNTER(enc->htim, 0u);
    enc->last_raw = 0u;
    enc->total_count = 0;
    enc->last_delta = 0;
    enc->speed_mmps = 0.0f;
}

void Encode_Update(Encode_t *enc, float dt_s) {
    uint16_t now_raw;
    int32_t delta;
    float raw_mmps;
    float alpha;

    if (enc == NULL || enc->htim == NULL) return;
    if (enc->running == 0u) return;
    if (!(dt_s > 0.0f)) return;

    /* ── 1) 读计数、算有符号增量、累加位置 ──
     * (int16_t) 做差利用二进制补码自动处理 16 位计数器回绕。 */
    now_raw = (uint16_t)__HAL_TIM_GET_COUNTER(enc->htim);
    delta = (int32_t)(int16_t)((uint16_t)(now_raw - enc->last_raw));
    enc->last_raw = now_raw;

    if (enc->sign < 0) {
        delta = -delta;
    }
    enc->last_delta = delta;
    enc->total_count += delta;

    /* ── 2) 单周期测速 ──
     * raw_mmps = 计数增量 × (mm/count) ÷ dt。
     * ENCODER_COUNTS_TO_MMPS = 轮周长 / 每轮计数 = 188.4956 / 1560 = 0.12083。 */
    raw_mmps = ((float)delta * ENCODER_COUNTS_TO_MMPS) / dt_s;

    /* ── 3) 一级一阶低通（唯一的一级滤波，下游不再滤波）──
     * RC 由调用方给出（VEL_FEEDBACK_LPF_RC_S）；RC 为 0 时直接取原值。 */
    if (enc->speed_filter_rc_s <= 0.0f) {
        alpha = 1.0f;
    } else {
        alpha = dt_s / (enc->speed_filter_rc_s + dt_s);
        if (alpha > 1.0f) alpha = 1.0f;
    }
    enc->speed_mmps += alpha * (raw_mmps - enc->speed_mmps);
}

int32_t Encode_GetCount(const Encode_t *enc) {
    return (enc == NULL) ? 0 : enc->total_count;
}

int32_t Encode_GetLastDelta(const Encode_t *enc) {
    return (enc == NULL) ? 0 : enc->last_delta;
}

float Encode_GetSpeedMmps(const Encode_t *enc) {
    return (enc == NULL) ? 0.0f : enc->speed_mmps;
}

float Encode_GetSpeedCountsPerSec(const Encode_t *enc) {
    if (enc == NULL) return 0.0f;
    return enc->speed_mmps / ENCODER_COUNTS_TO_MMPS;
}

float Encode_GetSpeedRpm(const Encode_t *enc) {
    if (enc == NULL) return 0.0f;
    /* mm/s → 轮转/s → rpm：mm/s ÷ 轮周长(mm) × 60。 */
    return enc->speed_mmps * 60.0f / WHEEL_PERIMETER_MM;
}
