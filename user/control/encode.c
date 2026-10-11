/**
  ******************************************************************************
  * @file    encode.c
  * @brief   四路增量编码器接口的实现（多实例 + 定窗口测速）。
  *
  * 调用约定：Encode_Update() 每 1 ms 调用一次（由 Chassis 任务调用），
  *           每 window_ms（默认 10 ms）更新一次速度。
  *
  * ★ 关于 16 位计数器回绕：
  *   TIM1/TIM3/TIM4 的计数器是 16 位，编码器正转越过 0xFFFF 会回到 0，
  *   反转越过 0 会回到 0xFFFF。这里用 (int16_t)(now - last) 做差，
  *   利用二进制补码减法自动得到正确的有符号增量，**前提是两次读取之间
  *   计数变化不超过 ±32767**。
  *   本车满速 1046 mm/s、44 counts/轮转、1 ms 读取周期：
  *     满速计数率 = 1046/188.5 × 44 = 244 counts/s → 每毫秒仅 0.24 个计数，
  *     距离 ±32767 有 5 个数量级的余量，绝对安全。
  *   （TIM2 是 32 位计数器，更宽松，但用同样的写法也正确。）
  ******************************************************************************
  */

#include "encode.h"

#include <stddef.h>

/* 计数差值的类型：用 int16_t 做减法再提升为 int32_t，天然处理回绕。 */
#define ENCODE_RAW_DIFF(now, last) ((int32_t)(int16_t)((uint16_t)(now) - (uint16_t)(last)))

void Encode_Init(Encode_t *enc, TIM_HandleTypeDef *htim, int8_t sign, uint32_t window_ms) {
    if (enc == NULL) return;

    enc->htim = htim;
    enc->sign = (sign < 0) ? (int8_t)-1 : (int8_t)1;
    enc->window_ms = (window_ms == 0u) ? 1u : window_ms;

    enc->last_raw = 0u;
    enc->total_count = 0;
    enc->window_count = 0;
    enc->speed_counts_s = 0;
    enc->elapsed_ms = 0;
    enc->running = 0u;
}

void Encode_Start(Encode_t *enc) {
    if (enc == NULL || enc->htim == NULL) return;

    /* 两个通道都要启动（TI12 模式下 CH1/CH2 都是输入，HAL 要求 CHANNEL_ALL）。 */
    (void)HAL_TIM_Encoder_Start(enc->htim, TIM_CHANNEL_ALL);

    /* 以当前计数值为基准，避免把“上电前残留的计数值”当成运动。 */
    enc->last_raw = (uint16_t)__HAL_TIM_GET_COUNTER(enc->htim);
    enc->total_count = 0;
    enc->window_count = 0;
    enc->speed_counts_s = 0;
    enc->elapsed_ms = 0;
    enc->running = 1u;
}

void Encode_Reset(Encode_t *enc) {
    if (enc == NULL || enc->htim == NULL) return;
    __HAL_TIM_SET_COUNTER(enc->htim, 0u);
    enc->last_raw = 0u;
    enc->total_count = 0;
    enc->window_count = 0;
    enc->speed_counts_s = 0;
    enc->elapsed_ms = 0;
}

void Encode_Update(Encode_t *enc) {
    uint16_t now_raw;
    int32_t delta;

    if (enc == NULL || enc->htim == NULL) return;
    if (enc->running == 0u) return;

    /* ── 1) 读硬件计数器并算出有符号增量（自动处理 16 位回绕）── */
    now_raw = (uint16_t)__HAL_TIM_GET_COUNTER(enc->htim);
    delta = ENCODE_RAW_DIFF(now_raw, enc->last_raw);
    enc->last_raw = now_raw;

    if (delta != 0) {
        /* 乘 sign 之后，正数一律表示“该轮物理前进”。 */
        int32_t signed_delta = (enc->sign > 0) ? delta : -delta;
        enc->total_count += signed_delta;
        enc->window_count += signed_delta;
    }

    /* ── 2) 窗口计时；未到窗口末尾直接返回，不做除法 ── */
    enc->elapsed_ms += 1;
    if ((uint32_t)enc->elapsed_ms < enc->window_ms) {
        return;
    }

    /* ── 3) 窗口结束：counts/s = 窗口计数 × 1000 / 窗口毫秒 ──
     * 先乘 1000 再除，避免整数除法把低速段全部截断成 0。
     * 窗口计数满速时约 2~3，乘 1000 后仍在 int32 范围内，不会溢出。 */
    enc->speed_counts_s = (enc->window_count * 1000) / (int32_t)enc->window_ms;

    enc->window_count = 0;
    enc->elapsed_ms = 0;
}

int32_t Encode_GetCount(const Encode_t *enc) {
    return (enc == NULL) ? 0 : enc->total_count;
}

int32_t Encode_GetSpeedCountsPerSec(const Encode_t *enc) {
    return (enc == NULL) ? 0 : enc->speed_counts_s;
}

float Encode_GetSpeedMmps(const Encode_t *enc) {
    if (enc == NULL) return 0.0f;
    /* counts/s → mm/s：1 counting/s 对应 4.284 mm/s（见 app_config.h 第 7 节）。 */
    return (float)enc->speed_counts_s * ENCODER_COUNTS_TO_MMPS;
}

float Encode_GetSpeedRpm(const Encode_t *enc) {
    if (enc == NULL) return 0.0f;
    /* counts/s ÷ counts_per_wheel_rev = 轮转/s，再 × 60 = rpm。 */
    return (float)enc->speed_counts_s * 60.0f / ENCODER_COUNT_PER_WHEEL_REV;
}
