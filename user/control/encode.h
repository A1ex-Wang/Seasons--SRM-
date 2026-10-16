/**
  ******************************************************************************
  * @file    encode.h
  * @brief   四路增量编码器接口（定时器编码器模式 TI12 四倍频 + 单周期测速）。
  *
  * ★ 本文件**刻意保持极简**，逻辑与作者已经跑通的项目
  *   D:\STM32\PID_Freertos\user\motor\encode.c 完全一致，只做两件事：
  *
  *       ① 读定时器计数，算有符号增量（自动处理 16 位回绕）；
  *       ② raw = 增量 × (mm/count) / dt，再过一级一阶低通得到 speed_mmps。
  *
  *   就这么两行。**没有窗口、没有累加器、没有限幅、没有守卫、没有自检。**
  *
  * ★★★ 为什么必须这么极简（本工程的惨痛教训）★★★
  *   这个速度环先后被加过下面这些"保护"，每一样都制造了新的故障：
  *
  *     ① 定窗口累加测速（window_count / elapsed_ms）
  *        → 中间状态被写坏时算出物理不可能的速度（-148150 mm/s）
  *     ② "拒绝采样"：检测到坏值就 return，本周期不更新 speed_mmps
  *        → **永久死锁**：speed_mmps 恒为 0，PID 看到 meas=0 输出饱和、轮子狂转
  *     ③ 变化率限幅器 + 独立二次 LPF + s_meas_prev
  *        → 限幅时篡改滤波器历史，滤波器失效
  *     ④ 速度物理上限 clamp（阈值取 WHEEL_MAX_SPEED_MMPS 本身）
  *        → 正常运转就 27% 的周期触发限幅，speed 被压在 1046，
  *          与同周期的 last_delta 自相矛盾（13 个计数 vs 1046 mm/s，差 6.7 倍）
  *     ⑤ DWT 自测 dt_s
  *        → 差值出现 23 µs 这种物理不可能的短间隔
  *     ⑥ dt_s 合法区间守卫
  *        → 又一次"换掉分母"，链路被切断
  *
  *   **结论：这条链上每加一层"保护"，就多一条故障路径。**
  *   参考项目没有这一层，它跑通了；所以本文件也全部删除，回到两行。
  *
  * ★ 关于 dt（参考项目 control_time.c 用 DWT 实测）★
  *   本工程的速度环是由 FreeRTOS tick（1 ms）驱动的：Chassis 任务每 tick 跑一次、
  *   每 VEL_LOOP_PERIOD_MS 个 tick 调一次速度环。节拍本身就是 10 ms，
  *   所以这里直接用**标称周期 0.01 s**，并让测速与 PID 共用**同一个** dt，
  *   两者不可能不一致。周期偶有抖动只是把 Kp/Ki 的等效增益按比例缩放几个百分点，
  *   不改变闭环性质。
  *
  * ★ 计数器回绕：用 (int16_t)(now - last) 做差，依赖二进制补码自动得到
  *   有符号增量，前提是两次读取之间计数变化不超过 ±32767。
  *   本车满速 1046 mm/s、1560 counts/轮转、10 ms 周期 → 每周期约 87 个计数，
  *   余量两个数量级，绝对安全（TIM2 是 32 位，写法相同也正确）。
  *
  * 为什么不能用"输入捕获测周法（T 法）"：
  *   四路编码器占满 TIM1/2/3/4 的 CH1/CH2，输入捕获需要同样的通道，互斥；
  *   且 PB6 在 F407 上只能作 TIM4_CH1、没有 CH2 可用。只能走计数差（M 法）。
  *
  * ★★ 编码器定时器的数字滤波器必须两相相同 ★★
  *   Core/Src/tim.c 里 TIM1/2/3/4 的 IC1Filter 曾经是 0（不滤波）而 IC2Filter 是 8。
  *   正交解码靠"哪个通道先跳变"判方向，两相滤波不对称会让同一个真实边沿被
  *   算成"先正后反"两次计数。参考项目用的是对称的 5，本工程现在是对称的 8。
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

/**
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

/**
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

/**
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
