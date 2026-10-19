/**
  ******************************************************************************
  * @file    controller.c
  * @brief   通用 PID 控制器（源自 RM 标准库风格，本工程按其使用场景修过若干行为）。
  *
  * ★ 调用约定（最容易踩的坑）★
  *   PIDCalculate(pid, measure, ref, dt_s) 的最后一个参数是
  *   **dt_s —— 单位【秒】，不是毫秒！**
  *   100 Hz 速度环要传 0.01f，不是 10。传错会让 I 项差 1000 倍，
  *   现象是“车冲出去并剧烈振荡”，而代码看起来完全正常。
  *   下面有运行期保护把 dt_s 限制在合理区间，但请仍然按约定传值。
  *
  * 速度环相关行为：dt_s 非法时保持输出；DeadBand > 0 时才启用误差死区。
  * 总输出包含前馈，饱和时禁止加深饱和的积分，但允许反向释放积分。
  ******************************************************************************
  */

#include "controller.h"

#include <math.h>
#include <string.h>

/** @brief dt_s 的合法上限（秒）。超过它认为调用方传了毫秒或发生了调度异常。 */
#define PID_DT_MAX_S 0.1f

/** @brief dt_s 的合法下限（秒）。低于它认为时间没有推进。 */
#define PID_DT_MIN_S 0.000001f

static float LimitFloat(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void PIDReset(PIDInstance *pid) {
    pid->Measure = 0.0f;
    pid->Last_Measure = 0.0f;
    pid->Ref = 0.0f;
    pid->Err = 0.0f;
    pid->Last_Err = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->ITerm = 0.0f;
    pid->Output = 0.0f;
    pid->Last_Output = 0.0f;
    pid->Last_Dout = 0.0f;
    pid->FeedForward = 0.0f;
    pid->IntegralHold = 0u;
    pid->initialized = 0U;
}

void PID_FreezeIntegral(PIDInstance *pid) {
    if (pid == NULL) return;

    /* Iout 非有限值（NaN/Inf，例如曾经被坏采样污染过）时直接清零，
     * 否则任何限幅比较都会失效，积分就再也管不住了。 */
    if (!isfinite(pid->Iout)) {
        pid->Iout = 0.0f;
    }
    /* 只置标志：PIDCalculate() 看到它就不更新积分（保持当前值不动，
     * 既不夹边界也不清零），处理完自动清除。 */
    pid->IntegralHold = 1u;
}

void PIDInit(PIDInstance *pid, const PID_Init_Config_s *config) {
    if (pid == NULL || config == NULL) return;

    memset(pid, 0, sizeof(*pid));
    pid->Kp = config->Kp;
    pid->Ki = config->Ki;
    pid->Kd = config->Kd;
    pid->MaxOut = config->MaxOut;
    pid->DeadBand = config->DeadBand;
    pid->Improve = config->Improve;
    pid->IntegralLimit = config->IntegralLimit;
    pid->CoefA = config->CoefA;
    pid->CoefB = config->CoefB;
    pid->Output_LPF_RC = config->Output_LPF_RC;
    pid->Derivative_LPF_RC = config->Derivative_LPF_RC;
    PIDReset(pid);
}

float PIDCalculate(PIDInstance *pid, float measure, float ref, float dt_s) {
    float candidate_i;
    float candidate_output;
    float dt;

    if (pid == NULL || pid->MaxOut <= 0.0f) {
        return 0.0f;
    }
    /* dt_s 保护：非法就直接返回，绝不用错误的 dt 去累积积分。 */
    if (!(dt_s > PID_DT_MIN_S) || (dt_s > PID_DT_MAX_S)) {
        return pid->Output;
    }
    dt = dt_s;

    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = ref - measure;
    if (pid->initialized == 0U) {
        pid->Last_Err = pid->Err;
        pid->Last_Measure = pid->Measure;
        pid->initialized = 1U;
    }

    /* ── 死区分支 ──────────────────────────────────────────────────────────
     * 与原版的区别：不动 Output、不动 Iout，只是不再累积新的积分。
     * 这样“误差进入死区”时输出保持在上一次的值，电机不会被突然断输出，
     * 也就不会出现“断输出→掉速→误差变大→再给输出”的极限环。
     * DeadBand = 0 时禁用本分支，误差恰好为零也重新合成输出及前馈。 */
    if (pid->DeadBand > 0.0f && fabsf(pid->Err) <= pid->DeadBand) {
        pid->ITerm = 0.0f;
        pid->Last_Measure = pid->Measure;
        pid->Last_Err = pid->Err;
        pid->Last_Output = pid->Output;
        pid->Last_Dout = pid->Dout;
        return pid->Output;
    }

    pid->Pout = pid->Kp * pid->Err;
    pid->ITerm = pid->Ki * pid->Err * dt;
    pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / dt;

    if (pid->Improve & PID_Trapezoid_Intergral) {
        pid->ITerm = pid->Ki * (pid->Err + pid->Last_Err) * dt * 0.5f;
    }
    if (pid->Improve & PID_ChangingIntegrationRate) {
        float abs_error = fabsf(pid->Err);
        if (pid->Err * pid->Iout > 0.0f) {
            if (abs_error > pid->CoefB && abs_error <= pid->CoefA + pid->CoefB) {
                pid->ITerm *= (pid->CoefA - abs_error + pid->CoefB) / pid->CoefA;
            } else if (abs_error > pid->CoefA + pid->CoefB) {
                pid->ITerm = 0.0f;
            }
        }
    }
    if (pid->Improve & PID_Derivative_On_Measurement) {
        pid->Dout = -pid->Kd * (pid->Measure - pid->Last_Measure) / dt;
    }
    if ((pid->Improve & PID_DerivativeFilter) && pid->Derivative_LPF_RC > 0.0f) {
        pid->Dout = pid->Dout * dt / (pid->Derivative_LPF_RC + dt)
                    + pid->Last_Dout * pid->Derivative_LPF_RC
                          / (pid->Derivative_LPF_RC + dt);
    }

    /* ── 积分保持（冻结）──────────────────────────────────────────────────
     * 调用方（速度环）在"低速段反馈不可信"时会调 PID_FreezeIntegral() 置位。
     * 这里让 candidate_i 保持为当前值，既不累积也不被限幅夹到别处。
     * 处理完立刻清标志，调用方不需要负责复位，因此下一个周期自动恢复正常积分。 */
    if (pid->IntegralHold != 0u) {
        pid->IntegralHold = 0u;
        candidate_i = pid->Iout;
    } else {
        candidate_i = pid->Iout + pid->ITerm;
        if (pid->Improve & PID_Integral_Limit) {
            candidate_i = LimitFloat(candidate_i, -pid->IntegralLimit, pid->IntegralLimit);
        }
    }
    /* ★ 前馈必须计入饱和判断 ★
     * pid->FeedForward 由调用方在每个周期开始前写入（占空比分数，与 Output 同量纲）。
     * 如果前馈加在 PIDCalculate() 之外（调用方自己 +），那么下面这个抗饱和判据
     * 就看不到真实的总输出：明明已经饱和了，PID 却以为还有余量，于是继续累积积分，
     * 等退出饱和时就会"抽"一下。
     * 注意分母：candidate_output 是"P+I+D"三项之和，加前馈后才是真正写入执行器的量。 */
    candidate_output = pid->Pout + candidate_i + pid->Dout + pid->FeedForward;

    /* 含前馈的总输出饱和时，只阻止继续加深饱和的积分增量。
     * 允许反向积分释放已有积累，避免饱和后积分始终无法恢复。
     * 用实际增量判断，兼容积分限幅、梯形积分和冻结积分。 */
    if (((candidate_output > pid->MaxOut) && (candidate_i > pid->Iout)) ||
        ((candidate_output < -pid->MaxOut) && (candidate_i < pid->Iout))) {
        candidate_i = pid->Iout;
    }
    pid->Iout = candidate_i;

    /* 输出 = P + I + D + 前馈，然后统一限幅。
     * 内部保留完整值供遥测/抗饱和使用，最后再钳到 ±MaxOut。 */
    pid->Output = pid->Pout + pid->Iout + pid->Dout + pid->FeedForward;

    if ((pid->Improve & PID_OutputFilter) && pid->Output_LPF_RC > 0.0f) {
        pid->Output = pid->Output * dt / (pid->Output_LPF_RC + dt)
                      + pid->Last_Output * pid->Output_LPF_RC
                            / (pid->Output_LPF_RC + dt);
    }
    pid->Output = LimitFloat(pid->Output, -pid->MaxOut, pid->MaxOut);

    pid->Last_Measure = pid->Measure;
    pid->Last_Err = pid->Err;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    return pid->Output;
}
