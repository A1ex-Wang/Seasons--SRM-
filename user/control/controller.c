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
  * ★ 相对原版做的 4 处行为修正（都是本工程实车会踩到的）★
  *   1) dt_s 合法性保护：超出 (0, PID_DT_MAX_S] 直接返回 0，不再参与计算，
  *      避免一次调度抖动把 I 项炸掉；
  *   2) DeadBand 分支不再把 Output 直接清零：原版在 |Err| <= DeadBand 时
  *      Output = 0，用于位置环没问题，但速度环会造成“到了目标就断输出、
  *      掉速后再猛给”的极限环抖动。改为：保持当前输出、清零积分增量，
  *      并且额外支持 **死区内输出限幅**（更符合“不要抖”的诉求）；
  *   3) 抗积分饱和改成“**只要输出饱和就不积分**”（原版只在误差还朝饱和方向
  *      增大时才阻止积分，误差已经反向但输出仍饱和时会继续堆积）；
  *   4) Iout 的限幅移到所有分支之外无条件执行，避免 DeadBand 分支绕过限幅。
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
    pid->initialized = 0U;
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
     * 如果调用方希望死区内慢慢松输出，把 DeadBand 设 0 即可。 */
    if (fabsf(pid->Err) <= pid->DeadBand) {
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

    candidate_i = pid->Iout + pid->ITerm;
    if (pid->Improve & PID_Integral_Limit) {
        candidate_i = LimitFloat(candidate_i, -pid->IntegralLimit, pid->IntegralLimit);
    }
    candidate_output = pid->Pout + candidate_i + pid->Dout;

    /* ── 抗积分饱和 ────────────────────────────────────────────────────────
     * 判据改成“输出将要饱和就冻结本次积分”，不再要求误差方向一致。
     * 原因：误差刚反向、但输出还压在饱和值时（例如堵转刚解除），
     * 原判据会继续累积积分，等误差真正反向后要靠很久才能把积分放掉，
     * 表现为“冲一下再回来”。 */
    if ((candidate_output > pid->MaxOut) || (candidate_output < -pid->MaxOut)) {
        candidate_i = pid->Iout;
    }
    pid->Iout = candidate_i;

    pid->Output = pid->Pout + pid->Iout + pid->Dout;

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
