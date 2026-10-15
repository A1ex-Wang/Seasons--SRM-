//
// Created by 18045 on 2026/9/2.
//
#ifndef USER_MOTOR_CONTROLLER_H
#define USER_MOTOR_CONTROLLER_H
#include <stdint.h>
typedef enum {
    PID_IMPROVE_NONE = 0U,
    PID_Integral_Limit = 1U << 0,
    PID_Derivative_On_Measurement = 1U << 1,
    PID_Trapezoid_Intergral = 1U << 2,
    PID_Proportional_On_Measurement = 1U << 3,
    PID_OutputFilter = 1U << 4,
    PID_ChangingIntegrationRate = 1U << 5,
    PID_DerivativeFilter = 1U << 6,
    PID_ErrorHandle = 1U << 7
    } PID_Improvement_e;
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float DeadBand;
    PID_Improvement_e Improve;
    float IntegralLimit;
    float CoefA;
    float CoefB;
    float Output_LPF_RC;
    float Derivative_LPF_RC;
} PID_Init_Config_s;
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float DeadBand;
    PID_Improvement_e Improve;
    float IntegralLimit;
    float CoefA;
    float CoefB;
    float Output_LPF_RC;
    float Derivative_LPF_RC;

    /* ── 下面两个字段的名字来自 RM 标准库，**语义与名字相反，务必看清** ──────
     * ⚠ pid->Measure = 调用方传入的【目标值】(ref)
     * ⚠ pid->Ref     = 调用方传入的【实测值】(measure)
     * ⚠ pid->Err     = 目标值 − 实测值（这个是对的）
     *
     * 例：目标是 450 mm/s、实测是 380 mm/s 时，
     *       pid->Measure == 450.0f   ← 名字叫 Measure 但装的是目标
     *       pid->Ref     == 380.0f   ← 名字叫 Ref 但装的是实测
     *       pid->Err     == 70.0f
     * 在 Ozone 里 Watch 时**不要**拿 Ref 当目标去比，否则会把"实测"误读成"目标"。
     * 想看得清楚就用 VelCtl_GetWheelTargetMmps() / VelCtl_GetWheelSpeedMmps()。 */
    float Measure;      /**< ⚠ 实际装的是【目标值】 */
    float Last_Measure; /**< ⚠ 上一周期的【目标值】 */
    float Ref;          /**< ⚠ 实际装的是【实测值】 */
    float Err;          /**< 目标 − 实测（这个语义正确）*/
    float Last_Err;
    float Pout;
    float Iout;
    float Dout;
    float ITerm;
    float Output;
    float Last_Output;
    float Last_Dout;

    /* 前馈项（占空比分数，与 Output 同量纲）。由调用方在每个周期开始前写入，
     * 由 PIDCalculate() 计入饱和判断与抗积分饱和。
     * 放在结构体里（而不是让调用方在 PIDCalculate 之外自己加）的原因：
     * 如果前馈加在函数外，PID 内部的抗饱和看不到它，会误判"输出没饱和"
     * 而继续累积积分，退出饱和时就会"抽"一下。 */
    float FeedForward;

    /* 积分保持标志：1 = 本周期不更新积分（P 项照常工作）。
     * 由调用方在需要时置位（例如低速段反馈不可信），PIDCalculate() 处理完后
     * **自动清零**，所以调用方不需要负责复位。
     * 为什么需要它：光靠 IntegralLimit 做不到"冻结"，因为限幅会把超出的值
     * 夹到边界而不是保持不变；而用 fabsf(Iout) 做限幅又会让负积分被夹到 0
     * （不对称）。用一个显式标志最直白、也最安全。 */
    uint8_t IntegralHold;

    uint8_t initialized;
} PIDInstance;
void PIDInit(PIDInstance *pid, const PID_Init_Config_s *config);
void PIDReset(PIDInstance *pid);

void PID_FreezeIntegral(PIDInstance *pid);

/**
  * @brief 计算一次 PID 输出。
  *
  * ⚠ 参数顺序是 (pid, measure, ref, dt_s)：第 2 个是【反馈值】，第 3 个是【目标值】。
  *   注意这与 PIDInstance 内部字段名相反（见上方关于 Measure/Ref 的警告）。
  *
  * ⚠ 最后一个参数 dt_s 的单位是【秒】，不是毫秒。
  *   100 Hz 的速度环请传 0.01f；传 10 会让 I 项差 1000 倍并导致整车失控振荡。
  *
  * 前馈由 pid->FeedForward 提供（调用方每周期开始时写入），会一并计入
  * 饱和判断与输出，因此调用方**不要**再在函数外加前馈。
  *
  * @param pid       PID 实例
  * @param measure   反馈值（实测）
  * @param ref       目标值
  * @param dt_s      距上次调用的时间，单位秒（合法区间约 (1e-6, 0.1]）
  * @return 限幅后的输出，范围 ±pid->MaxOut（含前馈）
  */
float PIDCalculate(PIDInstance *pid, float measure, float ref, float dt_s);
#endif