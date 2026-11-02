

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

    /* PID 状态：Measure 为反馈值，Ref 为目标值，Err = Ref - Measure。 */
    float Measure;      /**<  当前反馈值 */
    float Last_Measure; /**<  上一周期反馈值 */
    float Ref;          /**<  当前目标值 */
    float Err;          /**< 目标值减反馈值*/
    float Last_Err;
    float Pout;
    float Iout;
    float Dout;
    float ITerm;
    float Output;
    float Last_Output;
    float Last_Dout;

    /* 前馈与 Output 同量纲，参与总输出限幅和抗积分饱和。 */
    float FeedForward;

    /* 本周期冻结积分；PIDCalculate() 使用后清除此标志。 */
    uint8_t IntegralHold;

    uint8_t initialized;
} PIDInstance;
void PIDInit(PIDInstance *pid, const PID_Init_Config_s *config);
void PIDReset(PIDInstance *pid);

void PID_FreezeIntegral(PIDInstance *pid);

/**
 * @brief 计算一次 PID 输出。
 * @param pid       PID 实例
 * @param measure   反馈值（实测）
 * @param ref       目标值
 * @param dt_s      距上次调用的时间，单位秒（合法区间约 (1e-6, 0.1]）
 * @return 限幅后的输出，范围 ±pid->MaxOut（含前馈）
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref, float dt_s);
#endif