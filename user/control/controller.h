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
    float Measure;
    float Last_Measure;
    float Ref;
    float Err;
    float Last_Err;
    float Pout;
    float Iout;
    float Dout;
    float ITerm;
    float Output;
    float Last_Output;
    float Last_Dout;
    uint8_t initialized;
} PIDInstance;
void PIDInit(PIDInstance *pid, const PID_Init_Config_s *config);
void PIDReset(PIDInstance *pid);

/**
  * @brief 计算一次 PID 输出。
  *
  * ⚠ 最后一个参数 dt_s 的单位是【秒】，不是毫秒。
  *   100 Hz 的速度环请传 0.01f；传 10 会让 I 项差 1000 倍并导致整车失控振荡。
  *
  * @param pid       PID 实例
  * @param measure   反馈值
  * @param ref       目标值
  * @param dt_s      距上次调用的时间，单位秒（合法区间约 (1e-6, 0.1]）
  * @return 限幅后的输出，范围 ±pid->MaxOut
  */
float PIDCalculate(PIDInstance *pid, float measure, float ref, float dt_s);
#endif