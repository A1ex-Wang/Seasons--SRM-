/**
  ******************************************************************************
  * @file    velocity_control.c
  * @brief   底盘速度控制：遥控映射 → 加速度限幅 → 麦轮逆运动学 → 四路独立 PI。
  *
  * 调用：由 Chassis 任务每 VEL_LOOP_PERIOD_MS（10 ms）调用 VelCtl_Feed() 一次。
  *       本文件不做任何阻塞操作，全部是定长整数/浮点运算。
  *
  * ★ 坐标与量纲约定（全工程统一）★
  *   车体坐标系：x 向前、y 向左、ω 逆时针为正（俯视，右手系 z 向上）
  *   单位：v 用 mm/s，ω 用 rad/s（内部乘 CHASSIS_K_MM 换算成轮子的 mm/s）
  *
  * ★ 摇杆映射（按你和协议约定）★
  *   ly > 0 = 前进 → vy = +ly/512 × MAX_VXY
  *   ly < 0 = 后退
  *   lx < 0 = 向左 → vx = −lx/512 × MAX_VXY   （负号来自“左 = y 正方向”）
  *   lx > 0 = 向右
  *   rx > 0 = 顺时针 → ω = −rx/512 × MAX_OMEGA （负号来自“逆时针为正”）
  *   ry / lt / rt 本阶段不参与控制，原样透传（照旧打印）。
  *
  * ★ 麦轮逆运动学（X 型布置，辊子 45°）★
  *   w_FL = +vx +vy +k·ω
  *   w_FR = +vx −vy −k·ω
  *   w_RL = +vx −vy +k·ω
  *   w_RR = +vx +vy −k·ω      k = CHASSIS_K_MM = 262 mm
  *   符号组合取决于辊子朝向与电机左右安装，**必须用 P1 标定实测确认**；
  *   如果实测发现某一项反号，改的是 app_config.h 的 MOTOR_SIGN_* ，不是这个公式。
  *
  * ★ 饱和处理 ★
  *   若任一 |w_i| 超过 WHEEL_MAX_SPEED_MMPS，则四个轮速**整体等比缩放**。
  *   注意这个策略的副作用：满速直行时 ω 会被砍到 0（因为 0.9 m/s 已经接近
  *   轮速上限）。所以 MAX_VXY_MS 刻意没有取到物理极限 1.046。若日后觉得
  *   “高速时转不动弯”很别扭，把这里换成“平移优先、只压缩 ω”的策略即可。
  ******************************************************************************
  */

#include "velocity_control.h"

#include <math.h>
#include <stddef.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_config.h"
#include "app_types.h"
#include "control/controller.h"
#include "control/encode.h"
#include "motor_driver.h"
#include "stm32f4xx_hal_tim.h"

/* CubeMX 在 Core/Src/tim.c 生成的编码器定时器句柄。 */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/*============================================================================*/
/* 静态对象（全部 .bss / SRAM1，无 heap、无 CCM）                              */
/*============================================================================*/

static Encode_t s_enc[WHEEL_COUNT];
static PIDInstance s_pid[WHEEL_COUNT];

/** @brief 遥测用快照，不参与控制计算。 */
static motor_cmd_t s_cmd_snapshot;
motor_cmd_t g_motor_cmd;

/** @brief 四轮速度反馈的滤波状态（mm/s），初值在 VelCtl_Init() 里清零。 */
static float s_speed_lpf[WHEEL_COUNT];

/* 上一周期的车体指令（用于加速度限幅）。单位 mm/s、mrad/s。 */
static float s_vx_prev = 0.0f;
static float s_vy_prev = 0.0f;
static float s_omega_prev = 0.0f;

/** @brief 链路无效时的安全态标志：1 = 已经处于全零输出，不必重复写 PWM。 */
static uint8_t s_safe_latched = 1u;

/*============================================================================*/
/* 工具函数                                                                    */
/*============================================================================*/

/**
  * @brief 摇杆轴 → 归一化 [-1, 1]，带死区。
  *
  * 死区内的值直接当 0；死区外**不做拉伸重映射**，这样刚出死区时出力是连续的
  * （如果做重映射，摇杆一离开死区就会跳到一个较大的值，手感很硬）。
  *
  * 标定模式与闭环模式都要用，所以放在条件编译之外。
  */
static float axis_normalized(int16_t raw) {
    if (raw > REMOTE_DEADZONE) {
        return (float)raw / REMOTE_AXIS_FULL_SCALE;
    }
    if (raw < -REMOTE_DEADZONE) {
        return (float)raw / REMOTE_AXIS_FULL_SCALE;
    }
    return 0.0f;
}

/* 下面两个工具只被闭环路径使用；标定模式下整体不编译，避免无用告警。 */
#if (MOTOR_TEST_MODE == 0)

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int32_t clampi32(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/**
  * @brief 一阶低通（用于速度反馈滤波）。
  * @param prev 上一次的滤波输出
  * @param now  本次原始值
  * @param rc_s RC 时间常数（秒）
  * @param dt_s 采样周期（秒）
  */
static float lpf1(float prev, float now, float rc_s, float dt_s) {
    float alpha;

    if (rc_s <= 0.0f) {
        return now;
    }
    alpha = dt_s / (rc_s + dt_s);
    if (alpha > 1.0f) alpha = 1.0f;
    return prev + alpha * (now - prev);
}

#endif /* MOTOR_TEST_MODE == 0 */

/*============================================================================*/
/* 初始化                                                                      */
/*============================================================================*/

/** @brief 四个 PID 的整定参数。每个轮子一份，标定后可以分别改。 */
static const PID_Init_Config_s s_pid_cfg[WHEEL_COUNT] = {
    /* 四个轮子先给同一组初值。因为四台电机型号相同、机械结构对称，
     * 在“已经用 SIGN_* 和 DEADBAND_* 把每轮归一化到同样的 mm/s 量纲”之后，
     * Kp/Ki 本来就应该接近一致。如果标定后某一轮明显不同（比如阻力大），
     * 只改那一份即可 —— 这正是把 PID 做成每轮一份的意义。 */
    {VEL_KP, VEL_KI, VEL_KD, 1.0f, 0.0f, PID_Integral_Limit, VEL_INTEGRAL_LIMIT,
     0.0f, 0.0f, VEL_OUTPUT_LPF_RC_S, 0.0f},
    {VEL_KP, VEL_KI, VEL_KD, 1.0f, 0.0f, PID_Integral_Limit, VEL_INTEGRAL_LIMIT,
     0.0f, 0.0f, VEL_OUTPUT_LPF_RC_S, 0.0f},
    {VEL_KP, VEL_KI, VEL_KD, 1.0f, 0.0f, PID_Integral_Limit, VEL_INTEGRAL_LIMIT,
     0.0f, 0.0f, VEL_OUTPUT_LPF_RC_S, 0.0f},
    {VEL_KP, VEL_KI, VEL_KD, 1.0f, 0.0f, PID_Integral_Limit, VEL_INTEGRAL_LIMIT,
     0.0f, 0.0f, VEL_OUTPUT_LPF_RC_S, 0.0f},
};

void VelCtl_Init(void) {
    int i;

    /* 编码器绑定表。顺序与 WheelId_e 一致：FL, FR, RL, RR。 */
    Encode_Init(&s_enc[WHEEL_FL], &htim2, (int8_t)MOTOR_SIGN_ENC_FL, ENCODE_WINDOW_MS);
    Encode_Init(&s_enc[WHEEL_FR], &htim1, (int8_t)MOTOR_SIGN_ENC_FR, ENCODE_WINDOW_MS);
    Encode_Init(&s_enc[WHEEL_RL], &htim4, (int8_t)MOTOR_SIGN_ENC_RL, ENCODE_WINDOW_MS);
    Encode_Init(&s_enc[WHEEL_RR], &htim3, (int8_t)MOTOR_SIGN_ENC_RR, ENCODE_WINDOW_MS);

    /* 先读一次初值再启动，避免把上电前的残留计数当成运动。 */
    for (i = 0; i < (int)WHEEL_COUNT; i++) {
        Encode_Start(&s_enc[i]);
        s_speed_lpf[i] = 0.0f;
    }

    for (i = 0; i < (int)WHEEL_COUNT; i++) {
        PIDInit(&s_pid[i], &s_pid_cfg[i]);
    }

    s_vx_prev = 0.0f;
    s_vy_prev = 0.0f;
    s_omega_prev = 0.0f;
    s_safe_latched = 1u;

    for (i = 0; i < (int)WHEEL_COUNT; i++) {
        s_cmd_snapshot.duty[i] = 0;
        s_cmd_snapshot.active[i] = 0u;
    }
    g_motor_cmd = s_cmd_snapshot;

    /* PWM 与方向脚的初始化放在最后：先把内部状态准备好，
     * 再让电机具备输出的能力，避免上电瞬间出现未定义占空比。 */
    Motor_Init();
}

void VelCtl_SafeState(void) {
    int i;

    /* 安全态要做的三件事，缺一不可：
     *   ① PWM 立即归零（不要等 PID 慢慢收敛）；
     *   ② 积分清零（否则恢复链路瞬间会带着旧积分“抽”一下）；
     *   ③ 指令历史归零（否则加速度限幅会以为车还在高速）。 */
    Motor_StopAll();

    for (i = 0; i < (int)WHEEL_COUNT; i++) {
        PIDReset(&s_pid[i]);
        s_cmd_snapshot.duty[i] = 0;
        s_cmd_snapshot.active[i] = 0u;
    }
    g_motor_cmd = s_cmd_snapshot;

    s_vx_prev = 0.0f;
    s_vy_prev = 0.0f;
    s_omega_prev = 0.0f;
    s_safe_latched = 1u;
}

/*============================================================================*/
/* 主控制流程（闭环部分）                                                      */
/*============================================================================*/

/* 标定模式（MOTOR_TEST_MODE = 1）下速度环不工作，这些函数整体不参与编译，
 * 这样既不会出现“定义了但没用”的编译告警，也保证标定期间绝对不可能
 * 意外跑进闭环逻辑。 */
#if (MOTOR_TEST_MODE == 0)

/**
  * @brief 摇杆映射 + 加速度限幅，得到本轮的车体指令。（（l*映射为v*）
  * @param cmd   遥控帧
  * @param dt_s  采样周期（秒）
  * @param vx    输出：向右为正（mm/s）
  * @param vy    输出：向前为正（mm/s）
  * @param omega 输出：逆时针为正（rad/s）
  */
static void compute_body_cmd(const remote_in_t *cmd, float dt_s,
                             float *vx, float *vy, float *omega) {
    float n_ly;
    float n_lx;
    float n_rx;
    float vx_t;
    float vy_t;
    float om_t;
    float max_dv;
    float max_domega;

    n_ly = axis_normalized(cmd->ly);
    n_lx = axis_normalized(cmd->lx);
    n_rx = axis_normalized(cmd->rx);

    /* ly 正 = 前进（你和协议约定的），直接用；单位 mm/s。 */
    vy_t = n_ly * (MAX_VXY_MS * 1000.0f);
    /* lx 负 = 向左，而车体 y 轴向左为正，所以这里要取负号。 */
    vx_t = -n_lx * (MAX_VXY_MS * 1000.0f);
    /* rx 正 = 顺时针，而 ω 以逆时针为正，所以取负号。 */
    om_t = -n_rx * MAX_OMEGA_RADS;

    /* 摇杆推到底之前先把车体指令限幅到“不超轮速”的范围内，
     * 否则运动学输出必然饱和，会被等比缩放砍掉 ω，用户感觉“转不动”。 */
    {
        float mag = sqrtf(vx_t * vx_t + vy_t * vy_t);
        float max_xy = MAX_VXY_MS * 1000.0f;
        if (mag > max_xy && mag > 0.0f) {
            float k = max_xy / mag;
            vx_t *= k;
            vy_t *= k;
        }
    }

    /* 指令死区：非常小的指令直接归零，避免摇杆边缘抖动被放大成电机嗡鸣。 */
    if (fabsf(vy_t) < CMD_DEADBAND_MMPS) vy_t = 0.0f;
    if (fabsf(vx_t) < CMD_DEADBAND_MMPS) vx_t = 0.0f;
    if (fabsf(om_t) < (CMD_DEADBAND_MMPS / CHASSIS_K_MM)) om_t = 0.0f;

    /* ── 加速度限幅（指令斜坡）──
     * 直接原因：TB6612 单路连续 1.2 A，而堵转 6 A。指令从 0 跳到 0.9 m/s
     * 会让电流瞬间顶到限流/过温，并且麦轮会打滑（打滑时编码器测到的是
     * 轮子空转，闭环反而更错）。 */
    max_dv = MAX_ACC_MS2 * 1000.0f * dt_s;      /* mm/s 每周期允许的变化量 */
    max_domega = MAX_ALPHA_RADS2 * dt_s;        /* rad/s 每周期允许的变化量 */

    vx_t = clampf(vx_t, s_vx_prev - max_dv, s_vx_prev + max_dv);
    vy_t = clampf(vy_t, s_vy_prev - max_dv, s_vy_prev + max_dv);
    om_t = clampf(om_t, s_omega_prev - max_domega, s_omega_prev + max_domega);

    s_vx_prev = vx_t;
    s_vy_prev = vy_t;
    s_omega_prev = om_t;

    *vx = vx_t;
    *vy = vy_t;
    *omega = om_t;
}

/**
  * @brief 麦轮逆运动学 + 饱和等比缩放（在保证航向不变时避免超速）。
  * @param w 输入输出：四个轮子的目标线速度（mm/s），顺序 FL/FR/RL/RR
  */
static void mecanum_inverse(float vx, float vy, float omega, float w[WHEEL_COUNT]) {
    float k_omega = CHASSIS_K_MM * omega; /* mm/s：自转贡献到每个轮子的线速度 */
    float peak = 0.0f;
    int i;

    w[WHEEL_FL] = vx + vy + k_omega;
    w[WHEEL_FR] = vx - vy - k_omega;
    w[WHEEL_RL] = vx - vy + k_omega;
    w[WHEEL_RR] = vx + vy - k_omega;

    /* 找峰值，超过轮速上限就整体等比缩放（保持运动方向不变）。 */
    for (i = 0; i < (int)WHEEL_COUNT; i++) {
        float a = fabsf(w[i]);
        if (a > peak) peak = a;
    }
    if (peak > WHEEL_MAX_SPEED_MMPS) {
        float scale = WHEEL_MAX_SPEED_MMPS / peak;
        for (i = 0; i < (int)WHEEL_COUNT; i++) {
            w[i] *= scale;
        }
    }
}
// 如果满速转弯会导致v被砍，那么需要改进此部分代码，保证转弯时的直线速度
/**
  * @brief 单轮 PI（含前馈、低速冻结积分、死区补偿、限幅）。
  *
  * @param id      轮标识
  * @param ref_mmps 目标轮速（mm/s，该轮前进为正）
  * @param dt_s    采样周期（秒）
  * @return 实际写到 PWM 比较寄存器的带符号占空比
  */
static int32_t wheel_loop_step(WheelId_e id, float ref_mmps, float dt_s) {
    float raw_mmps;
    float meas_mmps;
    float duty_frac;
    int32_t duty_counts;
    float abs_ref;
    float abs_meas;

    /* 先做一阶低通再进 PID：编码器只有 44 counts/轮转，10 ms 窗口下
     * 量化噪声地板约 ±43 mm/s（见 app_config.h 第 7 节）。
     * 滤波器不能消除这个地板，但能阻止它被 Kp 直接放大成电机嗡鸣。
     * s_speed_lpf 同时也是遥测打印用的值，保证“看到的”就是“控制用的”。 */
    raw_mmps = Encode_GetSpeedMmps(&s_enc[id]);
    s_speed_lpf[id] = lpf1(s_speed_lpf[id], raw_mmps, VEL_FEEDBACK_LPF_RC_S, dt_s);
    meas_mmps = s_speed_lpf[id];

    /* 低速段保护：目标速度和反馈速度都很小的时候，反馈被量化噪声淹没
     * （1 个计数 / 10 ms = 43 mm/s 的噪声地板，见 app_config.h 第 7 节）。
     * 此时把**积分上限收紧到当前积分值**，等于冻结积分累积，
     * 只用 P 项和前馈——避免 I 项被噪声喂偏导致停止时整车抖动。 */
    abs_ref = fabsf(ref_mmps);
    abs_meas = fabsf(meas_mmps);
    if ((abs_ref < VEL_LOWSPEED_FREEZE_MMPS) && (abs_meas < VEL_LOWSPEED_FREEZE_MMPS)) {
        float i_now = fabsf(s_pid[id].Iout);

        s_pid[id].IntegralLimit = i_now;
        duty_frac = PIDCalculate(&s_pid[id], meas_mmps, ref_mmps, dt_s);
        s_pid[id].IntegralLimit = VEL_INTEGRAL_LIMIT;
    } else {
        duty_frac = PIDCalculate(&s_pid[id], meas_mmps, ref_mmps, dt_s);
    }

    /* 前馈：按“满占空比 ≈ 空载最高轮速”的斜率直接给出直流工作点，
     * PI 只负责修模型误差。这一项能显著减小低速段的稳态误差，
     * 因为低速段 I 项被冻结了。 */
    duty_frac += VEL_FF_GAIN * ref_mmps;

    duty_frac = clampf(duty_frac, -1.0f, 1.0f);
    duty_counts = (int32_t)(duty_frac * (float)MOTOR_PWM_FULL);

    Motor_SetDuty(id, duty_counts);

    /* 回读实际写入值，供遥测与限幅验证。 */
    return duty_counts;
}

#endif /* MOTOR_TEST_MODE == 0 */

#if (MOTOR_TEST_MODE != 0)
/**
  * @brief P1 开环标定模式：四个轮子吃固定占空比，用遥控 ly 调制大小与方向。
  *
  * 用法（详见 README 的最后一步）：
  *   1) 上电，把 ly 从 0 缓慢推到最大，观察四个轮子的**转向**与**编码器读数符号**；
  *   2) 逐个记录：某一轮正转时 SIGN_DIR/PWM/ENC 是否一致；
  *   3) 缓慢减小占空比，记录每个轮子“刚开始转”的占空比 → DEADBAND_*；
  *   4) 把结果填回 app_config.h，然后把 MOTOR_TEST_MODE 改成 0。
  */
static void run_open_loop_test(const remote_in_t *cmd) {
    float n_ly = axis_normalized(cmd->ly);
    int32_t duty = MOTOR_TEST_DUTY_BASE + (int32_t)(n_ly * (float)MOTOR_TEST_DUTY_SPAN);
    int i;

    for (i = 0; i < (int)WHEEL_COUNT; i++) {
        /* 标定要用**原始**占空比（绕过死区补偿），这样才能量出真实死区。 */
        Motor_SetRawDuty((WheelId_e)i, duty);
        s_cmd_snapshot.duty[i] = (int16_t)duty;
        s_cmd_snapshot.active[i] = (duty != 0) ? 1u : 0u;
    }
    g_motor_cmd = s_cmd_snapshot;
}
#endif /* MOTOR_TEST_MODE */

void VelCtl_Feed(const remote_in_t *cmd, float dt_s) {
    int i;

    if (cmd == NULL || dt_s <= 0.0f) {
        return;
    }

    /* ── 每 1 ms 累积编码器计数（Encode_Update 内部按窗口分频才做除法）── */
    for (i = 0; i < (int)WHEEL_COUNT; i++) {
        Encode_Update(&s_enc[i]);
    }

    /* ── 链路检查放在最前面，优先级最高 ──
     * 断蓝牙后协议层已经把数值置零，但 PID 里还有上一次的积分，
     * 如果不在这里硬性拦一道，车会带着残余速度继续动。 */
    if (cmd->online == 0u) {
        if (s_safe_latched == 0u) {
            VelCtl_SafeState();
        }
        return;
    }
    s_safe_latched = 0u;

#if (MOTOR_TEST_MODE != 0)
    /* 标定模式：不走闭环、不走运动学。 */
    run_open_loop_test(cmd);
#else
    {
        float vx;
        float vy;
        float omega;
        float w[WHEEL_COUNT];

        compute_body_cmd(cmd, dt_s, &vx, &vy, &omega);
        mecanum_inverse(vx, vy, omega, w);

        for (i = 0; i < (int)WHEEL_COUNT; i++) {
            int32_t duty;

            /* 目标轮速先钳到物理上限：即使上一级已经缩放过，
             * 这里再保险一次，防止任何路径漏算导致超速。 */
            w[i] = clampf(w[i], -WHEEL_MAX_SPEED_MMPS, WHEEL_MAX_SPEED_MMPS);

            duty = wheel_loop_step((WheelId_e)i, w[i], dt_s);
            s_cmd_snapshot.duty[i] = (int16_t)clampi32(duty, -32768, 32767);
            s_cmd_snapshot.active[i] = (duty != 0) ? 1u : 0u;
        }
        g_motor_cmd = s_cmd_snapshot;
    }
#endif /* MOTOR_TEST_MODE */
}

/*============================================================================*/
/* 遥测用只读接口                                                              */
/*============================================================================*/

float VelCtl_GetWheelSpeedMmps(WheelId_e id) {
    if ((int)id >= (int)WHEEL_COUNT) return 0.0f;
    /* 刻意返回**未滤波**的原始速度：遥测看到的必须就是编码器算出来的东西。
     * 标定模式下轮速不经过 wheel_loop_step()，如果这里返回滤波值就会永远是 0，
     * 也就没法用打印判断编码器方向了。闭环时控制内部用自己的滤波副本
     * （s_speed_lpf），与这里互不影响。 */
    return Encode_GetSpeedMmps(&s_enc[id]);
}

int32_t VelCtl_GetWheelDuty(WheelId_e id) {
    if ((int)id >= (int)WHEEL_COUNT) return 0;
    return (int32_t)s_cmd_snapshot.duty[id];
}
