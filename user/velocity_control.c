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

#if (MOTOR_TEST_MODE == 0)
/* 速度环补偿参数：不改遥控、运动学、编码器和驱动层。
 * kV 单位为占空比分数/(mm/s)，kS 为占空比分数。
 * 以下由现有最大轮速和死区估算，尚非负载实测值。
 * 后续可分别标定每轮正反转；令对应 kV/kS 为 0 可独立关闭该项。
 * kV = (u2-u1)/(v2-v1)，kS = u1-kV*v1（同方向稳态样本）。 */
typedef struct {
    float kv_positive;
    float kv_negative;
    float ks_positive;
    float ks_negative;
} wheel_ff_config_t;

#define VEL_FF_INITIAL_KS(deadband) ((float)(deadband) / (float)MOTOR_PWM_FULL)
#define VEL_FF_INITIAL_KV(deadband) ((1.0f - VEL_FF_INITIAL_KS(deadband)) / WHEEL_MAX_SPEED_MMPS)
#define VEL_FF_INITIAL(deadband) {VEL_FF_INITIAL_KV(deadband), VEL_FF_INITIAL_KV(deadband), \
                                  VEL_FF_INITIAL_KS(deadband), VEL_FF_INITIAL_KS(deadband)}
static const wheel_ff_config_t s_ff_cfg[WHEEL_COUNT] = {
    VEL_FF_INITIAL(MOTOR_DEADBAND_FL), VEL_FF_INITIAL(MOTOR_DEADBAND_FR),
    VEL_FF_INITIAL(MOTOR_DEADBAND_RL), VEL_FF_INITIAL(MOTOR_DEADBAND_RR)
};

#define VEL_FRICTION_BLEND_MMPS 60.0f /* 0..此轮速内以 smoothstep 平滑建立摩擦补偿 */
#define VEL_STOP_ENTER_MMPS 15.0f
#define VEL_STOP_EXIT_MMPS 30.0f
#define VEL_STOP_SETTLE_S 0.10f
static float s_stop_elapsed_s;
static uint8_t s_stop_requested;
static uint8_t s_stopped;
#endif

/** @brief 遥测用快照，不参与控制计算。 */
static motor_cmd_t s_cmd_snapshot;
motor_cmd_t g_motor_cmd;

/** @brief 上一周期发给该轮的目标速度（mm/s），仅供遥测显示用。 */
static float s_target_mmps[WHEEL_COUNT];

/* 注：这里曾经有 s_speed_lpf[] 和 s_meas_prev[] 两个中间数组，
 * 分别用于"下游二次低通"和"变化率限幅（剔除坏采样）"。**已全部删除**：
 *   · 速度滤波只在编码器模块内部做一级（user/control/encode.c），
 *     下游再滤一次只会白送相位滞后；
 *   · "拒绝坏采样"的限幅器在本工程踩过两次永久死锁的坑，
 *     现在改为在编码器内部**限幅**（钳到物理边界后照常送滤波器）。
 * 保留它们只会让人误以为控制路径上还有这两级处理。 */

/* 上一周期的车体指令（用于加速度限幅）。单位 mm/s、rad/s。 */
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

/* 注：这里曾经有一个 lpf1() 一阶低通，用于对编码器反馈做二次滤波。
 * 已经删除 —— 编码器模块（user/control/encode.c）内部做了**单周期测速 + 一级
 * 一阶低通**，反馈链上再叠一级滤波只会白送相位滞后、拖累 Kp。
 * 全工程的速度滤波只有一处，参数是 app_config.h 的 VEL_FEEDBACK_LPF_RC_S。 */

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

    /* 编码器绑定表。顺序与 WheelId_e 一致：FL, FR, RL, RR。
     * 低通 RC 直接由编码器模块内部维护（单周期测速 + 一级 LPF），
     * 下游 velocity 环节不再做任何速度滤波。 */
    Encode_Init(&s_enc[WHEEL_FL], &htim2, (int8_t)MOTOR_SIGN_ENC_FL, VEL_FEEDBACK_LPF_RC_S);
    Encode_Init(&s_enc[WHEEL_FR], &htim1, (int8_t)MOTOR_SIGN_ENC_FR, VEL_FEEDBACK_LPF_RC_S);
    Encode_Init(&s_enc[WHEEL_RL], &htim4, (int8_t)MOTOR_SIGN_ENC_RL, VEL_FEEDBACK_LPF_RC_S);
    Encode_Init(&s_enc[WHEEL_RR], &htim3, (int8_t)MOTOR_SIGN_ENC_RR, VEL_FEEDBACK_LPF_RC_S);

    /* 先读一次初值再启动，避免把上电前的残留计数当成运动。 */
    for (i = 0; i < (int)WHEEL_COUNT; i++) {
        Encode_Start(&s_enc[i]);
        s_target_mmps[i] = 0.0f;
    }

    for (i = 0; i < (int)WHEEL_COUNT; i++) {
        PIDInit(&s_pid[i], &s_pid_cfg[i]);
    }

    s_vx_prev = 0.0f;
    s_vy_prev = 0.0f;
    s_omega_prev = 0.0f;
    s_safe_latched = 1u;

#if (MOTOR_TEST_MODE == 0)
    s_stop_elapsed_s = 0.0f;
    s_stop_requested = 0u;
    s_stopped = 0u;
#endif

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
#if (MOTOR_TEST_MODE == 0)
    s_stop_elapsed_s = 0.0f;
    s_stop_requested = 0u;
    s_stopped = 0u;
#endif
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

    /* ── ly → vy（前后）──────────────────────────────────────────────────
     *
     * ★ 这里对 ly 取负，是本工程实测标定的结果，不要改回正号 ★
     *
     * 协议层对 ly 的定义是"**正 = 摇杆向下**"（见 app_types.h 里 remote_in_t
     * 的字段注释），而遥控器习惯里"左摇杆向下"就是后退。所以：
     *
     *     ly > 0  →  摇杆向下  →  应当后退
     *     ly < 0  →  摇杆向上  →  应当前进
     *
     * 而本工程的内部约定是"**vy > 0 = 车体前进**"（mecanum_inverse 里
     * w[FL]=vx+vy、w[RR]=vx+vy，正向 vy 让左前/右后轮一起往前推）。
     * 所以要取负号把两者对齐：
     *
     *     vy_t = -n_ly * MAX_VXY
     *
     * 实测验证（改之前）：ly=+256 时 w[RL] = -vy = -450（ref 为负）且车实际后退；
     * 改之后 ly=+256 → vy = -450 → w[RL] = -vy = **+450**（ref 为正）且车前进。
     *
     * ⚠ 只影响前后分量。vx（左右）与 ω（自旋）的公式一个字都没动，
     *   所以左右方向和自旋方向保持标定后的正确状态；
     *   PID 反馈方向、编码器 MOTOR_SIGN_* 也完全不受影响。 */
    vy_t = -n_ly * (MAX_VXY_MS * 1000.0f);
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
    
    w[WHEEL_FL] = vx + vy - k_omega;
    w[WHEEL_FR] = vx - vy + k_omega;
    w[WHEEL_RL] = vx - vy - k_omega;
    w[WHEEL_RR] = vx + vy + k_omega;

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
/* 速度前馈 + 平滑摩擦前馈，方向取自目标轮速，不跟随反馈噪声翻转。
 * x*x*(3-2*x) 在零点和混合区边界的斜率均为零，没有固定 PWM 跳变。
 * 最终仍用原始 PWM 接口，避免与驱动层旧死区映射重复补偿。 */
static float wheel_feedforward(WheelId_e id, float ref_mmps) {
    const wheel_ff_config_t *cfg = &s_ff_cfg[id];
    float speed = fabsf(ref_mmps);
    float x = clampf(speed / VEL_FRICTION_BLEND_MMPS, 0.0f, 1.0f);
    float blend = x * x * (3.0f - 2.0f * x);
    float kv = (ref_mmps >= 0.0f) ? cfg->kv_positive : cfg->kv_negative;
    float ks = (ref_mmps >= 0.0f) ? cfg->ks_positive : cfg->ks_negative;
    float output = kv * speed + ks * blend;
    return (ref_mmps >= 0.0f) ? output : -output;
}

/* 只在整车指令回中且斜坡目标全部到零后进入停车流程。
 * 单轮目标为零的组合运动不触发。停车请求建立时清旧积分，减速期间
 * 保留 P 制动但冻结积分；低速持续 100 ms 后归零输出。外力推动车辆
 * 超过退出阈值时恢复速度环。这是零输出静止态，不是坡道位置保持。 */
static uint8_t update_stop_state(const remote_in_t *cmd,
                                 const float w[WHEEL_COUNT], float dt_s) {
    uint8_t requested = (axis_normalized(cmd->lx) == 0.0f &&
                         axis_normalized(cmd->ly) == 0.0f &&
                         axis_normalized(cmd->rx) == 0.0f);
    float peak_speed = 0.0f;
    int i;
    for (i = 0; i < (int)WHEEL_COUNT; i++) {
        if (w[i] != 0.0f) requested = 0u;
        float speed = fabsf(Encode_GetSpeedMmps(&s_enc[i]));
        if (speed > peak_speed) peak_speed = speed;
    }
    if (requested == 0u) {
        s_stop_requested = 0u;
        s_stopped = 0u;
        s_stop_elapsed_s = 0.0f;
        return 0u;
    }
    if (s_stop_requested == 0u) {
        for (i = 0; i < (int)WHEEL_COUNT; i++) PIDReset(&s_pid[i]);
        s_stop_requested = 1u;
    }
    if (s_stopped != 0u && peak_speed > VEL_STOP_EXIT_MMPS) {
        s_stopped = 0u;
        s_stop_elapsed_s = 0.0f;
    }
    if (s_stopped == 0u) {
        if (peak_speed <= VEL_STOP_ENTER_MMPS) {
            s_stop_elapsed_s += dt_s;
            if (s_stop_elapsed_s >= VEL_STOP_SETTLE_S) s_stopped = 1u;
        } else {
            s_stop_elapsed_s = 0.0f;
        }
    }
    return s_stopped;
}

/**
  * @brief 单轮 PI + 速度前馈 + 平滑摩擦补偿。
  * 前馈写入 PIDInstance，由控制器统一处理总输出限幅和抗积分饱和。
  * 输出直接交给 Motor_SetRawDuty，不使用驱动层旧死区映射。
  * 停车请求期间冻结积分；正常运动（含单轮零目标）继续闭环。
  */
static int32_t wheel_loop_step(WheelId_e id, float ref_mmps, float dt_s) {
    float duty_frac;
    int32_t duty_counts;
    uint8_t use_fb;

    /* ── 该轮是否接入了编码器反馈 ────────────────────────────────────────
     * ★ 四轮项目必须显式判断，不能假设"四个编码器都在" ★
     * 没接编码器时 Encode_GetSpeedMmps() 恒返回 0，若照常喂给 PID：
     *     Err = ref − 0 = ref  →  Pout 远超 MaxOut  →  Output 被钳到 ±1.0
     *     →  duty = ±8400 满占空比，每周期如此 → 电机颤抖而不转。
     * 这是四轮项目特有的状态（参考项目是单电机单编码器，不存在这一步）。
     * 开关在 app_config.h 第 7-b 节。 */
    switch (id) {
        case WHEEL_FL: use_fb = (uint8_t)VEL_USE_FB_FL; break;
        case WHEEL_FR: use_fb = (uint8_t)VEL_USE_FB_FR; break;
        case WHEEL_RL: use_fb = (uint8_t)VEL_USE_FB_RL; break;
        case WHEEL_RR: use_fb = (uint8_t)VEL_USE_FB_RR; break;
        default:       use_fb = 0u;                    break;
    }

    /* 遥测用：本周期发给该轮的目标值 */
    s_target_mmps[id] = ref_mmps;

    if (use_fb == 0u) {
        /* 没有编码器反馈：不闭环、也不给开环输出，避免乱转干扰调试。
         * 要让这一路动起来，先把编码器接上并把对应宏改成 1。 */
        PIDReset(&s_pid[id]);
        Motor_SetRawDuty(id, 0);
        return 0;
    }

    /* ★ 整个速度环★
     * 参数顺序是 (pid, measure, ref, dt_s) —— "实测值、目标值"的顺序，
         * 量纲：meas 与 ref 同为 mm/s，Kp/Ki 的单位是"占空比分数 / (mm/s)"。 */
    s_pid[id].FeedForward = wheel_feedforward(id, ref_mmps);
    if (s_stop_requested != 0u) PID_FreezeIntegral(&s_pid[id]);
    duty_frac = PIDCalculate(&s_pid[id], Encode_GetSpeedMmps(&s_enc[id]), ref_mmps, dt_s);

    duty_frac = clampf(duty_frac, -1.0f, 1.0f);
    duty_counts = (int32_t)(duty_frac * (float)MOTOR_PWM_FULL);

    /* 前馈和摩擦补偿已计入 PID 总输出及抗饱和，不能再叠加旧死区映射。 */
    Motor_SetRawDuty(id, duty_counts);

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

    /* ── 更新编码器：读计数增量 + 单周期测速 ──────────────────────────────
     * ⚠ 本函数（以及这里的 Encode_Update）是**每 VEL_LOOP_PERIOD_MS = 10 ms**
     *   才被 Chassis 任务调用一次，不是 1 ms！
     * 现在就是最简单的：**谁调用，谁提供真实周期**，编码器内部再校验一次区间。 */
    for (i = 0; i < (int)WHEEL_COUNT; i++) {
        Encode_Update(&s_enc[i], dt_s);
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

    /* 挡位互锁：CONTROL 帧的 S1/S2 位于 remote_in_t.buttons 的 bit4/bit5。
     * 两个开关都关闭才是底盘挡；机械臂粗调、精调以及无效组合都必须让
     * 底盘立即停止，避免机械臂操作时同一组摇杆同时驱动车体。 */
    if ((cmd->buttons & ((1UL << 4) | (1UL << 5))) != 0U) {
        VelCtl_SafeState();
        return;
    }

#if (MOTOR_TEST_MODE != 0)
    /* 标定模式：不走闭环、不走运动学。 */
    run_open_loop_test(cmd);
#else
    {
        float vx;
        float vy;
        float omega;
        float w[WHEEL_COUNT];
        uint8_t stopped;

        compute_body_cmd(cmd, dt_s, &vx, &vy, &omega);
        mecanum_inverse(vx, vy, omega, w);
        stopped = update_stop_state(cmd, w, dt_s);

        for (i = 0; i < (int)WHEEL_COUNT; i++) {
            int32_t duty;

            /* 目标轮速先钳到物理上限：即使上一级已经缩放过，
             * 这里再保险一次，防止任何路径漏算导致超速。 */
            w[i] = clampf(w[i], -WHEEL_MAX_SPEED_MMPS, WHEEL_MAX_SPEED_MMPS);

            if (stopped != 0u) {
                PIDReset(&s_pid[i]);
                s_target_mmps[i] = w[i];
                Motor_SetRawDuty((WheelId_e)i, 0);
                duty = 0;
            } else {
                duty = wheel_loop_step((WheelId_e)i, w[i], dt_s);
            }
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

float VelCtl_GetWheelTargetMmps(WheelId_e id) {
    if ((int)id >= (int)WHEEL_COUNT) return 0.0f;
    /* 专门存一份目标值，避免去读 PIDInstance 里那个名字是反的 Ref 字段
     * （见 controller.h 的警告注释）。 */
    return s_target_mmps[id];
}

/*============================================================================*/
/* 单例遥测接口（Ozone 专用，★调试时请优先用这几个★）                          */
/*============================================================================*/
/*
 * ★ 为什么需要这几个函数 ★
 *
 * Ozone 在 watch 里写 `s_enc[2].xxx` 时，会自己按**它缓存的数组元素步长**
 * 算地址，而字段偏移则按**它缓存的结构体布局**算。本工程改过 Encode_t 的
 * 布局（删掉 window_count/speed_counts_s/elapsed_ms，加上 speed_mmps 等），
 * 结果调试器把新字段偏移套在旧步长上，算出来的地址整体偏了 4 字节 ——
 * 实测到过 s_enc[2] 显示成 0x2000D090（真值 0x2000D08C），于是看到
 * "speed_mmps = -194864 mm/s" 这种物理不可能的读数，把排查带偏。
 *
 * 下面这几个函数**参数是编译期常量、返回标量**，Ozone 直接写函数名即可求值，
 * 完全不经过数组下标计算，因此不会受调试信息缓存的影响。
 *
 * 配合 chassiscontrol.c 里已经存在的串口打印，两条路都能拿到真值。
 */

/** @brief FL 轮实测轮速 [mm/s]（前进为正，已滤波）。 */
float VelCtl_TelemSpeedFL(void) { return Encode_GetSpeedMmps(&s_enc[WHEEL_FL]); }
/** @brief FR 轮实测轮速 [mm/s]。 */
float VelCtl_TelemSpeedFR(void) { return Encode_GetSpeedMmps(&s_enc[WHEEL_FR]); }
/*============================================================================*/
/* 遥测接口（只读，调试用，不参与控制）                                        */
/*============================================================================*/
/*
 * ★ 为什么要有这组函数 ★
 *
 * Ozone 在 watch 里写 `s_enc[2].speed_mmps` 时，会按**它自己缓存的**数组元素
 * 步长算地址、按**它自己缓存的**结构体布局算字段偏移。本工程反复改过 Encode_t
 * 的布局，一旦调试信息与固件不同步，就会算出错位地址、显示物理不可能的读数。
 *
 * 下面这组函数**参数是编译期常量、返回标量**，Ozone 直接写函数名求值，
 * 不经过数组下标计算，不受调试信息缓存影响。
 *
 * ★ 函数 ↔ 变量 对照（M3 = RL = 索引 2）
 *   VelCtl_TelemTargetRL() ↔ s_target_mmps[2]      （目标，= 运动学输出）
 *   VelCtl_TelemSpeedRL()  ↔ s_enc[2].speed_mmps   （实测，= PID 的 measure）
 *   VelCtl_TelemDeltaRL()  ↔ s_enc[2].last_delta   （本周期计数增量）
 *   VelCtl_TelemCountRL()  ↔ s_enc[2].total_count  （累计位置）
 *   VelCtl_TelemDutyRL()   ↔ s_cmd_snapshot.duty[2]（写入的带符号占空比）
 */

/**
  * @brief 车体前进速度指令 vy [mm/s]（**>0 = 车体前进**）。
  *
  * ⚠ 注意符号关系：因为协议里 ly 是"正 = 摇杆向下"，所以
  *     ly > 0  →  vy **< 0**（摇杆向下 = 后退）
  *     ly < 0  →  vy **> 0**（摇杆向上 = 前进）
  *   判断"是前进还是后退"请看 vy 的符号，或直接看 s_target_mmps 与实车动作，
  *   不要拿 ly 的符号去对。
  */
float VelCtl_TelemCmdVy(void) { return s_vy_prev; }

/** @brief RL 轮（M3）目标轮速 [mm/s]，正数 = 该轮物理前进。 */
float VelCtl_TelemTargetRL(void) { return s_target_mmps[WHEEL_RL]; }

/** @brief RL 轮（M3）实测轮速 [mm/s]，与目标值同量纲、同符号约定。 */
float VelCtl_TelemSpeedRL(void) { return Encode_GetSpeedMmps(&s_enc[WHEEL_RL]); }

/** @brief RL 轮（M3）本周期有符号计数增量（满速 ±87）。 */
int32_t VelCtl_TelemDeltaRL(void) { return Encode_GetLastDelta(&s_enc[WHEEL_RL]); }

/** @brief RL 轮（M3）累计位置计数，用来判断"真在转"还是"数错了"。 */
int32_t VelCtl_TelemCountRL(void) { return Encode_GetCount(&s_enc[WHEEL_RL]); }

/** @brief RL 轮（M3）当前带符号占空比（PWM 计数，满量程 8400）。 */
int32_t VelCtl_TelemDutyRL(void) { return (int32_t)s_cmd_snapshot.duty[WHEEL_RL]; }
