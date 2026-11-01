#include "grab.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "tim.h"
#include "app_config.h"
#include "chassiscontrol.h"
#include "stepper.h"

/* PWM 已由 CubeMX 配置为 1 MHz 计数、20 ms 周期。每个 CCR 单位为 1 us。 */
#define SERVO_PWM_0_US              500U
#define SERVO_PWM_180_US            2500U

/* S1/S2 在 CONTROL 帧中被 remote.c 放入 buttons 的 bit4/bit5。 */
#define GRAB_SWITCH_S1_MASK         (1UL << 4)
#define GRAB_SWITCH_S2_MASK         (1UL << 5)

#define GRAB_AXIS_DEADBAND          40
#define GRAB_AXIS_RELEASE_ZONE      25
#define GRAB_AXIS_DOMINANT_GAP      20

#define GRAB_COARSE_SPEED_DEG_S     30.0f
#define GRAB_FINE_SPEED_DEG_S       GRAB_COARSE_SPEED_DEG_S
#define GRAB_ROTATE_SPEED_DEG_S     65.0f
#define GRAB_GRIPPER_SPEED_DEG_S    GRAB_COARSE_SPEED_DEG_S

/* 每个舵机独立方向宏：1 = 逻辑角度增加对应舵机电气角度增加，
 * -1 = 逻辑角度增加对应舵机电气角度减小。 */
#define GRAB_J2_DIR                  (1)
#define GRAB_J3_DIR                  (1)
#define GRAB_J4_DIR                  (1)
#define GRAB_J5_DIR                 (-1)
#define GRAB_J6_DIR                  (1)

/* 每个关节的逻辑软件限位，刻意比机械可动范围窄。 */
typedef struct {
    TIM_HandleTypeDef *timer;
    uint32_t channel;
    float min_angle;
    float max_angle;
    float target_angle;
    float speed_deg_s;
    int8_t direction;
} GrabServo_t;

static GrabServo_t s_servo[GRAB_SERVO_COUNT] = {
    /* J2：0 水平，90 竖直；超过 90 后继续下降 */
    {&htim10, TIM_CHANNEL_1,   8.0f, 175.0f,  90.0f, GRAB_COARSE_SPEED_DEG_S, GRAB_J2_DIR},
    /* J3：实物约 15 度限位，软件从 20 度开始 */
    {&htim11, TIM_CHANNEL_1,  20.0f, 175.0f,  90.0f, GRAB_COARSE_SPEED_DEG_S, GRAB_J3_DIR},
    /* J4：硬件装反，先按角度增加为上升 */
    {&htim13, TIM_CHANNEL_1,  0.0f, 175.0f,  90.0f, GRAB_FINE_SPEED_DEG_S,   GRAB_J4_DIR},
    /* J5：DIR=1 时角度增加为逆时针，逻辑正方向要求顺时针，因此取反 */
    {&htim14, TIM_CHANNEL_1,   0.0f, 180.0f,  20.0f, GRAB_ROTATE_SPEED_DEG_S, GRAB_J5_DIR},
    /* J6：100 开、139 闭，软件再缩窄一点 */
    {&htim12, TIM_CHANNEL_2, 102.0f, 137.0f, 120.0f, GRAB_GRIPPER_SPEED_DEG_S, GRAB_J6_DIR}
};

static GrabMode_e s_mode = GRAB_MODE_CHASSIS;
/* 左摇杆主导轴：-1=未锁定，0=LX，1=LY。 */
static int8_t s_left_axis_lock = -1;

static uint32_t grab_angle_to_pulse(const GrabServo_t *servo);

/* Ozone 调试专用：直接 watch 这些变量，不需要调用带形参的函数。 */
volatile float g_grab_j2_target_angle_deg;
volatile float g_grab_j3_target_angle_deg;
volatile float g_grab_j4_target_angle_deg;
volatile float g_grab_j5_target_angle_deg;
volatile float g_grab_j6_target_angle_deg;
volatile GrabServoWatch_t g_grab_j2_watch;
volatile GrabServoWatch_t g_grab_j3_watch;
volatile GrabServoWatch_t g_grab_j4_watch;
volatile GrabServoWatch_t g_grab_j5_watch;
volatile GrabServoWatch_t g_grab_j6_watch;
volatile GrabInputWatch_t g_grab_input_watch;

static void grab_publish_debug_targets(void)
{
    g_grab_j2_target_angle_deg = s_servo[GRAB_J2_BIG_ARM].target_angle;
    g_grab_j3_target_angle_deg = s_servo[GRAB_J3_FOREARM].target_angle;
    g_grab_j4_target_angle_deg = s_servo[GRAB_J4_PITCH].target_angle;
    g_grab_j5_target_angle_deg = s_servo[GRAB_J5_ROTATE].target_angle;
    g_grab_j6_target_angle_deg = s_servo[GRAB_J6_GRIPPER].target_angle;

    g_grab_j2_watch.target_angle_deg = s_servo[GRAB_J2_BIG_ARM].target_angle;
    g_grab_j2_watch.pulse_us = grab_angle_to_pulse(&s_servo[GRAB_J2_BIG_ARM]);
    g_grab_j3_watch.target_angle_deg = s_servo[GRAB_J3_FOREARM].target_angle;
    g_grab_j3_watch.pulse_us = grab_angle_to_pulse(&s_servo[GRAB_J3_FOREARM]);
    g_grab_j4_watch.target_angle_deg = s_servo[GRAB_J4_PITCH].target_angle;
    g_grab_j4_watch.pulse_us = grab_angle_to_pulse(&s_servo[GRAB_J4_PITCH]);
    g_grab_j5_watch.target_angle_deg = s_servo[GRAB_J5_ROTATE].target_angle;
    g_grab_j5_watch.pulse_us = grab_angle_to_pulse(&s_servo[GRAB_J5_ROTATE]);
    g_grab_j6_watch.target_angle_deg = s_servo[GRAB_J6_GRIPPER].target_angle;
    g_grab_j6_watch.pulse_us = grab_angle_to_pulse(&s_servo[GRAB_J6_GRIPPER]);
}

static void grab_publish_stepper_input(const remote_in_t *cmd,
                                       int8_t selected_lx,
                                       int8_t selected_ly)
{
    if (cmd == NULL) {
        g_stepper_debug_watch.lx = 0;
        g_stepper_debug_watch.ly = 0;
        g_stepper_debug_watch.rx = 0;
        g_stepper_debug_watch.ry = 0;
        g_stepper_debug_watch.buttons = 0U;
        g_stepper_debug_watch.online = 0U;
    } else {
        g_stepper_debug_watch.lx = cmd->lx;
        g_stepper_debug_watch.ly = cmd->ly;
        g_stepper_debug_watch.rx = cmd->rx;
        g_stepper_debug_watch.ry = cmd->ry;
        g_stepper_debug_watch.buttons = cmd->buttons;
        g_stepper_debug_watch.online = cmd->online;
    }
    g_stepper_debug_watch.grab_mode = (uint8_t)s_mode;
    g_stepper_debug_watch.axis_lock = s_left_axis_lock;
    g_stepper_debug_watch.selected_lx = selected_lx;
    g_stepper_debug_watch.selected_ly = selected_ly;
}

static float grab_clampf(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static int8_t grab_axis_sign(int16_t axis)
{
    if (axis > GRAB_AXIS_DEADBAND) return 1;
    if (axis < -GRAB_AXIS_DEADBAND) return -1;
    return 0;
}

/**
 * @brief 左摇杆单轴选择。
 *
 * 机械臂的 LX/LY 分别对应两个不同关节。操作者推动一个方向时，另一
 * 轴通常会有少量串扰；这里选定主导轴并锁定，直到两个轴都回中，避免
 * 两个舵机同时动作或在两个轴之间抖动切换。
 */
static void grab_select_left_axis(int16_t lx, int16_t ly,
                                  int8_t *out_lx, int8_t *out_ly)
{
    int32_t ax = (lx >= 0) ? lx : -(int32_t)lx;
    int32_t ay = (ly >= 0) ? ly : -(int32_t)ly;

    *out_lx = 0;
    *out_ly = 0;

    if (s_left_axis_lock == 0) {
        if (ax <= GRAB_AXIS_RELEASE_ZONE && ay <= GRAB_AXIS_RELEASE_ZONE) {
            s_left_axis_lock = -1;
        } else {
            *out_lx = grab_axis_sign(lx);
            return;
        }
    } else if (s_left_axis_lock == 1) {
        if (ax <= GRAB_AXIS_RELEASE_ZONE && ay <= GRAB_AXIS_RELEASE_ZONE) {
            s_left_axis_lock = -1;
        } else {
            *out_ly = grab_axis_sign(ly);
            return;
        }
    }

    if (ax < GRAB_AXIS_DEADBAND && ay < GRAB_AXIS_DEADBAND) return;

    if (ax >= ay + GRAB_AXIS_DOMINANT_GAP) {
        s_left_axis_lock = 0;
        *out_lx = grab_axis_sign(lx);
    } else if (ay >= ax + GRAB_AXIS_DOMINANT_GAP) {
        s_left_axis_lock = 1;
        *out_ly = grab_axis_sign(ly);
    }
    /* 两轴幅值过于接近时保持输出为零，等操作者给出明确方向。 */
}

static void grab_update_mode(uint32_t buttons)
{
    uint8_t s1 = ((buttons & GRAB_SWITCH_S1_MASK) != 0U) ? 1U : 0U;
    uint8_t s2 = ((buttons & GRAB_SWITCH_S2_MASK) != 0U) ? 1U : 0U;

    GrabMode_e new_mode;

    if (!s1 && !s2) new_mode = GRAB_MODE_CHASSIS;
    else if (s1 && !s2) new_mode = GRAB_MODE_COARSE;
    else if (!s1 && s2) new_mode = GRAB_MODE_FINE;
    else new_mode = GRAB_MODE_SAFE;

    if (new_mode != s_mode) {
        /* 切换挡位后必须重新判定主导轴，避免旧挡位的锁定状态带入新挡位。 */
        s_left_axis_lock = -1;
    }
    s_mode = new_mode;
}

static uint32_t grab_angle_to_pulse(const GrabServo_t *servo)
{
    float electrical_angle;
    float pulse;

    electrical_angle = (servo->direction >= 0)
                     ? servo->target_angle
                     : (180.0f - servo->target_angle);
    electrical_angle = grab_clampf(electrical_angle, 0.0f, 180.0f);
    pulse = (float)SERVO_PWM_0_US
          + electrical_angle * (float)(SERVO_PWM_180_US - SERVO_PWM_0_US) / 180.0f;
    return (uint32_t)(pulse + 0.5f);
}

static void grab_write_servo(GrabServo_t *servo)
{
    __HAL_TIM_SET_COMPARE(servo->timer, servo->channel, grab_angle_to_pulse(servo));
}

static void grab_move_servo(GrabServo_t *servo, int8_t command_sign, float dt_s)
{
    if (command_sign == 0 || dt_s <= 0.0f) return;
    servo->target_angle = grab_clampf(
        servo->target_angle + (float)command_sign * servo->speed_deg_s * dt_s,
        servo->min_angle,
        servo->max_angle);
    grab_write_servo(servo);
}

void Grab_Init(void)
{
    uint32_t i;

    (void)HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    Stepper_Init();

    for (i = 0U; i < GRAB_SERVO_COUNT; ++i) {
        grab_write_servo(&s_servo[i]);
    }
    grab_publish_debug_targets();
}

void Grab_Feed(const remote_in_t *cmd, float dt_s)
{
    int8_t left_x_sign;
    int8_t left_y_sign;

    if (cmd == NULL || dt_s <= 0.0f) {
        g_grab_input_watch.online = 0U;
        g_grab_input_watch.mode = (uint8_t)GRAB_MODE_SAFE;
        g_grab_input_watch.lx = 0;
        g_grab_input_watch.ly = 0;
        g_grab_input_watch.selected_lx = 0;
        g_grab_input_watch.selected_ly = 0;
        g_grab_input_watch.axis_lock = -1;
        grab_publish_stepper_input(NULL, 0, 0);
        Stepper_Feed(0, dt_s);
        return;
    }

    g_grab_input_watch.online = cmd->online;
    g_grab_input_watch.lx = cmd->lx;
    g_grab_input_watch.ly = cmd->ly;

    if (cmd->online == 0U) {
        s_mode = GRAB_MODE_SAFE;
        s_left_axis_lock = -1;
        Stepper_Feed(0, dt_s);
        g_grab_input_watch.mode = (uint8_t)s_mode;
        g_grab_input_watch.selected_lx = 0;
        g_grab_input_watch.selected_ly = 0;
        g_grab_input_watch.axis_lock = s_left_axis_lock;
        grab_publish_stepper_input(cmd, 0, 0);
        return;
    }

    grab_update_mode(cmd->buttons);
    grab_select_left_axis(cmd->lx, cmd->ly, &left_x_sign, &left_y_sign);
    g_grab_input_watch.mode = (uint8_t)s_mode;
    g_grab_input_watch.selected_lx = left_x_sign;
    g_grab_input_watch.selected_ly = left_y_sign;
    g_grab_input_watch.axis_lock = s_left_axis_lock;
    grab_publish_stepper_input(cmd, left_x_sign, left_y_sign);

    if (s_mode == GRAB_MODE_COARSE) {
        /* LY 正值：大臂目标角增加；RY 正值：小臂目标角增加。 */
        grab_move_servo(&s_servo[GRAB_J2_BIG_ARM], left_y_sign, dt_s);
        grab_move_servo(&s_servo[GRAB_J3_FOREARM], grab_axis_sign(cmd->ry), dt_s);
        /* LX：J1 步进旋转电机，正负方向由 DIR 宏/接线标定。 */
        Stepper_Feed(left_x_sign, dt_s);
    } else if (s_mode == GRAB_MODE_FINE) {
        Stepper_Feed(0, dt_s);
        /* LY：俯仰；LX：旋转臂逻辑方向，正值为顺时针；RX：抓夹。 */
        grab_move_servo(&s_servo[GRAB_J4_PITCH], left_y_sign, dt_s);
        grab_move_servo(&s_servo[GRAB_J5_ROTATE], left_x_sign, dt_s);
        grab_move_servo(&s_servo[GRAB_J6_GRIPPER], grab_axis_sign(cmd->rx), dt_s);
    } else {
        /* 底盘挡和安全挡都必须停止 J1，避免切挡后沿用上一帧指令。 */
        Stepper_Feed(0, dt_s);
    }

    grab_publish_debug_targets();
}

GrabMode_e Grab_GetMode(void)
{
    return s_mode;
}

float Grab_GetTargetAngle(GrabServoId_e id)
{
    if ((uint32_t)id >= GRAB_SERVO_COUNT) return 0.0f;
    return s_servo[id].target_angle;
}

void StartGrabTask(void *argument)
{
    remote_in_t command;

    (void)argument;
    Grab_Init();
    (void)memset(&command, 0, sizeof(command));

    for (;;) {
        Chassis_TakeLatest(&command);
        Grab_Feed(&command, 0.010f);
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}
