/** @file motor_driver.c
 * @brief 底盘四路 TB6612FNG 通道的底层驱动实现。
 */

#include "motor_driver.h"

#include <stddef.h>

#include "main.h"

#include "app_config.h"
#include "app_types.h"

/* CubeMX 在 Core/Src/tim.c 生成的 PWM 定时器句柄。 */
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;

/* 每轮硬件绑定表 */

typedef struct {
    TIM_HandleTypeDef *htim;  /**< PWM 定时器 */
    uint32_t channel;         /**< PWM 通道 */
    GPIO_TypeDef *dir_a_port; /**< TB6612 的 AIN1 */
    uint16_t dir_a_pin;
    GPIO_TypeDef *dir_b_port; /**< TB6612 的 AIN2 */
    uint16_t dir_b_pin;
    int8_t sign_dir;   /**< +1/-1：该轮“物理前进”时 AIN1/AIN2 的极性 */
    int8_t sign_pwm;   /**< +1/-1：PWM 计数符号与物理前向的对应 */
    uint32_t deadband; /**< 启动死区（PWM 计数） */
} motor_hw_t;

/* FL: TIM9_CH1 / PE1、PE2；FR: TIM9_CH2 / PE3、PE4。
 * RL: TIM8_CH3 / PG3、PG4；RR: TIM8_CH4 / PG5、PG6。
 * 数组顺序：左前、右前、左后、右后。 */
static const motor_hw_t s_motor_hw[WHEEL_COUNT] = {
    /* WHEEL_FL —— M1：TIM9_CH1(PE5)，DIR = PE1/PE2 */
    {&htim9, TIM_CHANNEL_1, GPIOE, M1_DIR1_Pin, GPIOE, M1_DIR2_Pin,
     MOTOR_SIGN_DIR_FL, MOTOR_SIGN_PWM_FL, MOTOR_DEADBAND_FL},
    /* WHEEL_FR —— M2：TIM9_CH2(PE6)，DIR = PE3/PE4 */
    {&htim9, TIM_CHANNEL_2, GPIOE, M2_DIR1_Pin, GPIOE, M2_DIR2_Pin,
     MOTOR_SIGN_DIR_FR, MOTOR_SIGN_PWM_FR, MOTOR_DEADBAND_FR},
    /* WHEEL_RL —— M3：TIM8_CH3(PC8)，DIR = PG3/PG4 */
    {&htim8, TIM_CHANNEL_3, GPIOG, M3_DIR1_Pin, GPIOG, M3_DIR2_Pin,
     MOTOR_SIGN_DIR_RL, MOTOR_SIGN_PWM_RL, MOTOR_DEADBAND_RL},
    /* WHEEL_RR —— M4：TIM8_CH4(PC9)，DIR = PG5/PG6 */
    {&htim8, TIM_CHANNEL_4, GPIOG, M4_DIR1_Pin, GPIOG, M4_DIR2_Pin,
     MOTOR_SIGN_DIR_RR, MOTOR_SIGN_PWM_RR, MOTOR_DEADBAND_RR},
};

/** @brief 当前实际写到 PWM 比较寄存器的值（带符号），供遥测观察。 */
static int32_t s_applied_duty[WHEEL_COUNT];

/* 内部工具 */

/*
  * @brief 按带符号占空比设置 TB6612 的方向脚。
  *
  * forward > 0 表示该轮物理前进。AIN1/AIN2 的具体电平由 sign_dir 决定：
  *   sign_dir = +1 → 前进时 AIN1=0, AIN2=1
  *   sign_dir = -1 → 前进时 AIN1=1, AIN2=0
  * 占空比为 0 时两脚都置 0（滑行），避免零占空比下堵转发热。
 */
static void set_direction(const motor_hw_t *hw, int forward) {
    int a;
    int b;

    if (forward > 0) {
        a = (hw->sign_dir > 0) ? 0 : 1;
        b = (hw->sign_dir > 0) ? 1 : 0;
    } else if (forward < 0) {
        a = (hw->sign_dir > 0) ? 1 : 0;
        b = (hw->sign_dir > 0) ? 0 : 1;
    } else {
        a = 0; /* 滑行 */
        b = 0;
    }

    HAL_GPIO_WritePin(hw->dir_a_port, hw->dir_a_pin,
                      (a != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hw->dir_b_port, hw->dir_b_pin,
                      (b != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/** @brief 把带符号占空比写入 PWM 比较寄存器（绝对值已钳到满量程）。 */
static void write_pwm(const motor_hw_t *hw, int32_t duty) {
    int32_t magnitude = (duty < 0) ? -duty : duty;

    if (magnitude > (int32_t)MOTOR_PWM_FULL) {
        magnitude = (int32_t)MOTOR_PWM_FULL;
    }
    __HAL_TIM_SET_COMPARE(hw->htim, hw->channel, (uint32_t)magnitude);
}

/* 对外接口 */

/**
 * @brief 死区补偿的“直通带”阈值（占空比计数）。
 */
#define MOTOR_COMP_LINEAR_BAND 200

int32_t Motor_ApplyDeadbandComp(int32_t desired, uint32_t deadband) {
    int32_t sign;
    int32_t magnitude;
    int32_t span;
    int32_t output;

    if (desired == 0) {
        return 0;
    }
    if (deadband >= (uint32_t)MOTOR_PWM_FULL) {
        /* 死区配置不合理（≥ 满量程）：不做补偿，直接饱和，避免除零。 */
        return (desired > 0) ? (int32_t)MOTOR_PWM_FULL : -(int32_t)MOTOR_PWM_FULL;
    }

    sign = (desired > 0) ? 1 : -1;
    magnitude = (desired > 0) ? desired : -desired;
    if (magnitude > (int32_t)MOTOR_PWM_FULL) {
        magnitude = (int32_t)MOTOR_PWM_FULL; /* 先钳位，避免乘法溢出 */
    }

    /* 分段映射 */
    if (magnitude <= (int32_t)MOTOR_COMP_LINEAR_BAND) {
        return sign * magnitude;
    }

    span = (int32_t)MOTOR_PWM_FULL - (int32_t)deadband;
    output = (int32_t)deadband + (magnitude * span) / (int32_t)MOTOR_PWM_FULL;

    if (output > (int32_t)MOTOR_PWM_FULL) {
        output = (int32_t)MOTOR_PWM_FULL;
    }
    return sign * output;
}

void Motor_SetRawDuty(WheelId_e id, int32_t duty) {
    const motor_hw_t *hw;

    if ((int)id >= (int)WHEEL_COUNT) return;   /* id 是枚举，非负；只查上界，避免 -Wtype-limits 告警 */
    hw = &s_motor_hw[id];

    /* sign_pwm 把“控制器意义的前进”翻成“该轮物理前进”，再交给方向脚逻辑。 */
    if (hw->sign_pwm < 0) {
        duty = -duty;
    }
    if (duty > (int32_t)MOTOR_PWM_FULL) duty = (int32_t)MOTOR_PWM_FULL;
    if (duty < -(int32_t)MOTOR_PWM_FULL) duty = -(int32_t)MOTOR_PWM_FULL;

    set_direction(hw, (duty > 0) ? 1 : ((duty < 0) ? -1 : 0));
    write_pwm(hw, duty);
    s_applied_duty[id] = duty;
}

void Motor_SetDuty(WheelId_e id, int32_t duty) {
    if ((int)id >= (int)WHEEL_COUNT) return;   /* id 是枚举，非负；只查上界，避免 -Wtype-limits 告警 */
    Motor_SetRawDuty(id, Motor_ApplyDeadbandComp(duty, Motor_GetDeadband(id)));
}

void Motor_Stop(WheelId_e id) {
    Motor_SetRawDuty(id, 0);
}

void Motor_StopAll(void) {
    int i;

    for (i = 0; i < (int)WHEEL_COUNT; i++) {
        Motor_SetRawDuty((WheelId_e)i, 0);
    }
}

uint32_t Motor_GetDeadband(WheelId_e id) {
    if ((int)id >= (int)WHEEL_COUNT) return 0u; /* id 是枚举，非负 */
    return s_motor_hw[id].deadband;
}

void Motor_Init(void) {
    int i;

    /* 先把方向脚清成滑行状态，再启动 PWM，避免上电瞬间冲一下。 */
    for (i = 0; i < (int)WHEEL_COUNT; i++) {
        const motor_hw_t *hw = &s_motor_hw[i];
        HAL_GPIO_WritePin(hw->dir_a_port, hw->dir_a_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(hw->dir_b_port, hw->dir_b_pin, GPIO_PIN_RESET);
        s_applied_duty[i] = 0;
    }

    /* 启动四路 PWM（CubeMX 已把 Period 设为 8400-1，即 20 kHz）。 */
    (void)HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1); /* FL */
    (void)HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2); /* FR */
    (void)HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); /* RL */
    (void)HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4); /* RR */

    Motor_StopAll();
}
