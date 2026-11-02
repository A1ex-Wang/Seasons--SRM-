#ifndef ROBO_STEPPER_H
#define ROBO_STEPPER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int8_t command_sign;
    int8_t requested_dir;
    int8_t active_dir;
    int32_t position_steps;
    float step_hz;
    uint8_t timer_running;
    uint32_t tim5_cr1;
    uint32_t tim5_dier;
    uint8_t step_pin_level;
    uint8_t dir_pin_level;
} StepperWatch_t;

/* 步进控制状态快照：遥控输入、模式、选轴和脉冲输出。 */
typedef struct {
    int16_t lx;
    int16_t ly;
    int16_t rx;
    int16_t ry;
    uint32_t buttons;
    uint8_t online;
    uint8_t grab_mode;
    int8_t axis_lock;
    int8_t selected_lx;
    int8_t selected_ly;
    int8_t command_sign;
    int8_t requested_dir;
    int8_t active_dir;
    int32_t position_steps;
    float step_hz;
    uint8_t timer_running;
    uint32_t tim5_cr1;
    uint32_t tim5_dier;
    uint8_t step_pin_level;
    uint8_t dir_pin_level;
} StepperDebugWatch_t;

extern volatile StepperWatch_t g_stepper_watch;
extern volatile StepperDebugWatch_t g_stepper_debug_watch;

/* J1 步进旋转关节：正负号只表示逻辑方向，实际电气方向由宏/接线标定。 */
void Stepper_Init(void);
void Stepper_Feed(int8_t command_sign, float dt_s);
void Stepper_TIM5_PeriodElapsed(void);
int32_t Stepper_GetPositionSteps(void);
uint8_t Stepper_IsRunning(void);

#ifdef __cplusplus
}
#endif

#endif
