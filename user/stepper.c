#include "stepper.h"

#include "main.h"
#include "tim.h"
#include "app_config.h"

static volatile int32_t s_position_steps;
static volatile int8_t s_requested_dir;
static volatile int8_t s_active_dir;
static volatile float s_step_hz;
static volatile uint8_t s_timer_running;

static void stepper_stop_timer(void)
{
    __HAL_TIM_DISABLE_IT(&htim5, TIM_IT_UPDATE);
    __HAL_TIM_DISABLE(&htim5);
    s_timer_running = 0U;
    HAL_GPIO_WritePin(ARME_STEP_GPIO_Port, ARME_STEP_Pin, GPIO_PIN_RESET);
}

static void stepper_set_dir(int8_t direction)
{
    GPIO_PinState level = (direction * STEPPER_DIR_POLARITY > 0)
                        ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(ARME_DIR_GPIO_Port, ARME_DIR_Pin, level);
}

static void stepper_set_frequency(float step_hz)
{
    float edge_hz;
    uint32_t arr;

    if (step_hz < STEPPER_MIN_STEP_HZ) step_hz = STEPPER_MIN_STEP_HZ;
    edge_hz = step_hz * 2.0f; /* STEP 高、低各占一个 TIM5 周期。 */
    arr = (uint32_t)(STEPPER_TIMER_TICK_HZ / edge_hz);
    if (arr < 2U) arr = 2U;
    __HAL_TIM_SET_AUTORELOAD(&htim5, arr - 1U);
}

static void stepper_start_timer(void)
{
    if (s_timer_running != 0U) return;
    __HAL_TIM_SET_COUNTER(&htim5, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(&htim5);
    s_timer_running = 1U;
}

void Stepper_Init(void)
{
    s_position_steps = 0;
    s_requested_dir = 0;
    s_active_dir = 1;
    s_step_hz = 0.0f;
    s_timer_running = 0U;
    stepper_stop_timer();
    stepper_set_dir(1);
}

void Stepper_Feed(int8_t command_sign, float dt_s)
{
    float target_hz;
    float delta_hz;
    int8_t bounded_command = command_sign;

    if (dt_s <= 0.0f) return;
    if (bounded_command > 0 && s_position_steps >= (int32_t)STEPPER_MAX_POSITION_STEPS)
        bounded_command = 0;
    if (bounded_command < 0 && s_position_steps <= 0)
        bounded_command = 0;
    s_requested_dir = bounded_command;

    /* 换向时先减速到零，再改变 DIR，避免运行中直接反向。 */
    target_hz = (s_requested_dir != 0 && s_requested_dir == s_active_dir)
              ? STEPPER_TARGET_STEP_HZ : 0.0f;
    delta_hz = STEPPER_ACCEL_STEP_HZ_S * dt_s;
    if (s_step_hz < target_hz) {
        s_step_hz += delta_hz;
        if (s_step_hz > target_hz) s_step_hz = target_hz;
    } else if (s_step_hz > target_hz) {
        s_step_hz -= delta_hz;
        if (s_step_hz < target_hz) s_step_hz = target_hz;
    }

    if (s_step_hz <= 0.0f) {
        s_step_hz = 0.0f;
        stepper_stop_timer();
        if (s_requested_dir != 0 && s_requested_dir != s_active_dir) {
            s_active_dir = s_requested_dir;
            stepper_set_dir(s_active_dir);
        }
        return;
    }

    stepper_set_frequency(s_step_hz);
    stepper_set_dir(s_active_dir);
    stepper_start_timer();
}

void Stepper_TIM5_PeriodElapsed(void)
{
    GPIO_PinState current;

    if (s_timer_running == 0U) return;
    current = HAL_GPIO_ReadPin(ARME_STEP_GPIO_Port, ARME_STEP_Pin);
    if (current == GPIO_PIN_RESET) {
        /* 只在上升沿计一步，A4988 的 STEP 上升沿有效。 */
        if ((s_active_dir > 0 && s_position_steps >= (int32_t)STEPPER_MAX_POSITION_STEPS) ||
            (s_active_dir < 0 && s_position_steps <= 0)) {
            stepper_stop_timer();
            return;
        }
        s_position_steps += (s_active_dir > 0) ? 1 : -1;
        HAL_GPIO_WritePin(ARME_STEP_GPIO_Port, ARME_STEP_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(ARME_STEP_GPIO_Port, ARME_STEP_Pin, GPIO_PIN_RESET);
    }
}

int32_t Stepper_GetPositionSteps(void)
{
    return s_position_steps;
}

uint8_t Stepper_IsRunning(void)
{
    return s_timer_running;
}
