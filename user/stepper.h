#ifndef ROBO_STEPPER_H
#define ROBO_STEPPER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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
