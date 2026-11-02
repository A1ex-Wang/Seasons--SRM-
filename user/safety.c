/** @file safety.c
 * @brief 安全任务（Safety）。
 */

#include "safety.h"

#include "main.h" /* 寄存器定义（stm32f4xx.h → CMSIS device header）*/

#include "app_config.h"

void Safety_WatchdogStart(void) {
    /* 写 IWDG 寄存器需要先解锁：向 KR 写 APP_IWDG_KEY_UNLOCK。
     * HAL_IWDG_Init() 内部做的是同一套动作，这里直接展开，原因见 app_config.h：
     * 当前 CubeMX 配置没有使能 HAL IWDG 模块，也没有生成 IWDG 句柄；
     * 直接用寄存器可以让 CubeMX 生成的 hal_conf.h 与 CMakeLists 保持原样，
     * 重新生成代码时不会被冲掉。 */

    /* ① 解锁 PR 与 RLR */
    IWDG->KR = APP_IWDG_KEY_UNLOCK;

    /* ② 预分频：计数器时钟 = LSI(32 kHz) / (4 * 2^APP_IWDG_PRESCALER) = 125 Hz，
     *    即 8 ms 一个计数。 */
    IWDG->PR = APP_IWDG_PRESCALER;

    /* ③ 重装值：62 → 62 × 8 ms ≈ 0.5 s（LSI 频率存在误差）。 */
    IWDG->RLR = APP_IWDG_RELOAD;

    /* ④ 等待 SR 里的 PVU/RVU 清零，确认 PR/RLR 已生效（LSI 由硬件自动开启）。 */
    while ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0u) {
        /* 短暂等待：这里在任务上下文，几微秒量级，无需让出 CPU。 */
    }

    /* ⑤ 刷新计数器并启动：先重装，再启动。 */
    IWDG->KR = APP_IWDG_KEY_RELOAD;
    IWDG->KR = APP_IWDG_KEY_START;
}

void StartSafetyTask(void *argument) {
    TickType_t last_wake;

    (void)argument;

    /* 必须在开始循环前启动 IWDG：一旦启动就无法关闭，因此要保证启动之后 */
#ifndef DEBUG_NO_WATCHDOG
#define DEBUG_NO_WATCHDOG 0
#endif

#if (DEBUG_NO_WATCHDOG == 0)
    Safety_WatchdogStart();
#endif

    /* 使用 app_config.h 中的任务优先级。 */
    (void)vTaskPrioritySet(NULL, (UBaseType_t)PRIO_SAFETY);

    last_wake = xTaskGetTickCount();

    for (;;) {
        /* 喂狗：向 KR 写 APP_IWDG_KEY_RELOAD，等同于 HAL_IWDG_Refresh()。
         * 本任务除了喂狗不做任何事——这也是它优先级最高的原因。 */
        IWDG->KR = APP_IWDG_KEY_RELOAD;

        /* 10 ms 固定周期。用 vTaskDelayUntil 保证节拍不漂移；
         * 不用 HAL_Delay()（在 FreeRTOS 下它只是忙等，会白占 CPU）。 */
        vTaskDelayUntil(&last_wake, (TickType_t)pdMS_TO_TICKS(SAFETY_PERIOD_MS));
    }
}
