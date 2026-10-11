/**
  ******************************************************************************
  * @file    safety.c
  * @brief   安全任务（Safety）。
  *
  * 任务职责：以 10 ms 周期喂独立看门狗（IWDG）。若调度器停摆、任务被无限阻塞
  *           或发生非法死循环，IWDG 会在约 4 s 后复位 MCU —— 这是遥控机器人
  *           最基本的一道保命措施（本阶段虽然不驱动电机，但链路必须从第一天
  *           就带着看门狗跑，避免后期把“忘了喂狗”当成业务 bug 排查）。
  *
  * 周期：10 ms。优先级：34（全工程最高）。
  ******************************************************************************
  */

#include "safety.h"

#include "main.h" /* 寄存器定义（stm32f4xx.h → CMSIS device header）*/

#include "app_config.h"

void Safety_WatchdogStart(void) {
    /* 写 IWDG 寄存器需要先解锁：向 KR 写 IWDG_KEY_UNLOCK。
     * HAL_IWDG_Init() 内部做的是同一套动作，这里直接展开，原因见 app_config.h：
     * 当前 CubeMX 配置没有使能 HAL IWDG 模块，也没有生成 IWDG 句柄；
     * 直接用寄存器可以让 CubeMX 生成的 hal_conf.h 与 CMakeLists 保持原样，
     * 重新生成代码时不会被冲掉。 */

    /* ① 解锁 PR 与 RLR */
    IWDG->KR = IWDG_KEY_UNLOCK;

    /* ② 预分频：计数器时钟 = LSI(32 kHz) / (4 * 2^IWDG_PRESCALER) = 125 Hz，
     *    即 8 ms 一个计数。 */
    IWDG->PR = IWDG_PRESCALER;

    /* ③ 重装值：499 → 499 × 8 ms ≈ 4.0 s。
     *    取值偏长的理由：调试器（Ozone）下断点/单步会停住 CPU，喂狗任务同样
     *    停住；超时太短会在单步调试时被反复复位。4 s 仍能兜住真正的死锁。 */
    IWDG->RLR = IWDG_RELOAD;

    /* ④ 等待 SR 里的 PVU/RVU 清零，确认 PR/RLR 已生效（LSI 由硬件自动开启）。 */
    while ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0u) {
        /* 短暂等待：这里在任务上下文，几微秒量级，无需让出 CPU。 */
    }

    /* ⑤ 刷新计数器并启动：先重装，再启动。 */
    IWDG->KR = IWDG_KEY_RELOAD;
    IWDG->KR = IWDG_KEY_START;
}

void StartSafetyTask(void *argument) {
    TickType_t last_wake;

    (void)argument;

    /* 必须在开始循环前启动 IWDG：一旦启动就无法关闭，因此要保证启动之后
     * 立刻就有喂狗动作跟着，中间不能出现长耗时操作。 */
    Safety_WatchdogStart();

    last_wake = xTaskGetTickCount();

    for (;;) {
        /* 喂狗：向 KR 写 IWDG_KEY_RELOAD，等同于 HAL_IWDG_Refresh()。
         * 本任务除了喂狗不做任何事——这也是它优先级最高的原因。 */
        IWDG->KR = IWDG_KEY_RELOAD;

        /* 10 ms 固定周期。用 vTaskDelayUntil 保证节拍不漂移；
         * 不用 HAL_Delay()（在 FreeRTOS 下它只是忙等，会白占 CPU）。 */
        vTaskDelayUntil(&last_wake, (TickType_t)pdMS_TO_TICKS(SAFETY_PERIOD_MS));
    }
}
