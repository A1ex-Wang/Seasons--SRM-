/** @file chassiscontrol.c
 * @brief 底盘控制任务（Chassis）。
 */

#include "chassiscontrol.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "app_config.h"
#include "app_types.h"
#include "velocity_control.h"

/* 最近一次收到的遥控输入。生产者（Remote）只投队列，不直接写它；
 * 由本任务每周期覆盖，因此读到的永远是“队列里最新的一帧”。 */
static remote_in_t s_last_cmd;

/* 速度环节拍计数（1 ms 累加，达到 VEL_LOOP_PERIOD_MS 触发一次）。 */
static uint32_t s_vel_tick = 0u;

void Chassis_TakeLatest(remote_in_t *out) {
    if (out != NULL) {
        *out = s_last_cmd;
    }
}

/* 遥测输出由 CHASSIS_PRINT_ENABLE 控制。 */
#if (CHASSIS_PRINT_ENABLE != 0)

/** @brief 打印遥控原始值（第 1 行）。 */
static void print_remote_line(const remote_in_t *last) {
    (void)printf("LX=%4d LY=%4d RX=%4d RY=%4d BTN=0x%06lX LT=%3u RT=%3u %s\r\n",
                 (int)last->lx, (int)last->ly, (int)last->rx, (int)last->ry,
                 (unsigned long)last->buttons, (unsigned)last->lt, (unsigned)last->rt,
                 (last->online != 0u) ? "ON" : "LOST");
}

/**
 * @brief 打印四轮实测速度（mm/s）与速度环输出占空比（第 2 行）。
 */
static void print_wheel_line(void) {
    (void)printf("WHEEL mm/s FL=%5ld FR=%5ld RL=%5ld RR=%5ld | DUTY FL=%5ld FR=%5ld RL=%5ld RR=%5ld\r\n",
                 (long)VelCtl_GetWheelSpeedMmps(WHEEL_FL),
                 (long)VelCtl_GetWheelSpeedMmps(WHEEL_FR),
                 (long)VelCtl_GetWheelSpeedMmps(WHEEL_RL),
                 (long)VelCtl_GetWheelSpeedMmps(WHEEL_RR),
                 (long)VelCtl_GetWheelDuty(WHEEL_FL),
                 (long)VelCtl_GetWheelDuty(WHEEL_FR),
                 (long)VelCtl_GetWheelDuty(WHEEL_RL),
                 (long)VelCtl_GetWheelDuty(WHEEL_RR));
}

static void print_enc_line(void) {
    (void)printf("RL VY=%5ld TGT=%5ld SPD=%6ld DELTA=%6ld CNT=%8ld DUTY=%5ld\r\n",
                 (long)VelCtl_TelemCmdVy(),
                 (long)VelCtl_TelemTargetRL(),
                 (long)VelCtl_TelemSpeedRL(),
                 (long)VelCtl_TelemDeltaRL(),
                 (long)VelCtl_TelemCountRL(),
                 (long)VelCtl_TelemDutyRL());
}

#else  /* CHASSIS_PRINT_ENABLE == 0 */

#endif /* CHASSIS_PRINT_ENABLE */

/**
 * @brief 执行一次速度环。
 */
static void vel_loop_tick(void) {
    VelCtl_Feed(&s_last_cmd, (float)VEL_LOOP_PERIOD_MS / 1000.0f);
}

void StartChassisTask(void *argument) {
    TickType_t last_wake;
    uint32_t print_counter = 0u;

    (void)argument;

    /* 上电初值：全零 + online = 0。速度环第一次执行时会因为 online == 0
     * 直接进入安全态并把 PWM 清零，所以上电期间四个轮子是静止的。 */
    (void)memset(&s_last_cmd, 0, sizeof(s_last_cmd));
    VelCtl_Init();

    (void)vTaskPrioritySet(NULL, (UBaseType_t)PRIO_CHASSIS);

    last_wake = xTaskGetTickCount();

    for (;;) {
        remote_in_t incoming;

        /* 非阻塞取空队列，仅保留最新一帧。 */
        while (xQueueReceive(q_remote_cmd, &incoming, (TickType_t)0) == pdPASS) {
            s_last_cmd = incoming;
        }

        /* 按 VEL_LOOP_PERIOD_MS 执行速度环，测速与 PID 使用相同周期。 */
        if (++s_vel_tick >= VEL_LOOP_PERIOD_MS) {
            s_vel_tick = 0u;
            vel_loop_tick();
        }

        /* 打印降频：10 Hz */
#if (CHASSIS_PRINT_ENABLE != 0)
        if (++print_counter >= CHASSIS_PRINT_DIVIDER) {
            print_counter = 0u;
            print_remote_line(&s_last_cmd);
            print_wheel_line();
            print_enc_line();
        }
#else
        (void)print_counter;   /* 关闭打印时避免“变量只写不读”的多余告警 */
#endif

        /* 使用绝对节拍周期唤醒，减少累计调度漂移。 */
        vTaskDelayUntil(&last_wake, (TickType_t)pdMS_TO_TICKS(CHASSIS_PERIOD_MS));
    }
}
