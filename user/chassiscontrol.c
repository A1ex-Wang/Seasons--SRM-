/**
  ******************************************************************************
  * @file    chassiscontrol.c
  * @brief   底盘控制任务（Chassis）。
  *
  * 任务职责：
  *   1) 每 1 ms 从 q_remote_cmd 非阻塞抽干队列，保存为“最近一次遥控输入”；
  *   2) 每 VEL_LOOP_PERIOD_MS（10 ms）调用一次速度环 VelCtl_Feed()：
  *      摇杆 → 车体速度 → 麦轮逆运动学 → 四路独立 PI → PWM；
  *   3) 每 100 ms（10 Hz）打印一行遥测，供实车标定与调参。
  *
  * 周期：1 ms（速度环 10 ms、打印 100 ms 都由本任务的计数器分频得到）。
  *
  * ★ 安全：只要遥控链路无效（online == 0），速度环会立即把 PWM 清零并清积分，
  *   不依赖任何“慢慢收敛”的过程。见 VelCtl_Feed() 里的链路检查。
  ******************************************************************************
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

/* 上一轮速度环执行的 tick，用于算真实的 dt_s。 */
static TickType_t s_vel_last_tick = 0u;

/* 速度环是否已经执行过第一次。
 * ⚠ 不能拿 s_vel_last_tick == 0 当"还没执行过"的哨兵：tick 0 是合法值
 * （调度器刚启动时第一次读 tick 完全可能为 0），一旦命中就会一直走"标称周期"
 * 分支，直到 tick 绕回 0（约 49.7 天）。所以用一个独立标志。 */
static uint8_t s_vel_started = 0u;

void Chassis_TakeLatest(remote_in_t *out) {
    if (out != NULL) {
        *out = s_last_cmd;
    }
}

/** @brief 打印遥控原始值（第 1 行）。 */
static void print_remote_line(const remote_in_t *last) {
    (void)printf("LX=%4d LY=%4d RX=%4d RY=%4d BTN=0x%06lX LT=%3u RT=%3u %s\r\n",
                 (int)last->lx, (int)last->ly, (int)last->rx, (int)last->ry,
                 (unsigned long)last->buttons, (unsigned)last->lt, (unsigned)last->rt,
                 (last->online != 0u) ? "ON" : "LOST");
}

/**
  * @brief 打印四轮实测速度（mm/s）与速度环输出占空比（第 2 行）。
  *
  * 标定/PID 调参时主要看这一行：
  *   - 标定阶段（MOTOR_TEST_MODE = 1）：给固定占空比，看哪个轮子的实测速度
  *     符号和车体前进方向不一致 → 那个轮的 MOTOR_SIGN_ENC_* 要取反；
  *     再看哪个轮子在别的轮子都转的时候还不转 → 那个轮的 DEADBAND 偏大。
  *   - 闭环阶段：对比实测速度与输出占空比，判断 P/I 是否合理。
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

/** @brief 执行一次速度环，并用 FreeRTOS tick 算出真实的 dt_s（单位：秒）。 */
static void vel_loop_tick(void) {
    TickType_t now = xTaskGetTickCount();
    float dt_s;

    if (s_vel_started == 0u) {
        /* 第一次调用：还没有上一次的时刻，用标称周期。 */
        dt_s = (float)VEL_LOOP_PERIOD_MS / 1000.0f;
        s_vel_started = 1u;
    } else {
        dt_s = (float)(now - s_vel_last_tick) / (float)APP_TICK_RATE_HZ;
        if (dt_s <= 0.0f) {
            /* tick 没有推进（理论上不该发生）：退化成标称周期，绝不把 0 传给 PID。 */
            dt_s = (float)VEL_LOOP_PERIOD_MS / 1000.0f;
        }
    }
    s_vel_last_tick = now;

    VelCtl_Feed(&s_last_cmd, dt_s);
}

void StartChassisTask(void *argument) {
    TickType_t last_wake;
    uint32_t print_counter = 0u;

    (void)argument;

    /* 上电初值：全零 + online = 0。速度环第一次执行时会因为 online == 0
     * 直接进入安全态并把 PWM 清零，所以上电期间四个轮子是静止的。 */
    (void)memset(&s_last_cmd, 0, sizeof(s_last_cmd));
    VelCtl_Init();

    last_wake = xTaskGetTickCount();

    for (;;) {
        remote_in_t incoming;

        /* ── 接收端“丢旧语义” ────────────────────────────────────────────────
         * xQueueReceive(..., 0) 是非阻塞取：取不到就沿用上一次的值。
         * 这里必须把队列**抽干**，只保留最后（最新）的一帧，否则队列会堆积成
         * FIFO，控制延迟随时间无上限增长（遥控链路里过期的摇杆位置没有价值）。 */
        while (xQueueReceive(q_remote_cmd, &incoming, (TickType_t)0) == pdPASS) {
            s_last_cmd = incoming;
        }

        /* ── 速度环：每 10 ms 执行一次 ─────────────────────────────────────
         * 为什么是 10 ms（100 Hz）而不是 1 ms，写在 app_config.h 第 7 节：
         * 本车 counts/轮转只有 44，1 ms 窗口里连 1 个计数都不到，
         * 测速会被量化噪声完全淹没。 */
        if (++s_vel_tick >= VEL_LOOP_PERIOD_MS) {
            s_vel_tick = 0u;
            vel_loop_tick();
        }

        /* ── 打印降频：10 Hz ───────────────────────────────────────────────
         * 1 kHz 打印会把串口带宽和 CPU 全部吃掉（每行约 90 字节 × 2 行），
         * 所以每 CHASSIS_PRINT_DIVIDER 个周期打印一次。 */
        if (++print_counter >= CHASSIS_PRINT_DIVIDER) {
            print_counter = 0u;
            print_remote_line(&s_last_cmd);
            print_wheel_line();
        }

        /* 固定周期节拍：vTaskDelayUntil 以绝对时间为基准，不会累积漂移。
         * 不用 HAL_Delay()：在 FreeRTOS 下它只是忙等 SysTick 计数，
         * 既费 CPU 又会破坏任务的周期精度。 */
        vTaskDelayUntil(&last_wake, (TickType_t)pdMS_TO_TICKS(CHASSIS_PERIOD_MS));
    }
}
