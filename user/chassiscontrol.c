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

/* 注：这里曾经有 s_vel_last_tick / s_vel_started，用来把"上一轮速度环的 tick"
 * 记下来算真实 dt_s。已删除 —— 现在 dt_s 直接用固定标称周期
 * （VEL_LOOP_PERIOD_MS / 1000），理由见 vel_loop_tick() 的注释：
 * 测速与 PID 用同一个常数 dt，时间维度上不再有任何变量，
 * 也就不存在"dt 掉到 1 ms 导致速度放大 10 倍"这类故障。 */

void Chassis_TakeLatest(remote_in_t *out) {
    if (out != NULL) {
        *out = s_last_cmd;
    }
}

/* ★ 三个打印函数整体受 CHASSIS_PRINT_ENABLE 保护 ★
 * 关闭时连函数体都不编译，避免 -Wunused-function，
 * 也彻底不占代码空间（printf 的格式化代码会被 --gc-sections 回收）。 */
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

/**
  * @brief 打印 M3（RL）轮的编码器诊断值（第 3 行）。
  *
  * ★ 为什么单独加这一行 ★
  *   Ozone 的 watch 在数组下标上可能用**过期的步长**算地址（本工程改过
  *   Encode_t 的布局），从而显示物理不可能的读数。串口打印走的是真实的
  *   取值函数，**不受调试器符号缓存影响**，所以调参阶段以这一行为准。
  *
  * 判读方法（ly 从小往大推）：
  *   - DELTA：本周期（10 ms）的有符号计数增量。
  *     满速 = 1046 mm/s → 约 87 个计数/周期；ly=64（112 mm/s）→ 约 9 个。
  *     若出现几百几千 → 编码器计数被干扰，测速不可信。
  *   - COUNT：累计位置。正常单向转动时**单调递增**；来回跳 = 真在正反转。
  *   - TGT ：该轮目标轮速（mm/s），正数 = 物理前进。
  *   - DUTY：写入的比较寄存器值（带符号，满量程 8400）。
  *
  * 全部用整数打印：printf 里用 %f 会链入完整的软浮点格式化代码，
  * 在 F407 上要多吃好几 KB Flash，这里没必要。
  */
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

/* 关闭打印：三个函数不编译。任务循环里对应分支也用 #if 去掉了，
 * 所以不会产生“定义了未使用”的告警。 */

#endif /* CHASSIS_PRINT_ENABLE */


/**
  * @brief 执行一次速度环。
  *
  * ★ dt_s 用固定标称周期（VEL_LOOP_PERIOD_MS / 1000 = 0.01 s）★
  *
  * 为什么不再测量：本工程的节拍本来就是 FreeRTOS tick 驱动的 —— Chassis 任务
  * 每 tick（1 ms）跑一次，每 VEL_LOOP_PERIOD_MS 个 tick 调一次速度环，
  * 所以真实周期稳定在 10 ms。用固定常数有三个好处：
  *
  *   1) **测速与 PID 用的是同一个 dt**，两者不可能不一致（这是本工程反复
  *      出问题的地方：速度按 A 算、积分按 B 算）；
  *   2) 时间维度上不再有任何变量，也就不存在"dt 掉到 1 ms 导致速度放大 10 倍"
  *      这种故障（本工程实测出现过 0.001 s、甚至 23 µs 的 dt）；
  *   3) 与参考项目 D:\STM32\PID_Freertos 的做法等价 —— 它虽然用 DWT 实测，
  *      但实测值在 100 Hz 节拍下恒为 10 ms，等价于常数。
  *
  * 节拍偶有抖动（例如 printf 阻塞）只会让等效 Kp/Ki 按比例缩放几个百分点，
  * 不改变闭环性质，也不需要补偿。
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

    /* ── ★把优先级强制成 app_config.h 的约定值★ ──────────────────────────
     *
     * 为什么要在运行期设：本任务的创建发生在 CubeMX 生成区
     * （Core/Src/freertos.c 的 osThreadNew(StartChassisTask, ...)），
     * 优先级来自 Robo.ioc 的 "FREERTOS.Tasks01"（那里是 47），
     * 而本工程的约定值在 app_config.h 的 PRIO_CHASSIS（32）。
     * 两套数字并存会造成两个真麻烦：
     *
     *   ① 优先级次序被破坏。app_config.h:45-54 明确要求
     *      "OLED 任务必须排在 Chassis 之下"，因为 OLED 里有阻塞式
     *      HAL_I2C_Mem_Write（HAL_MAX_DELAY），一旦它优先级更高就会
     *      把速度环彻底饿死 —— 现象是"串口没输出、s_vel_tick 停住、电机不动"，
     *      而 Safety 优先级更高仍在喂狗，看门狗也不会复位，非常隐蔽。
     *      而 .ioc 里 OLEDPowerTask = 35 > PRIO_CHASSIS(32)，
     *      正好会踩这个坑。
     *
     *   ② 调试时看到的优先级与实际不符，查问题会绕远路。
     *
     * 处理办法：**让 app_config.h 成为唯一真相来源**，在任务体开头用
     * vTaskPrioritySet() 覆盖 .ioc 的值。这样 CubeMX 里怎么填都不影响实际行为，
     * 也不需要每次改 .ioc。
     *
     * ⚠ 必须放在 VelCtl_Init() **之后**：初始化期间保持 CubeMX 给的高优先级，
     *   可以避免被 OLED 的阻塞 I2C 打断。 */
    (void)vTaskPrioritySet(NULL, (UBaseType_t)PRIO_CHASSIS);

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
         * 所以每 CHASSIS_PRINT_DIVIDER 个周期打印一次。
         *
         * ★ 调蓝牙遥控时把 app_config.h 的 CHASSIS_PRINT_ENABLE 设为 0 ★
         *   否则这 3 行 10 Hz 的打印会把 USART6 占满（约 2700 B/s），
         *   遥控任务的诊断信息就被冲掉了。 */
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

        /* 固定周期节拍：vTaskDelayUntil 以绝对时间为基准，不会累积漂移。
         * 不用 HAL_Delay()：在 FreeRTOS 下它只是忙等 SysTick 计数，
         * 既费 CPU 又会破坏任务的周期精度。 */
        vTaskDelayUntil(&last_wake, (TickType_t)pdMS_TO_TICKS(CHASSIS_PERIOD_MS));
    }
}
