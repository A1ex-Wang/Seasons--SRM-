/** @file app_config.h
 * @brief 全局编译期配置：所有魔数、任务参数、队列尺寸、机械参数、符号约定。
 */

#ifndef ROBO_APP_CONFIG_H
#define ROBO_APP_CONFIG_H

#include <stdint.h>

/* 1. 时钟与节拍 */

/* FreeRTOS 节拍 1 kHz（见 Core/Inc/FreeRTOSConfig.h 的 configTICK_RATE_HZ）。
 * 1 个 tick = 1 ms，因此“周期_ms”与“周期_tick”数值相等，代码里统一用 TickType_t。 */
#define APP_TICK_RATE_HZ 1000u

/* ===== 任务周期（毫秒）===== */
#define SAFETY_PERIOD_MS 10u   /* StartSafetyTask：喂狗 */
#define CHASSIS_PERIOD_MS 1u   /* StartChassisTask：读队列 / 速度环节拍 */
#define REMOTE_PERIOD_MS 1u    /* StartRemoteTask：SRM_Remote_Poll + 投队列 */
#define VEL_LOOP_PERIOD_MS 10u /* 速度环：100 Hz */

/* ===== 任务栈深度（单位：字 = 4 字节，与 xTaskCreateStatic 的 usStackDepth 语义一致）===== */
#define STACK_SAFETY_WORDS 256u
#define STACK_CHASSIS_WORDS 512u
#define STACK_REMOTE_WORDS 512u

/* ===== 任务优先级（FreeRTOS 数值越大优先级越高，上限 configMAX_PRIORITIES = 56）===== */
#define PRIO_SAFETY 34u     /* 看门狗任务 */
#define PRIO_CHASSIS 32u    /* 速度环：必须在 OLED 之上 */
#define PRIO_REMOTE 26u     /* 蓝牙字节流解析、投队列 */
#define PRIO_OLEDPOWER 20u  /* 低优先级显示任务 */

/* 2A. J1 步进旋转关节（A4988，GPIO STEP/DIR + TIM5 更新中断） */

/* 17HS4401：1.8°/步；A4988 使用 1/8 细分；同步带减速 1:5。 */
#define STEPPER_MOTOR_STEPS_REV       200u
#define STEPPER_MICROSTEP             8u
#define STEPPER_GEAR_RATIO            5u

/* 输出轴初始目标速度：15 rpm = 2000 个电机 STEP 脉冲/秒。后续实测可调。 */
#define STEPPER_TARGET_STEP_HZ        2000.0f
#define STEPPER_MIN_STEP_HZ           300.0f
#define STEPPER_ACCEL_STEP_HZ_S       2500.0f

/* TIM5 预分频为 84-1，系统时钟配置下计数频率为 1 MHz。 */
#define STEPPER_TIMER_TICK_HZ         1000000.0f
#define STEPPER_DIR_POLARITY          (1) /* 改为 -1 可整体反向 */

/* MPU6050 角速度环：I2C1，Z 轴量程 ±500 °/s，100 Hz 读取。 */
#define IMU_GYRO_Z_SIGN               (1) /* 安装方向反了就改为 -1 */
#define IMU_GYRO_CAL_SAMPLES          200u
#define IMU_GYRO_LPF_ALPHA            0.25f
#define YAW_RATE_KP                   0.05f
#define YAW_RATE_KI                   0.00f
#define YAW_RATE_INTEGRAL_LIMIT       0.80f
#define YAW_RATE_OUTPUT_LIMIT_RADS    2.0f

/* ===== 任务名称（调试器里看到的字符串，长度受 configMAX_TASK_NAME_LEN = 16 限制）===== */
#define NAME_SAFETY "Safety"
#define NAME_CHASSIS "Chassis"
#define NAME_REMOTE "Remote"

/* 2. 蓝牙链路（JDY-31 透传模块，接 USART2） */

/* 蓝牙 UART 波特率；须与 JDY-31 的设置一致，初始化时同步至 USART2。 */
#define BT_UART_BAUD 9600u

/* 蓝牙串口句柄：CubeMX 在 Core/Src/usart.c 中生成，C 文件里的变量名固定。 */
#define BT_UART_HANDLE huart2

/* DMA 环形接收缓冲大小（字节）。由上游 srm_uart.h 的 SRM_UART_RX_DMA_SIZE 决定，
 * srm_uart_init() 会校验“传进去的大小”必须等于该宏，这里只做一处引用和断言。 */
#define REMOTE_RX_DMA_SIZE 256u

/* 3. 调试串口（USART6，PG14=TX / PG9=RX，独立于蓝牙链路） */

/* Chassis 的 printf 走 USART6，不用蓝牙口：蓝牙链路是给协议用的， */
#define DEBUG_UART_BAUD 115200u
#define DEBUG_UART_INSTANCE USART6

/* printf 单字节写等待 TXE 的循环上限。防止 USART6 未使能（例如被 Ozone 复位过）
 * 时死等，配合 -O0 调试也足够宽松。 */
#define DEBUG_UART_TXE_TIMEOUT 200000u

/* 4. 队列（静态分配，不占用 FreeRTOS heap） */

#define Q_REMOTE_LEN 2u

/* 打印降频：每 N 个 Chassis 周期（1 ms）打印一次 = 10 Hz。
 * 不做内容去重的话，1 kHz 打印会把串口和 CPU 全部吃掉。 */
#define CHASSIS_PRINT_DIVIDER 100u

/* 底盘任务串口打印总开关 */
#define CHASSIS_PRINT_ENABLE 0u

/* 5. 失联保护 */

/* 上游协议层（srm_remote.c）已经实现了“超过 600 ms 没有合法控制帧就自动以
 * 全零状态再次回调板级函数”的保护。本工程不重复实现超时计时，
 * 只把这个数值写在这里用于文档化，并断言它与上游宏一致（见第 13 节）。 */
#define REMOTE_TIMEOUT_MS 600u

/* 6. 机械参数（JGB37-520-0650-EN，12 V，减速比 30，无同步带直连轮） */

/* 电机：减速箱输出轴空载 333 rpm / 额定负载 250 rpm（12 V） */
#define MOTOR_NO_LOAD_RPM 333.0f
#define MOTOR_RATED_RPM 250.0f

/* 麦轮外径 60 mm。轮周长 = π × 60 = 188.5 mm */
#define WHEEL_DIAMETER_MM 60.0f
#define WHEEL_PERIMETER_MM 188.4955592f /* π × 60 */

/* 编码器到轮轴的总机械减速比（不是同步带比，是整条传动链的总比）。
 * 无同步带、减速箱输出轴直连轮，但编码器装在电机轴（减速箱之前），
 * 所以从编码器轴到轮轴仍然是 1:30 的总减速。
 *
 * 实测依据：手动把轮子转一整圈，s_enc[i].total_count 增加约 1560，
 * 与 13 PPR × 4 倍频 × 30 = 1560 完全吻合。改这个值前请重新实测。 */
#define DRIVE_RATIO 30.0f

/* 轮子最大线速度（mm/s）。空载 333 rpm → 188.5 mm/rev × 333/60 = 1046 mm/s。
 * 这个值同时用作“运动学输出的物理上限”，速度环的限幅用它。 */
#define WHEEL_MAX_SPEED_MMPS 1046.0f

/* 车体几何：L = 前后轴距 250 mm，W = 左右轮距 274 mm。
 * 麦轮逆运动学的 k = (L + W) / 2 = 262 mm：
 * k 是“纯自转时每个轮子的线速度 / 车体角速度”，单位 mm。 */
#define CHASSIS_WHEELBASE_MM 250.0f /* L */
#define CHASSIS_TRACK_MM 274.0f     /* W */
#define CHASSIS_K_MM ((CHASSIS_WHEELBASE_MM + CHASSIS_TRACK_MM) * 0.5f) /* = 262.0 mm */

/* 7. 编码器与测速 */

#define ENCODER_PPR_MOTOR 13.0f
#define ENCODER_QUADRATURE 4.0f
#define ENCODER_COUNT_PER_MOTOR_REV (ENCODER_PPR_MOTOR * ENCODER_QUADRATURE)   /* 52 */
#define ENCODER_COUNT_PER_WHEEL_REV (ENCODER_COUNT_PER_MOTOR_REV * DRIVE_RATIO) /* 1560 */

#define ENCODER_COUNTS_TO_MMPS (WHEEL_PERIMETER_MM / ENCODER_COUNT_PER_WHEEL_REV)
#define MMPS_TO_ENCODER_COUNTS (ENCODER_COUNT_PER_WHEEL_REV / WHEEL_PERIMETER_MM)

/* 速度反馈低通时间常数（秒），alpha = dt / (RC + dt)。 */
#define VEL_FEEDBACK_LPF_RC_S 0.0085f

/* 7-b. 逐轮反馈使能（没接编码器的轮子必须置 0） */

/* 逐轮反馈开关：1 = 闭环控制，0 = 关闭该轮输出。 */
#define VEL_USE_FB_FL 1u  /* 反馈使能：1 = 闭环，0 = 关闭输出。 */
#define VEL_USE_FB_FR 1u  /* 反馈使能：1 = 闭环，0 = 关闭输出。 */
#define VEL_USE_FB_RL 1u  /* 反馈使能：1 = 闭环，0 = 关闭输出。 */
#define VEL_USE_FB_RR 1u  /* 反馈使能：1 = 闭环，0 = 关闭输出。 */

/* 8. 运动学与整定参数 */

/* 摇杆原始量纲：-512 .. 511 */
#define REMOTE_AXIS_MIN (-512)
#define REMOTE_AXIS_MAX (511)
#define REMOTE_AXIS_FULL_SCALE 512.0f

/* 摇杆死区内归零，死区外按原量程归一化。 */
#define REMOTE_DEADZONE 20

/* 车体速度上限：
 *   MAX_VXY_MS 取 0.9 而不是物理极限 1.046，是为了保留转弯余量：
 *   ω_max = (WHEEL_MAX − |v|) / k，满速 1.046 时 ω_max = 0，也就是推到底完全转不了弯。
 *   取 0.9 → 满速直行时仍留约 0.56 rad/s（32°/s）。 */
#define MAX_VXY_MS 0.9f
#define MAX_OMEGA_RADS 2.0f

/* 指令加速度限幅。保护对象不是电机（扭矩很宽裕），而是：
 *   ① TB6612 的电流（堵转 6 A/电机，四轮 24 A 远超驱动器能力）；
 *   ② 麦轮横移方向的抓地力（约 2~3 m/s² 就开始打滑）。
 * 2.5 m/s² → 0→0.9 m/s 约 0.36 s。 */
#define MAX_ACC_MS2 2.5f
#define MAX_ALPHA_RADS2 12.0f /* 角加速度限幅 rad/s² */

/* 指令幅值小于该值（mm/s）时直接归零，避免死区边缘的指令抖动。 */
#define CMD_DEADBAND_MMPS 5.0f

/* 9. 电机驱动（TB6612FNG × 4）与每轮标定 */

/* PWM：TIM8_CH3/CH4 = M3/M4，TIM9_CH1/CH2 = M1/M2，Period = 8400-1 → 20 kHz。
 * 占空比以“计数”为单位，满量程 MOTOR_PWM_FULL = 8400。 */
#define MOTOR_PWM_FULL 8400u

/* TB6612 的 STBY 由硬件外接 3V3 常高，不需要软件控制。 */

/* 每轮标定表（P1 开环标定要填的就是这里） */

#define MOTOR_SIGN_DIR_FL (1)
#define MOTOR_SIGN_PWM_FL (1)
#define MOTOR_SIGN_ENC_FL (-1)   
#define MOTOR_DEADBAND_FL 1000u

#define MOTOR_SIGN_DIR_FR (1)
#define MOTOR_SIGN_PWM_FR (1)
#define MOTOR_SIGN_ENC_FR (-1)
#define MOTOR_DEADBAND_FR 1000u

#define MOTOR_SIGN_DIR_RL (-1)
#define MOTOR_SIGN_PWM_RL (1)
#define MOTOR_SIGN_ENC_RL (-1)
#define MOTOR_DEADBAND_RL 1000u

#define MOTOR_SIGN_DIR_RR (1)
#define MOTOR_SIGN_PWM_RR (1)
#define MOTOR_SIGN_ENC_RR (1)
#define MOTOR_DEADBAND_RR 1000u

/* 开环标定模式：置 1 后速度环停摆，四个轮子直接吃由摇杆调制的固定占空比，
 * 串口打印各轮原始计数与换算速度。用来把上面的 SIGN_* 和 DEADBAND_* 全部量出来。
 * 标定完成后改回 0。
 *
 * 支持用编译选项覆盖（-DMOTOR_TEST_MODE=0），这样同一份源码可以同时
 * 编译“标定固件”和“闭环固件”而不用改文件；下面的 #ifndef 保证不会重定义。 */
#ifndef MOTOR_TEST_MODE
#define MOTOR_TEST_MODE 0
#endif

/* 标定模式下的占空比范围（计数）。占空比 = MOTOR_TEST_DUTY_BASE ±
 * |摇杆| × MOTOR_TEST_DUTY_SPAN / 512。 */
#define MOTOR_TEST_DUTY_BASE 1
#define MOTOR_TEST_DUTY_SPAN 3000

/* 10. 速度环整定参数（每个轮子一份 PIDInstance，先给同一组初值） */

#define VEL_KP 0.010f
#define VEL_KI 0.0020f
#define VEL_KD 0.0f

/* 积分项上限（占空比分数）。必须有，否则堵转/饱和后会堆积分然后“抽”一下。 */
#define VEL_INTEGRAL_LIMIT 0.5f

/* 输出侧低通（RC，秒）。0 = 不滤波。速度环输出已经比较干净，先不开。 */
#define VEL_OUTPUT_LPF_RC_S 0.0f

/* 11. 独立看门狗（IWDG） */

/* 直接操作寄存器而不引入 stm32f4xx_hal_iwdg.c，原因有两个： */
#define APP_IWDG_PRESCALER 6u /* PR[2:0]：6 → 分频 4 * 2^6 = 256 */
#define APP_IWDG_RELOAD 62u   /* RLR：约 0.50 s @ 32 kHz LSI */

/* IWDG 键寄存器（IWDG_KR）的魔术值。STM32 的设备头文件只定义了 IWDG_KR_KEY */
#define APP_IWDG_KEY_UNLOCK 0x5555u /* 解除 PR / RLR 的写保护 */
#define APP_IWDG_KEY_RELOAD 0xAAAAu /* 重装计数器（等价于 HAL_IWDG_Refresh()）*/
#define APP_IWDG_KEY_START 0xCCCCu  /* 启动看门狗（启动后无法停止）*/

/* 协议状态及 DMA 缓冲区放在 DMA 可访问的 SRAM，不能放入 CCM RAM。 */

/* STM32F407 的 CCM RAM：只挂在 CPU 的 D-bus 上，DMA 控制器无法访问。
 * 任何交给 DMA 的缓冲（本工程就是 srm_remote_t 里的 rx_buffer[256]）都必须
 * 落在 SRAM1（0x20000000 段，本工程的 RAM 区）。
 * 这两个常量用于运行期地址自检，见 remote.c 的 Remote_ProtocolInit()。 */
#define CCM_RAM_BASE 0x10000000u
#define CCM_RAM_SIZE 0x00010000u /* 64 KB */
#define CCM_RAM_END (CCM_RAM_BASE + CCM_RAM_SIZE)
/* 注意：名字不能叫 SRAM_BASE —— CMSIS 的 stm32f407xx.h 已经用了这个宏，
 * 重复定义会产生重定义警告。 */
#define APP_SRAM1_BASE 0x20000000u
#define APP_SRAM1_END 0x20020000u /* SRAM1 128 KB，见 STM32F407xx_FLASH.ld 的 RAM 区 */

/* 13. 编译期校验（配置写错 = 编译失败，而不是运行时静默出错） */

/* 头文件需要 C11 的 _Static_assert；cmake/stm32cubemx/CMakeLists.txt 已经强制要求 C11。 */
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
#define APP_STATIC_ASSERT(cond, msg) _Static_assert((cond), msg)
#else
#define APP_STATIC_ASSERT(cond, msg) \
    typedef char app_static_assert_##__LINE__[(cond) ? 1 : -1]
#endif

/* 检查队列配置；遥控结构体尺寸在 app_types.h 中校验。 */
APP_STATIC_ASSERT((Q_REMOTE_LEN > 0u), "Q_REMOTE_LEN must be > 0");

/* 复用上游协议失联超时，编译期校验配置一致性。 */
#define APP_SRM_TIMEOUT_MS 600u
APP_STATIC_ASSERT((REMOTE_TIMEOUT_MS == APP_SRM_TIMEOUT_MS),
                  "REMOTE_TIMEOUT_MS must match upstream SRM_REMOTE_CONTROL_TIMEOUT_MS");

/* 运动学与驱动配置的自检：写错这些在实车上表现为“车乱跑”，所以钉死在编译期。 */
APP_STATIC_ASSERT((MOTOR_PWM_FULL > 0u), "MOTOR_PWM_FULL must be > 0");
APP_STATIC_ASSERT((CHASSIS_K_MM > 0.0f), "CHASSIS_K_MM must be > 0");
APP_STATIC_ASSERT((MAX_VXY_MS > 0.0f), "MAX_VXY_MS must be > 0");
APP_STATIC_ASSERT((MAX_OMEGA_RADS > 0.0f), "MAX_OMEGA_RADS must be > 0");
APP_STATIC_ASSERT((MAX_ACC_MS2 > 0.0f), "MAX_ACC_MS2 must be > 0");
APP_STATIC_ASSERT((MAX_ALPHA_RADS2 > 0.0f), "MAX_ALPHA_RADS2 must be > 0");
APP_STATIC_ASSERT((WHEEL_MAX_SPEED_MMPS > 0.0f), "WHEEL_MAX_SPEED_MMPS must be > 0");
APP_STATIC_ASSERT((ENCODER_COUNT_PER_WHEEL_REV > 0.0f),
                  "ENCODER_COUNT_PER_WHEEL_REV must be > 0");
APP_STATIC_ASSERT((VEL_LOOP_PERIOD_MS > 0u), "VEL_LOOP_PERIOD_MS must be > 0");
APP_STATIC_ASSERT((VEL_INTEGRAL_LIMIT > 0.0f), "VEL_INTEGRAL_LIMIT must be > 0");
APP_STATIC_ASSERT((VEL_FEEDBACK_LPF_RC_S >= 0.0f), "VEL_FEEDBACK_LPF_RC_S must be >= 0");
APP_STATIC_ASSERT((REMOTE_DEADZONE >= 0), "REMOTE_DEADZONE must be >= 0");
APP_STATIC_ASSERT((REMOTE_DEADZONE < REMOTE_AXIS_FULL_SCALE),
                  "REMOTE_DEADZONE must be smaller than the axis full scale");
/* 四个轮的启动死区必须留出可用的线性区间，否则死区补偿会退化。 */
APP_STATIC_ASSERT((MOTOR_DEADBAND_FL < MOTOR_PWM_FULL), "MOTOR_DEADBAND_FL out of range");
APP_STATIC_ASSERT((MOTOR_DEADBAND_FR < MOTOR_PWM_FULL), "MOTOR_DEADBAND_FR out of range");
APP_STATIC_ASSERT((MOTOR_DEADBAND_RL < MOTOR_PWM_FULL), "MOTOR_DEADBAND_RL out of range");
APP_STATIC_ASSERT((MOTOR_DEADBAND_RR < MOTOR_PWM_FULL), "MOTOR_DEADBAND_RR out of range");

/* 14. 电池功率与电量监控（ADC3 + INA240A1 + SSD1306） */

/* PF3/ADC3_IN9 电流：1 mΩ + INA240A1，REF 接地；PF4/ADC3_IN14 电压：11:1 分压。 */

#define POWER_SHUNT_OHM 0.001f
#define POWER_AMP_GAIN 20.0f
#define POWER_AMPS_TO_VOLTS (POWER_SHUNT_OHM * POWER_AMP_GAIN)
#define POWER_CURRENT_BIDIRECTIONAL 0
#define POWER_DIVIDER_RATIO 11.0f
#define POWER_ADC_VREF 3.3f
#define POWER_ADC_FULL_SCALE 4095.0f
#define POWER_CURRENT_FLOOR_A 0.0f
#define POWER_ZERO_CAL_SAMPLES 100u
#define POWER_ZERO_CAL_AT_BOOT 0u
#define POWER_CURRENT_ZERO_V 0.0f
#define POWER_CURRENT_GAIN_CORRECTION 1.0f
#define POWER_ZERO_CAL_WINDOW_MS 200u
#define POWER_SAMPLE_STALE_MS 100u
#define POWER_SOC_REST_MS 3000u
#define POWER_SOC_STEP_MS 5000u
#define POWER_SOC_STABLE_BAND_V 0.05f
#define POWER_LOW_VOLTAGE_V POWER_V_EMPTY
#define POWER_LOW_VOLTAGE_CLEAR_V (POWER_V_EMPTY + 0.3f)
#define POWER_LOW_VOLTAGE_HOLD_MS 1000u
#define POWER_MEASURE_PRIORITY (PRIO_OLEDPOWER + 1u)
#define OLED_REFRESH_PERIOD_MS 200u
#define OLED_RETRY_PERIOD_MS 1000u
#define OLED_I2C_LOCK_TIMEOUT_MS 5u
#define POWER_SAMPLE_PERIOD_MS 10u
#define POWER_LPF_ALPHA 0.90f
#define POWER_V_FULL 12.6f
#define POWER_V_EMPTY 10.5f
#define POWER_SOC_UPDATE_MAX_A 0.5f
#define OLED_POWER_PERIOD_MS POWER_SAMPLE_PERIOD_MS
#define OLED_REFRESH_DIVIDER (OLED_REFRESH_PERIOD_MS / POWER_SAMPLE_PERIOD_MS)
#define STACK_OLEDPOWER_WORDS 768u
#define NAME_OLEDPOWER "OLEDPower"

/*  本节自检 */
APP_STATIC_ASSERT((POWER_SHUNT_OHM > 0.0f), "POWER_SHUNT_OHM must be > 0");
APP_STATIC_ASSERT((POWER_AMP_GAIN > 0.0f), "POWER_AMP_GAIN must be > 0");
APP_STATIC_ASSERT((POWER_DIVIDER_RATIO > 1.0f),
                  "POWER_DIVIDER_RATIO must be > 1 (it is a step-down divider)");
APP_STATIC_ASSERT((POWER_ADC_VREF > 0.0f), "POWER_ADC_VREF must be > 0");
APP_STATIC_ASSERT((POWER_ADC_FULL_SCALE > 0.0f), "POWER_ADC_FULL_SCALE must be > 0");
APP_STATIC_ASSERT((POWER_ZERO_CAL_SAMPLES > 0u), "POWER_ZERO_CAL_SAMPLES must be > 0");
APP_STATIC_ASSERT((POWER_SAMPLE_PERIOD_MS > 0u), "POWER_SAMPLE_PERIOD_MS must be > 0");
APP_STATIC_ASSERT((POWER_LPF_ALPHA > 0.0f) && (POWER_LPF_ALPHA < 1.0f),
                  "POWER_LPF_ALPHA must be in (0, 1) or the filter is unstable");
APP_STATIC_ASSERT((POWER_V_FULL > POWER_V_EMPTY), "POWER_V_FULL must be above POWER_V_EMPTY");
APP_STATIC_ASSERT((OLED_REFRESH_DIVIDER > 0u), "OLED_REFRESH_DIVIDER must be > 0");

#endif /* ROBO_APP_CONFIG_H */
