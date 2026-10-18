/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   全局编译期配置：所有魔数、任务参数、队列尺寸、机械参数、符号约定。
  *
  * 设计原则：
  *   1) 所有 .c 里不允许出现裸数字（魔数），一律用本文件的宏；
  *   2) 通过改这里的宏就能改变行为（尤其是 SIGN_* / *_DEADBAND），不用改逻辑；
  *   3) 编译期断言放在这里，配置写错时直接编译失败，而不是运行时静默出错。
  *
  * 本文件只有宏和断言，没有 .c，不需要参与编译单元。
  ******************************************************************************
  */

#ifndef ROBO_APP_CONFIG_H
#define ROBO_APP_CONFIG_H

#include <stdint.h>

/*============================================================================*/
/* 1. 时钟与节拍                                                               */
/*============================================================================*/

/* FreeRTOS 节拍 1 kHz（见 Core/Inc/FreeRTOSConfig.h 的 configTICK_RATE_HZ）。
 * 1 个 tick = 1 ms，因此“周期_ms”与“周期_tick”数值相等，代码里统一用 TickType_t。 */
#define APP_TICK_RATE_HZ 1000u

/* ===== 任务周期（毫秒）===== */
#define SAFETY_PERIOD_MS 10u   /* StartSafetyTask：喂狗 */
#define CHASSIS_PERIOD_MS 1u   /* StartChassisTask：读队列 / 速度环节拍 */
#define REMOTE_PERIOD_MS 1u    /* StartRemoteTask：SRM_Remote_Poll + 投队列 */
#define VEL_LOOP_PERIOD_MS 10u /* 速度环：100 Hz（理由见第 7 节的量化分析） */

/* ===== 任务栈深度（单位：字 = 4 字节，与 xTaskCreateStatic 的 usStackDepth 语义一致）===== */
#define STACK_SAFETY_WORDS 256u
#define STACK_CHASSIS_WORDS 512u
#define STACK_REMOTE_WORDS 512u

/* ===== 任务优先级（FreeRTOS 数值越大优先级越高，上限 configMAX_PRIORITIES = 56）===== */
#define PRIO_SAFETY 34u     /* 最高业务任务：喂狗必须最准时 */
#define PRIO_CHASSIS 32u    /* 速度环：必须在 OLED 之上（见下方说明） */
#define PRIO_REMOTE 26u     /* 蓝牙字节流解析、投队列 */
#define PRIO_OLEDPOWER 20u  /* 最低：OLED 刷屏是"有就行"的功能，不能挡路 */

/* ★ 关于 PRIO_OLEDPOWER 必须最低（踩过的坑）★
 * OLED 任务里有两处会长时间占住 CPU，必须让它们随时能被底盘任务抢占：
 *   1) StartOLEDPower 启动时调用 ssd1306_Init()，内部是**阻塞式**
 *      HAL_I2C_Mem_Write，超时参数是 HAL_MAX_DELAY —— OLED 没接好/不响应时会
 *      一直等 I2C；
 *   2) 每 100 ms 一次 oled_render()，同样是阻塞 I2C。
 * 一旦 OLED 任务的优先级高于 Chassis，上面任一处卡住就会把速度环彻底饿死：
 * 现象是"串口完全没输出、s_vel_tick 停住不动、电机不动"，而 Safety 任务
 * 因为优先级更高仍在喂狗，所以**看门狗也不会复位**，故障表现得很隐蔽。
 * 因此 OLED 必须排在 Remote(26) 之下。 */


/* ===== 任务名称（调试器里看到的字符串，长度受 configMAX_TASK_NAME_LEN = 16 限制）===== */
#define NAME_SAFETY "Safety"
#define NAME_CHASSIS "Chassis"
#define NAME_REMOTE "Remote"

/*============================================================================*/
/* 2. 蓝牙链路（JDY-31 透传模块，接 USART2）                                   */
/*============================================================================*/

/* 蓝牙模块（JDY-31）的 UART 波特率。
 *
 * ★★★ 这是**唯一真值来源**，改这里就够了 ★★★
 *   Core/Src/usart.c 的 USER CODE 区会在 MX_USART2_UART_Init() 末尾
 *   强制把 huart2 的波特率与 BRR 对齐到本宏：
 *       if (huart2.Init.BaudRate != BT_UART_BAUD) {
 *           huart2.Init.BaudRate = BT_UART_BAUD;
 *           huart2.Instance->BRR = ...;
 *       }
 *   所以在 CubeMX 里改 USART2.BaudRate、或直接改 usart.c:46 的 115200，
 *   **都会被这里拉回来**。要改波特率只能改这个宏。
 *
 * ★ 必须与 JDY-31 模块自己的 UART 波特率一致，否则收到的全是乱码 ★
 *   JDY-31 出厂默认 9600。SRM 参考固件（stm32f103_hal）与 SRM 库注释
 *   （srm_uart.h:18「115200 bps 下 256 字节…」）都按 115200 设计。
 *
 *   两种情况二选一，**两边必须同时是同一个值**：
 *     A) 模块保持出厂 9600        → 本宏写 9600u
 *     B) 模块用 AT 命令改成 115200 → 本宏写 115200u
 *
 *   用 AT 命令把 JDY-31 改成 115200 的方法（USB 转串口接模块，9600 8N1 进入）：
 *       AT+BAUD8        （8 = 115200；具体编号以模块手册为准）
 *       或 AT+BAUD=115200
 *   改完后模块重新上电，之后必须用 115200 与它通信。 */
#define BT_UART_BAUD 9600u

/* 蓝牙串口句柄：CubeMX 在 Core/Src/usart.c 中生成，C 文件里的变量名固定。 */
#define BT_UART_HANDLE huart2

/* DMA 环形接收缓冲大小（字节）。由上游 srm_uart.h 的 SRM_UART_RX_DMA_SIZE 决定，
 * srm_uart_init() 会校验“传进去的大小”必须等于该宏，这里只做一处引用和断言。 */
#define REMOTE_RX_DMA_SIZE 256u

/*============================================================================*/
/* 3. 调试串口（USART6，PG14=TX / PG9=RX，独立于蓝牙链路）                      */
/*============================================================================*/

/* Chassis 的 printf 走 USART6，不用蓝牙口：蓝牙链路是给协议用的，
 * 调试文本混进去会污染 ACK/ERROR 帧，也会被 App 当成噪声。
 *
 * 这里刻意使用 115200 而不是蓝牙的 9600：
 *   Chassis 每 100 ms 打印一行约 90 字节，若用 9600 bps，一行要占用约 94 ms 的
 *   发送时间，会把低优先级的 Remote 任务饿住并导致 DMA 环形缓冲被蓝牙数据冲掉。
 *   115200 下一行只需约 7.8 ms，配合 1 kHz 的 Remote 任务抢占，不会丢字节。 */
#define DEBUG_UART_BAUD 115200u
#define DEBUG_UART_INSTANCE USART6

/* printf 单字节写等待 TXE 的循环上限。防止 USART6 未使能（例如被 Ozone 复位过）
 * 时死等，配合 -O0 调试也足够宽松。 */
#define DEBUG_UART_TXE_TIMEOUT 200000u

/*============================================================================*/
/* 4. 队列（静态分配，不占用 FreeRTOS heap）                                   */
/*============================================================================*/

/* 队列深度：只要 2 就够。遥控链路的“丢旧语义”靠两点共同保证：
 *   - 发送端：队列满时 xQueueSend(..., 0) 直接失败，绝不阻塞在生产者里；
 *   - 接收端：xQueueReceive(..., 0) 非阻塞取，取空了就沿用上一帧。
 * 深度取 1 也不是不行（配 xQueueOverwrite），但 2 能在“发送端刚被抢占、
 * 还没来得及消费”的极短窗口里少丢一次数据，代价只有 56 字节 RAM。 */
#define Q_REMOTE_LEN 2u

/* 打印降频：每 N 个 Chassis 周期（1 ms）打印一次 = 10 Hz。
 * 不做内容去重的话，1 kHz 打印会把串口和 CPU 全部吃掉。 */
#define CHASSIS_PRINT_DIVIDER 100u

/* ★★★ 底盘任务串口打印总开关 ★★★
 *
 * 1 = 打印（正常调试底盘/PID 时用）
 * 0 = 关闭（**调蓝牙遥控时务必设 0**）
 *
 * 为什么调遥控时必须关：
 *   底盘的 3 行打印是 10 Hz、每轮约 270 字节，占用串口约 2700 B/s；
 *   而遥控任务的诊断信息只有 0.5 Hz、每行约 80 字节。
 *   两者共用同一个 printf → USART6（见 debug_uart.c 的重定向链路），
 *   底盘把带宽占满之后，遥控那几行（尤其是**开机只打一次**的
 *   `[remote] protocol init OK`）会被冲掉或者错过，
 *   表现为"看不到 remote 任务的串口输出"。
 *
 *   另外 printf 在 DebugUart_PutChar() 里是**阻塞等 TXE** 的：
 *   底盘一次打 3 行约占用 23 ms/100 ms，而 Chassis 优先级(32) 高于
 *   Remote(26)，所以这段时间遥控任务完全得不到执行，
 *   SRM_Remote_Poll() 也被推迟 —— 调蓝牙时更需要让出带宽和 CPU。 */
#define CHASSIS_PRINT_ENABLE 0u

/*============================================================================*/
/* 5. 失联保护                                                                */
/*============================================================================*/

/* 上游协议层（srm_remote.c）已经实现了“超过 600 ms 没有合法控制帧就自动以
 * 全零状态再次回调板级函数”的保护。本工程**不重复实现超时计时**，
 * 只把这个数值写在这里用于文档化，并断言它与上游宏一致（见第 13 节）。 */
#define REMOTE_TIMEOUT_MS 600u

/*============================================================================*/
/* 6. 机械参数（JGB37-520-0650-EN，12 V，减速比 30，无同步带直连轮）            */
/*============================================================================*/

/* 电机：减速箱输出轴空载 333 rpm / 额定负载 250 rpm（12 V） */
#define MOTOR_NO_LOAD_RPM 333.0f
#define MOTOR_RATED_RPM 250.0f

/* 麦轮外径 60 mm。轮周长 = π × 60 = 188.5 mm */
#define WHEEL_DIAMETER_MM 60.0f
#define WHEEL_PERIMETER_MM 188.4955592f /* π × 60 */

/* 编码器到轮轴的总机械减速比（**不是同步带比，是整条传动链的总比**）。
 * 无同步带、减速箱输出轴直连轮，但编码器装在**电机轴（减速箱之前）**，
 * 所以从编码器轴到轮轴仍然是 1:30 的总减速。
 *
 * 实测依据：手动把轮子转一整圈，s_enc[i].total_count 增加约 1560，
 * 与 13 PPR × 4 倍频 × 30 = 1560 完全吻合。改这个值前请重新实测。 */
#define DRIVE_RATIO 30.0f

/* 轮子最大线速度（mm/s）。空载 333 rpm → 188.5 mm/rev × 333/60 = 1046 mm/s。
 * 这个值同时用作“运动学输出的物理上限”，速度环的限幅用它。 */
#define WHEEL_MAX_SPEED_MMPS 1046.0f

/* 车体几何：L = 前后轴距 250 mm，W = 左右轮距 274 mm（你实测值）。
 * 麦轮逆运动学的 k = (L + W) / 2 = 262 mm：
 * k 是“纯自转时每个轮子的线速度 / 车体角速度”，单位 mm。 */
#define CHASSIS_WHEELBASE_MM 250.0f /* L */
#define CHASSIS_TRACK_MM 274.0f     /* W */
#define CHASSIS_K_MM ((CHASSIS_WHEELBASE_MM + CHASSIS_TRACK_MM) * 0.5f) /* = 262.0 mm */

/*============================================================================*/
/* 7. 编码器与测速                                                            */
/*============================================================================*/

/* 编码器：13 PPR 霍尔编码器，装在**电机轴**（减速箱之前）。
 * STM32 编码器接口按 TI12 双边沿计数 → 4 倍频：
 *   每电机转计数 = 13 × 4 = 52
 *   每轮转计数   = 52 × DRIVE_RATIO(30) = **1560**   ← 已用"手转一圈"实测确认
 * counts/mm = 1560 / 188.5 = 8.276 → 1 个计数 = 0.121 mm 位移
 *
 * ★ 测速方案与量化误差（这几个数是实车参数决定的，改参数后要重算）：
 *   满速 1046 mm/s = 5.55 转/秒 = 8657 counts/s，因此 10 ms 周期里有约 **87 个计数**，
 *   量化噪声地板是
 *     1 计数 / 10 ms = 100 counts/s = **12 mm/s**（仅满速的 1.1%）。
 *   这个分辨率很好，所以：
 *     - 速度环跑 100 Hz（10 ms）完全够用，不需要更长窗口；
 *     - 0.2 m/s 时仍有约 16.6 个计数（±6%），低速也可信；
 *     - Kp 可以给到 0.05 量级，不必因为噪声而压低带宽。
 *
 *   ⚠ 这一段曾经按"11 PPR × DRIVE_RATIO=1 → 44 counts/轮转"写过，那时算出的噪声
 *     地板是 ±428 mm/s，导致结论完全相反（"低速不可信、Kp 不要超 5"）。
 *     实测 1560 之后结论翻过来了——**改任何机械/编码器参数都要回来重算这段**。 */
#define ENCODER_PPR_MOTOR 13.0f
#define ENCODER_QUADRATURE 4.0f
#define ENCODER_COUNT_PER_MOTOR_REV (ENCODER_PPR_MOTOR * ENCODER_QUADRATURE)   /* 52 */
#define ENCODER_COUNT_PER_WHEEL_REV (ENCODER_COUNT_PER_MOTOR_REV * DRIVE_RATIO) /* 1560 */

/* counts/s → mm/s 的换算系数：
 *   mm_per_count = WHEEL_PERIMETER_MM / ENCODER_COUNT_PER_WHEEL_REV = 0.1208
 *   所以 1 counts/s = 0.1208 mm/s，反之 1 mm/s = 8.276 counts/s。
 *
 * ★★★ 这里曾经写成 WHEEL_PERIMETER_MM * 1000.0f / ... （多乘 1000 倍）★★★
 *   上游原来的量纲是"counts/s → mm/s"，而那个写法相当于把 1 个计数当成
 *   120.83 mm 而不是 0.12083 mm，**整条测速链被放大 1000 倍**。
 *   症状：编码器增量 13 个计数 / 10 ms（真实约 157 mm/s）被算成
 *         speed_mmps = 143264 mm/s（Ozone 实测值，与 13×120.83/0.01 完全吻合）。
 *   注意上面第 178 行的注释一直写的是正确的 0.1208 —— 代码和注释不一致，
 *   这种"注释对、代码错"的情况最容易蒙混过关，改参数时务必以公式为准。
 *
 *   物理校验（改完必须满足）：
 *     1 个计数 = 0.12083 mm
 *     13 个计数 / 10 ms = 157.08 mm/s
 *     满速 8657 counts/s × 0.12083 = 1046 mm/s = WHEEL_MAX_SPEED_MMPS ✔ */
#define ENCODER_COUNTS_TO_MMPS (WHEEL_PERIMETER_MM / ENCODER_COUNT_PER_WHEEL_REV)
#define MMPS_TO_ENCODER_COUNTS (ENCODER_COUNT_PER_WHEEL_REV / WHEEL_PERIMETER_MM)

/* 速度反馈一阶低通的 RC 时间常数（秒）。alpha = dt / (RC + dt)。
 * 与参考项目 PID_Freertos 的 Encode_Init(..., 0.0085f) 一致。
 * 10 ms 采样下 alpha = 0.54，等效截止约 19 Hz。
 * 滤波器只能平滑，不能消除量化噪声的地板（噪声由 ±1 计数决定），
 * 调大只会增加相位滞后、拖累 Kp。 */
#define VEL_FEEDBACK_LPF_RC_S 0.0085f

/*============================================================================*/
/* 7-b. 逐轮反馈使能（★没接编码器的轮子必须置 0★）                             */
/*============================================================================*/

/* ★ 为什么必须有这一组开关 ★
 *
 * 本工程有四个轮子，但**编码器可以一个一个接**（实物调试就是逐轮来的）。
 * 如果某一轮的编码器没接，`Encode_GetSpeedMmps()` 永远返回 0，于是：
 *
 *     Err = ref − 0 = ref          （恒为满目标值）
 *     Pout = Kp × Err              （例如 0.02 × 112.5 = 2.25）
 *     Output 被 MaxOut 钳到 ±1.0   →  duty = ±8400（满量程）
 *
 * 也就是**这一轮每周期都全速输出**，再叠加 motor_driver.c 的死区补偿跳变，
 * 表现就是"电机一直在颤抖、不转"。这不是 PID 参数问题，是反馈缺失。
 *
 * 对比作者另一个已跑通的项目 D:\STM32\PID_Freertos：它是**单电机 + 单编码器**，
 * `Encode_Init()` 只有一路，从来不存在"某一路反馈恒为 0"的情形，
 * 所以那边 Kp=50 也不会炸。本工程是四轮，必须显式处理"未接编码器"这个状态。
 *
 * 置 0 之后该轮**完全不输出**（不闭环、也不给开环前馈），避免它乱转干扰调试；
 * 想让它动，先把编码器接上再把对应的宏改成 1。
 *
 * 当前状态：只接了 M3（= RL）的编码器反馈，其余三轮保持 0。 */
#define VEL_USE_FB_FL 1u  /* M1 编码器：未接 → 不输出 */
#define VEL_USE_FB_FR 1u  /* M2 编码器：未接 → 不输出 */
#define VEL_USE_FB_RL 1u  /* M3 编码器：已接 → 闭环（当前唯一在调的轮子） */
#define VEL_USE_FB_RR 1u  /* M4 编码器：未接 → 不输出 */

/*============================================================================*/
/* 8. 运动学与整定参数                                                        */
/*============================================================================*/

/* 摇杆原始量纲：-512 .. 511 */
#define REMOTE_AXIS_MIN (-512)
#define REMOTE_AXIS_MAX (511)
#define REMOTE_AXIS_FULL_SCALE 512.0f

/* 摇杆死区（原始量纲）。摇杆回中不可能精确为 0，不死区车会自己爬。
 * 20 / 512 ≈ 3.9%。死区内的值直接当 0，不做“重映射拉伸”，
 * 这样刚出死区时出力是连续的（不会跳一下）。 */
#define REMOTE_DEADZONE 20

/* 车体速度上限：
 *   MAX_VXY_MS 取 0.9 而不是物理极限 1.046，是为了**保留转弯余量**：
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

/*============================================================================*/
/* 9. 电机驱动（TB6612FNG × 4）与每轮标定                                     */
/*============================================================================*/

/* PWM：TIM8_CH3/CH4 = M3/M4，TIM9_CH1/CH2 = M1/M2，Period = 8400-1 → 20 kHz。
 * 占空比以“计数”为单位，满量程 MOTOR_PWM_FULL = 8400。 */
#define MOTOR_PWM_FULL 8400u

/* TB6612 的 STBY 由硬件外接 3V3 常高，不需要软件控制。 */

/* ★ 每轮标定表（P1 开环标定要填的就是这里）★
 *
 * SIGN_*（取值 +1 / -1）三个符号是**独立**的，必须分别实测：
 *   - SIGN_DIR ：方向脚极性。TB6612 用 AIN1/AIN2 两个脚，正转时哪个为高要实测；
 *   - SIGN_PWM ：PWM 占空比符号与“该轮物理前向”的对应关系；
 *   - SIGN_ENC ：编码器计数增减与“该轮物理前向”的对应关系。
 * 三者都可能因为电机左右对称安装、接线顺序不同而各不相同。
 *
 * DEADBAND_PWM：电机启动死区（占空比计数）。这是麦轮直行不跑偏的关键，
 *   必须每轮单独实测：缓慢加占空比，记录轮子刚开始转动时的值。
 *   下面 1000/8400 ≈ 12% 只是估计值，P1 标定后必须替换。
 *
 * M1 = 左前 FL，M2 = 右前 FR，M3 = 左后 RL，M4 = 右后 RR
 * （顺序是你实车上认定的位置，标定时请核对，与 CubeMX 标签无关） */

/* ★★★ 闭环稳定的唯一条件：每轮 MOTOR_SIGN_PWM_* 与 MOTOR_SIGN_ENC_* 必须同号 ★★★
 *
 * 推导（只需两步）：
 *   motor_driver.c:180  `if (sign_pwm < 0) duty = -duty;` 然后按翻转后的 duty 定方向；
 *   encode.c            `signed_delta = (sign < 0) ? -delta : -delta 的相反数`。
 *
 *   · PWM 与 ENC 同号 → 该轮是负反馈：实测偏小 → 加大出力 → 收敛。**稳定**
 *   · PWM 与 ENC 反号 → 该轮是**正反馈**：实测偏小 → 反而减小出力 → 更偏 →
 *     输出在 ±MOTOR_PWM_FULL 之间来回翻转 → 表现为**电机来回震荡、不转**。
 *
 * 实测证据：本文件曾经是 FL: PWM=+1 / ENC=-1（四个轮子里唯一反号的一个），
 *   症状正是"电机抖动 + 缓慢转动、SPD 在正负之间交替"。
 *   四轮全部改成同号（都取 +1）后，四个轮子的反馈方向都统一成
 *   "正占空比 ⇒ 编码器读数变正"，闭环才是负反馈。
 *
 * 标定方法（用 MOTOR_TEST_MODE=1 的开环模式逐轮做，见第 10 节的说明）：
 *   1) 给一个固定正占空比，确认该轮**物理正转**；
 *   2) 同时看它的编码器读数：为正 → ENC 保持 +1；为负 → 把 ENC 改成 -1，
 *      并且**同时把 PWM 也改成 -1**（两者必须同号，改就一起改）。
 *   注意：这里说的"为正"是 relative 该轮的物理前进方向，不是车体前进方向 ——
 *   麦轮直行时左右两侧轮子的物理转向本来就相反，那是由运动学的目标值符号
 *   区分的（见 velocity_control.c 的 mecanum_inverse），与本组标定宏无关。 */
#define MOTOR_SIGN_DIR_FL (-1)
#define MOTOR_SIGN_PWM_FL (1)
#define MOTOR_SIGN_ENC_FL (1)   /* 曾经是 -1，与 PWM_FL=+1 反号 → 正反馈震荡，已修正 */
#define MOTOR_DEADBAND_FL 1000u

#define MOTOR_SIGN_DIR_FR (1)
#define MOTOR_SIGN_PWM_FR (1)
#define MOTOR_SIGN_ENC_FR (-1)
#define MOTOR_DEADBAND_FR 1000u

#define MOTOR_SIGN_DIR_RL (-1)
#define MOTOR_SIGN_PWM_RL (1)
#define MOTOR_SIGN_ENC_RL (-1)
#define MOTOR_DEADBAND_RL 1000u

#define MOTOR_SIGN_DIR_RR (-1)
#define MOTOR_SIGN_PWM_RR (1)
#define MOTOR_SIGN_ENC_RR (-1)
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

/*============================================================================*/
/* 10. 速度环整定参数（每个轮子一份 PIDInstance，先给同一组初值）               */
/*============================================================================*/

/* PI 初值。量纲：误差单位 mm/s，输出是占空比分数（-1..1）。
 *
 * ★ 这两个数是从作者已经跑通的项目 D:\STM32\PID_Freertos **换算**来的 ★
 *
 * 那个项目的速度环参数（motor.h:33-47）：
 *     .Kp = 50.0f,  .Ki = 1000.0f,  .Kd = 0.0f,  .MaxOut = PWM_LIMIT = 3599
 *     .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral |
 *                PID_Derivative_On_Measurement
 * 它的反馈量纲是 **RPM**（moter.c:58 传 Encode_GetSpeedRpm()），
 * 而本工程的反馈量纲是 **mm/s**。换算关系：
 *     1 轮转/s = 60 RPM = 188.4956 mm/s   →   1 RPM = 3.1416 mm/s
 *
 *   Kp: 50（每 RPM 给 50 个 PWM 计数）→ 每 mm/s 给 50/3.1416 = 15.92 计数
 *       → 换成占空比分数：15.92 / 3600 = 0.00442
 *   Ki: 1000 × Err(rpm) × dt = 1000 × (E_mmps/3.1416) × 0.01
 *       = 3.18 × E_mmps × dt  （PWM 计数）
 *       → 换成占空比分数：3.18 / 3600 = 0.000884
 *
 * 再考虑两点后取整：
 *   ① 本工程 Kd = 0（旧项目也是 0），所以不需要微分项；
 *   ② 保守起步：Kp 取换算值的约 2 倍（换算值偏小是因为旧项目的电机
 *      带 21.3:1 减速箱、转速范围只有 ±230 RPM，而本车 ±333 RPM 但轮径大、
 *      惯量大），Ki 取换算值附近。
 *
 * 调参顺序（每个轮子单独调，因为机械差异）：
 *   ① 先把 Kp 和 Ki **都置 0**，确认电机不抖、`SPD` 能跟着手动转动变化；
 *   ② 只加 Kp：从 0.004 开始每次 ×1.5，加到轮速能跟上指令、
 *      但开始明显抖动/啸叫为止，再退回上一档；
 *   ③ 再加 Ki：从 0.0008 开始往上加，直到稳态误差消失；
 *   ④ 低速段抖得厉害就先降 Kp。
 *
 * ⚠ 判据（量纲必须记牢）：Kp 的单位是"占空比分数 / (mm/s)"，
 *   所以 Kp = 0.005 时，200 mm/s 的误差才刚好对应满输出。
 *   历史上这里曾经是 3.0f（量纲错约 600 倍），误差 0.33 mm/s 就满输出，
 *   PID 退化成继电器，电机每 10 ms 正反翻转 —— 那是"电机颤抖"的原因之一。 */
#define VEL_KP 0.010f
#define VEL_KI 0.0010f
#define VEL_KD 0.0f

/* 积分项上限（占空比分数）。必须有，否则堵转/饱和后会堆积分然后“抽”一下。 */
#define VEL_INTEGRAL_LIMIT 0.5f

/* 输出侧低通（RC，秒）。0 = 不滤波。速度环输出已经比较干净，先不开。 */
#define VEL_OUTPUT_LPF_RC_S 0.0f

/*============================================================================*/
/* 11. 独立看门狗（IWDG）                                                      */
/*============================================================================*/

/* 直接操作寄存器而不引入 stm32f4xx_hal_iwdg.c，原因有两个：
 *   1) 当前 CubeMX 配置没有使能 HAL_IWDG_MODULE_ENABLED，也没有生成 IWDG 句柄；
 *   2) IWDG 一共只有两个寄存器（KR 键寄存器 + PR/RLR 预分频与重装值），
 *      直接用寄存器可以让 Core/Inc/stm32f4xx_hal_conf.h 和 cmake/stm32cubemx/
 *      CMakeLists.txt 保持 CubeMX 原样，重新生成代码不会把配置冲掉。
 *
 * 超时时间 = (4 * 2^PR) * RLR / f_LSI，f_LSI 典型值 32 kHz。
 * PR = 6 → 分频 4 * 2^6 = 256 → 计数器时钟 = 32 kHz / 256 = 125 Hz → 8 ms/计数。
 * RLR = 499 → 499 * 8 ms ≈ 4.0 s。
 * 取 4 s 而不是更短：Ozone 单步/断点会长时间停住 CPU，喂狗任务同样停住，
 * 超时太短会在调试时被反复复位。4 s 依然能兜住真正的死锁。 */
#define APP_IWDG_PRESCALER 6u /* PR[2:0]：6 → 分频 4 * 2^6 = 256 */
#define APP_IWDG_RELOAD 499u  /* RLR：约 4.0 s @ 32 kHz LSI */

/* IWDG 键寄存器（IWDG_KR）的魔术值。STM32 的设备头文件只定义了 IWDG_KR_KEY
 * 位域，没有给这几个键值起名字，所以在这里集中定义，避免 .c 里出现裸数字。
 * 取值来自参考手册 RM0090 的 IWDG 章节。
 *
 * ⚠ 命名必须带 APP_ 前缀：如果启用了 HAL_IWDG_MODULE_ENABLED，
 *   stm32f4xx_hal_iwdg.h 里已经定义了 IWDG_KEY_RELOAD 等宏，
 *   不带前缀会与之重定义冲突（-Wmacro-redefined 警告）。 */
#define APP_IWDG_KEY_UNLOCK 0x5555u /* 解除 PR / RLR 的写保护 */
#define APP_IWDG_KEY_RELOAD 0xAAAAu /* 重装计数器（等价于 HAL_IWDG_Refresh()）*/
#define APP_IWDG_KEY_START 0xCCCCu  /* 启动看门狗（启动后无法停止）*/

/*============================================================================*/
/* 12. 内存布局常量（CCM RAM 红线）                                            */
/*============================================================================*/

/* STM32F407 的 CCM RAM：只挂在 CPU 的 D-bus 上，**DMA 控制器无法访问**。
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

/*============================================================================*/
/* 13. 编译期校验（配置写错 = 编译失败，而不是运行时静默出错）                   */
/*============================================================================*/

/* 头文件需要 C11 的 _Static_assert；cmake/stm32cubemx/CMakeLists.txt 已经强制要求 C11。 */
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
#define APP_STATIC_ASSERT(cond, msg) _Static_assert((cond), msg)
#else
#define APP_STATIC_ASSERT(cond, msg) \
    typedef char app_static_assert_##__LINE__[(cond) ? 1 : -1]
#endif

/* 队列元素必须是 4 字节的整数倍：FreeRTOS 按字（4 字节）对齐拷贝队列元素，
 * 加入填充字节后若总长不是 4 的倍数，元素之间会出现错位，收到的是垃圾数据。
 * remote_in_t 的尺寸断言写在 app_types.h（那里能看到结构体定义）。 */
APP_STATIC_ASSERT((Q_REMOTE_LEN > 0u), "Q_REMOTE_LEN must be > 0");

/* 本工程的失联超时值必须与上游协议层的宏一致，防止两边被改成不同数值。 */
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

/*============================================================================*/
/* 14. 电池功率与电量监控（ADC3 + INA240A1 + SSD1306）                          */
/*============================================================================*/

/* 本节全部是 OLED 数显模块（user/OLEDpower.c）的编译期参数。
 * 与第 1~13 节的底盘/遥控参数完全无关，改动本节不会影响运动学、速度环、
 * 编码器或蓝牙协议解析。 */

/* 硬件链条（已实测确认；换硬件时只改本节前面几个宏，算法不用动）：
 *
 *   电池 ──[1 mΩ 采样电阻]── 负载
 *              │
 *         INA240A1（增益 20 V/V，REF1 = REF2 = GND → **单向**检测）
 *              │  输出 0 V(0 A) … 3.3 V(165 A)，零点无偏置
 *           PF3 = ADC3_IN9      → 电流通道（规则组 rank1）
 *
 *   电池 ──[100 kΩ ±5%]──┬──[10 kΩ ±1%]── GND
 *                        │
 *                    PF4 = ADC3_IN14 → 电压通道（规则组 rank2，分压比 11 : 1）
 *
 * ★ REF 接地带来的三个后果（与常见 VCC/2 双向接法不同，务必理解）：
 *   1) 零电流时运放输出 ≈ 0 V，不是 1.65 V，所以电流**恒为非负**；
 *      代码仍按带符号处理并做下限幅；
 *   2) 运放输出贴近地轨时线性度变差（输出下限约 10 mV → 折合 0.5 A），
 *      因此 0.5 A 以下不保证精度，也不参与电量更新；
 *   3) 满量程 = 3.3 V / 0.02 V/A = 165 A，远超实际电流，无饱和风险。 */

#define POWER_SHUNT_OHM 0.001f /* 采样电阻 1 mΩ（硬件不可改，已确认） */
#define POWER_AMP_GAIN 20.0f   /* INA240A1 增益 20 V/V */
/* 采样电阻 + 运放合并后的转换系数：I[A] = (V_adc − V_zero) / POWER_AMPS_TO_VOLTS */
#define POWER_AMPS_TO_VOLTS (POWER_SHUNT_OHM * POWER_AMP_GAIN) /* = 0.02 V/A */

/* 1 = 双向（REF 接 VCC/2），0 = 单向（REF 接 GND，本工程当前硬件）。
 * 置 0 时负电流被限幅到 POWER_CURRENT_FLOOR_A。 */
#define POWER_CURRENT_BIDIRECTIONAL 0

/* 分压比 = (100k + 10k) / 10k = 11.0。
 * ⚠ 精度提醒：100 kΩ 是 ±5%、10 kΩ 是 ±1%，最坏情况下分压比落在 10.52…11.61
 * （约 ±5.5% 误差），已经吃掉一半的 ±10% 指标预算。后续实测修正时直接改这个
 * 数字（例如实测比值 11.08 就写 11.08f），不要改公式。 */
#define POWER_DIVIDER_RATIO 11.0f

/* ADC 参考电压。暂按 3.3 V；实测 VREF+ 后改成实测值（例如 3.287f）。
 * 它是全链路的比例因子，偏 3% 则电压/电流/功率一起偏 3%。 */
#define POWER_ADC_VREF 3.3f
#define POWER_ADC_FULL_SCALE 4095.0f /* 12 位，右对齐 */

/* 电流下限（A）。单向检测没有负电流，低于 0 一律归零。 */
#define POWER_CURRENT_FLOOR_A 0.0f

/* 零点校准采样次数：在调度器启动前（PWM 从未输出过，物理上必然零电流）
 * 连续采样求平均赋给 zero_offset_v。100 次求平均把量化噪声压到 0.1 LSB 量级。 */
#define POWER_ZERO_CAL_SAMPLES 100u

/* 过采样窗长（ms）。ADC3 在 DMA 循环模式下自由运行，采样率由 adc.c 的配置决定：
 *     f_ADCCLK = PCLK2 / 6 = 84 MHz / 6 = 14 MHz
 *     单通道转换 = 144(采样) + 12(12 位逐次逼近) = 156 个 ADC 周期 ≈ 11.1 µs
 *     一轮扫描 = 2 个通道 ≈ 22.3 µs  →  **约 45 kHz**，DMA 完成中断同频
 * 也就是说 10 ms 窗内约有 450 轮转换（900 个采样点）。
 * DMA 完成中断里只做整数累加、不做任何浮点，任务每 10 ms 取一次均值，
 * 量化噪声按 √450 ≈ 21 倍下降 —— 这是把「1 LSB = 40 mA」这个硬分辨率救回来的
 * 关键手段，也是守住 ±10% 的必要条件。 */
#define POWER_SAMPLE_PERIOD_MS 10u

/* 一阶低通「守旧系数」（0~1，越大越平滑）。等价时间常数：
 *     τ = dt × alpha / (1 − alpha)，alpha = 0.90、dt = 10 ms → τ ≈ 90 ms
 * 注意 dt 是 10 ms，所以 τ 是**毫秒**量级，不是"90 秒"。
 * alpha 再往上加（0.99 → τ ≈ 1 s）会让电量显示严重滞后，不建议。 */
#define POWER_LPF_ALPHA 0.90f

/* 电量百分比映射端点（3S 锂电池：满 12.6 V / 空 10.5 V）。
 * ⚠ 这是很粗的模型：锂电放电曲线中段平坦，线性映射误差远大于显示精度，
 * 它只能当"还能不能跑"的粗略指示，不是真实 SOC。要更准需库仑积分或分段查表。 */
#define POWER_V_FULL 12.6f
#define POWER_V_EMPTY 10.5f

/* 只有在电流小于该值时才更新电量百分比：急加速时电池被拉低，
 * 此时若更新，电量会跟着油门一起狂掉。0.5 A 同时是运放近地轨的精度下限。 */
#define POWER_SOC_UPDATE_MAX_A 0.5f

/* ── OLED 任务参数 ────────────────────────────────────────────────────────── */
#define OLED_POWER_PERIOD_MS POWER_SAMPLE_PERIOD_MS /* 10 ms 严格周期 */

/* 刷屏分频：10 次 × 10 ms = 100 ms 刷一次（10 Hz）。
 * ★ 分频改变不了"那一次刷屏"的耗时：ssd1306_UpdateScreen() 要在 400 kHz 下
 *   发 1024 字节数据 + 24 字节命令，实测约 25 ms 阻塞。期间 vTaskDelayUntil
 *   会顺延 2~3 个 tick。这是把 I2C 屏塞进 10 ms 任务的固有代价，换来的是
 *   "不丢采样"——采样由 DMA + 中断累加，任务晚一点取均值不影响结果。 */
#define OLED_REFRESH_DIVIDER 10u

/* 任务栈（字）与优先级。任务用 xTaskCreateStatic 静态创建（不占 FreeRTOS heap）。
 *
 * ★ PRIO_OLEDPOWER 由本节统一规定，是**全系统最低**的优先级。
 *   原因（踩过的坑）：OLED 任务里有两处会长时间占住 CPU，必须让它们随时能被
 *   底盘任务抢占：
 *     1) StartOLEDPower 启动时调用 ssd1306_Init()，内部是**阻塞式**
 *        HAL_I2C_Mem_Write，超时参数是 HAL_MAX_DELAY —— OLED 没接好/不响应
 *        时会一直等 I2C；
 *     2) 每 100 ms 一次 oled_render()，同样是阻塞 I2C。
 *   一旦 OLED 优先级高于 Chassis，上面任一处卡住就会把速度环彻底饿死：
 *   现象是"串口完全没输出、s_vel_tick 停住不动、电机不动"，而 Safety 任务
 *   因为优先级更高仍在喂狗，所以**看门狗也不会复位**，故障表现得非常隐蔽。
 *
 *   定义只在下面这一处，数值见第 1 节（PRIO_OLEDPOWER = 20）。
 *   注意"数值越大优先级越高"：20 低于 Remote(26)，因此 OLED 不会挡住任何任务。 */
#define STACK_OLEDPOWER_WORDS 768u
#define NAME_OLEDPOWER "OLEDPower"

/* ── 本节自检 ─────────────────────────────────────────────────────────────── */
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
