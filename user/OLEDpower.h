/**
  ******************************************************************************
  * @file    OLEDpower.h
  * @brief   电池功率与电量监控：ADC3(INA240 电流 + 分压电压) → SSD1306 OLED。
  *
  * 数据链（硬件已实测确认，参数宏全部在 user/app_config.h 第 14 节）：
  *
  *   PF3 = ADC3_IN9  ← INA240A1(增益 20，REF=GND 单向) ← 1 mΩ 采样电阻
  *   PF4 = ADC3_IN14 ← 100 kΩ : 10 kΩ 分压（11 : 1）
  *
  *   ADC3 扫描模式 + 连续转换 + DMA2_Stream0 循环搬运
  *        ↓ 每完成一轮序列触发一次 DMA 完成中断（约 45 kHz）
  *   HAL_ADC_ConvCpltCallback 里**只做整数累加**（见 OLEDpower.c）
  *        ↓ 任务每 10 ms 取一次均值（10 ms 窗约 450 轮转换）
  *   零点校准 → 一阶低通 → 电压 / 电流 / 功率 / 电量
  *        ↓ 分频 10（100 ms 一次）
  *   SSD1306（I2C1 @400 kHz，地址 0x3C）四行显示
  *
  * ★ 调用顺序要求（写错就全是 0）：
  *
  *     App_Init()  ──►  OLEDPower_Init()      ← 必须在 osKernelStart() 之前！
  *                      （启动 ADC + DMA、做零点校准）
  *     osKernelStart()
  *        └─ OLEDPowerTask  ──►  StartOLEDPower()   ← 任务体，只管换算与刷屏
  *
  *   为什么必须先初始化：
  *     1) ADC 必须有人调用 HAL_ADC_Start_DMA()。CubeMX 只生成了配置函数
  *        MX_ADC3_Init()，**工程里没有任何地方启动转换**，不启动则 DMA 缓冲区
  *        恒为 0，电压/电流/功率/电量全为 0，而且不会报任何错；
  *     2) 零点校准要在"电机肯定没转"的时候做。TB6612 的 STBY 是硬件接 3V3、
  *        没有 GPIO 可控（见 app_config.h 第 9 节），无法靠拉低 STBY 保证，
  *        因此改为在调度器启动前采样 —— 此刻 Chassis 任务还没被创建、PWM 从未
  *        输出过，物理上必然是零电流，比软件控制 STBY 更可靠。
  ******************************************************************************
  */

#ifndef ROBO_OLEDPOWER_H
#define ROBO_OLEDPOWER_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "app_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* 对外可见的测量结果                                                          */
/*============================================================================*/

/**
  * @brief 电池功率监控的最新结果快照。
  *
  * 全部为 volatile：写侧是 OLEDPower 任务（10 ms 周期），读侧可以是遥测任务或
  * 调试打印。多任务只读不会出问题；写侧只有 OLEDPower 一个，因此无需加锁。
  *
  * 单位与符号约定：
  *   - current_a：放电为正。当前硬件 REF1/REF2 接 GND（单向检测），所以不会出现
  *     负值；代码仍做了下限幅，以后把 REF 改接 VCC/2 只需把 app_config.h 里的
  *     POWER_CURRENT_BIDIRECTIONAL 改成 1；
  *   - battery_v：**已经过滤波**的电池电压（τ ≈ 90 ms），电量百分比由它算出；
  *   - ratio_pct：0..100 的整数百分比（12.6 V~10.5 V 线性映射，见 app_config.h）。
  */
typedef struct {
    volatile float current_a;   /**< 电池电流，A（放电为正） */
    volatile float battery_v;   /**< 电池电压（低通后），V */
    volatile float power_w;     /**< 电池功率，W = battery_v × current_a */
    volatile uint8_t ratio_pct; /**< 剩余电量 0..100 %（仅在电流 < 0.5 A 时更新） */
} power_monitor_t;

/** @brief 最新测量结果（供遥测 / 调试打印只读）。 */
extern power_monitor_t g_power;

/*============================================================================*/
/* I2C 总线互斥锁                                                              */
/*============================================================================*/

/**
  * @brief OLED 所在 I2C2 的总线互斥锁。
  *
  * OLED 挂在 I2C2（PB10 = SCL / PB11 = SDA，400 kHz）。
  * 当前 I2C2 上只有 OLED 一个设备（user/sensorimu.c 仍是空壳），所以并不存在争用；
  * 但按要求仍然保留这把锁，作为"以后往 I2C2 上再加设备"时的唯一入口。
  *
  * ⚠ 若以后真的把 IMU 挂到 I2C2，**必须复用这把锁**，否则两条任务同时访问
  *   I2C2 会互相打断（表现为 NACK、花屏、或 IMU 数据错乱）。
  *
  * ★ 使用注意（一定要看）：拿锁期间不要做别的事。
  *   ssd1306_UpdateScreen() 要在 400 kHz 下连续发 1024 字节数据 + 24 字节命令，
  *   实测约 25 ms 阻塞，这段时间锁一直被持有。任何新加的 I2C2 代码必须遵循
  *   "一次事务 = 一次拿锁"：
  *       拿锁 → 完成一次完整的 I2C 事务 → 放锁
  *   绝不能只拿锁写半帧就松开，否则 OLED 会花屏；
  *   也不能在拿锁期间 vTaskDelay 或调 HAL_Delay，否则等于把总线锁死了。
  */
extern SemaphoreHandle_t g_oled_i2c_mutex;

/*============================================================================*/
/* 接口                                                                        */
/*============================================================================*/

/**
  * @brief 功率监控的**阻塞式**初始化：启动 ADC + DMA、做零点校准、创建任务。
  *
  * 必须在 osKernelStart() 之前调用（本工程从 App_Init() 里调用，也可以放在
  * main.c 的 USER CODE 2 区）。内部用忙等采样而不是 vTaskDelay，
  * 因为此刻调度器还没跑起来，FreeRTOS 的延时不可用。
  *
  * 完成后：
  *   - ADC3 在 DMA 循环模式下以约 45 kHz 自由运行，缓冲区持续刷新；
  *   - HAL_ADC_ConvCpltCallback 里的累加器开始工作（采样不依赖任务调度）；
  *   - 电流零点 zero_offset_v 由 POWER_ZERO_CAL_SAMPLES 次采样求平均得到；
  *   - OLEDPower 任务已用 xTaskCreateStatic 建好（不占 FreeRTOS heap）。
  *
  * 注意：本函数**不会**初始化 OLED 屏。ssd1306_Init() 由任务体自己调用，
  * 因为它内部有 HAL_Delay(100) 和几十条 I2C 命令，放在任务里才能被调试器
  * 与看门狗正常观察到。
  */
void OLEDPower_Init(void);

/**
  * @brief OLEDPower 任务体：10 ms 严格周期采样换算，100 ms 分频刷屏。
  *
  * 本函数用**强定义**覆盖 Core/Src/freertos.c 里 CubeMX 生成的同名 __weak 空壳
  * （与 StartSafetyTask / StartChassisTask / StartRemoteTask 同样的做法），
  * 因此不需要修改任何 CubeMX 生成的文件。任务由 OLEDPower_Init() 用
  * xTaskCreateStatic 创建，优先级与栈大小取 app_config.h 里的
  * PRIO_OLEDPOWER / STACK_OLEDPOWER_WORDS。
  *
  * 任务启动时会在临界区内创建 g_oled_i2c_mutex，并从任务体内调用 ssd1306_Init()。
  *
  * @param argument 未使用
  */
void StartOLEDPower(void *argument);

/*============================================================================*/
/* 遥测接口（只读，调电压/电流采样专用）                                       */
/*============================================================================*/
/*
 * ★ 为什么需要这组函数 ★
 *
 * 真正决定"采样对不对"的几个量（原始 ADC 码、零点、每周期样本数、ADC 启动状态）
 * 都是 OLEDpower.c 里的 static 变量，Ozone 里看不到；而 s_* 前缀的静态量一旦改了
 * 名字或布局，watch 表达式就失效。这组函数返回标量、参数是编译期常量，
 * 直接写函数名求值即可，不受调试信息影响。
 *
 * ★ 排障顺序（电压采样不对时按这个顺序看）★
 *
 *   1) Power_TelemAdcStatus()  必须 = 0（HAL_OK）
 *        ≠ 0 → HAL_ADC_Start_DMA() 失败，整个采样从未启动（此时下面全是 0）
 *
 *   2) Power_TelemSampleCount()  每 10 ms 周期的样本数
 *        = 0        → DMA 没在跑（或 ADC 中断没进）
 *        ≈ 670      正常（67 kHz × 10 ms；若为 450 左右说明 ADC 时钟更慢，也正常）
 *        远小于预期 → 任务被饿死，或 ADC 停了
 *
 *   3) Power_TelemRawVoltageCode() / Power_TelemRawCurrentCode()  原始 12 位码
 *        电压通道：12.6 V 电池经 11:1 分压 → 1.145 V → 约 1421 码
 *                  10.5 V → 0.955 V → 约 1185 码
 *        电流通道：零电流时约等于零点码；满量程 20 A → 0.4 V → 约 496 码
 *        若是 0 或 4095 卡住 → 引脚没接 / 分压电阻缺失 / ADC 通道号错
 *
 *   4) Power_TelemBatteryMv()  换算后的电压（mV）
 *        与第 3 步的码值必须自洽：mV ≈ code / 4095 × 3300 × 11
 *        不自洽说明 POWER_DIVIDER_RATIO 或 POWER_ADC_VREF 配错
 *
 *   5) Power_TelemZeroOffsetMv()  电流零点（mV）
 *        正常应接近 0（REF 接 GND）。若明显偏大（如 >100 mV）→ REF 接线有问题
 */

/** @brief ADC 启动状态：0 = HAL_OK，非 0 = 启动失败（此时所有采样值恒为 0）。 */
int32_t Power_TelemAdcStatus(void);

/** @brief 上一个 10 ms 周期内累计的样本轮数（0 = DMA 没在跑）。 */
uint32_t Power_TelemSampleCount(void);

/** @brief 电压通道上一周期的过采样均值（12 位原始码，0..4095）。 */
uint32_t Power_TelemRawVoltageCode(void);

/** @brief 电流通道上一周期的过采样均值（12 位原始码，0..4095）。 */
uint32_t Power_TelemRawCurrentCode(void);

/** @brief 电流通道零点电压，单位 mV（由 OLEDPower_Init 的校准得到）。 */
int32_t Power_TelemZeroOffsetMv(void);

/** @brief 电池电压（低通后），单位 mV。 */
int32_t Power_TelemBatteryMv(void);

/** @brief 电池电流（低通后），单位 mA（放电为正）。 */
int32_t Power_TelemCurrentMa(void);

/** @brief 电池功率，单位 mW。 */
int32_t Power_TelemPowerMw(void);


/** @brief 电压通道（PF4/ADC3_IN14）**瞬时**原始码，用来判断引脚上到底是什么电平。 */
uint32_t Power_TelemRawDmaVoltageCode(void);

/** @brief 电流通道（PF3/ADC3_IN9）**瞬时**原始码。 */
uint32_t Power_TelemRawDmaCurrentCode(void);

/**
  * @brief 本任务循环的真实周期（ms），期望恒等于 OLED_POWER_PERIOD_MS(10)。
  *
  * 远大于 10 说明任务被阻塞了。历史上就是这个问题：
  * user/ssd1306/ssd1306.c 的 I2C 写用 HAL_MAX_DELAY（无限超时），
  * 屏不在线时一次 ssd1306_UpdateScreen()（32 笔事务）约 772 ms，
  * 把 10 ms 采样周期拖成约 862 ms，遥测里每周期样本数从 449 变成 38715。
  */
uint32_t Power_TelemLoopPeriodMs(void);

/** @brief OLED 屏探测状态：1 = 在线（在刷屏），2 = 离线（已禁用刷屏）。 */
uint32_t Power_TelemOledState(void);

/** @brief 剩余电量百分比 0..100。 */
uint32_t Power_TelemSocPct(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBO_OLEDPOWER_H */
