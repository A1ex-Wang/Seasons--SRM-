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
  * @brief I2C1 总线互斥锁（OLED 与"以后挂在 I2C1 上的任何设备"共用）。
  *
  * 实测本工程 IMU 挂在 I2C2（PB10/PB11，100 kHz），与 OLED 的 I2C1 互相独立，
  * 当前并不存在争用；但按要求仍然保留这把锁，作为 I2C1 上再加设备时的唯一入口。
  *
  * ★ 使用注意（一定要看）：拿锁期间不要做别的事。
  *   ssd1306_UpdateScreen() 要在 400 kHz 下连续发 1024 字节数据 + 24 字节命令，
  *   实测约 **25 ms** 阻塞，这段时间锁一直被持有。任何新加的 I2C1 代码必须：
  *
  *       xSemaphoreTake(g_i2c1_mutex, portMAX_DELAY);
  *       ... 一次完整的 I2C 事务 ...
  *       xSemaphoreGive(g_i2c1_mutex);
  *
  *   即"一次事务 = 一次拿锁"。绝不能只拿锁写半帧就松开，否则 OLED 会花屏；
  *   也不能在拿锁期间 vTaskDelay 或调 HAL_Delay，否则等于把总线锁死了。
  */
extern SemaphoreHandle_t g_i2c1_mutex;

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
  * 任务启动时会在临界区内创建 g_i2c1_mutex，并从任务体内调用 ssd1306_Init()。
  *
  * @param argument 未使用
  */
void StartOLEDPower(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* ROBO_OLEDPOWER_H */
