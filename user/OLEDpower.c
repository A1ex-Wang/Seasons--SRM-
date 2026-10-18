/**
  ******************************************************************************
  * @file    OLEDpower.c
  * @brief   电池功率与电量监控实现。设计说明、调用顺序、口径约定见 OLEDpower.h。
  *
  * 责任分工（别搞混）：
  *
  *   中断上下文（DMA 完成回调，约 45 kHz）：
  *       只做整数累加。**不取锁、不刷屏、不做浮点、不调任何阻塞 API**。
  *       正因为采样由中断累加、不依赖任务调度，10 ms 任务周期被刷屏拖慢也不会
  *       丢样本 —— 这是本模块守住 ±10% 的关键，也是"任务里直接读内存全局变量"
  *       这一要求在本工程最稳妥的实现方式。
  *
  *   任务上下文（OLEDPower 任务，10 ms 周期）：
  *       取累加器均值 → 换算物理量 → 一阶低通 → 电量百分比 → 每 100 ms 刷屏。
  *
  *   调度器启动前（OLEDPower_Init）：
  *       启动 ADC + DMA，做零点校准。
  ******************************************************************************
  */

#include "OLEDpower.h"

/* ── 标准库 ───────────────────────────────────────────────────────────────── */
#include <stdio.h>
#include <string.h>

/* ── 本工程 ───────────────────────────────────────────────────────────────── */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cmsis_os.h"

#include "adc.h"   /* hadc3 */
#include "dma.h"   /* MX_DMA_Init 已在 main.c 里调用，DMA 控制器时钟已开 */
#include "i2c.h"   /* hi2c2：SSD1306 的 I2C 端口（PB10=SCL / PB11=SDA） */

#include "ssd1306.h"
#include "ssd1306_fonts.h"

/*============================================================================*/
/* 编译期约束                                                                  */
/*============================================================================*/

#if defined(SSD1306_USE_SPI)
#error "本模块按 I2C 接口编写（ssd1306_conf.h 里的 SSD1306_USE_I2C）。改用 SPI 屏请同步修改本文件。"
#endif

#if !defined(SSD1306_INCLUDE_FONT_11x18)
#error "ssd1306_conf.h 必须打开 SSD1306_INCLUDE_FONT_11x18：四行数据用的就是这套字体。"
#endif

/* ADC 规则组序列长度必须与 Core/Src/adc.c 的 NbrOfConversion 一致，
 * 顺序（rank1 → rank2）也必须与下面两个下标一一对应：
 *      rank1 = ADC_CHANNEL_9  (PF3) → 电流
 *      rank2 = ADC_CHANNEL_14 (PF4) → 电压
 * 下标写错不会报错，只会让电压显示成电流、电流显示成电压。 */
#define ADC_CH_COUNT 2u
#define ADC_CH_CURRENT 0u
#define ADC_CH_VOLTAGE 1u

/*============================================================================*/
/* ADC + DMA 原始数据                                                          */
/*============================================================================*/

/**
  * @brief DMA 目标缓冲：一轮序列 2 个半字，[0] = 电流通道，[1] = 电压通道。
  *
  * ★ 两个硬性要求（违反其一就是"全是 0"或者随机数）：
  *   1) 必须按半字对齐。DMA 配的是 MEMALIGN_HALFWORD。这里额外做了 4 字节对齐，
  *      以后若改成 WORD 传输也不用动；
  *   2) 必须落在 SRAM，不能进 CCM RAM。F407 的 CCM RAM（0x10000000）只挂在 CPU
  *      D-bus 上，**DMA 控制器访问不到**（见 app_config.h 第 12 节的红线）。
  *      本变量是普通 static，链接脚本把它放进 0x20000000 段的 RAM，安全。
  *
  * 本变量不加 volatile：DMA 在硬件层写内存，编译器看不到别名，但这份内存的
  * "新鲜度"由 HAL_ADC_ConvCpltCallback 触发的中断负责说明；
  * 真正跨上下文共享的是下面的累加器（那个才是 volatile）。
  */
static uint16_t g_adc_dma[ADC_CH_COUNT] __attribute__((aligned(4)));

/**
  * @brief 过采样累加器：中断里写，任务里读。
  *
  * uint32 累加不会溢出：12 位满量程 0x0FFF × 0x00FFFFFE 次 ≈ 2^38，
  * 而 10 ms 窗只有约 450 轮（见 app_config.h 的 POWER_SAMPLE_PERIOD_MS），
  * 余量 6 个数量级（另有 add_sample_to_accumulator 里的溢出保护）。
  */
static volatile uint32_t s_acc_sum[ADC_CH_COUNT];
static volatile uint32_t s_acc_cnt;
static volatile float s_zero_offset_v; /**< 电流通道零点电压（V），由校准得到 */

/*============================================================================*/
/* 输出与内部状态                                                              */
/*============================================================================*/

power_monitor_t g_power;        /* 对外快照，定义见 OLEDpower.h */
SemaphoreHandle_t g_oled_i2c_mutex; /* OLED 所在 I2C2 的总线互斥锁，定义见 OLEDpower.h */

/* 任务本体、静态栈与任务控制块。
 * 为什么用 xTaskCreateStatic 而不是 CubeMX 里那个 osThreadNew：
 *   1) CubeMX 为 OLEDPowerTask 生成的是**动态**创建（osThreadNew + heap_4），
 *      而本工程的约定是"任务一律静态分配、不占 FreeRTOS heap"
 *      （见 app_types.h 里对 configSUPPORT_STATIC_ALLOCATION 的断言说明）。
 *   2) 静态创建不会失败、不需要在调度器启动前屏蔽中断去分配内存。
 * 三个变量必须放在文件作用域：任务永不退出，栈的生命周期必须等于程序。 */
/* ★★ 这里曾经有 s_oled_tcb / s_oled_stack / s_oled_task 三个静态缓冲，已全部删除 ★★
 *
 * Robo.ioc 的 FREERTOS.Tasks01 里 OLEDPowerTask 是 Dynamic，MX_FREERTOS_Init()
 * 已经用 osThreadNew 创建过一次。本文件曾经再用 xTaskCreateStatic 建第二次，
 * 于是**两个 OLED 任务实例同时在跑**，而它们共用同一个 I2C2 与同一个
 * g_oled_i2c_mutex：两个实例各建一次互斥锁（第二次覆盖全局句柄）、刷屏互相打断
 * → 屏幕撕裂、I2C 报错，并且白吃一份 CPU（每次刷屏约 25 ms 阻塞）。
 *
 * 这和 Chassis / Safety / Remote 是同一个 bug（详见 user/remote.c 的 App_Init()
 * 注释）。现在统一交给 CubeMX 创建，优先级在任务体开头用 vTaskPrioritySet()
 * 强制成 app_config.h 的约定值。 */

/* 一阶低通的内部状态。只有 OLEDPower 任务会碰，不需要 volatile。 */
static float s_filtered_v;  /**< 电压低通状态 */
static float s_filtered_i;  /**< 电流低通状态 */
static uint8_t s_ratio_pct; /**< 上次更新到的电量百分比（大电流时保持不变） */

/*============================================================================*/
/* 遥测快照（只读，调试电压/电流采样专用；不参与任何控制计算）                  */
/*============================================================================*/

/**
  * @brief ADC 启动结果（HAL_ADC_Start_DMA 的返回值）。
  * 0 = HAL_OK。非 0 说明采样从未启动，此时下面所有量都恒为 0 —— 排障第一站。
  */
static HAL_StatusTypeDef s_adc_st;

/** @brief 上一个 10 ms 周期累计的样本轮数。0 = DMA 没在跑。 */
static volatile uint32_t s_telem_n;

/** @brief 电压通道过采样均值（12 位原始码）。 */
static volatile uint32_t s_telem_raw_v;

/** @brief 电流通道过采样均值（12 位原始码）。 */
static volatile uint32_t s_telem_raw_i;

/**
  * @brief 瞬时 DMA 缓冲值（每周期刷新，不看平均值，用来判断"引脚上到底是什么电平"）。
  * [0] = rank1 = ADC_CHANNEL_9（PF3，电流通道）
  * [1] = rank2 = ADC_CHANNEL_14（PF4，电压通道）
  */
static volatile uint32_t s_telem_dma_i;
static volatile uint32_t s_telem_dma_v;

/** @brief 上一次任务循环的实际周期（ms）。用来发现"10 ms 被拖长了"。 */
static volatile uint32_t s_telem_period_ms;

/**
  * @brief OLED 屏探测状态：0 = 未探测，1 = 在线（正常刷屏），2 = 离线（跳过刷屏）。
  *
  * ★★ 为什么必须探测（本工程踩过的坑）★★
  *
  *   user/ssd1306/ssd1306.c 里所有 I2C 写都是
  *       HAL_I2C_Mem_Write(..., HAL_MAX_DELAY);      // 无限超时
  *   而 ssd1306_UpdateScreen() 一次要发 32 笔事务（24 条命令 + 8 页数据）。
  *   **OLED 没接或不应答时，每一笔都在等超时**，一次刷屏实测约 772 ms，
  *   把本任务 10 ms 的采样周期拖成约 862 ms。
  *
  *   后果（实测）：遥测里每周期样本数 n = 38715，
  *   而理论上 10 ms 只有 44900 × 0.01 = 449 个。也就是说
  *   "10 ms 过采样"实际上变成了"862 ms 过采样" —— 电压/电流的低通时间常数、
  *   电量更新的节拍全部被拉长 86 倍。
  *
  *   本任务优先级是最低的 PRIO_OLEDPOWER(20)，所以底盘/遥控不受影响；
  *   但采样节拍被破坏，调电压时数据会失真。
  *
  *   对策：刷屏前用 HAL_I2C_IsDeviceReady() 做一次**有界超时**的探测
  *   （2 次尝试 × 5 ms），不应答就直接跳过这一帧。探测本身耗时 <10 ms，
  *   远小于原来的 772 ms，采样周期立刻回到 10 ms。
  */
static volatile uint8_t s_oled_state;

/** @brief 屏探测参数：尝试次数 / 每次超时（ms）。都是小数，探测开销可忽略。 */
#define OLED_PROBE_TRIALS 2u
#define OLED_PROBE_TIMEOUT_MS 5u

/**
  * @brief 调试打印开关（1 = 每 POWER_DEBUG_PRINT_PERIOD_MS 打一行到 USART6）。
  *
  * ★ 当前值为 1（打开）—— 正处在调电压采样的阶段 ★
  *   调完电压后建议改成 0，让调试串口恢复干净。
  *
  * 为什么打开它不会干扰遥控/底盘：
  *   1) 打印走 printf → USART6（见 debug_uart.c），与蓝牙口 USART2 **物理隔离**，
  *      不会污染 SRM 帧；
  *   2) 频率只有 2 Hz（约 110 字节/次 ≈ 220 B/s），而底盘那 3 行 10 Hz 约
  *      2700 B/s —— 差一个数量级；
  *   3) 本任务优先级 PRIO_OLEDPOWER(20) **低于** Chassis(32)/Remote(26)/Safety(34)，
  *      打印期间随时被它们抢占，不会拖慢速度环或蓝牙解析。
  *
  * 可用编译选项覆盖：-DPOWER_DEBUG_PRINT=0 */
#ifndef POWER_DEBUG_PRINT
#define POWER_DEBUG_PRINT 1
#endif

/** @brief 调试打印周期（ms）。2 Hz 足够看趋势，又不会刷屏。 */
#define POWER_DEBUG_PRINT_PERIOD_MS 500u

/*============================================================================*/
/* 临界区                                                                      */
/*============================================================================*/

/* 用 portENTER/EXIT_CRITICAL 而不是 vTaskEnter/EXIT_CRITICAL：后者在临界区内还会
 * 屏蔽 configMAX_SYSCALL_INTERRUPT_PRIORITY 以上的中断（把靠 FreeRTOS API 唤醒的
 * 中断一并挂起），对本模块没有必要。这里只需要挡住"DMA 完成中断去改累加器"。
 * 临界区长度必须是常数个周期：只做 4 次 volatile 访问，不会干扰 10 ms 节拍。 */
#define POWER_ENTER_CRITICAL() portENTER_CRITICAL()
#define POWER_EXIT_CRITICAL()  portEXIT_CRITICAL()

/*============================================================================*/
/* 中断上下文：纯整数累加                                                      */
/*============================================================================*/

/**
  * @brief 把 DMA 缓冲里刚完成的一轮序列累加到过采样累加器。
  *
  * 只在 DMA 完成中断里被调用，约 45 kHz，**必须保持极短**：
  * 2 次数组索引 + 2 次读取 + 3 次加减法，没有除法、没有浮点、没有函数调用。
  */
static void add_sample_to_accumulator(void)
{
    if (s_acc_cnt >= 0x00FFFFFEu) {
        /* 极端情况（例如任务优先级被降到极低、连续很久没被调度）下保护累加器，
         * 丢掉当前样本，等任务把累加器消费掉。 */
        return;
    }
    s_acc_sum[ADC_CH_CURRENT] += (uint32_t)g_adc_dma[ADC_CH_CURRENT];
    s_acc_sum[ADC_CH_VOLTAGE] += (uint32_t)g_adc_dma[ADC_CH_VOLTAGE];
    s_acc_cnt++;
}

/**
  * @brief ADC 规则组转换完成回调（覆盖 HAL 的 __weak 空实现）。
  *
  * 调用链（已逐行核对 stm32f4xx_hal_adc.c 的 ADC_DMAConvCplt）：
  *     DMA2_Stream0 传输完成中断
  *       → HAL_DMA_IRQHandler(&hdma_adc3)        （Core/Src/stm32f4xx_it.c）
  *         → ADC_DMAConvCplt()
  *           → HAL_ADC_ConvCpltCallback()        ← 就是本函数
  *
  * 中断优先级是 5（Core/Src/dma.c 设置），等于
  * configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY，也就是说这里**可以**调
  * FromISR 版 API。但本函数刻意什么都不调。
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    /* 显式过滤实例：本工程只启动了 ADC3。留着这一句，是为了以后有人再加
     * ADC1/ADC2 的 DMA 时，别的通道数据不会混进本模块的累加器。 */
    if (hadc->Instance != ADC3) {
        return;
    }
    add_sample_to_accumulator();
}

/*============================================================================*/
/* 累加器消费（任务上下文）                                                    */
/*============================================================================*/

/**
  * @brief 原子地取走累加器内容并清零。返回 0 表示这 10 ms 内一个新样本都没有。
  *
  * 为什么要在临界区里做：累加是"读-改-写"，若中断恰好插在"读计数"和"清零"
  * 之间，那一次样本会被清零动作抹掉（只丢 1 个样本、影响可忽略，但没必要留洞）。
  */
static uint32_t take_accumulator(uint32_t *sum_current, uint32_t *sum_voltage)
{
    uint32_t cnt;

    POWER_ENTER_CRITICAL();
    *sum_current = s_acc_sum[ADC_CH_CURRENT];
    *sum_voltage = s_acc_sum[ADC_CH_VOLTAGE];
    cnt = s_acc_cnt;
    s_acc_sum[ADC_CH_CURRENT] = 0u;
    s_acc_sum[ADC_CH_VOLTAGE] = 0u;
    s_acc_cnt = 0u;
    POWER_EXIT_CRITICAL();

    return cnt;
}

/*============================================================================*/
/* 物理量换算                                                                  */
/*============================================================================*/

/** @brief ADC 原始码 → 电压（V）。12 位右对齐，参考电压取 POWER_ADC_VREF。 */
static float raw_to_volts(uint32_t raw)
{
    return ((float)raw / POWER_ADC_FULL_SCALE) * POWER_ADC_VREF;
}

/**
  * @brief 过采样均值 → 电压（V）。
  *
  * 单独写一个 float 版本而不是复用 raw_to_volts((uint32_t)mean)：均值的整数部分
  * 已经包含了过采样的全部小数位，强转成 uint32 再传进去虽然结果正确（值域远小于
  * 2^24，float 表示精确），但意图含混，以后有人把窗长改大就可能踩到精度边界。
  */
static float mean_to_volts(float mean_raw)
{
    return (mean_raw / POWER_ADC_FULL_SCALE) * POWER_ADC_VREF;
}

/**
  * @brief 10 ms 一次的测量更新：取均值 → 换算 → 一阶低通 → 电量。
  *
  * 精度说明（为什么必须做过采样）：
  *   1 mΩ × 增益 20 = 0.02 V/A，而 1 LSB = 3.3 V / 4095 = 0.806 mV，
  *   折合**每个 LSB 就是 40 mA**。若单次采样直接换算，电流只能精确到 0.04 A
  *   一档。本模块在 10 ms 内平均约 450 轮转换（见 app_config.h 的推导），
  *   量化噪声按 √450 ≈ 21 倍下降，等效分辨率进入 mA 量级 —— 这是守住 ±10% 的
  *   必要条件。（若平均的是"被 PWM 开关噪声污染"的样本，均值下降的是随机噪声；
  *   与 PWM 同步的确定性干扰靠 144 周期积分采样 + 后级低通一起压。）
  *
  * 电量防跳变的两道措施：
  *   ① 电压先过一阶低通（α = 0.9，τ ≈ 90 ms）；
  *   ② 只有电流小于 POWER_SOC_UPDATE_MAX_A(0.5 A) 时才更新百分比，
  *      即"大电流期间电量数字冻结"，避免油门一大电量就狂掉。
  */
static void power_measurement_update(void)
{
    uint32_t sum_i;
    uint32_t sum_v;
    uint32_t n;
    float mean_i_raw;
    float mean_v_raw;
    float v_battery_raw;
    float current_a;
    float power_w;
    float min_current;

    n = take_accumulator(&sum_i, &sum_v);

    /* 遥测：样本数与原始码先记下来（即使 n==0 也要记录，便于看出 DMA 停了）。 */
    s_telem_n = n;

    /* ── 还没有样本：保持上一次结果，绝不把 0 灌进滤波器 ────────────────────
     * 只在启动头几毫秒（DMA 刚启动）或 DMA 被停掉时出现。此时若用 0 更新低通，
     * 滤波状态会被拉坏，要几百毫秒才能恢复。 */
    if (n == 0u) {
        return;
    }

    mean_i_raw = (float)sum_i / (float)n;
    mean_v_raw = (float)sum_v / (float)n;

    /* 遥测：把过采样均值存成整数原始码（四舍五入），供 Ozone / 串口直接看。
     * 用整数是为了避免在 Ozone 里看到 "2047.9998" 这种浮点精度噪声。 */
    s_telem_raw_i = (uint32_t)(mean_i_raw + 0.5f);
    s_telem_raw_v = (uint32_t)(mean_v_raw + 0.5f);

    /* 遥测：瞬时 DMA 值。【用它判断"引脚上到底是什么电平"】
     * rawI/rawV 是 10 ms 内的均值，适合看稳定性；
     * dmaI/dmaV 是"此刻"那一轮转换的结果，适合排查"是不是某一通道恒 0/恒满"。 */
    s_telem_dma_i = (uint32_t)g_adc_dma[ADC_CH_CURRENT];
    s_telem_dma_v = (uint32_t)g_adc_dma[ADC_CH_VOLTAGE];

    /* ── 电流 ───────────────────────────────────────────────────────────────
     * I[A] = (V_adc − V_zero) / (R_shunt × Gain) = (V_adc − V_zero) / 0.02
     * 零点由 OLEDPower_Init() 实测得到。减法顺序不能反。 */
    current_a = (mean_to_volts(mean_i_raw) - s_zero_offset_v) / POWER_AMPS_TO_VOLTS;

    /* 单向检测（REF 接 GND，本工程当前硬件）时负值纯属噪声，限到 0；
     * 双向检测时保留下限幅但保留正负号。 */
#if (POWER_CURRENT_BIDIRECTIONAL == 1)
    min_current = POWER_CURRENT_FLOOR_A;
#else
    min_current = 0.0f;
#endif
    if (current_a < min_current) {
        current_a = min_current;
    }

    /* ── 电压 ───────────────────────────────────────────────────────────────
     * V_bat = V_adc × 分压比(11 : 1)。分压比与参考电压都在 app_config.h 里，
     * 实测修正时只改宏，不要改这里。 */
    v_battery_raw = mean_to_volts(mean_v_raw) * POWER_DIVIDER_RATIO;

    /* ── 一阶低通 ─────────────────────────────────────────────────────────
     * s = α·s + (1−α)·new，α = POWER_LPF_ALPHA = 0.9，dt = 10 ms → τ ≈ 90 ms。 */
    s_filtered_v = (POWER_LPF_ALPHA * s_filtered_v) + ((1.0f - POWER_LPF_ALPHA) * v_battery_raw);
    s_filtered_i = (POWER_LPF_ALPHA * s_filtered_i) + ((1.0f - POWER_LPF_ALPHA) * current_a);

    /* ── 功率：用滤波后的电压 × 滤波后的电流，避免两个噪声相乘被放大 ────── */
    power_w = s_filtered_v * s_filtered_i;

    /* ── 电量百分比：仅在电流小时更新（防跳变措施②） ─────────────────────── */
    if (s_filtered_i < POWER_SOC_UPDATE_MAX_A) {
        float pct = (s_filtered_v - POWER_V_EMPTY) / (POWER_V_FULL - POWER_V_EMPTY) * 100.0f;

        if (pct > 100.0f) {
            pct = 100.0f;
        } else if (pct < 0.0f) {
            pct = 0.0f;
        }
        s_ratio_pct = (uint8_t)pct;
    }

    g_power.battery_v = s_filtered_v;
    g_power.current_a = s_filtered_i;
    g_power.power_w = power_w;
    g_power.ratio_pct = s_ratio_pct;
}

/*============================================================================*/
/* OLED 显示                                                                   */
/*============================================================================*/

/* ★★★ 显示布局：4 行还是 3 行（本工程踩过的坑）★★★
 *
 * ssd1306_WriteChar()（user/ssd1306/ssd1306.c:231）在画任何像素之前有一条硬检查：
 *     if (SSD1306_WIDTH < (SSD1306.CurrentX + char_width) ||
 *         SSD1306_HEIGHT < (SSD1306.CurrentY + Font.height))
 *         return 0;                      // ← 整行直接放弃，一个像素都不画
 *
 * 屏高 SSD1306_HEIGHT = 64，于是：
 *
 *   「Font_11x18（高 18），行距 18，四行 y = 0/18/36/54」
 *       第 1 行 0+18  = 18 ≤ 64  ✔
 *       第 2 行 18+18 = 36 ≤ 64  ✔
 *       第 3 行 36+18 = 54 ≤ 64  ✔
 *       第 4 行 54+18 = 72 > 64  ✘ **整行不画** → 屏上只有三行
 *     （实测现象：V/I/P 有，SOC 完全不见。
 *       注意它**不是**"只显示上半部分"——在 WriteChar 里就 return 了，
 *       根本走不到 DrawPixel 的边界裁剪。旧注释写错了这一点。）
 *
 *   「Font_7x10（高 10），行距 16，四行 y = 0/16/32/48」
 *       最后一行 48+10 = 58 ≤ 64  ✔ 四行全部完整可见
 *
 * 所以：
 *   OLED_SOC_ON_4TH_LINE = 1（默认）→ 四行 Font_7x10，V/I/P/SOC 都看得见，字较小
 *   OLED_SOC_ON_4TH_LINE = 0        → 三行 Font_11x18，V/I/P 字号大很多，但 SOC 看不见 */
#ifndef OLED_SOC_ON_4TH_LINE
#define OLED_SOC_ON_4TH_LINE 1
#endif

/* Font_11x18 的行距（三行布局用） */
#define OLED_LINE_HEIGHT 18u

/* Font_7x10 的行距（四行布局用）。取 16 而不是 10，是为了在 10 px 字高之间
 * 留 6 px 空隙，看起来不挤；48 + 10 = 58 ≤ 64，四行仍安全。 */
#define OLED_LINE_PITCH_7x10 16u

/* ★ 行宽核算 ★
 *   Font_11x18：11 px/字符 → 每行最多 11 字符（128 / 11 = 11.6）
 *   Font_7x10 ： 7 px/字符 → 每行最多 18 字符（128 / 7  = 18.3）
 *   本模块实际使用的最长行是 "I:+20.00 A" = 10 字符：
 *       11x18 → 110 px ✔ 刚好放得下
 *        7x10 →  70 px ✔ 余量很大
 *   OLED_LABEL_COMPACT 的意义：在 11x18 下必须缩短标签，否则会裁尾。 */
#ifndef OLED_LABEL_COMPACT
#define OLED_LABEL_COMPACT 1
#endif

#if (OLED_LABEL_COMPACT == 1)
/* 紧凑标签：最宽 "V:xx.xx V" = 9 字符 = 99 px，留足宽度。 */
#define OLED_LABEL_VOLTAGE "V:"
#define OLED_LABEL_CURRENT "I:"
#define OLED_LABEL_POWER   "P:"
#define OLED_LABEL_BATTERY "SOC:"
#else
/* 字面格式（会裁字符，见上面的核算）。 */
#define OLED_LABEL_VOLTAGE "Voltage:"
#define OLED_LABEL_CURRENT "Current:"
#define OLED_LABEL_POWER   "Power:"
#define OLED_LABEL_BATTERY "Battery:"
#endif

/**
  * @brief 显示一个浮点数（自己实现的替代品）。
  *
  * ★ 为什么需要它：afiskon 的 stm32-ssd1306 库里**没有** ssd1306_WriteFloat()。
  *   已逐行核对 ssd1306.h 的声明区，API 只有 WriteChar / WriteString / DrawXXX
  *   这一套。所以按"绝不调用不存在的 API"的要求，这里写一个本地实现。
  *
  * ★ 为什么不用 sprintf("%f")：newlib 的 %f 会拉起整套浮点格式化代码
  *   （几 KB Flash + 上百字节栈），是本工程明确要避开的坑。本函数改成纯整数算法：
  *       ① 先把浮点数缩放到百分位（× 100）；
  *       ② 四舍五入成整数（正数加 0.5、负数减 0.5，不依赖 libm 的 roundf）；
  *       ③ 用整数除法 / 取模拆出整数部分与小数部分；
  *       ④ 最后只输出整数（%ld），全程不触发浮点格式化代码，栈占用几十字节。
  *
  * @param x     左上角 x 坐标（像素）
  * @param y     左上角 y 坐标（像素），需与字体行高对齐
  * @param value 待显示的值，固定两位小数，例如 -1.23 / 12.60
  * @param font  字体，本模块统一用 Font_11x18
  * @param color White / Black
  */
static void OLED_ShowFloat(uint8_t x, uint8_t y, float value, SSD1306_Font_t font, SSD1306_COLOR color)
{
    char buf[24];
    int32_t scaled; /* value × 100，单位是"百分位" */
    int32_t whole;
    int32_t frac;
    const char *sign = "";

    /* ① + ② 缩放到百分位并四舍五入 */
    if (value >= 0.0f) {
        scaled = (int32_t)((value * 100.0f) + 0.5f);
    } else {
        scaled = (int32_t)((value * 100.0f) - 0.5f);
    }

    /* 限幅：整数部分只保留两位（±99.99），防止坐标/缓冲区越界。
     * 本工程正常量程是 10.5~12.6 V、0~20 A、0~260 W，远碰不到这个边界。 */
    if (scaled > 9999) {
        scaled = 9999;
    } else if (scaled < -9999) {
        scaled = -9999;
    }

    /* ③ 拆整数与小数部分。负数取模结果仍为负，所以先取绝对值再拆。 */
    if (scaled < 0) {
        sign = "-";
        scaled = -scaled;
    }
    whole = scaled / 100;
    frac = scaled % 100;

    /* ④ 只格式化整数 */
    (void)snprintf(buf, sizeof(buf), "%s%ld.%02ld", sign, (long)whole, (long)frac);

    /* ⑤ 定位光标再写。
     * ★ 这一步不能省：库的 ssd1306_WriteString() 没有坐标参数，它从 SSD1306
     *   内部的 CurrentX/CurrentY 接着写，所以必须先 SetCursor，
     *   否则会从上次写到的位置继续输出（表现为文字乱串）。 */
    ssd1306_SetCursor(x, y);

    /* ssd1306_WriteString() 的形参不是 const char*，需要显式去 const */
    (void)ssd1306_WriteString(buf, font, color);
}

/**
  * @brief 把 float 格式化成"两位小数"的文本，**纯整数算法**。
  *
  * ★★ 为什么不能用 snprintf("%.2f")（本工程实测过的坑）★★
  *
  *   本工程链接的是 **newlib-nano**，它的 printf/snprintf 默认**不带浮点支持**。
  *   遇到 `%.2f` 不报错、也不打印 `%f` 字面量，而是**输出空字符串**。
  *
  *   后果（实测现象）：屏幕上只剩标签和单位，数值是空的 ——
  *       "V:%.2f V"  →  "V: V"
  *       "I:%+.2f A" →  "I: A"
  *       "P:%.2f W"  →  "P: W"
  *   这正是"显示屏只有单位、没有具体数值"的原因。串口那边的 `[power]` 行是
  *   整数格式（%ld），所以照常显示 —— 这也解释了为什么串口正常、屏不正常。
  *
  *   两种修法：
  *     a) 链接时加 `-u _printf_float`：能用 %f，但要多占 3~6 KB Flash；
  *     b) **整数算法自格式化**（本函数采用，与 OLED_ShowFloat 完全同一套做法）：
  *        ① ×100 缩放到百分位；② 四舍五入成整数；
  *        ③ 整数除法/取模拆出整数位与小数位；④ 只用 %ld 输出。
  *   选 b：Flash 零代价，行为确定，不依赖链接选项。
  *
  * @param buf        输出缓冲区（建议 ≥12 字节）
  * @param cap        缓冲区大小
  * @param value      待格式化的值
  * @param force_sign 1 = 非负数也带 '+'（用于电流方向）；0 = 只有负数带 '-'
  */
static void fmt_f2(char *buf, size_t cap, float value, uint8_t force_sign)
{
    int32_t scaled;
    int32_t whole;
    int32_t frac;
    uint8_t neg = 0u;

    if ((buf == NULL) || (cap == 0u)) {
        return;
    }

    /* ① + ② 缩放到百分位并四舍五入（正数加 0.5、负数减 0.5） */
    if (value >= 0.0f) {
        scaled = (int32_t)((value * 100.0f) + 0.5f);
    } else {
        scaled = (int32_t)((value * 100.0f) - 0.5f);
    }

    /* 限幅：整数部分最多 4 位（±9999.99），防止缓冲区越界。
     * 本工程正常量程是 10.5~12.6 V、0~20 A、0~260 W，远碰不到。 */
    if (scaled > 999999) {
        scaled = 999999;
    } else if (scaled < -999999) {
        scaled = -999999;
    }

    if (scaled < 0) {
        neg = 1u;
        scaled = -scaled;
    }
    whole = scaled / 100;
    frac = scaled % 100;

    /* ③ 只输出整数格式，绝不出现 %f */
    if (neg != 0u) {
        (void)snprintf(buf, cap, "-%ld.%02ld", (long)whole, (long)frac);
    } else if (force_sign != 0u) {
        (void)snprintf(buf, cap, "+%ld.%02ld", (long)whole, (long)frac);
    } else {
        (void)snprintf(buf, cap, "%ld.%02ld", (long)whole, (long)frac);
    }
}

/**
  * @brief 刷一帧：清缓冲 → 画四行 → 推屏。
  *
  * ★ 调用者必须已持有 g_oled_i2c_mutex。
  * ★ 耗时：ssd1306_UpdateScreen() 在 400 kHz 上发 1024 字节数据 + 24 字节命令，
  *   实测约 25 ms（库内是阻塞式 HAL_I2C_Mem_Write，超时参数 HAL_MAX_DELAY）。
  * ★ 所有浮点都用 fmt_f2() 转成文本，**这里不允许出现 "%.2f"**
  *   （newlib-nano 无浮点 printf 支持，会输出空字符串 → 屏上只剩单位）。
  *
  * ★★ 为什么用 Font_7x10 而不是 Font_11x18（本工程踩过的坑）★★
  *
  *   ssd1306_WriteChar()（ssd1306.c:231）在画任何像素之前有一条硬检查：
  *       if (... || SSD1306_HEIGHT < (SSD1306.CurrentY + Font.height)) return 0;
  *
  *   屏高 SSD1306_HEIGHT = 64。
  *     · Font_11x18 高 18：前三行 y=0/18/36 → 18/36/54 都 ≤ 64 ✔
  *       **第四行 y=54 → 54+18 = 72 > 64 → 整行直接 return，一个像素都不画。**
  *       （实测现象：屏上只有三行，SOC 完全不见。）
  *       注意这不是"只显示上半部分"——它根本走不到 DrawPixel 的边界裁剪。
  *     · Font_7x10 高 10，行距取 16：y=0/16/32/48 → 最多 58 ≤ 64 ✔ 四行全可见。
  *
  *   若想让字号更大，把 OLED_SOC_ON_4TH_LINE 设为 0：退回"三行 Font_11x18"，
  *   V/I/P 会明显更大，但 SOC 显示不出来（屏高不够第四行 18 px）。
  */
static void oled_render(void)
{
    char num[12];    /* 单个数值，如 "-12.34" */
    char line[28];   /* 一整行，如 "V:11.91 V" */
    /* 快照：g_power 是 volatile，先取到本地再格式化，避免读到"半新半旧"的组合 */
    float volt = g_power.battery_v;
    float curr = g_power.current_a;
    float powr = g_power.power_w;
    unsigned pct = (unsigned)g_power.ratio_pct;

#if (OLED_SOC_ON_4TH_LINE != 0)
    /* ── 四行布局（Font_7x10，行距 16，占 58/64 px）────────────────────────
     * 行宽：7 px/字符、屏宽 128 px → 每行最多 18 字符。
     *   "V:11.91 V" = 9 字符 = 63 px  ✔ 余量充足 */
    ssd1306_Fill(Black);

    /* 第一行：电压 */
    fmt_f2(num, sizeof(num), volt, 0u);
    (void)snprintf(line, sizeof(line), OLED_LABEL_VOLTAGE "%s V", num);
    ssd1306_SetCursor(0u, 0u);
    (void)ssd1306_WriteString(line, Font_7x10, White);

    /* 第二行：电流（强制带符号，看清方向） */
    fmt_f2(num, sizeof(num), curr, 1u);
    (void)snprintf(line, sizeof(line), OLED_LABEL_CURRENT "%s A", num);
    ssd1306_SetCursor(0u, OLED_LINE_PITCH_7x10);
    (void)ssd1306_WriteString(line, Font_7x10, White);

    /* 第三行：功率 */
    fmt_f2(num, sizeof(num), powr, 0u);
    (void)snprintf(line, sizeof(line), OLED_LABEL_POWER "%s W", num);
    ssd1306_SetCursor(0u, 2u * OLED_LINE_PITCH_7x10);
    (void)ssd1306_WriteString(line, Font_7x10, White);

    /* 第四行：剩余电量（本来就是整数，%3u 在 newlib-nano 下完全正常） */
    (void)snprintf(line, sizeof(line), OLED_LABEL_BATTERY "%3u%%", pct);
    ssd1306_SetCursor(0u, 3u * OLED_LINE_PITCH_7x10);
    (void)ssd1306_WriteString(line, Font_7x10, White);

#else
    /* ── 三行大字号布局（Font_11x18，行距 18，占 54/64 px）──────────────
     * 字号明显更大、更好看，但屏高 64 px 放不下第四行 18 px 的字，
     * 所以 **SOC 无法显示**（原因见上面的函数注释）。 */
    ssd1306_Fill(Black);

    fmt_f2(num, sizeof(num), volt, 0u);
    (void)snprintf(line, sizeof(line), OLED_LABEL_VOLTAGE "%s V", num);
    ssd1306_SetCursor(0u, 0u);
    (void)ssd1306_WriteString(line, Font_11x18, White);

    fmt_f2(num, sizeof(num), curr, 1u);
    (void)snprintf(line, sizeof(line), OLED_LABEL_CURRENT "%s A", num);
    ssd1306_SetCursor(0u, 1u * OLED_LINE_HEIGHT);
    (void)ssd1306_WriteString(line, Font_11x18, White);

    fmt_f2(num, sizeof(num), powr, 0u);
    (void)snprintf(line, sizeof(line), OLED_LABEL_POWER "%s W", num);
    ssd1306_SetCursor(0u, 2u * OLED_LINE_HEIGHT);
    (void)ssd1306_WriteString(line, Font_11x18, White);
#endif /* OLED_SOC_ON_4TH_LINE */

    /* 推屏（唯一真正走 I2C 的动作） */
    ssd1306_UpdateScreen();
}

/**
  * @brief 上电自检画面。调用者必须已持有 g_oled_i2c_mutex。
  *
  * 把实测到的零点电压直接显示出来：这是现场判断"校准是否成功"的唯一直观依据。
  * REF1/REF2 接 GND 时这里应该是几毫伏；若显示 0.8 V 以上，说明 REF 实际接的
  * 不是地（例如误接 VCC/2），此时电流读数会整体偏掉。
  */
static void oled_show_banner(void)
{
    ssd1306_Fill(Black);

    ssd1306_SetCursor(0u, 0u);
    (void)ssd1306_WriteString("Power Monitor", Font_11x18, White);

    /* 零点电压保留两位小数：这是本模块唯一需要分辨到 mV 的读数。
     * 正常（REF 接地）约 0.000~0.005 V，显示 "0.00" 即为正常；
     * 若误接 VCC/2 会显示 1.65，一眼就能看出来。
     * x=88 起放标签，给左边留 8 个字符位置（0.00 ≈ 4~5 字符，1.65 也够）。 */
    OLED_ShowFloat(0u, OLED_LINE_HEIGHT, s_zero_offset_v, Font_11x18, White);
    ssd1306_SetCursor(88u, OLED_LINE_HEIGHT);
    (void)ssd1306_WriteString("Vzero", Font_11x18, White);

    ssd1306_SetCursor(0u, 2u * OLED_LINE_HEIGHT);
    (void)ssd1306_WriteString("CH9/CH14 I2C2", Font_11x18, White);

    ssd1306_UpdateScreen();
}

/*============================================================================*/
/* 初始化（调度器启动前调用）                                                  */
/*============================================================================*/

void OLEDPower_Init(void)
{
    HAL_StatusTypeDef st;
    uint32_t guard;

    s_zero_offset_v = 0.0f;
    s_acc_sum[ADC_CH_CURRENT] = 0u;
    s_acc_sum[ADC_CH_VOLTAGE] = 0u;
    s_acc_cnt = 0u;

    memset(&g_power, 0, sizeof(g_power));
    s_filtered_v = 0.0f;
    s_filtered_i = 0.0f;
    s_ratio_pct = 0u;

    /* ── 第 1 步：启动 ADC + DMA ────────────────────────────────────────────
     * 这是全工程唯一一处启动 ADC 的地方。CubeMX 只生成了 MX_ADC3_Init()
     * （写配置寄存器），没有任何代码启动转换；不调用本函数就永远是 0。
     *
     * 参数说明：
     *   - 长度 = 2 = 规则组序列长度（adc.c 里 NbrOfConversion = 2），必须是 2，
     *     多一个少一个都会让两个通道的数据整体错位；
     *   - 必须强制转换成 uint32_t*：HAL 的形参类型就是 uint32_t*，而真实的数据
     *     宽度由 hdma_adc3 的 MemDataAlignment = HALFWORD 决定，不由指针类型决定。
     *     传 uint16_t 数组的地址是正确用法，反过来把缓冲声明成 uint32_t 数组才错。
     *   - Circular + ContinuousConvMode 已在 Core/Src/adc.c 配好，所以这一次调用
     *     之后 ADC 自由运行、DMA 自动循环搬运，**不需要任何定时器触发**，
     *     也不会产生"每次转换一次软件启动"的开销。 */
    st = HAL_ADC_Start_DMA(&hadc3, (uint32_t *)g_adc_dma, ADC_CH_COUNT);
    s_adc_st = st;                       /* 遥测：0 = HAL_OK，非 0 = 启动失败 */
    configASSERT(st == HAL_OK);

    /* ── 第 2 步：零点校准 ────────────────────────────────────────────────
     * 前提：此刻调度器还没启动、Chassis 任务还没创建、PWM 从未输出过，因此电机
     *       物理上零电流。TB6612 的 STBY 是硬件接 3V3、软件关不掉（见 app_config.h
     *       第 9 节），所以只能靠这个"时间窗口"来保证零电流条件。
     *
     * 做法：等累加器攒够 POWER_ZERO_CAL_SAMPLES 个样本再取平均。
     * 以约 45 kHz 的转换率（一轮扫描 2 个通道 ≈ 22.3 µs），100 个样本约 2.2 ms。
     * guard 只用于防止"DMA 没启动
     * 成功"时在这里死等 —— 那种情况应该由 configASSERT 或界面去暴露，而不是卡死。 */
    guard = 0u;
    while ((s_acc_cnt < POWER_ZERO_CAL_SAMPLES) && (guard < 2000000u)) {
        guard++;
    }

    if (s_acc_cnt > 0u) {
        uint32_t sum_i;
        uint32_t sum_v;

        /* 校准阶段没有真正的并发：调度器未启动，唯一会改累加器的 DMA 中断
         * 优先级低于当前上下文（中断总是抢占线程模式），不会有撕裂。
         * 但仍然走 take_accumulator()，与运行期保持同一套访问方式，
         * 以后若有人把本函数挪进任务里也不会出错。 */
        (void)take_accumulator(&sum_i, &sum_v);
        (void)sum_v; /* 电压通道不需要零点：分压器直流耦合，地就是真正的地 */

        s_zero_offset_v = raw_to_volts(sum_i / POWER_ZERO_CAL_SAMPLES);
    }

    /* ── 第 3 步：OLEDPower 任务**不在这里创建** ─────────────────────────────
     *
     * 由 CubeMX 的 MX_FREERTOS_Init() 创建（Core/Src/freertos.c 的
     * osThreadNew(StartOLEDPower, ...)，参数取自 Robo.ioc 的 FREERTOS.Tasks01）。
     *
     * ★ 这里绝对不能再建第二次 ★
     * 曾经用 xTaskCreateStatic 建过第二次，于是两个 OLED 任务实例同时在跑，
     * 而它们共用同一个 I2C2 与同一个 g_oled_i2c_mutex：两个实例各执行一次
     * xSemaphoreCreateMutex()（后者覆盖全局句柄）、刷屏互相打断 →
     * 屏幕撕裂、I2C 报错，并且白吃一份 CPU（每次刷屏约 25 ms 阻塞）。
     * 这和 Chassis / Safety / Remote 是同一个 bug，详见 remote.c 的 App_Init()。
     *
     * 优先级不在创建时指定，而是在任务体开头用
     * vTaskPrioritySet(NULL, PRIO_OLEDPOWER) 强制成 app_config.h 的约定值，
     * 这样 Robo.ioc 里的数字怎么填都不会破坏
     * "OLED 必须低于 Remote/Chassis" 的次序要求（见 app_config.h:45-54）。 */
}

/*============================================================================*/
/* 任务                                                                        */
/*============================================================================*/

/**
  * @brief OLEDPower 任务体。
  *
  * 周期策略（严格按 10 ms + 分频 10）：
  *   主节拍用 vTaskDelayUntil，周期 OLED_POWER_PERIOD_MS = 10 ms；
  *   用分频计数器每 OLED_REFRESH_DIVIDER(10) 次刷一屏 = 100 ms 一帧。
  *
  * ★ 为什么"分频"不能省：刷屏一次约 25 ms 阻塞。若每 10 ms 都刷，这个任务会
  *   一直追进度，并把所有低优先级任务饿死。分频到 100 ms 后，CPU 占用约 25%。
  */
void StartOLEDPower(void *argument)
{
    TickType_t last_wake;
    TickType_t last_loop_tick;
    uint32_t divider = 0u;
    const TickType_t period = pdMS_TO_TICKS(OLED_POWER_PERIOD_MS);
#if (POWER_DEBUG_PRINT != 0)
    TickType_t last_print_tick;
#endif

    (void)argument;

    /* ── 第 0 步：★把优先级强制成 app_config.h 的约定值★ ────────────────────
     * 本任务由 CubeMX 创建（osThreadNew），优先级取自 Robo.ioc（那里是 35），
     * 而本工程的约定值是 PRIO_OLEDPOWER（20），并且 app_config.h:45-54 明确要求
     * **OLED 必须低于 Remote(26) 与 Chassis** —— 因为下面第 2 步的 ssd1306_Init()
     * 和每 100 ms 一次的刷屏都是**阻塞式** HAL_I2C_Mem_Write（超时 HAL_MAX_DELAY）。
     * 一旦它的优先级高于速度环，OLED 没接好时就会把速度环彻底饿死，
     * 现象是"串口没输出、s_vel_tick 停住、电机不动"，而 Safety 仍在喂狗、
     * 看门狗也不会复位，故障非常隐蔽。
     * 用 vTaskPrioritySet 覆盖，让 app_config.h 成为唯一真相来源。
     * ⚠ 必须放在阻塞式 I2C 之前，否则还没降下来就被自己卡住。 */
    (void)vTaskPrioritySet(NULL, (UBaseType_t)PRIO_OLEDPOWER);

    /* ── 第 1 步：创建 OLED 所在 I2C2 的总线互斥锁 ──────────────────────────
     * 本任务由 CubeMX 的 MX_FREERTOS_Init() 创建（Core/Src/freertos.c 的
     * osThreadNew），此时调度器已经启动，所以走 else 分支；
     * xTaskGetSchedulerState() 的判断保留下来，是为了以后有人把它挪到
     * 调度器启动之前调用时也不会破坏 heap_4 的空闲链表。
     *
     * ⚠ 这把锁保护的是 I2C2（OLED 在 PB10/PB11 上）。
     *   目前 I2C2 只有 OLED 一个设备（user/sensorimu.c 还是空壳）。
     *   以后若把 IMU 挂到 I2C2，**必须复用这把锁**，否则两条任务同时访问会互相打断。 */
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        taskENTER_CRITICAL();
        g_oled_i2c_mutex = xSemaphoreCreateMutex();
        taskEXIT_CRITICAL();
    } else {
        g_oled_i2c_mutex = xSemaphoreCreateMutex();
    }
    configASSERT(g_oled_i2c_mutex != NULL);

    /* ── 第 2 步：探测 + 初始化 OLED ────────────────────────────────────────
     *
     * ★ 先用**有界超时**探测屏在不在（见 s_oled_state 的说明）★
     *   user/ssd1306/ssd1306.c 里所有 I2C 写都用 HAL_MAX_DELAY（无限超时），
     *   而 ssd1306_UpdateScreen() 一次 32 笔事务。屏不应答时每笔都等超时，
     *   一次刷屏实测约 772 ms，会把本任务的 10 ms 采样周期拖成 862 ms
     *   （遥测里表现为 n 从 449 变成 38715）。
     *
     *   所以先探测：不在线就**完全不碰** ssd1306 的那套 API，
     *   连 ssd1306_Init() 也跳过（它同样会死在 HAL_MAX_DELAY 上）。
     *   探测本身最多 2 × 5 ms = 10 ms，有界。 */
    if (HAL_I2C_IsDeviceReady(&SSD1306_I2C_PORT, (uint16_t)SSD1306_I2C_ADDR,
                              OLED_PROBE_TRIALS, OLED_PROBE_TIMEOUT_MS) != HAL_OK) {
        s_oled_state = 2u;   /* 离线 */
        (void)printf("[power] OLED not responding at 0x%02X -> display disabled, "
                     "sampling stays at %ums\r\n",
                     (unsigned)(SSD1306_I2C_ADDR >> 1), (unsigned)OLED_POWER_PERIOD_MS);
    } else {
        s_oled_state = 1u;   /* 在线 */
        if (xSemaphoreTake(g_oled_i2c_mutex, portMAX_DELAY) == pdTRUE) {
            ssd1306_Init();
            oled_show_banner();
            (void)xSemaphoreGive(g_oled_i2c_mutex);
        }
    }

    /* 基准时刻必须在"这一轮初始化"之后取，否则 last_wake 会停留在 130 ms 之前，
     * vTaskDelayUntil 会连续追赶十几个周期（表现为起步时刷屏飞快）。 */
    last_wake = xTaskGetTickCount();
    last_loop_tick = last_wake;
#if (POWER_DEBUG_PRINT != 0)
    last_print_tick = last_wake;
#endif

    for (;;) {
        TickType_t now_tick;

        /* ── 10 ms 固定周期 ────────────────────────────────────────────────
         * 用 vTaskDelayUntil 而不是 vTaskDelay：前者以"绝对时刻"为基准，
         * 会把本周期的执行耗时自动扣掉，长期平均周期严格等于 10 ms；
         * 后者是"执行完再等 10 ms"，周期会被执行时间越拖越长。
         * 严禁使用 HAL_Delay（忙等占住 CPU）。 */
        vTaskDelayUntil(&last_wake, period);

        /* 遥测：本周期相对上一周期的真实间隔。
         * 正常应恒为 OLED_POWER_PERIOD_MS(10)。若明显更大，
         * 说明本任务被阻塞了（历史上就是屏不在线时 I2C 无限超时导致的）。 */
        now_tick = xTaskGetTickCount();
        s_telem_period_ms = (uint32_t)(now_tick - last_loop_tick);
        last_loop_tick = now_tick;

        /* ① 采集 + 换算 + 滤波 + 电量：只碰内存与 FPU，耗时几十微秒。 */
        power_measurement_update();

#if (POWER_DEBUG_PRINT != 0)
        /* ── 调试打印（默认 2 Hz）────────────────────────────────────────────
         * 全部用整数（mV / mA / mW），不触发 printf 的浮点格式化代码。
         *
         * 判读：
         *   adc=0        HAL_ADC_Start_DMA 成功；非 0 则采样根本没启动
         *   dt=10        本任务循环的真实周期（ms）。**必须接近 10**；
         *                远大于 10 说明任务被阻塞了（历史上是屏 I2C 无限超时）
         *   n=449        本周期样本数。44.9 kHz × 10 ms ≈ 449。0 = DMA 停了
         *   oled=1       1=屏在线在刷屏；2=屏不在线已禁用刷屏（采样周期因此正常）
         *   dmaV/dmaI    **瞬时**那一轮转换的原始码 —— 判断引脚电平就看这个
         *   rawV/rawI    本周期均值（原始码），看稳定性
         *   zero         电流零点（mV），应接近 0
         *   V/I/P/SOC    换算后的物理量（mV / mA / mW / %）
         *
         * 自洽性校验（手动算一遍就能确认换算链没错）：
         *   V[mV] ≈ rawV / 4095 × 3300 × 11
         *   例：rawV=1421 → 1421/4095×3300×11 ≈ 12594 mV ≈ 12.59 V ✔
         *   若 rawV≈0 而 V 也是 0 → **ADC 引脚上确实没有电压**（接线/电池） */
        if ((TickType_t)(xTaskGetTickCount() - last_print_tick)
                >= (TickType_t)pdMS_TO_TICKS(POWER_DEBUG_PRINT_PERIOD_MS)) {
            last_print_tick = xTaskGetTickCount();
            (void)printf("[power] adc=%d dt=%lu oled=%u n=%lu | "
                         "dma V=%4lu I=%4lu | raw V=%4lu I=%4lu zero=%ldmV | "
                         "V=%ldmV I=%ldmA P=%ldmW SOC=%lu%%\r\n",
                         (int)s_adc_st,
                         (unsigned long)s_telem_period_ms,
                         (unsigned)s_oled_state,
                         (unsigned long)s_telem_n,
                         (unsigned long)s_telem_dma_v,
                         (unsigned long)s_telem_dma_i,
                         (unsigned long)s_telem_raw_v,
                         (unsigned long)s_telem_raw_i,
                         (long)(s_zero_offset_v * 1000.0f),
                         (long)(g_power.battery_v * 1000.0f),
                         (long)(g_power.current_a * 1000.0f),
                         (long)(g_power.power_w * 1000.0f),
                         (unsigned long)g_power.ratio_pct);
        }
#endif

        /* ② 分频刷屏：10 次 × 10 ms = 100 ms 一帧（10 Hz）。 */
        divider++;
        if (divider >= OLED_REFRESH_DIVIDER) {
            divider = 0u;

            /* 拿 I2C2 总线锁。目前 I2C2 上只有 OLED（IMU 尚未实现），
             * 所以这里正常是"零等待"。
             *
             * 刻意**不用 portMAX_DELAY**：万一以后有别的任务（比如 IMU）
             * 拿住 I2C2 不放，本任务优先级最低(20)，若无限等待就会白白占着
             * 时间片。限时 200 ms（约 2 帧刷屏周期）后放弃这一帧，
             * 下一帧（100 ms 后）自然重试，界面只是偶尔不更新，不会拖垮系统。 */
            /* 只有屏在线才刷，否则完全不碰 ssd1306 的 HAL_MAX_DELAY 那套 API
             * （见 s_oled_state 的说明：屏不在线时一次刷屏会拖 772 ms）。 */
            if ((s_oled_state == 1u) &&
                (xSemaphoreTake(g_oled_i2c_mutex, pdMS_TO_TICKS(200)) == pdTRUE)) {
                oled_render();
                (void)xSemaphoreGive(g_oled_i2c_mutex);
            }
        }
    }
}

/*============================================================================*/
/* 遥测接口（只读，调电压/电流采样专用）                                       */
/*============================================================================*/
/*
 * 这组函数只在调试时使用，不参与任何控制计算，也不会被控制路径调用。
 * 存在的唯一目的：让 Ozone / 串口能读到 OLEDpower.c 内部的 static 量
 * （原始 ADC 码、零点、样本数、ADC 启动状态），这些量在 Ozone 里直接
 * 看 static 变量既不可靠（改了布局/名字就失效）也不直观。
 *
 * 排障顺序见 OLEDpower.h 里这组声明上方的说明。
 */

int32_t Power_TelemAdcStatus(void)
{
    return (int32_t)s_adc_st;
}

uint32_t Power_TelemSampleCount(void)
{
    return s_telem_n;
}

uint32_t Power_TelemRawVoltageCode(void)
{
    return s_telem_raw_v;
}

uint32_t Power_TelemRawCurrentCode(void)
{
    return s_telem_raw_i;
}

int32_t Power_TelemZeroOffsetMv(void)
{
    float mv = s_zero_offset_v * 1000.0f;
    return (int32_t)((mv >= 0.0f) ? (mv + 0.5f) : (mv - 0.5f));
}

int32_t Power_TelemBatteryMv(void)
{
    float mv = g_power.battery_v * 1000.0f;
    return (int32_t)((mv >= 0.0f) ? (mv + 0.5f) : (mv - 0.5f));
}

int32_t Power_TelemCurrentMa(void)
{
    float ma = g_power.current_a * 1000.0f;
    return (int32_t)((ma >= 0.0f) ? (ma + 0.5f) : (ma - 0.5f));
}

int32_t Power_TelemPowerMw(void)
{
    float mw = g_power.power_w * 1000.0f;
    return (int32_t)((mw >= 0.0f) ? (mw + 0.5f) : (mw - 0.5f));
}

uint32_t Power_TelemSocPct(void)
{
    return (uint32_t)g_power.ratio_pct;
}

uint32_t Power_TelemRawDmaVoltageCode(void)
{
    return s_telem_dma_v;
}

uint32_t Power_TelemRawDmaCurrentCode(void)
{
    return s_telem_dma_i;
}

uint32_t Power_TelemLoopPeriodMs(void)
{
    return s_telem_period_ms;
}

uint32_t Power_TelemOledState(void)
{
    return (uint32_t)s_oled_state;
}
