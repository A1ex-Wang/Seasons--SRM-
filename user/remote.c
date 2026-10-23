/**
  ******************************************************************************
  * @file    remote.c
  * @brief   遥控任务（Remote）与 SRM v4 协议接入层。
  *
  * 任务职责：以 1 ms 周期调用 SRM_Remote_Poll() 消费 USART2 的 DMA 环形缓冲
  *           中的新字节，解析出 v4 控制帧；在检测到新帧时把它打包成
  *           remote_in_t 投递到 q_remote_cmd。
  *
  * 数据流：
  *   蓝牙模块 → USART2 RX DMA(Circular, 无 HT/TC 中断) → rx_buffer[256]
  *            → SRM_Remote_Poll()（本任务，1 kHz）→ SRM_BoardApplyControl /
  *              SRM_BoardApplyProControl（同步回调，只存快照 + 置标志）
  *            → q_remote_cmd（静态队列）→ StartChassisTask
  *
  * 本阶段不驱动任何电机。
  ******************************************************************************
  */

#include "remote.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "main.h" /* HAL 与 CubeMX 生成的类型定义 */

#include "srm_remote.h" /* 上游协议层（Middlewares/Third_Party/SRM） */

/*============================================================================*/
/* ★ 调试开关（必须放在所有 #if 之前！）★                                      */
/*============================================================================*/
/*
 * DEBUG_NO_REMOTE = 1：无蓝牙模块时的调试旁路。
 *   - 跳过 Remote_ProtocolInit()，避开 upstream 里
 *       configASSERT(init_status == HAL_OK)
 *     （本工程的 configASSERT 是 {taskDISABLE_INTERRUPTS(); for(;;);}，
 *      会关中断并永久卡死 Remote 任务 → 调度器停摆 → Chassis 任务不执行）
 *   - 整段跳过"失联兜底投帧"：没有蓝牙时 g_remote.last_control_ms 恒为 0，
 *     上电满 600 ms 后 remote_link_is_online() 由 1 翻 0 会投出一帧全零的
 *     remote_in_t，把 Ozone 里手写的 s_last_cmd.online / ly 覆盖掉
 *     （现象：改完值能保持一会儿，然后自己变回 0）。
 *   于是 s_last_cmd 完全归调试器掌控。
 *
 * ⚠ 正常固件（要蓝牙遥控）必须为 0。可用 -DDEBUG_NO_REMOTE=0 覆盖。
 *
 * ⚠⚠ 这个 #define 必须出现在文件里**所有 #if (DEBUG_NO_REMOTE ...) 之前**。
 *   预处理是自上而下的：如果定义在守卫之后，守卫处该宏还是"未定义"，
 *   而在 #if 里未定义标识符按 0 处理，于是 (0 == 0) 恒真、
 *   守卫完全失效（函数照样被编译进去并触发 -Wunused-function）。
 *   这个坑本文件踩过一次，改动时不要把这段往下挪。 */
#ifndef DEBUG_NO_REMOTE
#define DEBUG_NO_REMOTE 0
#endif

/*============================================================================*/
/* 调试串口输出总开关                                                          */
/*============================================================================*/

/* ★ REMOTE_DEBUG_PRINT：Remote 任务是否往 USART6 打调试信息 ★
 *
 * 1 = 打印（调蓝牙/遥控时用）
 * 0 = 关闭（**调其它模块时务必设 0**）
 *
 * 为什么要能关：
 *   本任务原本每 500 ms 打一行 `[remote] rx=... online=...`，
 *   在还没通的时候还会附 32 字节的缓冲十六进制转储 + M0AR/SR 寄存器行。
 *   而 OLEDpower 模块调电压时同样每 500 ms 打一行 `[power] ...`。
 *   两路 2 Hz 的输出交织在一起，串口里几乎没法读。
 *
 * 关掉之后 USART6 上就只剩其它模块的输出，互不干扰。
 * 本宏只影响**打印**，协议解析、投队列、online 判定一律照常工作。
 *
 * 用法：调电压/底盘/PID 时设 0；回头查蓝牙链路时再设 1。
 *      也可用编译选项覆盖：-DREMOTE_DEBUG_PRINT=1 */
#ifndef REMOTE_DEBUG_PRINT
#define REMOTE_DEBUG_PRINT 0
#endif

#if (REMOTE_DEBUG_PRINT != 0)
#define REMOTE_LOG(...)   REMOTE_LOG(__VA_ARGS__)
#else
/* 关掉时展开成空语句：参数不会被求值，也就不会产生"变量只写不读"的告警
 * （相关变量本身也用 #if 一起排除了，见任务体里的说明）。 */
#define REMOTE_LOG(...)   do { } while (0)
#endif

#include "chassiscontrol.h"
#include "debug_uart.h"
#include "safety.h"
#include "OLEDpower.h" /* OLEDPower_Init()：ADC+DMA 启动 + 电流零点校准 + 建 OLED 数显任务 */

/*============================================================================*/
/* 静态对象：队列                                                              */
/*============================================================================*/

/* 队列缓冲区与静态队列控制块。必须是文件作用域 static（.bss / SRAM1），
 * 这样队列在程序生命周期内一直有效，且完全不经过 FreeRTOS heap。
 *
 * 注意：**调试旁路下也要保留这两个缓冲**——DEBUG_NO_REMOTE=1 只是没人往里投帧，
 * App_Init() 仍然会创建队列（chassis 侧也照常取），所以它们并非"未使用"。 */
static uint8_t s_queue_storage[Q_REMOTE_LEN * sizeof(remote_in_t)];
static StaticQueue_t s_queue_cb;

QueueHandle_t q_remote_cmd = NULL;

/*============================================================================*/
/* 静态对象：协议状态                                                          */
/*============================================================================*/

/**
  * @brief 上游协议状态（含 256 字节 DMA 缓冲）。
  *
  * ★★★ CCM RAM 红线 ★★★
  * STM32F407 的 CCM RAM 位于 0x10000000 段，它只挂在 CPU 的 D-bus 上，
  * **DMA 控制器根本无法访问**。一旦 g_remote（或其中任何一个字段）被链接器
  * 放进 CCM RAM，DMA 的写操作会被静默丢弃：不报错、不产生中断、不置标志，
  * 现象就是“一个字节都收不到、代码看起来完全正常”。
  *
  * 本工程的链接脚本 STM32F407xx_FLASH.ld 里 .ccmram 段只收集 *(.ccmram*)，
  * 而这里**没有**任何 __attribute__((section(".ccmram"))) 标注，
  * 因此 g_remote 落在 .bss → RAM (0x20000000 段，SRAM1)，DMA 可见。
  * Remote_ProtocolInit() 里还有一次运行期地址断言兜底（见 CCM_RAM_BASE）。
  */
static srm_remote_t g_remote;

/* 最近一帧的原始状态（两个回调各自写入自己的那一份）。 */
static srm_control_state_t g_last_control;
static srm_pro_control_state_t g_last_pro;

static volatile TickType_t g_last_frame_tick; /**< 最近一帧到达时的 tick，用于时间戳 */
static volatile uint8_t g_frame_valid;        /**< 本轮 SRM_Remote_Poll() 是否产生了新状态 */
static volatile uint8_t g_frame_is_pro;       /**< 1 = 数据源是 PRO_CONTROL，0 = CONTROL */
/** @brief 因队列满而被丢弃的新帧计数（诊断用），仅生产分支使用。 */
#if (DEBUG_NO_REMOTE == 0)
static uint32_t g_frames_dropped;
#endif

/*============================================================================*/
/* 板级回调：只能做「拷贝字段 + 置标志」                                       */
/*============================================================================*/

/*
 * 下面两个回调都是**从 SRM_Remote_Poll() 的解析路径里同步调用**的。
 * 它们运行在 Remote 任务上下文（不在中断里），但这不改变约束：
 *   只允许 ① 把字段拷贝到静态结构体  ② 置一个 volatile 标志。
 * 严禁在此投队列、调串口发送、调 printf、或做任何可能阻塞/耗时的操作——
 * 解析路径被拖慢会直接把 DMA 环形缓冲耗光，丢掉后面的遥控帧。
 * 真正的投队列动作放在 Remote 的任务循环里，由标志位触发。
 */
//control模式为只有ABXY、十字键等基础键，具体应用为手机App的简单模式、老版本设备
void SRM_BoardApplyControl(const srm_control_state_t *s) {
    if (s == NULL) return;
    g_last_control = *s;             /* ① 拷贝字段 */
    g_frame_is_pro = 0u;             /* 记录数据源，投队列时按此取快照 */
    g_last_frame_tick = xTaskGetTickCount();
    g_frame_valid = 1u;              /* ② 置标志 */
}
//procontrol模式为包含ABXY、LB/RB、扳机、摇杆按压、Start/Select等，具体应用为Xbox手柄、PC端中继器、手机App的高级模式
void SRM_BoardApplyProControl(const srm_pro_control_state_t *s) {
    if (s == NULL) return;
    g_last_pro = *s;
    g_frame_is_pro = 1u;
    g_last_frame_tick = xTaskGetTickCount();
    g_frame_valid = 1u;
}

/*
 * 关于“online / LOST 是怎么判出来的”（对应约束 7：不要重复实现失联保护）
 *
 * 上游 srm_remote.c 已经实现了 600 ms 失联保护：超时后会以全零状态再调用一次
 * 板级回调（apply_safe()），随后把 control_active 清零、不再回调。
 *
 * 因此**本工程不写任何自己的超时计时器**，online 完全由上游的既有状态推导：
 *   online = (当前 tick - g_remote.last_control_ms) <= REMOTE_TIMEOUT_MS
 * g_remote.last_control_ms 是上游每收到一帧合法控制帧就刷新的时间戳，
 * 也是上游自己判定超时所用的同一个变量；超过 600 ms 未被刷新就说明
 * 上游已进入 / 即将进入安全状态。这样做的好处：
 *   - 只有一处计时逻辑（在上游），不存在两套阈值互相打架；
 *   - 不会把“摇杆回中、按键全松”的真实全零帧误判成失联
 *     （真实帧一定会刷新 last_control_ms）。
 * app_config.h 里有编译期断言保证 REMOTE_TIMEOUT_MS 与上游宏一致。
 */

/*============================================================================*/
/* 板级回调：UART 错误                                                         */
/*============================================================================*/

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == NULL) return;
    if (huart->Instance != BT_UART_HANDLE.Instance) return;

    /* ⚠ 中断上下文，只做两件事：
     *   ① 通知上游协议层“需要重启 DMA”（内部只是置一个 volatile 标志）；
     *   ② 清掉 ORE（过载）标志。
     * 真正的 HAL_UART_DMAStop() + HAL_UART_Receive_DMA() 由任务里的
     * srm_uart_poll() 执行——中断里重启 DMA 会和任务里正在读的
     * rx_tail/DMA 计数器打架，也可能长时间占用中断。 */
    SRM_Remote_OnUartError(&g_remote, huart);

    /* F4 的 UART：ORE 置位后若不清，会一直触发“过载错误中断”，
     * 形成中断风暴并把整机卡死在 ISR 里。HAL 的 UART_EndRxTransfer() 清掉了
     * PE/FE/NE/ORE，但错误回调是在它之后进入的，期间新到的字节仍可能再置 ORE，
     * 因此这里补一次。HAL_UART_Receive_DMA() 在重启时也会清一次，两次不冲突。 */
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
}

/*============================================================================*/
/* 队列投递                                                                    */
/*============================================================================*/

/**
  * @brief 当前链路是否有效。
  *
  * 读取的是**上游协议层自己的**时间戳 srm_remote_t::last_control_ms —— 上游每收到
  * 一帧通过 CRC 与范围校验的控制帧就刷新它，并用它判断 600 ms 超时。
  * 这里只是复用该判据做显示，没有新建任何计时器（约束 7）。
  */
//online函数在线置1，超时置0
#if (DEBUG_NO_REMOTE == 0)
static uint8_t remote_link_is_online(void) {
    /* ★★★ 必须用 HAL_GetTick()，不能换 xTaskGetTickCount() ★★★
     *
     * 上游（Middlewares/Third_Party/SRM/srm_remote.c:78）是这样写时间戳的：
     *     remote.last_control_ms = HAL_GetTick();
     * 而本工程用了 Core/Src/stm32f4xx_hal_timebase_tim.c ——
     * **HAL 的时基被改到 TIM6**（1 kHz），从 HAL_Init() 起就开始计数；
     * 而 FreeRTOS 的 tick 来自 SysTick，从 osKernelStart() 才开始计数。
     *
     * 两者相差 = 从 HAL_Init() 到 osKernelStart() 的全部初始化时间
     * （外设初始化 + ADC 校准 + OLED 的 HAL_Delay(100) …，实测数百毫秒）。
     *
     * 若这里用 xTaskGetTickCount() 去减 last_control_ms，得到的是
     *     (t_now - t_sched_start) - t_frame = elapsed - t_sched_start
     * 也就是真实间隔被减掉了整段初始化时间。上电后第一帧到达时 elapsed≈0，
     * 该值为负 → 按 uint32 处理就是巨大数 → 恒 > 600
     * → **永远判为 LOST，底盘永远不动，遥控完全不工作**。
     * 即使初始化时间小于 600 ms 也只是"侥幸"，有效超时被压缩成
     * 600 ms - 初始化时间，链路会莫名很容易断。
     *
     * 所以这里必须与上游**同一个时基**：HAL_GetTick()。 */
    return ((uint32_t)(HAL_GetTick() - g_remote.last_control_ms)
            <= (uint32_t)REMOTE_TIMEOUT_MS)
               ? 1u
               : 0u;
}

/* 帧构建与投递只被生产分支使用；调试旁路（DEBUG_NO_REMOTE=1）下整体不编译，
 * 既避免"定义了但没用"的编译告警，也保证调试期绝对不可能投出任何帧。
 * 同一个 #if 块覆盖：remote_link_is_online / build_remote_in / post_current_frame。 */

/** @brief 按当前数据源把快照转成 remote_in_t。 */
static void build_remote_in(remote_in_t *out) {
    if (out == NULL) return;
    (void)memset(out, 0, sizeof(*out));

    if (g_frame_is_pro != 0u) {
        /* PRO_CONTROL：四个轴 + 24 位按键 + 双扳机，全部是线上原始量纲。 */
        out->lx = g_last_pro.left_x;
        out->ly = g_last_pro.left_y;
        out->rx = g_last_pro.right_x;
        out->ry = g_last_pro.right_y;
        out->buttons = g_last_pro.buttons;
        out->lt = g_last_pro.left_trigger;
        out->rt = g_last_pro.right_trigger;
    } else {
        /* 兼容传统 CONTROL 帧（部分平板 App / PC 中继走这一条）：
         * 只有 8 位按键与 6 位拨杆，没有扳机。这里把按键映射到同一位图的前 8 位，
         * 扳机填 0，避免上层出现两套解析。 */
        out->lx = g_last_control.left_x;
        out->ly = g_last_control.left_y;
        out->rx = g_last_control.right_x;
        out->ry = g_last_control.right_y;
        /* CONTROL 的 buttons、S1..S6 和方向键统一压入 remote_in_t.buttons：
         * bit0..3=A/B/X/Y，bit4..9=S1..S6，bit10..12=D-pad。 */
        out->buttons = (uint32_t)g_last_control.buttons
                     | ((uint32_t)g_last_control.switches << 4)
                     | ((uint32_t)g_last_control.dpad << 10);
        out->lt = 0u;
        out->rt = 0u;
    }

    out->stamp = (uint32_t)g_last_frame_tick; /* FreeRTOS tick（1 kHz，即 ms）*/
    out->online = remote_link_is_online();
}

/** @brief 把当前快照投进队列（丢旧语义）。 */
static void post_current_frame(void) {
    remote_in_t frame;
    build_remote_in(&frame);

    if (xQueueSend(q_remote_cmd, &frame, (TickType_t)0) != pdPASS) {
        g_frames_dropped++;
    }
}

#endif /* DEBUG_NO_REMOTE == 0 */

/*============================================================================*/
/* 初始化                                                                      */
/*============================================================================*/

BaseType_t Remote_ProtocolInit(void) {
    /* 取 g_remote 里 rx_buffer 的地址（链接期决定，运行期检查）。 */
    uint32_t rx_addr;

    /* 编译期一致性：本工程配置的缓冲大小必须等于上游宏的大小，
     * 否则 srm_uart_init() 会直接返回 HAL_ERROR（它要求两者相等）。 */
    APP_STATIC_ASSERT((REMOTE_RX_DMA_SIZE == SRM_UART_RX_DMA_SIZE),
                      "REMOTE_RX_DMA_SIZE must equal upstream SRM_UART_RX_DMA_SIZE");

    rx_addr = (uint32_t)(uintptr_t)g_remote.rx_buffer;

    /* ── 运行期断言 1：DMA 缓冲绝不能落在 CCM RAM（见 app_config.h 第 8 节）────
     * DMA 访问 CCM RAM 会静默失败：收不到任何字节且不报任何错。
     *
     * ★ 这里**不用 configASSERT** ★
     *   本工程的 configASSERT 是 {taskDISABLE_INTERRUPTS(); for(;;);} ——
     *   会关中断 + 死循环，把 Remote 任务（优先级 26）永久卡死，
     *   进而让调度器停摆、Chassis 任务（优先级 32）永远得不到执行。
     *   遥控调试期间最常见的问题恰恰是"蓝牙没接好/波特率不对/DMA 没起来"，
     *   如果每次都让整机瘫痪，就完全无法定位问题（连串口都停了）。
     *   所以改成**打印原因 + 返回失败**：底盘照常运行（安全态停车），
     *   串口和控制台都能看到到底哪一项检查没过。
     *   地址检查本身在链接脚本确定后不可能失败，保留它是为了兜住
     *   "以后有人给 g_remote 加了 __attribute__((section(".ccmram")))" 这种情况。 */
    if (rx_addr >= CCM_RAM_BASE && rx_addr < CCM_RAM_END) {
        REMOTE_LOG("[remote] INIT FAIL: rx_buffer in CCM RAM (0x%08lX) - DMA cannot access it\r\n",
                     (unsigned long)rx_addr);
        return pdFAIL;
    }

    /* ── 运行期检查 2：DMA 缓冲必须落在 SRAM1 地址区间 ────────────────────
     * DMA 控制器访问不到 CCM RAM，但能访问 SRAM1/SRAM2。 */
    if (rx_addr < APP_SRAM1_BASE || rx_addr >= APP_SRAM1_END) {
        REMOTE_LOG("[remote] INIT FAIL: rx_buffer 0x%08lX not in SRAM1 [0x%08lX,0x%08lX)\r\n",
                     (unsigned long)rx_addr,
                     (unsigned long)APP_SRAM1_BASE, (unsigned long)APP_SRAM1_END);
        return pdFAIL;
    }

    /* ── 运行期检查 3：对齐 —— ★只需要字节对齐，不需要 4 字节★ ─────────────
     *
     * ★ 这里曾经要求 4 字节对齐，那是**错的**，而且直接把遥控功能废掉了 ★
     *
     * 为什么不需要 4 字节：
     *   Core/Src/usart.c:142-145 里 USART2_RX 的 DMA 配置是
     *       .MemDataAlignment = DMA_MDATAALIGN_BYTE   ← 逐字节传输
     *       .FIFOMode         = DMA_FIFOMODE_DISABLE  ← 没开 FIFO
     *   逐字节 + 无 FIFO 时，内存地址**只要求 1 字节对齐**，任意字节地址都合法。
     *   （只有 MEMDATAALIGN_HALFWORD/WORD、或开着 FIFO 才有对齐约束。）
     *
     * 而实际地址必然不是 4 的倍数 —— 这是结构体布局决定的：
     *   srm_remote_t 里 rx_buffer 是第 3 个成员（srm_remote.h:13-24）
     *       srm_uart_t  uart     @ 0    大小 40
     *       srm_parser_t parser  @ 40   大小 71（body[68] + state/position/expected）
     *       uint8_t rx_buffer[]  @ 111  ← 111 % 4 = 3
     *   所以 g_remote 即使 4 字节对齐，rx_buffer 也必然落在 4n+3。
     *   实测就是这个地址：0x2000AEFB（& 3 = 3）。
     *
     * 后果（本工程真实现象）：
     *   原来的断言让 Remote_ProtocolInit() 返回 pdFAIL →
     *   串口打出 "[remote] INIT FAIL: rx_buffer ... not 4-byte aligned" →
     *   **DMA 从未启动 → rx_bytes 永远是 0 → online 永远是 0**。
     *   （更早的版本这里是 configASSERT(0)，会直接关中断死循环，整机瘫痪。）
     *
     * 现在只做**字节对齐**检查（恒成立，纯属兜底），不再阻止初始化。 */
    (void)rx_addr;   /* 两个地址检查已经覆盖了必要的合法性；此处保留以示意 */

    (void)memset(&g_last_control, 0, sizeof(g_last_control));
    (void)memset(&g_last_pro, 0, sizeof(g_last_pro));

    /* 初始化上游协议层：内部会校验 USART2 的 RX DMA 是否为 Circular 模式，
     * 然后真正启动 HAL_UART_Receive_DMA()，并关闭 DMA 的 HT/TC 中断。
     *
     * 注意 SRM_Remote_Init() 是“按值返回”的：返回期间 DMA 已经在往**局部副本**
     * 的 rx_buffer 里写了，副本在返回后即失效。所以紧接着必须调用一次
     * SRM_Remote_Poll()：上游会在这时把 uart.rx_context 修正为 g_remote 的地址，
     * 之后的字节才会经过 g_remote 的缓冲与解析器。这一行不能省。 */
    g_remote = SRM_Remote_Init(&BT_UART_HANDLE);

    /* ⚠ init_status != HAL_OK 说明 DMA 没起来（常见原因：CubeMX 里 USART2_RX
     *   的 DMA Mode 不是 Circular，或 hdmarx 为 NULL）。
     *   **同样不用 configASSERT**（理由见上面的说明）：打印原因并返回失败，
     *   任务继续以 1 ms 周期运行、底盘处于安全态（电机不动），
     *   手机 APP 那边可以先把数据发起来，等串口报告修好后再看效果。 */
    if (g_remote.init_status != HAL_OK) {
        REMOTE_LOG("[remote] INIT FAIL: srm_uart_init status=%d, hdmarx=%p (expect Circular)\r\n",
                     (int)g_remote.init_status, (void *)BT_UART_HANDLE.hdmarx);
        return pdFAIL; /* 队列照常创建，chassis 会一直显示 LOST */
    }

    /* ★★★★★ 这里必须修补 rx_buffer 指针 ★★★★★
     *
     * 上游 SRM_Remote_Init() 是**按值返回**的：
     *     srm_remote_t SRM_Remote_Init(...) {
     *         srm_remote_t remote;                          // 栈上局部变量
     *         srm_uart_init(&remote.uart, uart,
     *                       remote.rx_buffer,               // 局部副本数组的地址
     *                       ...);
     *         return remote;                                // 按值拷贝到 g_remote
     *     }
     *
     * ⚠⚠ 关键在于 srm_uart_t::rx_buffer 是一个**指针**（srm_uart.h:43）：
     *       srm_uart_init() 里：uart->rx_buffer = rx_buffer;
     *   所以拷贝到 g_remote 之后，
     *       g_remote.uart.rx_buffer  ==  SRM_Remote_Init 那个【已失效的栈帧地址】
     *   而不是 g_remote.rx_buffer（真正的全局数组）。
     *
     * 后果（本工程的实测症状）：
     *   · start_rx_dma() 用这个悬空指针装载 DMA → **DMA 把收到的数据写进死栈帧**
     *   · srm_uart_poll() 也从同一个悬空指针读（srm_uart.c:46）
     *   · 死栈帧随后被 printf 等函数调用反复覆盖 →
     *     表现为 rx_bytes 一直增长（DMA 的 NDTR 确实在变）、
     *     但缓冲内容恒为 **00 00 00 …**、永远找不到 0xA5 → online 恒为 0。
     *
     *   注：只重新装载 DMA **不够** —— 上一版就是拿同一个悬空指针重启的，
     *       所以依然读不到数据。必须先把指针指回全局数组。
     *
     * 必须在调用 SRM_Remote_OnUartError()/Poll() **之前**改好指针，
     * 这样 start_rx_dma() 拿到的才是 g_remote.rx_buffer。 */
    g_remote.uart.rx_buffer = g_remote.rx_buffer;
    g_remote.uart.rx_context = &g_remote;

    /* 然后借用上游自己的重启机制，让 DMA 用**新指针**重新装载：
     *   SRM_Remote_OnUartError() 只是置 uart.restart_requested = 1；
     *   下一次 SRM_Remote_Poll() → srm_uart_poll() 看到该标志就执行
     *   HAL_UART_DMAStop() + start_rx_dma()，此时 uart->rx_buffer 已经正确。 */
    SRM_Remote_OnUartError(&g_remote, &BT_UART_HANDLE);
    SRM_Remote_Poll(&g_remote);

    /* 诊断：确认 DMA 目的地址就是全局数组 g_remote.rx_buffer，且落在 SRAM1 */
    REMOTE_LOG("[remote] DMA armed: buf=0x%08lX (static=%s) ptr_ok=%s restarts=%lu\r\n",
                 (unsigned long)(uintptr_t)g_remote.uart.rx_buffer,
                 (((uint32_t)(uintptr_t)g_remote.uart.rx_buffer >= APP_SRAM1_BASE)
                  && ((uint32_t)(uintptr_t)g_remote.uart.rx_buffer < APP_SRAM1_END))
                     ? "SRAM1 ok" : "BAD",
                 (g_remote.uart.rx_buffer == g_remote.rx_buffer) ? "YES" : "NO",
                 (unsigned long)g_remote.uart.stats.dma_restarts);

    g_frame_valid = 0u;
    return pdPASS;
}

/*============================================================================*/
/* 任务：Remote                                                                */
/*============================================================================*/

/* ★ 调试开关：没有蓝牙模块时用 Ozone 直接改 s_last_cmd 调底盘 ★
 *
 * 为什么需要它（这是实机调试踩过的坑，务必理解）：
 *   Remote_ProtocolInit() 里有一句
 *       configASSERT(g_remote.init_status == HAL_OK);
 *   而本工程的 configASSERT 定义是（见 Core/Inc/FreeRTOSConfig.h）
 *       #define configASSERT(x) if ((x) == 0) {taskDISABLE_INTERRUPTS(); for(;;);}
 *   —— **它会关中断并死循环**。
 *
 *   于是"没接蓝牙模块"这个纯调试场景会变成整机瘫痪：
 *     没接蓝牙 → BT_UART_HANDLE.hdmarx == NULL
 *     → 上游 srm_uart_init() 参数检查失败返回 HAL_ERROR
 *     → configASSERT 触发：关中断 + Remote 任务（优先级 26）永久卡死
 *     → SysTick 被屏蔽 → 调度器停摆
 *     → **Chassis 任务（优先级 32）永远得不到执行**
 *     → Motor_Init() 没跑、四路 PWM 没启动、s_last_cmd 没有任何人读
 *     → 现象：在 Ozone 里怎么改 s_last_cmd.ly / online，电机都不转。
 *
 * DEBUG_NO_REMOTE = 1 时：**跳过 Remote_ProtocolInit()**，但不删任务。
 *   任务照常创建、照常 1 ms 周期跑，只是不解析协议；
 *   由于 g_remote.init_status 保持为 0（≠ HAL_OK），SRM_Remote_Poll() 会
 *   在第一行就 return（上游有此保护），所以不会有任何帧被投递，
 *   **s_last_cmd 永远归你掌控**，可以放心用 Ozone 写它来测底盘。
 *
 * ⚠ 正常固件（要蓝牙遥控）必须保持 0，否则遥控链路完全不工作。
 *   可以用编译选项覆盖：-DDEBUG_NO_REMOTE=0。
 *
 * ⚠ 宏定义已移到文件顶部（所有 #if 之前），见上面第 4 节的说明。
 *   这里**不要**再写 #define，否则又是"定义晚于使用"的坑。 */

void StartRemoteTask(void *argument) {
    TickType_t last_wake;
#if (DEBUG_NO_REMOTE == 0)
    uint8_t link_online;   /* 仅在解析协议时需要记录上一周期在线状态 */
#if (REMOTE_DEBUG_PRINT != 0)
    uint8_t first_frame_reported;  /* 诊断：第一帧到达时打印一次 */
    TickType_t last_diag_tick;     /* 诊断：字节级统计的节拍 */
#endif
#endif

    (void)argument;

    /* ── 入口横幅：证明 Remote 任务确实跑起来了 ──────────────────────────
     * 这两行在 Remote_ProtocolInit() **之前**打印，因此即使协议初始化失败、
     * 甚至卡在里面，只要看到它们就说明任务本身被调度到了。
     * 如果连这两行都看不到，说明任务没被创建/没被调度（与蓝牙完全无关）。 */
    REMOTE_LOG("\r\n[remote] ===== Remote task STARTED (prio=%u) =====\r\n",
                 (unsigned)PRIO_REMOTE);

#if (DEBUG_NO_REMOTE == 0)
#if (REMOTE_DEBUG_PRINT != 0)
    first_frame_reported = 0u;
#endif
    if (Remote_ProtocolInit() != pdPASS) {
        REMOTE_LOG("[remote] SRM_Remote_Init FAILED: check USART2 RX DMA = Circular\r\n");
    } else {
        REMOTE_LOG("[remote] protocol init OK, waiting for frames on USART2 @%lu bps\r\n",
                     (unsigned long)BT_UART_BAUD);
    }
#if (REMOTE_DEBUG_PRINT != 0)
    last_diag_tick = xTaskGetTickCount();
#endif
#else
    /* 调试旁路：不调用 Remote_ProtocolInit()。
     * g_remote 全零 → init_status == 0(≠HAL_OK) → 下面循环里的
     * SRM_Remote_Poll() 会立刻返回，等价于"遥控链路不存在"，
     * 因此 s_last_cmd 不会被任何人写入。 */
    REMOTE_LOG("[remote] DEBUG_NO_REMOTE=1: protocol init skipped, Ozone drives s_last_cmd\r\n");
#endif

#if (DEBUG_NO_REMOTE == 0)
    link_online = remote_link_is_online();
#endif

    /* ── ★把优先级强制成 app_config.h 的约定值★ ──────────────────────────
     * 本任务由 CubeMX（Core/Src/freertos.c 的 osThreadNew）创建，优先级取自
     * Robo.ioc（那里是 32），而本工程的约定值是 PRIO_REMOTE（26）。
     * 让 app_config.h 成为唯一真相来源，避免"改了一处忘了另一处"。
     * 次序要求见 app_config.h:45-54：OLED 必须在 Remote 之下。 */
    (void)vTaskPrioritySet(NULL, (UBaseType_t)PRIO_REMOTE);

    last_wake = xTaskGetTickCount();

    for (;;) {
#if (DEBUG_NO_REMOTE != 0)
        /* ── 调试旁路：不碰链路状态、不投任何帧 ──────────────────────────────
         * 只跳过 Remote_ProtocolInit() 是不够的：没有蓝牙时
         * g_remote.last_control_ms 恒为 0，上电满 600 ms 后
         * remote_link_is_online() 会从 1 翻到 0，于是 now_online != link_online
         * 成立，**仍然会投出一帧全零的 remote_in_t**，把 Ozone 里手动写的
         * s_last_cmd.ly / s_last_cmd.online 覆盖成 0。
         * （现象：改完值能保持一会儿，然后自己变回 0。）
         * 所以这里干脆什么都不做，让 s_last_cmd 完全归调试器掌控。
         * 生产固件（DEBUG_NO_REMOTE=0）走下面的正常逻辑，行为一字不变。 */
#else
        uint8_t now_online;
        uint8_t need_post;

        /* 1) 消费 DMA 新字节 + 解析协议 + 由上游处理 600 ms 失联保护。
         *    必须高频调用：256 字节环形缓冲在 9600 bps 下约 266 ms 就会写满。 */
        g_frame_valid = 0u;
        SRM_Remote_Poll(&g_remote);

        now_online = remote_link_is_online();

        /* 2) 什么时候投队列：
         *    a) 上游产生了新状态（真实帧，或超时那一刻的全零填充帧）；
         *    b) online 状态发生翻转（1→0 显示 LOST，0→1 恢复显示 ON）。
         *    b) 是必需的：上游 apply_safe() 只在超时那一刻回调一次，之后
         *    control_active 被清零、不再回调，所以“失联”这件事只有一次投递机会；
         *    而 online 翻转检测保证即使那一帧被队列丢旧语义丢掉，
         *    Chassis 侧也能在一个周期内（1 ms）拿到在线/离线状态。 */
        need_post = ((g_frame_valid != 0u) || (now_online != link_online)) ? 1u : 0u;

        if (need_post != 0u) {
            post_current_frame();

#if (REMOTE_DEBUG_PRINT != 0)
            /* ── 诊断：第一帧到达时打印一次 ──────────────────────────────────
             * 只打印第一帧，避免 1 kHz 刷屏把串口带宽吃光。
             * 注意 post_current_frame() 在上一行、**不受本开关影响**。 */
            if (first_frame_reported == 0u) {
                if (now_online != 0u) {
                    first_frame_reported = 1u;
                    REMOTE_LOG("[remote] FIRST FRAME OK: LX=%d LY=%d RX=%d RY=%d BTN=0x%06lX LT=%u RT=%u\r\n",
                               (int)g_last_pro.left_x, (int)g_last_pro.left_y,
                               (int)g_last_pro.right_x, (int)g_last_pro.right_y,
                               (unsigned long)g_last_pro.buttons,
                               (unsigned)g_last_pro.left_trigger,
                               (unsigned)g_last_pro.right_trigger);
                }
            }
#endif /* REMOTE_DEBUG_PRINT */
        }
        link_online = now_online;

#if (REMOTE_DEBUG_PRINT != 0)
        /* ── 字节级 + 数值诊断（每 500 ms 一次，2 Hz，不刷屏）────────────
         * ★ 这是排查"蓝牙连上但 online=0"最有用的一行 ★
         * 它同时回答两个最底层的问题：
         *   ① MCU 的 USART2 到底有没有收到字节         → rx_bytes
         *   ② 收到的字节有没有被解析成合法控制帧        → online / AX
         *
         * rx_bytes 判读：
         *   = 0        → 一个字节都没到 UART。按顺序查：
         *                 ① DMA 没起来（看上面的 init OK / DMA armed 行）
         *                 ② 接线：USART2=PA2(TX)/PA3(RX)，与 JDY-31 **交叉**接
         *                 ③ 波特率：两边必须一致（本工程 BT_UART_BAUD=9600，
         *                    若 JDY-31 被 AT 命令改过就要对齐）
         *                 ④ JDY-31 没把 BLE 数据转发到 UART（模块侧）
         *   > 0 但不变  → 字节偶尔来，但不成流
         *   > 0 且持续增长：
         *       online=1 → 链路正常，看 AX 的数值调 APP
         *       online=0 → 字节在进但解析不出帧：波特率不对（乱码→CRC 全失败）
         *                  或 APP 发的不是 SRM v4 帧
         *
         * AX 是解析后的摇杆原始量纲（-512..511）+ 按键位图，
         * 与底盘打印的 LX/LY/RX/RY/BTN 完全同源，用来直接调 APP 的映射。 */
        if ((TickType_t)(xTaskGetTickCount() - last_diag_tick) >= (TickType_t)500u) {
            int ax_lx, ax_ly, ax_rx, ax_ry;
            unsigned long btn;
            unsigned is_pro;

            last_diag_tick = xTaskGetTickCount();

            /* 按当前数据源取快照（PRO_CONTROL 还是传统 CONTROL）。 */
            is_pro = (unsigned)g_frame_is_pro;
            if (g_frame_is_pro != 0u) {
                ax_lx = (int)g_last_pro.left_x;   ax_ly = (int)g_last_pro.left_y;
                ax_rx = (int)g_last_pro.right_x;  ax_ry = (int)g_last_pro.right_y;
                btn   = (unsigned long)g_last_pro.buttons;
            } else {
                ax_lx = (int)g_last_control.left_x;   ax_ly = (int)g_last_control.left_y;
                ax_rx = (int)g_last_control.right_x;  ax_ry = (int)g_last_control.right_y;
                btn   = (unsigned long)g_last_control.buttons;
            }

            REMOTE_LOG("[remote] rx=%lu err=%lu rest=%lu online=%u st=%u pro=%u | "
                         "AX %4d %4d %4d %4d BTN=0x%06lX\r\n",
                         (unsigned long)g_remote.uart.stats.rx_bytes,
                         (unsigned long)g_remote.uart.stats.uart_errors,
                         (unsigned long)g_remote.uart.stats.dma_restarts,
                         (unsigned)now_online,
                         (unsigned)g_remote.parser.state,
                         is_pro,
                         ax_lx, ax_ly, ax_rx, ax_ry, btn);

            /* ── ★原始字节转储★（只在还没解析出帧时打印，通了就不再打）──────
             *
             * 为什么需要它：当 rx 一直在涨但 online 一直是 0 时，靠上层状态
             * 无法区分"CRC 错"、"版本不是 4"、"根本不是 SRM 帧"这三种情况。
             * 直接把 DMA 环形缓冲里的字节按十六进制打出来，一眼就能看出
             * APP 到底在发什么。
             *
             * ★ 取样位置用 rx_tail，而不是 DMA 当前写指针 ★
             *   rx_tail 是 srm_uart_poll() 已经消费到的位置 —— 它之前的 32 个
             *   字节**确定已经被 DMA 写过**。而 DMA 写指针之前的区域在
             *   刚重启动 DMA 时可能是从未写过的全 0，会造成误判。
             *
             * 判读（SRM v4 一帧的线格式）：
             *     A5 5A | VTYPE | SEQ | LEN | payload... | CRC
             *   VTYPE 高 4 位必须是 4，所以第 3 个字节应该是 0x4X
             *   （CONTROL → 0x40，PRO_CONTROL → 0x47，HELLO → 0x41）
             *
             *   · A5 5A 47 / A5 5A 40 → 帧格式对，问题在 CRC 或后续字段
             *   · A5 5A 但不是 4X      → APP 的协议版本不是 v4
             *   · 看不到 A5 5A         → 波特率不对（乱码）或 APP 发的不是 SRM 帧
             *   · 全是 00              → 收到的确实是全零字节（模块侧或 APP 没发数据）
             *   · 全是 FF              → 线路悬空 / TX 没接 */
            if (now_online == 0u) {
                const uint16_t n_show = 32u;
                uint16_t tail = g_remote.uart.rx_tail;
                uint16_t k;
                uint16_t dma_remain;

                dma_remain = (uint16_t)__HAL_DMA_GET_COUNTER(g_remote.uart.handle->hdmarx);

                for (k = 0u; k < n_show; k++) {
                    uint16_t idx = (uint16_t)((tail + REMOTE_RX_DMA_SIZE - n_show + k)
                                              % REMOTE_RX_DMA_SIZE);
                    if ((k % 16u) == 0u) {
                        if (k != 0u) REMOTE_LOG("\r\n");
                        REMOTE_LOG("[remote] buf[%2u]: ", (unsigned)k);
                    }
                    REMOTE_LOG("%02X ", (unsigned)g_remote.uart.rx_buffer[idx]);
                }
                REMOTE_LOG("| tail=%u ndtr=%u\r\n",
                             (unsigned)tail, (unsigned)dma_remain);

                /* ── ★决定性检查：DMA 的内存目的地址 M0AR 到底指向哪★ ────────
                 *
                 * 现象是：tail 和 ndtr 都在推进（DMA 确实在传输），
                 *         但 g_remote.rx_buffer 一个字节都没被写过（保持 BSS 全零）。
                 *
                 * 这只有两种可能：
                 *   ① DMA 的 M0AR 指向别处（还是那个失效栈地址）→ 缓冲永远全零
                 *   ② M0AR 是对的，那么收到的字节**本身就是 0x00**
                 *
                 * 读 M0AR 就能一刀切开：
                 *   m0ar == want  → DMA 正在写这个缓冲 ⇒ 收到的数据真的是全零
                 *   m0ar != want  → DMA 写别处       ⇒ 指针/重装载没生效
                 *
                 * 同时打印 USART2 的 SR 与 CR1：
                 *   SR 的 ORE(bit3)/NE(bit2)/FE(bit1)/IDLE(bit4) 能看出
                 *   是不是线路异常（全零 + FE 置位 = 线路一直被拉低）；
                 *   CR1 的 RE(bit2) 必须为 1，否则接收器根本没开。 */
                {
                    uint32_t m0ar = (uint32_t)BT_UART_HANDLE.hdmarx->Instance->M0AR;
                    uint32_t want = (uint32_t)(uintptr_t)g_remote.rx_buffer;
                    uint32_t sr   = (uint32_t)BT_UART_HANDLE.Instance->SR;
                    uint32_t cr1  = (uint32_t)BT_UART_HANDLE.Instance->CR1;

                    /* 把 SR 的错误位解成文本，省得再查手册。
                     * FE（帧错误）是最重要的信号：
                     *   FE=1 且数据全 0x00  ⇒ **波特率不匹配**（或线路被拉低）。
                     *   这是 MCU 在错误的时刻采样停止位造成的，与 DMA、解析都无关。 */
                    REMOTE_LOG("[remote] M0AR=0x%08lX want=0x%08lX match=%s | "
                                 "SR=0x%04lX CR1=0x%04lX | %s%s%s%s "
                                 "(RE=%u)\r\n",
                                 (unsigned long)m0ar, (unsigned long)want,
                                 (m0ar == want) ? "YES" : "NO!!",
                                 (unsigned long)(sr & 0xFFFFu),
                                 (unsigned long)(cr1 & 0xFFFFu),
                                 ((sr & (1u << 1)) != 0u) ? "FE " : "",
                                 ((sr & (1u << 2)) != 0u) ? "NE " : "",
                                 ((sr & (1u << 3)) != 0u) ? "ORE " : "",
                                 ((sr & (1u << 4)) != 0u) ? "IDLE " : "",
                                 (unsigned)((cr1 >> 2) & 1u));
                }
            }
        }
#endif /* REMOTE_DEBUG_PRINT */
#endif /* DEBUG_NO_REMOTE */

        /* 3) 固定周期节拍。用 vTaskDelayUntil 而不是 osDelay/HAL_Delay */
        vTaskDelayUntil(&last_wake, (TickType_t)pdMS_TO_TICKS(REMOTE_PERIOD_MS));
    }
}

/*============================================================================*/
/* 应用层初始化（不再创建任务）                                                */
/*============================================================================*/

/* ★ 这里曾经有 s_stack_chassis / s_tcb_chassis / s_stack_safety / s_tcb_safety /
 *   s_stack_remote / s_tcb_remote 六个静态缓冲，以及对应的三次 xTaskCreateStatic。
 *   全部删除 —— 原因见 App_Init() 里的详细说明：
 *
 *   Robo.ioc 的 FREERTOS.Tasks01 已经为 ChassisTask / SafetyTask / RemoteTask
 *   （以及其余 7 个任务）配置好了参数，MX_FREERTOS_Init() 会各创建一次。
 *   本文件再静态创建一次就会**每个任务两个实例同时在跑**，而它们：
 *     · 各有一套 static 状态（s_vel_tick / s_vel_last_tick 等）
 *     · 却共用同一份硬件与数据结构（s_enc[] / s_pid[] / 电机 PWM / 遥控队列）
 *   实测后果：速度读数被放大约 10 倍（dt_s 掉到 1 ms）、看门狗被喂两遍、
 *   蓝牙帧被投两遍、s_target_mmps 里新旧运动学公式的符号模式混在一起。
 *
 *   现在由 CubeMX 独占创建权，优先级则在各任务体开头用 vTaskPrioritySet()
 *   强制成 app_config.h 的约定值（见 PRIO_SAFETY / PRIO_CHASSIS / PRIO_REMOTE）。 */

void App_Init(void) {
    /* ── 第 0 步：调试串口 ──────────────────────────────────────────────────
     * printf 走 USART6（独立于蓝牙口）。CubeMX 已经初始化了 USART6 的引脚与
     * 时钟，这里只统一波特率，必须在任何 printf 之前调用。 */
    DebugUart_Init();

    /* ── 第 0.5 步：OLED 数显模块的硬件初始化（必须在调度器启动前调用）─────
     * 只做三件事，**完全不涉及底盘控制与蓝牙协议**：
     *   1) 启动 ADC3 + DMA2_Stream0（循环模式）。CubeMX 只生成了配置函数
     *      MX_ADC3_Init()，全工程没有任何地方调用 HAL_ADC_Start_DMA()，
     *      不启动则 OLED 上的电压/电流/功率/电量永远是 0；
     *   2) 电流零点校准：要求"电机肯定没转"。TB6612 的 STBY 是硬件接 3V3、
     *      软件关不掉（见 app_config.h 第 9 节），只能抢在 Chassis 任务被创建
     *      之前、即 PWM 从未输出过的时间窗口里采样；
     *   3) 建 OLEDPower 任务（静态分配，优先级 35 < Chassis 47，
     *      不会抢占速度环）。
     * 该函数不做任何 PWM / 方向脚 / 串口操作，也不消费 q_remote_cmd。 */
    OLEDPower_Init();

    /* ── 第 1 步：先建队列 ──────────────────────────────────────────────────
     * 顺序要求：队列先于任务。这样两个任务开始运行时队列一定已经存在，
     * 不存在“生产者先跑、队列还是 NULL”的窗口。
     *
     * 必须用 xQueueCreateStatic：缓冲区与队列控制块都是 static 数组，
     * 不占用 FreeRTOS heap（硬性要求）。xQueueCreate 会去 heap_4 里申请内存，
     * 那是本工程明确禁止的。 */
    q_remote_cmd = xQueueCreateStatic((UBaseType_t)Q_REMOTE_LEN,
                                      (UBaseType_t)sizeof(remote_in_t),
                                      s_queue_storage,
                                      &s_queue_cb);
    configASSERT(q_remote_cmd != NULL);

    /* ── 第 2 步：Chassis 任务由 CubeMX（MX_FREERTOS_Init）创建 ─────────────
     *
     * ★★★ 这里绝对不要再创建一次 Chassis 任务 ★★★
     *
     * 本工程踩过这个大坑，现象是"速度读数被放大约 10 倍、PID 完全调不出来"：
     *
     *   Core/Src/freertos.c:183 已经用 osThreadNew(StartChassisTask, ...) 建过一次
     *   （CubeMX 生成，优先级 osPriorityHigh7 → FreeRTOS 优先级 47）。
     *   如果这里再用 xTaskCreateStatic 建第二次（优先级 PRIO_CHASSIS = 32），
     *   就会有**两个 Chassis 任务实例同时在跑**，而它们的：
     *
     *     s_vel_tick / s_vel_last_tick / s_vel_started   ← 各自一份（static 变量）
     *     s_enc[] / s_pid[] / s_target_mmps[] / 电机 PWM ← **共用同一份**
     *
     *   高优先级实例会抢占低优先级实例，于是"两次调用之间的真实时间"不再是 10 ms，
     *   chassiscontrol.c 算出的 dt_s 常常掉到 1 ms，而
     *       speed = delta × ENCODER_COUNTS_TO_MMPS / dt_s
     *   分母小 10 倍 → **速度读数大 10 倍**。
     *
     *   实测证据：s_enc[2].min_dt_s = 0.001（出现过 1 ms 的分母），
     *             s_enc[2].max_raw_mmps = 255086 mm/s（轮子上限只有 1046），
     *             s_target_mmps 里新旧运动学公式的符号模式混在一起
     *             （因为两个实例在同一时刻各自算了一遍指令）。
     *
     * ★ 正确的做法（本工程采用）★
     *   让 CubeMX 建的这一次成为**唯一**的一次：删掉本文件里重复的 xTaskCreateStatic。
     *   任务的优先级由任务体开头的 vTaskPrioritySet() 强制成 app_config.h 的约定值，
     *   于是"谁创建"和"参数从哪来"两件事都只有一个真相来源，
     *   且**不需要每次改完 .ioc 再手工修一遍代码**。
     *
     *   ⚠ Robo.ioc 里所有任务都是 Dynamic（osThreadNew，栈从 heap 拿）。
     *     这违背本工程"全部静态分配"的约定，但它是 CubeMX 的既定配置，
     *     改它需要动 .ioc；功能上无影响（heap 由 scheduler 起来后分配，
     *     本工程 configTOTAL_HEAP_SIZE 足够）。若以后要严格静态，
     *     在 MX 的 FreeRTOS 任务对话框里把 Allocation 改成 Static 即可。 */

    /* ── 第 2.5 / 3 步：Remote 与 Safety 任务同样**不在这里创建** ───────────
     *
     * 它们和 Chassis 是同一个问题：Robo.ioc 的 FREERTOS.Tasks01 里已经列了
     * RemoteTask(32) 和 SafetyTask(41)，MX_FREERTOS_Init() 会用 osThreadNew
     * 各建一次。这里如果再静态建一次，就会有两个实例同时在跑：
     *
     *   · Safety 跑两遍 → **看门狗被喂两遍**（掩盖真正的喂狗时序问题）
     *   · Remote 跑两遍 → 蓝牙帧被解析两遍、**队列被投两遍**（指令时序乱）
     *   · 三者的优先级还各不相同，互相抢占，dt_s 被算成 1 ms
     *
     * 所以三个任务统一交给 CubeMX 创建，优先级在各自任务体里用
     * vTaskPrioritySet() 强制成 app_config.h 的约定值。 */
}
