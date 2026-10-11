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

#include "chassiscontrol.h"
#include "debug_uart.h"
#include "safety.h"

/*============================================================================*/
/* 静态对象：队列                                                              */
/*============================================================================*/

/* 队列缓冲区与静态队列控制块。必须是文件作用域 static（.bss / SRAM1），
 * 这样队列在程序生命周期内一直有效，且完全不经过 FreeRTOS heap。 */
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
static uint32_t g_frames_dropped;             /**< 因队列满而被丢弃的新帧计数（诊断用） */

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
static uint8_t remote_link_is_online(void) {
    return ((uint32_t)(xTaskGetTickCount() - (TickType_t)g_remote.last_control_ms)
            <= (uint32_t)REMOTE_TIMEOUT_MS)
               ? 1u
               : 0u;
}

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
        out->buttons = (uint32_t)g_last_control.buttons;
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

    /* 丢旧语义（发送端）：非阻塞发送，队列满就**丢弃这一条**，绝不阻塞。
     * 为什么不能阻塞：Remote 任务被卡住 → SRM_Remote_Poll() 停摆 →
     * 256 字节环形缓冲被后续字节冲掉 → 后续帧全部 CRC 失败。
     * 为什么丢“新”而不是丢“旧”：队列深度只有 2，而帧率最高约 100 Hz、
     * 消费端 1 kHz 每周期都会取，队列真正满住的窗口只可能出现在消费端被
     * printf 占住的几毫秒内；这几毫秒之后马上会到达更新的一帧，
     * 数据新鲜度不受影响，而“丢最旧”需要额外的出队/入队两步、在非
     * 中断上下文里无法原子完成（xQueueOverwrite 只能用于长度 1 的队列）。 */
    if (xQueueSend(q_remote_cmd, &frame, (TickType_t)0) != pdPASS) {
        g_frames_dropped++;
    }
}

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
     * DMA 访问 CCM RAM 会静默失败：收不到任何字节且不报任何错。这里用地址范围
     * 检查把最坏的情况变成一次可定位的 configASSERT。 */
    if (rx_addr >= CCM_RAM_BASE && rx_addr < CCM_RAM_END) {
        configASSERT(0); /* rx_buffer 落在 CCM RAM —— DMA 会静默失效 */
    }

    /* ── 运行期断言 2：DMA 缓冲必须落在 SRAM1 地址区间，且 4 字节对齐 ────── */
    if (rx_addr < APP_SRAM1_BASE || rx_addr >= APP_SRAM1_END) {
        configASSERT(0); /* rx_buffer 不在 SRAM1 内（DMA 可能访问不到）*/
    }
    if ((rx_addr & 0x3u) != 0u) {
        configASSERT(0); /* rx_buffer 未按 4 字节对齐 */
    }

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

    /* ⚠ 不能静默忽略初始化结果：init_status != HAL_OK 说明 DMA 没起来
     * （常见原因：CubeMX 里 USART2_RX 的 DMA Mode 不是 Circular，或请求被改掉），
     * 此时收不到任何数据。用 configASSERT 让问题在第一次上电就暴露。
     * 注意上游返回的是结构体而不是状态码，状态在成员里，因此这里写
     * “.init_status != HAL_OK”而不是整个返回值。 */
    configASSERT(g_remote.init_status == HAL_OK);
    if (g_remote.init_status != HAL_OK) {
        return pdFAIL; /* 队列照常创建，chassis 会一直显示 LOST */
    }

    /* 修正回调上下文并驱动第一轮 poll（此时没有新数据，只是让内部状态一致）。 */
    SRM_Remote_Poll(&g_remote);
    g_frame_valid = 0u;
    return pdPASS;
}

/*============================================================================*/
/* 任务：Remote                                                                */
/*============================================================================*/

void StartRemoteTask(void *argument) {
    TickType_t last_wake;
    uint8_t link_online;

    (void)argument;

    if (Remote_ProtocolInit() != pdPASS) {
        (void)printf("[remote] SRM_Remote_Init FAILED: check USART2 RX DMA = Circular\r\n");
    }

    link_online = remote_link_is_online();
    last_wake = xTaskGetTickCount();

    for (;;) {
        uint8_t now_online;
        uint8_t need_post;

        /* 1) 消费 DMA 新字节 + 解析协议 + 由上游处理 600 ms 失联保护。
         *    必须高频调用：256 字节环形缓冲在 9600 bps 下约 266 ms 就会写满。 */
        g_frame_valid = 0u;
        SRM_Remote_Poll(&g_remote);

        now_online = remote_link_is_online();

        /* 2) 什么时候投队列：
         *    a) 上游产生了新状态（真实帧，或超时那一刻的全零填充帧）这里由g_frame_valid表征；
         *    b) online 状态发生翻转（1→0 显示 LOST，0→1 恢复显示 ON）。
         *    b) 是必需的：上游 apply_safe() 只在超时那一刻回调一次，之后
         *    control_active 被清零、不再回调，所以“失联”这件事只有一次投递机会；
         *    而 online 翻转检测保证即使那一帧被队列丢旧语义丢掉，
         *    Chassis 侧也能在一个周期内（1 ms）拿到在线/离线状态。 */
        need_post = ((g_frame_valid != 0u) || (now_online != link_online)) ? 1u : 0u;

        if (need_post != 0u) {
            post_current_frame();
        }
        link_online = now_online;

        /* 3) 固定周期节拍。用 vTaskDelayUntil 而不是 osDelay/HAL_Delay */
        vTaskDelayUntil(&last_wake, (TickType_t)pdMS_TO_TICKS(REMOTE_PERIOD_MS));
    }
}

/*============================================================================*/
/* 任务创建（全静态分配，不占用 FreeRTOS heap）                                */
/*============================================================================*/

/* 静态任务的栈与 TCB 缓冲。必须是文件作用域 static：
 *   - 生命周期 = 程序，任务永不退出，栈不会失效；
 *   - 不经过 FreeRTOS heap（本项目要求“不占用 FreeRTOS heap”，
 *     因此只能用 xTaskCreateStatic，绝不能用 xTaskCreate）。 */
static StackType_t s_stack_safety[STACK_SAFETY_WORDS];
static StaticTask_t s_tcb_safety;
static StackType_t s_stack_chassis[STACK_CHASSIS_WORDS];
static StaticTask_t s_tcb_chassis;
static StackType_t s_stack_remote[STACK_REMOTE_WORDS];
static StaticTask_t s_tcb_remote;

void App_Init(void) {
    /* ── 第 0 步：调试串口 ──────────────────────────────────────────────────
     * printf 走 USART6（独立于蓝牙口）。CubeMX 已经初始化了 USART6 的引脚与
     * 时钟，这里只统一波特率，必须在任何 printf 之前调用。 */
    DebugUart_Init();

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

    /* ── 第 2 步：先建消费者（Chassis），再建生产者（Remote）───────────────
     * 消费端先就位，生产者一开跑就有地方放数据。 */
    (void)xTaskCreateStatic(StartChassisTask,
                            NAME_CHASSIS,
                            (uint32_t)STACK_CHASSIS_WORDS,
                            NULL,
                            (UBaseType_t)PRIO_CHASSIS,
                            s_stack_chassis,
                            &s_tcb_chassis);

    /* Remote 任务在本任务体内调用 Remote_ProtocolInit()，也就是说
     * USART2 的 DMA 在调度器启动之后才真正开始接收。
     * 这是刻意的：SRM_Remote_Init() 内部会调用 HAL_UART_Receive_DMA()，
     * 放在调度器之前做也没有问题，但把“协议初始化”收敛到它自己的任务里，
     * 可以让初始化失败（init_status != HAL_OK）通过 configASSERT 明确暴露，
     * 并且 main.c 只需要插入一行 App_Init()。 */
    (void)xTaskCreateStatic(StartRemoteTask,
                            NAME_REMOTE,
                            (uint32_t)STACK_REMOTE_WORDS,
                            NULL,
                            (UBaseType_t)PRIO_REMOTE,
                            s_stack_remote,
                            &s_tcb_remote);

    /* ── 第 3 步：安全任务（最高优先级）──────────────────────────────────── */
    (void)xTaskCreateStatic(StartSafetyTask,
                            NAME_SAFETY,
                            (uint32_t)STACK_SAFETY_WORDS,
                            NULL,
                            (UBaseType_t)PRIO_SAFETY,
                            s_stack_safety,
                            &s_tcb_safety);
}
