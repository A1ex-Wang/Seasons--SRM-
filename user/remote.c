/** @file remote.c
 * @brief 遥控任务（Remote）与 SRM v4 协议接入层。
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

/*  调试开关（必须放在所有 #if 之前！） */

/* 置 1 跳过遥控解析和投帧，供调试器注入指令；正常遥控为 0。 */
#ifndef DEBUG_NO_REMOTE
#define DEBUG_NO_REMOTE 0
#endif

/* 调试串口输出总开关 */

/* 遥控诊断日志开关，仅控制串口输出，不影响协议解析。 */
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

/* 静态对象：队列 */

/* 队列缓冲区与静态队列控制块。必须是文件作用域 static（.bss / SRAM1），
 * 这样队列在程序生命周期内一直有效，且完全不经过 FreeRTOS heap。
 *
 * 注意：*调试旁路下也要保留这两个缓冲*——DEBUG_NO_REMOTE=1 只是没人往里投帧，
 * App_Init() 仍然会创建队列（chassis 侧也照常取），所以它们并非"未使用"。 */
static uint8_t s_queue_storage[Q_REMOTE_LEN * sizeof(remote_in_t)];
static StaticQueue_t s_queue_cb;

QueueHandle_t q_remote_cmd = NULL;

/* 静态对象：协议状态 */

/* 协议状态及 DMA 缓冲区放在 DMA 可访问的 SRAM，不能放入 CCM RAM。 */
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

/* 板级回调：只能做「拷贝字段 + 置标志」 */

/* 下面两个回调都是从 SRM_Remote_Poll() 的解析路径里同步调用的。 */
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

/* 复用协议层时间戳判定链路有效性。 */

/* 板级回调：UART 错误 */

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == NULL) return;
    if (huart->Instance != BT_UART_HANDLE.Instance) return;

    /* 中断上下文，只做两件事： */
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

/* 队列投递 */

/* 复用协议层时间戳判定链路有效性。 */
//online函数在线置1，超时置0
#if (DEBUG_NO_REMOTE == 0)
static uint8_t remote_link_is_online(void) {
    /* 必须用 HAL_GetTick()，不能换 xTaskGetTickCount() */
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

/* 初始化 */

BaseType_t Remote_ProtocolInit(void) {
    /* 取 g_remote 里 rx_buffer 的地址（链接期决定，运行期检查）。 */
    uint32_t rx_addr;

    /* 编译期一致性：本工程配置的缓冲大小必须等于上游宏的大小，
     * 否则 srm_uart_init() 会直接返回 HAL_ERROR（它要求两者相等）。 */
    APP_STATIC_ASSERT((REMOTE_RX_DMA_SIZE == SRM_UART_RX_DMA_SIZE),
                      "REMOTE_RX_DMA_SIZE must equal upstream SRM_UART_RX_DMA_SIZE");

    rx_addr = (uint32_t)(uintptr_t)g_remote.rx_buffer;

    /* 运行期断言 1：DMA 缓冲绝不能落在 CCM RAM */
    if (rx_addr >= CCM_RAM_BASE && rx_addr < CCM_RAM_END) {
        REMOTE_LOG("[remote] INIT FAIL: rx_buffer in CCM RAM (0x%08lX) - DMA cannot access it\r\n",
                     (unsigned long)rx_addr);
        return pdFAIL;
    }

    /*  运行期检查 2：DMA 缓冲必须落在 SRAM1 地址区间 
     * DMA 控制器访问不到 CCM RAM，但能访问 SRAM1/SRAM2。 */
    if (rx_addr < APP_SRAM1_BASE || rx_addr >= APP_SRAM1_END) {
        REMOTE_LOG("[remote] INIT FAIL: rx_buffer 0x%08lX not in SRAM1 [0x%08lX,0x%08lX)\r\n",
                     (unsigned long)rx_addr,
                     (unsigned long)APP_SRAM1_BASE, (unsigned long)APP_SRAM1_END);
        return pdFAIL;
    }

    (void)rx_addr;   /* 两个地址检查已经覆盖了必要的合法性；此处保留以示意 */

    (void)memset(&g_last_control, 0, sizeof(g_last_control));
    (void)memset(&g_last_pro, 0, sizeof(g_last_pro));

    /* 初始化上游协议层：内部会校验 USART2 的 RX DMA 是否为 Circular 模式， */
    g_remote = SRM_Remote_Init(&BT_UART_HANDLE);

    /* DMA 初始化失败时返回错误，由任务保留安全状态。 */
    if (g_remote.init_status != HAL_OK) {
        REMOTE_LOG("[remote] INIT FAIL: srm_uart_init status=%d, hdmarx=%p (expect Circular)\r\n",
                     (int)g_remote.init_status, (void *)BT_UART_HANDLE.hdmarx);
        return pdFAIL; /* 队列照常创建，chassis 会一直显示 LOST */
    }

    /* 协议结构体按值返回后，重新绑定全局缓冲区及回调上下文。 */
    g_remote.uart.rx_buffer = g_remote.rx_buffer;
    g_remote.uart.rx_context = &g_remote;

    /* 然后借用上游自己的重启机制，让 DMA 用*新指针*重新装载：
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

/* 任务：Remote */

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

    /* 输出任务启动日志。 */
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

    /* 使用 app_config.h 中的任务优先级。 */
    (void)vTaskPrioritySet(NULL, (UBaseType_t)PRIO_REMOTE);

    last_wake = xTaskGetTickCount();

    for (;;) {
#if (DEBUG_NO_REMOTE != 0)
        /* 调试旁路：不碰链路状态、不投任何帧 */
#else
        uint8_t now_online;
        uint8_t need_post;

        /* 1) 消费 DMA 新字节 + 解析协议 + 由上游处理 600 ms 失联保护。
         *    必须高频调用：256 字节环形缓冲在 9600 bps 下约 266 ms 就会写满。 */
        g_frame_valid = 0u;
        SRM_Remote_Poll(&g_remote);

        now_online = remote_link_is_online();

        /* 2) 什么时候投队列： */
        need_post = ((g_frame_valid != 0u) || (now_online != link_online)) ? 1u : 0u;

        if (need_post != 0u) {
            post_current_frame();

#if (REMOTE_DEBUG_PRINT != 0)
            /*  诊断：第一帧到达时打印一次 
             * 只打印第一帧，避免 1 kHz 刷屏把串口带宽吃光。
             * 注意 post_current_frame() 在上一行、*不受本开关影响*。 */
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
        /* 字节级 + 数值诊断（每 500 ms 一次，2 Hz，不刷屏） */
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

            /* 原始字节转储（只在还没解析出帧时打印，通了就不再打） */
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

                /* 决定性检查：DMA 的内存目的地址 M0AR 到底指向哪 */
                {
                    uint32_t m0ar = (uint32_t)BT_UART_HANDLE.hdmarx->Instance->M0AR;
                    uint32_t want = (uint32_t)(uintptr_t)g_remote.rx_buffer;
                    uint32_t sr   = (uint32_t)BT_UART_HANDLE.Instance->SR;
                    uint32_t cr1  = (uint32_t)BT_UART_HANDLE.Instance->CR1;

                    /* 把 SR 的错误位解成文本，省得再查手册。
                     * FE（帧错误）是最重要的信号：
                     *   FE=1 且数据全 0x00  ⇒ *波特率不匹配*（或线路被拉低）。
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

/* 应用层外设与队列初始化 */

void App_Init(void) {
    /* 初始化独立调试串口。 */
    DebugUart_Init();

    /* 初始化电源监测硬件。 */
    OLEDPower_Init();

    /* 创建静态遥控队列。 */
    q_remote_cmd = xQueueCreateStatic((UBaseType_t)Q_REMOTE_LEN,
                                      (UBaseType_t)sizeof(remote_in_t),
                                      s_queue_storage,
                                      &s_queue_cb);
    configASSERT(q_remote_cmd != NULL);

    /* Remote 与 Safety 任务由 CubeMX 创建。 */
}
