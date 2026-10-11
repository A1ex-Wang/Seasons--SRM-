#ifndef SRM_UART_H
#define SRM_UART_H

/* MODIFIED: F1->F4 HAL header */
/* 本工程 MCU 为 STM32F407ZGT6（F4 系列），上游原始文件为 STM32F103（F1）。
 * 唯一改动：把下面的 HAL 头文件由 stm32f1xx_hal.h 改为 stm32f4xx_hal.h。
 * 逻辑、寄存器访问方式与 API 均无改动（F1/F4 的 UART HAL API 兼容）。 */

#include <stddef.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * DMA 环形接收缓存。115200 bps 下 256 字节约能容纳 22 ms 连续数据，
 * 因此主循环必须频繁调用 SRM_Remote_Poll()，不能用长时间阻塞式 delay。
 */
#ifndef SRM_UART_RX_DMA_SIZE
#define SRM_UART_RX_DMA_SIZE 256u
#endif
#if SRM_UART_RX_DMA_SIZE == 0u || SRM_UART_RX_DMA_SIZE > 65535u
#error "SRM_UART_RX_DMA_SIZE must be in 1..65535"
#endif

typedef struct {
    uint32_t rx_bytes;
    uint32_t tx_frames;
    uint32_t tx_failures;
    uint32_t uart_errors;
    uint32_t dma_restarts;
} srm_uart_stats_t;

typedef void (*srm_uart_rx_callback_t)(void *context, uint8_t byte, uint32_t now_ms);

typedef struct {
    UART_HandleTypeDef *handle;
    srm_uart_rx_callback_t rx_callback;
    void *rx_context;
    uint8_t *rx_buffer;
    uint16_t rx_tail;
    volatile uint8_t restart_requested;
    srm_uart_stats_t stats;
} srm_uart_t;

/*
 * 初始化 STM32 HAL 平台层。
 * bluetooth_uart 必须已经由 CubeMX 初始化，且 RX DMA 必须配置为 Circular。
 * 成功返回 HAL_OK；配置缺失或启动 DMA 失败时返回相应 HAL 状态。
 */
HAL_StatusTypeDef srm_uart_init(srm_uart_t *uart, UART_HandleTypeDef *bluetooth_uart,
                                uint8_t *rx_buffer, uint16_t rx_buffer_size,
                                srm_uart_rx_callback_t rx_callback, void *rx_context);

/* 主循环每轮调用：取出 DMA 新字节、解析协议，并执行 600 ms 失联保护。 */
void srm_uart_poll(srm_uart_t *uart, uint32_t now_ms);

/* 在 HAL_UART_ErrorCallback() 中调用；真正的 DMA 重启延后到主循环执行。 */
void srm_uart_on_error(srm_uart_t *uart, UART_HandleTypeDef *failed_uart);

HAL_StatusTypeDef srm_uart_write(srm_uart_t *uart, const uint8_t *data, size_t length);

void srm_uart_copy_stats(const srm_uart_t *uart, srm_uart_stats_t *output);

#ifdef __cplusplus
}
#endif

#endif
