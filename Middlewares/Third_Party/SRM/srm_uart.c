#include "srm_uart.h"

#include <string.h>

#define SRM_UART_TX_TIMEOUT_MS 200u

static HAL_StatusTypeDef start_rx_dma(srm_uart_t *uart) {
    HAL_StatusTypeDef status;
    uart->rx_tail = 0u;
    uart->restart_requested = 0u;
    status = HAL_UART_Receive_DMA(uart->handle, uart->rx_buffer, SRM_UART_RX_DMA_SIZE);
    if (status != HAL_OK) return status;
    __HAL_DMA_DISABLE_IT(uart->handle->hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(uart->handle->hdmarx, DMA_IT_TC);
    return HAL_OK;
}

HAL_StatusTypeDef srm_uart_init(srm_uart_t *uart, UART_HandleTypeDef *bluetooth_uart,
                                uint8_t *rx_buffer, uint16_t rx_buffer_size,
                                srm_uart_rx_callback_t rx_callback, void *rx_context) {
    if (uart == NULL || bluetooth_uart == NULL || bluetooth_uart->hdmarx == NULL
            || bluetooth_uart->hdmarx->Init.Mode != DMA_CIRCULAR
            || rx_buffer == NULL || rx_buffer_size != SRM_UART_RX_DMA_SIZE
            || rx_callback == NULL) return HAL_ERROR;
    memset(uart, 0, sizeof(*uart));
    uart->handle = bluetooth_uart;
    uart->rx_buffer = rx_buffer;
    uart->rx_callback = rx_callback;
    uart->rx_context = rx_context;
    return start_rx_dma(uart);
}

void srm_uart_poll(srm_uart_t *uart, uint32_t now_ms) {
    uint16_t dma_position;
    if (uart == NULL || uart->handle == NULL) return;
    if (uart->restart_requested != 0u) {
        (void)HAL_UART_DMAStop(uart->handle);
        if (start_rx_dma(uart) == HAL_OK) uart->stats.dma_restarts++;
        else uart->restart_requested = 1u;
    }
    if (uart->restart_requested != 0u) return;
    dma_position = (uint16_t)(SRM_UART_RX_DMA_SIZE
            - __HAL_DMA_GET_COUNTER(uart->handle->hdmarx));
    if (dma_position >= SRM_UART_RX_DMA_SIZE) dma_position = 0u;
    while (uart->rx_tail != dma_position) {
        uart->rx_callback(uart->rx_context, uart->rx_buffer[uart->rx_tail], now_ms);
        uart->rx_tail++;
        if (uart->rx_tail >= SRM_UART_RX_DMA_SIZE) uart->rx_tail = 0u;
        uart->stats.rx_bytes++;
    }
}

void srm_uart_on_error(srm_uart_t *uart, UART_HandleTypeDef *failed_uart) {
    if (uart != NULL && uart->handle == failed_uart) {
        uart->stats.uart_errors++;
        uart->restart_requested = 1u;
    }
}

HAL_StatusTypeDef srm_uart_write(srm_uart_t *uart, const uint8_t *data, size_t length) {
    HAL_StatusTypeDef status;
    if (uart == NULL || uart->handle == NULL || data == NULL || length == 0u
            || length > UINT16_MAX) return HAL_ERROR;
    status = HAL_UART_Transmit(uart->handle, (uint8_t *)data, (uint16_t)length,
                               SRM_UART_TX_TIMEOUT_MS);
    if (status == HAL_OK) uart->stats.tx_frames++;
    else uart->stats.tx_failures++;
    return status;
}

void srm_uart_copy_stats(const srm_uart_t *uart, srm_uart_stats_t *output) {
    if (uart != NULL && output != NULL) *output = uart->stats;
}
