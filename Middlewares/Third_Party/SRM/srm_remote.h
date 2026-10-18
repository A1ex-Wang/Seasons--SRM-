#ifndef SRM_REMOTE_H
#define SRM_REMOTE_H

#include "srm_protocol.h"
#include "srm_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SRM_REMOTE_CONTROL_TIMEOUT_MS 600u

typedef struct {
    srm_uart_t uart;
    srm_parser_t parser;
    uint8_t rx_buffer[SRM_UART_RX_DMA_SIZE];
    uint32_t last_control_ms;
    uint8_t reply_sequence;
    uint8_t control_active;
    uint8_t pro_control_active;
    srm_control_state_t control;
    srm_pro_control_state_t pro_control;
    HAL_StatusTypeDef init_status;
} srm_remote_t;

/* 唯一初始化入口：传入已由 CubeMX 初始化且 RX DMA 为 Circular 的蓝牙 UART。 */
srm_remote_t SRM_Remote_Init(UART_HandleTypeDef *bluetooth_uart);

/* 主循环每轮调用；内部完成 DMA 消费、协议解码、回包和失联安全复位。 */
void SRM_Remote_Poll(srm_remote_t *remote);

/* 在 HAL_UART_ErrorCallback 中调用，随后由 SRM_Remote_Poll 自动重启 DMA。 */
void SRM_Remote_OnUartError(srm_remote_t *remote, UART_HandleTypeDef *uart);

/* 可选诊断快照。 */
const srm_uart_stats_t *SRM_Remote_Stats(const srm_remote_t *remote);
const srm_control_state_t *SRM_Remote_Control(const srm_remote_t *remote);
const srm_pro_control_state_t *SRM_Remote_ProControl(const srm_remote_t *remote);

/* 用户只需实现这三个板级回调；传入状态已完成完整协议校验。 */
void SRM_BoardApplyControl(const srm_control_state_t *state);
void SRM_BoardApplyProControl(const srm_pro_control_state_t *state);
uint8_t SRM_BoardHandleDebug(const uint8_t *utf8, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif
