#include "srm_remote.h"

#include <string.h>

static void apply_safe(srm_remote_t *remote) {
    static const srm_control_state_t safe_control = {0};
    static const srm_pro_control_state_t safe_pro = {0};
    if (remote->pro_control_active != 0u) {
        remote->pro_control = safe_pro;
        SRM_BoardApplyProControl(&remote->pro_control);
    } else {
        remote->control = safe_control;
        SRM_BoardApplyControl(&remote->control);
    }
    remote->control_active = 0u;
}

static void send_frame(srm_remote_t *remote, uint8_t type, const uint8_t *payload,
                       uint8_t length) {
    uint8_t wire[SRM_MAX_FRAME];
    size_t wire_length = srm_build_frame(wire, sizeof(wire), type,
                                         remote->reply_sequence++, payload, length);
    if (wire_length != 0u) (void)srm_uart_write(&remote->uart, wire, wire_length);
}

static void send_ack(srm_remote_t *remote, uint8_t sequence) {
    const uint8_t payload[2] = {sequence, 0u};
    send_frame(remote, SRM_TYPE_ACK, payload, (uint8_t)sizeof(payload));
}

static void send_error(srm_remote_t *remote, uint8_t sequence, uint8_t code) {
    const uint8_t payload[2] = {sequence, code};
    send_frame(remote, SRM_TYPE_ERROR, payload, (uint8_t)sizeof(payload));
}

static void handle_frame(srm_remote_t *remote, const srm_frame_t *frame, uint32_t now_ms) {
    if (frame->type == SRM_TYPE_CONTROL) {
        srm_control_state_t next;
        if (srm_decode_control(frame, &next)) {
            remote->control = next;
            remote->pro_control_active = 0u;
            remote->control_active = 1u;
            remote->last_control_ms = now_ms;
            SRM_BoardApplyControl(&remote->control);
        }
    } else if (frame->type == SRM_TYPE_PRO_CONTROL) {
        srm_pro_control_state_t next;
        if (srm_decode_pro_control(frame, &next)) {
            remote->pro_control = next;
            remote->pro_control_active = 1u;
            remote->control_active = 1u;
            remote->last_control_ms = now_ms;
            SRM_BoardApplyProControl(&remote->pro_control);
        }
    } else if (frame->type == SRM_TYPE_HELLO) {
        if (frame->length != 4u) send_error(remote, frame->sequence, 2u);
        else send_ack(remote, frame->sequence);
    } else if (frame->type == SRM_TYPE_DEBUG) {
        uint8_t code = SRM_BoardHandleDebug(frame->payload, frame->length);
        if (code == 0u) send_ack(remote, frame->sequence);
        else send_error(remote, frame->sequence, code);
    } else {
        send_error(remote, frame->sequence, 1u);
    }
}

static void on_rx(void *context, uint8_t byte, uint32_t now_ms) {
    srm_remote_t *remote = (srm_remote_t *)context;
    srm_frame_t frame;
    if (srm_parser_push(&remote->parser, byte, &frame) == SRM_PARSE_FRAME)
        handle_frame(remote, &frame, now_ms);
}

srm_remote_t SRM_Remote_Init(UART_HandleTypeDef *bluetooth_uart) {
    srm_remote_t remote;
    memset(&remote, 0, sizeof(remote));
    srm_parser_init(&remote.parser);
    remote.last_control_ms = HAL_GetTick();
    remote.init_status = srm_uart_init(&remote.uart, bluetooth_uart, remote.rx_buffer,
                                       (uint16_t)sizeof(remote.rx_buffer), on_rx, &remote);
    if (remote.init_status == HAL_OK) apply_safe(&remote);
    return remote;
}

void SRM_Remote_Poll(srm_remote_t *remote) {
    uint32_t now_ms;
    if (remote == NULL || remote->init_status != HAL_OK) return;
    /* 返回值初始化 API 允许按值接收对象；修正回调上下文为调用方对象地址。 */
    remote->uart.rx_context = remote;
    now_ms = HAL_GetTick();
    srm_uart_poll(&remote->uart, now_ms);
    if (remote->control_active != 0u
            && (uint32_t)(now_ms - remote->last_control_ms) > SRM_REMOTE_CONTROL_TIMEOUT_MS)
        apply_safe(remote);
}

void SRM_Remote_OnUartError(srm_remote_t *remote, UART_HandleTypeDef *uart) {
    if (remote != NULL) srm_uart_on_error(&remote->uart, uart);
}

const srm_uart_stats_t *SRM_Remote_Stats(const srm_remote_t *remote) {
    return remote == NULL ? NULL : &remote->uart.stats;
}

const srm_control_state_t *SRM_Remote_Control(const srm_remote_t *remote) {
    return remote == NULL ? NULL : &remote->control;
}

const srm_pro_control_state_t *SRM_Remote_ProControl(const srm_remote_t *remote) {
    return remote == NULL ? NULL : &remote->pro_control;
}

__weak void SRM_BoardApplyControl(const srm_control_state_t *state) { (void)state; }
__weak void SRM_BoardApplyProControl(const srm_pro_control_state_t *state) { (void)state; }

__weak uint8_t SRM_BoardHandleDebug(const uint8_t *data, uint8_t length) {
    static const uint8_t ping[] = {'P', 'I', 'N', 'G'};
    if (data == NULL) return 1u;
    if (length > 0u && data[length - 1u] == '\n') length--;
    if (length > 0u && data[length - 1u] == '\r') length--;
    return length == sizeof(ping) && memcmp(data, ping, sizeof(ping)) == 0 ? 0u : 1u;
}
