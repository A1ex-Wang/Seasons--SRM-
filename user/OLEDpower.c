/**
 * ADC3 DMA -> 10 ms 测量任务 -> 独立 200 ms 显示任务。
 * 所有 OLED 事务有界，失败即放弃本帧并在离线后重试；不影响 PID。
 */
#include "OLEDpower.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "adc.h"
#include "i2c.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"

#define ADC_CH_COUNT 2u
#define ADC_CH_CURRENT 0u
#define ADC_CH_VOLTAGE 1u
#define ADC_ACC_MAX_SAMPLES (UINT32_MAX / 4095u)
#define OLED_SCL_PIN GPIO_PIN_10
#define OLED_SDA_PIN GPIO_PIN_11
#define OLED_GPIO_PORT GPIOB
#define OLED_STARTUP_DELAY_MS 100u
#define OLED_LINE_PITCH 16u
#define ZERO_CAL_IDLE 0u
#define ZERO_CAL_BUSY 1u
#define ZERO_CAL_OK 2u
#define ZERO_CAL_FAILED 3u

static volatile uint16_t g_adc_dma[ADC_CH_COUNT] __attribute__((aligned(4)));
static volatile uint32_t s_acc_sum[ADC_CH_COUNT];
static volatile uint32_t s_acc_cnt;
static volatile float s_zero_offset_v;
static HAL_StatusTypeDef s_adc_st;
static volatile uint32_t s_telem_n, s_telem_raw_v, s_telem_raw_i;
static volatile uint32_t s_telem_dma_v, s_telem_dma_i, s_telem_period_ms;
static float s_filtered_v, s_filtered_i;
static uint8_t s_filter_ready, s_ratio_pct, s_soc_ready, s_rest_active;
static uint32_t s_last_sample_ms, s_rest_since_ms, s_soc_step_ms;
static float s_rest_anchor_v;
static uint8_t s_low_pending;
static uint32_t s_low_since_ms;
static uint64_t s_cal_sum;
static uint32_t s_cal_count, s_cal_since_ms;
static uint8_t s_cal_active;
static StaticTask_t s_display_tcb;
static StackType_t s_display_stack[STACK_OLEDPOWER_WORDS];
static StaticSemaphore_t s_i2c_mutex_storage;
power_monitor_t g_power;
power_diag_t g_power_diag;
volatile uint8_t g_power_zero_cal_request;
SemaphoreHandle_t g_oled_i2c_mutex;

APP_STATIC_ASSERT(POWER_MEASURE_PRIORITY < PRIO_REMOTE, "power sampling must remain below remote");
APP_STATIC_ASSERT(POWER_SOC_REST_MS > 0u && POWER_SOC_STEP_MS > 0u, "invalid SOC timing");
APP_STATIC_ASSERT(POWER_SOC_STABLE_BAND_V > 0.0f, "invalid SOC stability band");
APP_STATIC_ASSERT(POWER_CURRENT_GAIN_CORRECTION > 0.0f, "invalid current gain");
APP_STATIC_ASSERT(POWER_ZERO_CAL_WINDOW_MS > 0u, "invalid zero calibration timing");

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance != ADC3 || s_acc_cnt >= ADC_ACC_MAX_SAMPLES) return;
    s_acc_sum[ADC_CH_CURRENT] += g_adc_dma[ADC_CH_CURRENT];
    s_acc_sum[ADC_CH_VOLTAGE] += g_adc_dma[ADC_CH_VOLTAGE];
    s_acc_cnt++;
}

/* 可在调度器启动前使用，不改变 FreeRTOS 临界区嵌套状态。 */
static uint32_t take_accumulator(uint32_t *sum_i, uint32_t *sum_v) {
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint32_t n = s_acc_cnt;
    *sum_i = s_acc_sum[ADC_CH_CURRENT];
    *sum_v = s_acc_sum[ADC_CH_VOLTAGE];
    s_acc_sum[ADC_CH_CURRENT] = s_acc_sum[ADC_CH_VOLTAGE] = 0u;
    s_acc_cnt = 0u;
    __set_PRIMASK(mask);
    return n;
}
static float mean_to_volts(float code) {
    return code * POWER_ADC_VREF / POWER_ADC_FULL_SCALE;
}
void Power_GetSnapshot(power_monitor_t *out) {
    if (out == NULL) return;
    taskENTER_CRITICAL();
    *out = g_power;
    taskEXIT_CRITICAL();
}

/* 仍沿用已配置电池电压端点，不擅自假定未知电池的化学体系和放电曲线。 */
static uint8_t soc_from_voltage(float v) {
    float pct = (v - POWER_V_EMPTY) * 100.0f / (POWER_V_FULL - POWER_V_EMPTY);
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return (uint8_t)(pct + 0.5f);
}
static void soc_update(float v, float current, uint32_t now) {
    if (fabsf(current) >= POWER_SOC_UPDATE_MAX_A || s_cal_active != 0u) {
        s_rest_active = 0u;
        return;
    }
    if (s_rest_active == 0u || fabsf(v - s_rest_anchor_v) > POWER_SOC_STABLE_BAND_V) {
        s_rest_active = 1u;
        s_rest_since_ms = now;
        s_rest_anchor_v = v;
        return;
    }
    if ((uint32_t)(now - s_rest_since_ms) < POWER_SOC_REST_MS) return;
    uint8_t target = soc_from_voltage(v);
    if (s_soc_ready == 0u) {
        s_ratio_pct = target;
        s_soc_ready = 1u;
        s_soc_step_ms = now;
    } else if ((uint32_t)(now - s_soc_step_ms) >= POWER_SOC_STEP_MS) {
        if (target > s_ratio_pct) s_ratio_pct++;
        else if (target < s_ratio_pct) s_ratio_pct--;
        s_soc_step_ms = now;
    }
}
static void low_voltage_update(float v, uint32_t now) {
    if (v <= POWER_LOW_VOLTAGE_V) {
        if (s_low_pending == 0u) { s_low_pending = 1u; s_low_since_ms = now; }
        if ((uint32_t)(now - s_low_since_ms) >= POWER_LOW_VOLTAGE_HOLD_MS)
            g_power.low_voltage = 1u;
    } else {
        s_low_pending = 0u;
        if (v >= POWER_LOW_VOLTAGE_CLEAR_V) g_power.low_voltage = 0u;
    }
}
static void calibration_update(uint32_t sum_i, uint32_t n, uint32_t now) {
    if (g_power_zero_cal_request != 0u) {
        g_power_zero_cal_request = 0u;
        s_cal_active = 1u;
        s_cal_sum = 0u;
        s_cal_count = 0u;
        s_cal_since_ms = now;
        g_power_diag.zero_cal_state = ZERO_CAL_BUSY;
    }
    if (s_cal_active == 0u) return;
    s_cal_sum += sum_i;
    s_cal_count += n;
    if ((uint32_t)(now - s_cal_since_ms) >= POWER_ZERO_CAL_WINDOW_MS) {
        if (s_cal_count >= POWER_ZERO_CAL_SAMPLES) {
            /* 真正的实际样本数，保留均值小数位，不再固定除以 100。 */
            s_zero_offset_v = mean_to_volts((float)s_cal_sum / (float)s_cal_count);
            g_power_diag.zero_offset_v = s_zero_offset_v;
            g_power_diag.zero_cal_state = ZERO_CAL_OK;
            s_filter_ready = 0u;
        } else {
            g_power_diag.zero_cal_state = ZERO_CAL_FAILED;
        }
        s_cal_active = 0u;
    }
}
static void power_measurement_update(uint32_t now) {
    uint32_t sum_i, sum_v;
    uint32_t n = take_accumulator(&sum_i, &sum_v);
    s_telem_n = n;
    calibration_update(sum_i, n, now);
    if (n == 0u) {
        if ((uint32_t)(now - s_last_sample_ms) >= POWER_SAMPLE_STALE_MS) {
            g_power.valid = 0u;
            s_filter_ready = 0u;
            s_rest_active = 0u;
            s_low_pending = 0u;
        }
        return;
    }
    float mean_i = (float)sum_i / (float)n;
    float mean_v = (float)sum_v / (float)n;
    s_telem_raw_i = (uint32_t)(mean_i + 0.5f);
    s_telem_raw_v = (uint32_t)(mean_v + 0.5f);
    s_telem_dma_i = g_adc_dma[ADC_CH_CURRENT];
    s_telem_dma_v = g_adc_dma[ADC_CH_VOLTAGE];
    float current = (mean_to_volts(mean_i) - s_zero_offset_v)
                    / POWER_AMPS_TO_VOLTS * POWER_CURRENT_GAIN_CORRECTION;
#if (POWER_CURRENT_BIDIRECTIONAL == 0)
    if (current < 0.0f) current = 0.0f;
#else
    if (current < POWER_CURRENT_FLOOR_A) current = POWER_CURRENT_FLOOR_A;
#endif
    float v = mean_to_volts(mean_v) * POWER_DIVIDER_RATIO;
    if (s_filter_ready == 0u) {
        s_filtered_v = v; s_filtered_i = current; s_filter_ready = 1u;
    } else {
        s_filtered_v = POWER_LPF_ALPHA * s_filtered_v + (1.0f - POWER_LPF_ALPHA) * v;
        s_filtered_i = POWER_LPF_ALPHA * s_filtered_i + (1.0f - POWER_LPF_ALPHA) * current;
    }
    soc_update(s_filtered_v, s_filtered_i, now);
    low_voltage_update(s_filtered_v, now);
    g_power.battery_v = s_filtered_v;
    g_power.current_a = s_filtered_i;
    g_power.power_w = s_filtered_v * s_filtered_i;
    g_power.ratio_pct = s_ratio_pct;
    g_power.soc_valid = s_soc_ready;
    g_power.stamp_ms = now;
    g_power.valid = 1u;
    s_last_sample_ms = now;
}

/* PB10/PB11 只以开漏方式释放，不强推高电平；每次离线重试最多一次恢复。
 * I2C2 如新增设备，所有用户必须复用 g_oled_i2c_mutex。当前不触碰 I2C1。 */
static HAL_StatusTypeDef oled_bus_recover(void) {
    GPIO_InitTypeDef gpio = {0};
    HAL_StatusTypeDef status;
    g_power_diag.bus_recovery_attempts++;
    if (HAL_I2C_DeInit(&SSD1306_I2C_PORT) != HAL_OK) return HAL_ERROR;
    HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_SCL_PIN | OLED_SDA_PIN, GPIO_PIN_SET);
    gpio.Pin = OLED_SCL_PIN | OLED_SDA_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(OLED_GPIO_PORT, &gpio);
    for (uint32_t pulse = 0u; pulse < 9u; pulse++) {
        if (HAL_GPIO_ReadPin(OLED_GPIO_PORT, OLED_SCL_PIN) == GPIO_PIN_RESET) break;
        if (HAL_GPIO_ReadPin(OLED_GPIO_PORT, OLED_SDA_PIN) == GPIO_PIN_SET) break;
        HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_SCL_PIN, GPIO_PIN_RESET);
        vTaskDelay(pdMS_TO_TICKS(1u));
        HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_SCL_PIN, GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(1u));
    }
    if (HAL_GPIO_ReadPin(OLED_GPIO_PORT, OLED_SCL_PIN) == GPIO_PIN_SET) {
        HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_SCL_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_SDA_PIN, GPIO_PIN_RESET);
        vTaskDelay(pdMS_TO_TICKS(1u));
        HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_SCL_PIN, GPIO_PIN_SET);
        vTaskDelay(pdMS_TO_TICKS(1u));
        if (HAL_GPIO_ReadPin(OLED_GPIO_PORT, OLED_SCL_PIN) == GPIO_PIN_SET)
            HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_SDA_PIN, GPIO_PIN_SET);
    }
    /* 无论线路是否恢复都释放并还原外设；持续短路由后续探测报告离线。 */
    HAL_GPIO_WritePin(OLED_GPIO_PORT, OLED_SCL_PIN | OLED_SDA_PIN, GPIO_PIN_SET);
    status = HAL_I2C_Init(&SSD1306_I2C_PORT);
    if (HAL_GPIO_ReadPin(OLED_GPIO_PORT, OLED_SCL_PIN) == GPIO_PIN_RESET ||
        HAL_GPIO_ReadPin(OLED_GPIO_PORT, OLED_SDA_PIN) == GPIO_PIN_RESET) return HAL_ERROR;
    return status;
}
static void oled_line(uint8_t row, char *text) {
    ssd1306_SetCursor(0u, (uint8_t)(row * OLED_LINE_PITCH));
    (void)ssd1306_WriteString(text, Font_7x10, White);
}
static void oled_number(uint8_t row, const char *label, float value, const char *unit) {
    char text[24];
    int32_t hundred = (int32_t)(fabsf(value) * 100.0f + 0.5f);
    (void)snprintf(text, sizeof(text), "%s%s%ld.%02ld %s", label,
                   (value < 0.0f) ? "-" : "", (long)(hundred / 100),
                   (long)(hundred % 100), unit);
    oled_line(row, text);
}
static void oled_render(void) {
    power_monitor_t data;
    char text[24];
    Power_GetSnapshot(&data);
    ssd1306_Fill(Black);
    if (data.valid == 0u || (uint32_t)(HAL_GetTick() - data.stamp_ms) >= POWER_SAMPLE_STALE_MS) {
        oled_line(0u, "V:-- V"); oled_line(1u, "I:-- A");
        oled_line(2u, "P:-- W"); oled_line(3u, "ADC ERROR");
    } else {
        oled_number(0u, "V:", data.battery_v, "V");
        oled_number(1u, "I:", data.current_a, "A");
        oled_number(2u, "P:", data.power_w, "W");
        if (g_power_diag.zero_cal_state == ZERO_CAL_BUSY) oled_line(3u, "ZERO CAL...");
        else if (data.low_voltage != 0u) oled_line(3u, "LOW BATTERY");
        else if (data.soc_valid == 0u) oled_line(3u, "SOC:-- WAIT REST");
        else {
            (void)snprintf(text, sizeof(text), "SOC:%3u%%", (unsigned)data.ratio_pct);
            oled_line(3u, text);
        }
    }
    ssd1306_UpdateScreen();
}
static void display_task(void *argument) {
    uint32_t last_retry = 0u;
    uint8_t attempted = 0u;
    (void)argument;
    vTaskDelay(pdMS_TO_TICKS(OLED_STARTUP_DELAY_MS));
    for (;;) {
        uint32_t now = HAL_GetTick();
        uint8_t offline = (g_power_diag.oled_state != 1u);
        if (offline != 0u && attempted != 0u &&
            (uint32_t)(now - last_retry) < OLED_RETRY_PERIOD_MS) {
            vTaskDelay(pdMS_TO_TICKS(OLED_REFRESH_PERIOD_MS));
            continue;
        }
        if (xSemaphoreTake(g_oled_i2c_mutex, pdMS_TO_TICKS(OLED_I2C_LOCK_TIMEOUT_MS)) == pdTRUE) {
            HAL_StatusTypeDef status = HAL_OK;
            if (offline != 0u) {
                last_retry = now;
                if (attempted != 0u) status = oled_bus_recover();
                if (status == HAL_OK)
                    status = HAL_I2C_IsDeviceReady(&SSD1306_I2C_PORT, SSD1306_I2C_ADDR, 1u, 5u);
                if (status == HAL_OK) {
                    ssd1306_Init();
                    status = ssd1306_GetStatus();
                }
                if (status == HAL_OK && attempted != 0u) g_power_diag.oled_recoveries++;
                attempted = 1u;
            }
            if (status == HAL_OK) {
                ssd1306_ClearError();
                oled_render();
                status = ssd1306_GetStatus();
            }
            g_power_diag.last_i2c_status = status;
            if (status == HAL_OK) g_power_diag.oled_state = 1u;
            else {
                g_power_diag.oled_errors++;
                g_power_diag.oled_state = 2u;
                last_retry = HAL_GetTick();
            }
            (void)xSemaphoreGive(g_oled_i2c_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(OLED_REFRESH_PERIOD_MS));
    }
}
void OLEDPower_Init(void) {
    memset(&g_power, 0, sizeof(g_power));
    memset(&g_power_diag, 0, sizeof(g_power_diag));
    s_acc_sum[0] = s_acc_sum[1] = s_acc_cnt = 0u;
    s_zero_offset_v = POWER_CURRENT_ZERO_V;
    g_power_diag.zero_offset_v = s_zero_offset_v;
    s_filter_ready = s_soc_ready = s_rest_active = s_low_pending = 0u;
    s_ratio_pct = s_cal_active = 0u;
    g_power_zero_cal_request = 0u;
    s_adc_st = HAL_ADC_Start_DMA(&hadc3, (uint32_t *)(uintptr_t)g_adc_dma, ADC_CH_COUNT);
    if (s_adc_st != HAL_OK) return; /* 任务显示 ADC ERROR，不 assert 卡死 */
    /* 两个半字的半传输中断无业务用途，关闭以减少 IRQ 负担。 */
    __HAL_DMA_DISABLE_IT(hadc3.DMA_Handle, DMA_IT_HT);
#if (POWER_ZERO_CAL_AT_BOOT != 0)
    /* 仅可在确认被测回路真正零电流时启用，采样等待最多 100 ms。 */
    uint32_t started = HAL_GetTick();
    while (s_acc_cnt < POWER_ZERO_CAL_SAMPLES && (uint32_t)(HAL_GetTick() - started) < 100u) {}
    uint32_t sum_i, sum_v;
    uint32_t n = take_accumulator(&sum_i, &sum_v);
    if (n >= POWER_ZERO_CAL_SAMPLES) {
        s_zero_offset_v = mean_to_volts((float)sum_i / (float)n);
        g_power_diag.zero_offset_v = s_zero_offset_v;
        g_power_diag.zero_cal_state = ZERO_CAL_OK;
    } else g_power_diag.zero_cal_state = ZERO_CAL_FAILED;
#endif
    s_last_sample_ms = HAL_GetTick();
}
void StartOLEDPower(void *argument) {
    TickType_t last_wake, last_loop;
    (void)argument;
    vTaskPrioritySet(NULL, POWER_MEASURE_PRIORITY);
    g_oled_i2c_mutex = xSemaphoreCreateMutexStatic(&s_i2c_mutex_storage);
    configASSERT(g_oled_i2c_mutex != NULL);
    TaskHandle_t display = xTaskCreateStatic(display_task, "OLEDDisplay", STACK_OLEDPOWER_WORDS,
                                           NULL, PRIO_OLEDPOWER, s_display_stack, &s_display_tcb);
    configASSERT(display != NULL);
    last_loop = last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(POWER_SAMPLE_PERIOD_MS));
        TickType_t now = xTaskGetTickCount();
        s_telem_period_ms = (uint32_t)((now - last_loop) * 1000u / configTICK_RATE_HZ);
        last_loop = now;
        if (s_telem_period_ms > g_power_diag.loop_max_ms) g_power_diag.loop_max_ms = s_telem_period_ms;
        power_measurement_update(HAL_GetTick());
    }
}

int32_t Power_TelemAdcStatus(void) { return (int32_t)s_adc_st; }
uint32_t Power_TelemSampleCount(void) { return s_telem_n; }
uint32_t Power_TelemRawVoltageCode(void) { return s_telem_raw_v; }
uint32_t Power_TelemRawCurrentCode(void) { return s_telem_raw_i; }
int32_t Power_TelemZeroOffsetMv(void) { return (int32_t)(s_zero_offset_v * 1000.0f); }
int32_t Power_TelemBatteryMv(void) { return (int32_t)(g_power.battery_v * 1000.0f); }
int32_t Power_TelemCurrentMa(void) { return (int32_t)(g_power.current_a * 1000.0f); }
int32_t Power_TelemPowerMw(void) { return (int32_t)(g_power.power_w * 1000.0f); }
uint32_t Power_TelemRawDmaVoltageCode(void) { return s_telem_dma_v; }
uint32_t Power_TelemRawDmaCurrentCode(void) { return s_telem_dma_i; }
uint32_t Power_TelemLoopPeriodMs(void) { return s_telem_period_ms; }
uint32_t Power_TelemOledState(void) { return g_power_diag.oled_state; }
uint32_t Power_TelemSocPct(void) { return g_power.ratio_pct; }
