/** 电源测量与 OLED 显示。CubeMX 创建测量入口，入口再创建独立静态显示任务。 */
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

typedef struct {
    volatile float current_a;
    volatile float battery_v;
    volatile float power_w;
    volatile uint8_t ratio_pct;
    volatile uint8_t valid;       /* 0: 不得将保留的旧数值当作当前测量 */
    volatile uint8_t soc_valid;   /* 等待低负载稳定后才首次发布 SOC */
    volatile uint8_t low_voltage; /* 实测电压告警，不依赖 SOC 是否冻结 */
    volatile uint32_t stamp_ms;   /* 最近有效样本的 HAL 毫秒时间戳 */
} power_monitor_t;

typedef struct {
    volatile uint32_t oled_state; /* 0 启动，1 最近通信成功，2 离线 */
    volatile uint32_t oled_errors;
    volatile uint32_t oled_recoveries; /* 离线后成功重新初始化次数 */
    volatile uint32_t bus_recovery_attempts;
    volatile uint32_t last_i2c_status; /* HAL_OK/ERROR/BUSY/TIMEOUT */
    volatile uint32_t loop_max_ms;
    volatile uint32_t zero_cal_state; /* 0 固定零点，1 标定中，2 成功，3 失败 */
    volatile float zero_offset_v; /* 用于持久化零点，单位 V，保留亚毫伏精度 */
} power_diag_t;
extern power_monitor_t g_power;
extern power_diag_t g_power_diag;
/* 在 Ozone 写 1 请求标定；仅在确认被测支路真实零电流且保持静止时使用。
 * 新零点只存 RAM；确认后将其电压填入 POWER_CURRENT_ZERO_V，重编译持久化。 */
extern volatile uint8_t g_power_zero_cal_request;
extern SemaphoreHandle_t g_oled_i2c_mutex;
void OLEDPower_Init(void); /* 调度器启动前：启动 ADC，不创建重复测量任务 */
void StartOLEDPower(void *argument); /* 测量任务入口，10 ms；显示任务 200 ms */
void Power_GetSnapshot(power_monitor_t *out);
int32_t Power_TelemAdcStatus(void);
uint32_t Power_TelemSampleCount(void);
uint32_t Power_TelemRawVoltageCode(void);
uint32_t Power_TelemRawCurrentCode(void);
int32_t Power_TelemZeroOffsetMv(void);
int32_t Power_TelemBatteryMv(void);
int32_t Power_TelemCurrentMa(void);
int32_t Power_TelemPowerMw(void);
uint32_t Power_TelemRawDmaVoltageCode(void);
uint32_t Power_TelemRawDmaCurrentCode(void);
uint32_t Power_TelemLoopPeriodMs(void);
uint32_t Power_TelemOledState(void);
uint32_t Power_TelemSocPct(void);
#ifdef __cplusplus
}
#endif
#endif
