#include "sensorimu.h"

#include "FreeRTOS.h"
#include "task.h"
#include "i2c.h"
#include "app_config.h"
#include "MPU6050.h"

#define IMU_DPS_TO_RADS 0.0174532925199433f

static volatile float s_yaw_rate_dps;
static volatile uint8_t s_ready;
volatile SensorImuWatch_t g_imu_watch;

static uint8_t sensorimu_calibrate(void)
{
    int64_t sum_z = 0;
    uint32_t i;

    for (i = 0U; i < IMU_GYRO_CAL_SAMPLES; ++i) {
        MPU_readRawData(&hi2c1);
        sum_z += (int64_t)rawData.gz;
        vTaskDelay(pdMS_TO_TICKS(3U));
    }
    gyroCal.z = (float)sum_z / (float)IMU_GYRO_CAL_SAMPLES;
    s_yaw_rate_dps = 0.0f;
    g_imu_watch.bias_gz = gyroCal.z;
    g_imu_watch.filtered_dps = 0.0f;
    return 1U;
}

float SensorImu_GetYawRateDps(void)
{
    return s_yaw_rate_dps;
}

float SensorImu_GetYawRateRads(void)
{
    return s_yaw_rate_dps * IMU_DPS_TO_RADS;
}

uint8_t SensorImu_IsReady(void)
{
    return s_ready;
}

void StartSensorImuTask(void *argument)
{
    TickType_t last_wake;

    (void)argument;
    s_yaw_rate_dps = 0.0f;
    s_ready = 0U;
    g_imu_watch.raw_gz = 0;
    g_imu_watch.bias_gz = 0.0f;
    g_imu_watch.raw_dps = 0.0f;
    g_imu_watch.filtered_dps = 0.0f;
    g_imu_watch.ready = 0U;
    last_wake = xTaskGetTickCount();

    for (;;) {
        if (s_ready == 0U) {
            if (MPU_begin(&hi2c1, AD0_LOW, AFSR_2G, GFSR_500DPS,
                          0.98f, 0.010f) != 0U) {
                (void)sensorimu_calibrate();
                s_ready = 1U;
                g_imu_watch.ready = 1U;
            } else {
                vTaskDelay(pdMS_TO_TICKS(100U));
                continue;
            }
        }

        MPU_readRawData(&hi2c1);
        {
            float raw_dps = ((float)rawData.gz - gyroCal.z) / gScaleFactor;
            float measured_dps = raw_dps * (float)IMU_GYRO_Z_SIGN;
            s_yaw_rate_dps += IMU_GYRO_LPF_ALPHA * (measured_dps - s_yaw_rate_dps);
            g_imu_watch.raw_gz = rawData.gz;
            g_imu_watch.bias_gz = gyroCal.z;
            g_imu_watch.raw_dps = raw_dps;
            g_imu_watch.filtered_dps = s_yaw_rate_dps;
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10U));
    }
}
