//
// Created by 18045 on 2026/9/28.
//

#ifndef ROBO_IMU_H
#define ROBO_IMU_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t raw_gz;
    float bias_gz;
    float raw_dps;
    float filtered_dps;
    uint8_t ready;
} SensorImuWatch_t;

extern volatile SensorImuWatch_t g_imu_watch;

float SensorImu_GetYawRateDps(void);
float SensorImu_GetYawRateRads(void);
uint8_t SensorImu_IsReady(void);

void StartSensorImuTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif //ROBO_IMU_H
