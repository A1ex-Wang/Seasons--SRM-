

#include "control_time.h"
#include "main.h"
static uint32_t dwt_cpu_hz;
void ControlTime_DWT_Init(uint32_t cpu_hz)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    dwt_cpu_hz = cpu_hz;
}
float ControlTime_DWT_GetDelta(uint32_t *last_count)
{
    uint32_t now_count;
    uint32_t delta_count;
    if (last_count == 0 || dwt_cpu_hz == 0U)
        return 0.0f;
    now_count = DWT->CYCCNT;
    delta_count = now_count - *last_count;
    *last_count = now_count;
    return (float)delta_count / (float)dwt_cpu_hz;
}
