#pragma once
#include <stdint.h>
float DWT_GetDeltaT(uint32_t *count);
float DWT_GetTimeline_ms(void);
void DWT_Delay(float seconds);
