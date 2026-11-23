#pragma once
#include "bsp_can.h"
typedef struct { int unused; } SuperCapInstance;
typedef struct { CAN_Init_Config_s can_config; } SuperCap_Init_Config_s;
SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *config);
