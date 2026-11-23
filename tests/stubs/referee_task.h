#pragma once
#include "main.h"
typedef struct { int unused; } referee_info_t;
typedef struct { int unused; } Referee_Interactive_info_t;
referee_info_t *UITaskInit(UART_HandleTypeDef *uart, Referee_Interactive_info_t *ui);
