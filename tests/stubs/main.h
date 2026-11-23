#pragma once
#include <stdint.h>
#include <stddef.h>
typedef struct { int unused; } UART_HandleTypeDef;
typedef struct { int unused; } CAN_HandleTypeDef;
typedef struct { int unused; } SPI_HandleTypeDef;
extern UART_HandleTypeDef huart1, huart5, huart9;
extern CAN_HandleTypeDef hcan1, hcan2, hcan3;
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);
