#pragma once
#include "main.h"
typedef struct { uint8_t recv_buff[18]; } USARTInstance;
typedef struct { void (*module_callback)(void); UART_HandleTypeDef *usart_handle; uint8_t recv_buff_size; } USART_Init_Config_s;
USARTInstance *USARTRegister(USART_Init_Config_s *config);
void USARTServiceInit(USARTInstance *instance);
