#pragma once
#include "main.h"
typedef struct CANInstance CANInstance;
typedef struct {
    CAN_HandleTypeDef *can_handle;
    uint32_t tx_id, rx_id;
    void (*can_module_callback)(CANInstance *);
    void *id;
} CAN_Init_Config_s;
struct CANInstance { void *id; uint8_t rx_len, rx_buff[8], tx_buff[8]; };
typedef struct { uint32_t bus_off_count; } CANBusStatus;
CANInstance *CANRegister(CAN_Init_Config_s *config);
uint8_t CANTransmit(CANInstance *can, float timeout);
uint8_t CANIsReady(CANInstance *can);
const CANBusStatus *CANGetBusStatus(CANInstance *can);
void CANServiceTask(void);
