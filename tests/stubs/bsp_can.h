#pragma once
#include "main.h"
typedef struct CANInstance CANInstance;
typedef struct {
    CAN_HandleTypeDef *can_handle;
    uint32_t tx_id, rx_id;
    void (*can_module_callback)(CANInstance *);
    void *id;
} CAN_Init_Config_s;
#define CAN_ID_STD 0
#define CAN_RTR_DATA 0
struct CANInstance {
    void *id;
    uint8_t rx_len, rx_buff[8], tx_buff[8];
    CAN_HandleTypeDef *can_handle;
    uint32_t tx_id, rx_id;
    struct { uint32_t StdId, IDE, RTR, DLC; } txconf;
    void (*can_module_callback)(CANInstance *);
};
typedef struct { uint32_t bus_off_count; } CANBusStatus;
CANInstance *CANRegister(CAN_Init_Config_s *config);
uint8_t CANTransmit(CANInstance *can, float timeout);
void CANSetDLC(CANInstance *can, uint8_t length);
void CANSetLinkOnline(const CANInstance *can, uint8_t online);
uint8_t CANIsReady(const CANInstance *can);
const CANBusStatus *CANGetBusStatus(CANInstance *can);
void CANServiceTask(void);
