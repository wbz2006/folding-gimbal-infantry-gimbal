#pragma once
#ifndef DJI_MOTOR_H
#include "bsp_can.h"
#include "motor_def.h"
#include "daemon.h"
#define ECD_ANGLE_COEF_DJI 0.043945f
typedef struct { float total_angle, speed_aps; } DJI_Test_Measure_s;
typedef struct { float ref; int stopped; Motor_Working_Type_e stop_flag; Motor_Control_Setting_s motor_settings; DaemonInstance *daemon; CANInstance *motor_can_instance; DJI_Test_Measure_s measure; } DJIMotorInstance;
DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config);
void DJIMotorStop(DJIMotorInstance *motor);
void DJIMotorEnable(DJIMotorInstance *motor);
void DJIMotorSetRef(DJIMotorInstance *motor, float ref);
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e loop);
void DJIMotorResetPIDState(DJIMotorInstance *motor);
#endif
