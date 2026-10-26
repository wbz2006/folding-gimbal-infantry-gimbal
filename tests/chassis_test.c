#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include CHASSIS_SOURCE

UART_HandleTypeDef huart1, huart5, huart9;
CAN_HandleTypeDef hcan1, hcan2, hcan3;
static Chassis_Ctrl_Cmd_s incoming;
static uint8_t online = 1;
static DJIMotorInstance motors[4];
uint32_t HAL_GetTick(void) { return 0; }
attitude_t *INS_Init(void) { static attitude_t a; return &a; }
CANCommInstance *CANCommInit(CANComm_Init_Config_s *config)
{
    assert(config->recv_data_len == sizeof(Chassis_Ctrl_Cmd_s));
    assert(config->send_data_len == sizeof(Chassis_Upload_Data_s));
    return (CANCommInstance *)1;
}
uint8_t CANCommReceive(CANCommInstance *comm, void *data)
{ (void)comm; if (!online) return 0; memcpy(data, &incoming, sizeof(incoming)); return 1; }
void CANCommSend(CANCommInstance *comm, uint8_t *data) { (void)comm; (void)data; }
DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config)
{
    DaemonInstance *d = calloc(1, sizeof(*d));
    d->reload_count = config->reload_count ? config->reload_count : 100;
    d->temp_count = d->reload_count;
    d->has_reloaded = !config->require_first_reload;
    return d;
}
void DaemonReload(DaemonInstance *d)
{ if (d) { d->temp_count = d->reload_count; d->has_reloaded = 1; d->offline_reported = 0; } }
uint8_t DaemonIsOnline(DaemonInstance *d) { return d && d->has_reloaded && d->temp_count > 0; }
uint8_t DaemonHasReloaded(DaemonInstance *d) { return d && d->has_reloaded; }
void DaemonSetFeedbackValid(DaemonInstance *d, uint8_t valid) { if (d) d->feedback_valid = valid != 0; }
uint8_t DaemonIsFeedbackValid(DaemonInstance *d) { return d && d->feedback_valid; }
void DaemonSetFaultFlags(DaemonInstance *d, uint32_t flags) { if (d) d->fault_flags = flags; }
void DaemonAddFaultFlags(DaemonInstance *d, uint32_t flags) { if (d) d->fault_flags |= flags; }
void DaemonClearFaultFlags(DaemonInstance *d, uint32_t flags) { if (d) d->fault_flags &= ~flags; }
uint32_t DaemonGetFaultFlags(DaemonInstance *d) { return d ? d->fault_flags : UINT32_MAX; }
DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config)
{
    (void)config;
    static int i;
    DJIMotorInstance *m = &motors[i++];
    Daemon_Init_Config_s daemon_config = {.reload_count = 2, .require_first_reload = 1};
    m->daemon = DaemonRegister(&daemon_config);
    DaemonReload(m->daemon); // 测试夹具表示已收到有效电机反馈
    return m;
}
void DJIMotorStop(DJIMotorInstance *m) { m->stopped = 1; }
void DJIMotorEnable(DJIMotorInstance *m) { m->stopped = 0; }
void DJIMotorSetRef(DJIMotorInstance *m, float ref) { m->ref = ref; }
referee_info_t *UITaskInit(UART_HandleTypeDef *uart, Referee_Interactive_info_t *ui)
{ (void)uart; (void)ui; return NULL; }
SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *config) { (void)config; return NULL; }
int main(void)
{
    ChassisInit();
    for (int mode = CHASSIS_ROTATE; mode <= CHASSIS_FOLLOW_GIMBAL_YAW; ++mode)
        for (int angle = -360; angle <= 360; angle += 15)
        {
            incoming = (Chassis_Ctrl_Cmd_s){.vx=1200, .vy=-500, .offset_angle=angle, .chassis_mode=mode};
            ChassisTask();
            assert(fabsf(chassis_vx - (1200*cosf(angle*DEGREE_2_RAD)+500*sinf(angle*DEGREE_2_RAD))) < 0.001f);
            assert(fabsf(chassis_vy - (1200*sinf(angle*DEGREE_2_RAD)-500*cosf(angle*DEGREE_2_RAD))) < 0.001f);
            float expected = mode == CHASSIS_ROTATE ? 4000 : mode == CHASSIS_NO_FOLLOW ? 0 :
                float_constrain(-50*theta_format(angle), -4500, 4500);
            assert(fabsf(chassis_cmd_recv.wz - expected) < 0.001f);
            for (int i = 0; i < 4; ++i) assert(!motors[i].stopped);
        }
    incoming.chassis_mode = CHASSIS_FOLDED_ROTATE; ChassisTask();
    assert(chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE);
    for (int i = 0; i < 4; ++i) assert(motors[i].stopped);
    incoming.chassis_mode = CHASSIS_ROTATE; online = 0; chassis_cmd_daemon->temp_count = 0; ChassisTask();
    assert(chassis_feedback_data.fault_flags & APP_FAULT_COMMAND_STALE);
    for (int i = 0; i < 4; ++i) assert(motors[i].stopped);
    online = 1; incoming.chassis_mode = CHASSIS_ROTATE;
    motors[0].daemon->temp_count = 0; ChassisTask();
    assert(chassis_feedback_data.fault_flags & APP_FAULT_FEEDBACK_STALE);
    for (int i = 0; i < 4; ++i) assert(motors[i].stopped);
    puts("PASS production chassis: mode/angle cases, stale command/motor fault reports and fail-safe stop");
}
