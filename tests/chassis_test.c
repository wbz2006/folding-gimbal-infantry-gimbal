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
static unsigned imu_fault_reads, cap_initializations;
uint32_t HAL_GetTick(void) { return 0; }
uint8_t INS_GetFaultFlags(void) { ++imu_fault_reads; return 0; }
uint8_t CANIsReady(const CANInstance *can) { return can != NULL; }
attitude_t *INS_Init(void) { static attitude_t a; return &a; }
attitude_t *INS_GetData(void) { return INS_Init(); }
CANCommInstance *CANCommInit(CANComm_Init_Config_s *config)
{
    assert(config->can_config.can_handle == &hcan2);
    assert(config->can_config.tx_id == 0x311 && config->can_config.rx_id == 0x312);
    assert(config->recv_data_len == sizeof(Chassis_Ctrl_Cmd_s));
    assert(config->send_data_len == sizeof(Chassis_Upload_Data_s));
    static CANCommInstance comm;
    Daemon_Init_Config_s daemon_config = {.reload_count = 2, .require_first_reload = 1};
    comm.comm_daemon = DaemonRegister(&daemon_config);
    return &comm;
}
uint8_t CANCommReceive(CANCommInstance *comm, void *data)
{ (void)comm; if (!online) return 0; memcpy(data, &incoming, sizeof(incoming)); return 1; }
void CANCommSend(CANCommInstance *comm, uint8_t *data) { (void)comm; (void)data; }
uint32_t __get_PRIMASK(void) { return 0; }
void __disable_irq(void) {}
void __set_PRIMASK(uint32_t value) { (void)value; }
void CANServiceTask(void) {}

DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config)
{
    static int i;
    const unsigned motor_ids[] = {2, 1, 3, 4};
    assert(config->can_init_config.can_handle == &hcan3);
    assert(config->can_init_config.tx_id == motor_ids[i]);
    DJIMotorInstance *m = &motors[i++];
    m->motor_settings = config->controller_setting_init_config;
    Daemon_Init_Config_s daemon_config = {.reload_count = 2, .require_first_reload = 1};
    m->daemon = DaemonRegister(&daemon_config);
    DaemonReload(m->daemon); // 测试夹具表示已收到有效电机反馈
    m->motor_can_instance = (CANInstance *)1;
    return m;
}
void DJIMotorStop(DJIMotorInstance *m) { m->stopped = 1; m->stop_flag = MOTOR_STOP; }
void DJIMotorEnable(DJIMotorInstance *m) { m->stopped = 0; m->stop_flag = MOTOR_ENALBED; }
void DJIMotorResetPIDState(DJIMotorInstance *m) { (void)m; }
void DJIMotorSetRef(DJIMotorInstance *m, float ref) { m->ref = ref; }
referee_info_t *UITaskInit(UART_HandleTypeDef *uart, Referee_Interactive_info_t *ui)
{ (void)uart; (void)ui; return NULL; }
SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *config) { (void)config; ++cap_initializations; return NULL; }
int main(void)
{
    ChassisInit();
    assert(cap_initializations == 0);
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
    const float folded_speeds[] = {-5280, -1200, 0, 1800, 5280};
    for (unsigned i = 0; i < sizeof(folded_speeds) / sizeof(folded_speeds[0]); ++i)
    {
        incoming = (Chassis_Ctrl_Cmd_s){.vx=1200, .vy=-500, .offset_angle=0,
            .chassis_mode=CHASSIS_FOLDED_ROTATE, .wz=folded_speeds[i]};
        ChassisTask();
        assert(chassis_cmd_recv.chassis_mode == CHASSIS_FOLDED_ROTATE);
        assert(fabsf(chassis_cmd_recv.wz - 10.0f * float_constrain(folded_speeds[i], -4500, 4500)) < 0.001f);
        assert(chassis_vx == 1200 && chassis_vy == -500);
        for (int j = 0; j < 4; ++j) assert(!motors[j].stopped);
    }
    incoming.chassis_mode = (chassis_mode_e)5; ChassisTask();
    assert(chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE);
    for (int i = 0; i < 4; ++i) assert(motors[i].stopped);
    incoming.chassis_mode = CHASSIS_ROTATE; online = 0; chassis_cmd_daemon->temp_count = 0; ChassisTask();
    assert(chassis_feedback_data.fault.comm_fault_flags & APP_COMM_FAULT_COMMAND_OFFLINE);
    for (int i = 0; i < 4; ++i) assert(motors[i].stopped);
    online = 1; incoming.chassis_mode = CHASSIS_ROTATE;
    motors[0].daemon->temp_count = 0; ChassisTask();
    assert(chassis_feedback_data.fault.motor_offline_flags & MOTOR_1_OFFLINE);
    for (int i = 0; i < 4; ++i) assert(motors[i].stopped);
    assert(imu_fault_reads > 0 && chassis_feedback_data.fault.imu_fault_flags == APP_IMU_FAULT_NONE);
    puts("PASS production chassis: mode/angle cases, stale command/motor fault reports and fail-safe stop");
}
