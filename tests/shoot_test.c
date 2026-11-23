#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../modules/motor/DJImotor/dji_motor.h"
#include "../application/shoot/shoot.c"

UART_HandleTypeDef huart1, huart5, huart9;
CAN_HandleTypeDef hcan1, hcan2, hcan3;
static CANBusStatus bus;
static uint8_t bus_ready = 1;
static unsigned dji_motors;
static unsigned zero_position_commands;

uint32_t HAL_GetTick(void) { return 0; }
float DWT_GetDeltaT(uint32_t *count) { ++*count; return 0.001f; }
float DWT_GetTimeline_ms(void) { return 0.0f; }
void DWT_Delay(float seconds) { (void)seconds; }
uint32_t __get_PRIMASK(void) { return 0; }
void __disable_irq(void) {}
void __set_PRIMASK(uint32_t value) { (void)value; }
void CANServiceTask(void) {}
uint8_t CANIsReady(const CANInstance *can) { return can && bus_ready; }
const CANBusStatus *CANGetBusStatus(CANInstance *can) { return can ? &bus : NULL; }
uint8_t CANTransmit(CANInstance *can, float timeout)
{
    (void)timeout;
    if (can && can->tx_buff[7] == 0xfe) ++zero_position_commands;
    return 1;
}
CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    CANInstance *can = calloc(1, sizeof(*can));
    can->id = config->id;
    can->can_handle = config->can_handle;
    can->tx_id = config->tx_id;
    can->rx_id = config->rx_id;
    can->can_module_callback = config->can_module_callback;
    if (can->rx_id >= 0x201 && can->rx_id <= 0x208) ++dji_motors;
    return can;
}

static void DMFeedback(float position, uint8_t state)
{
    CANInstance *can = loader->motor_can_instace;
    uint16_t p = (uint16_t)((position - DM_P_MIN) * 65535.0f / (DM_P_MAX - DM_P_MIN));
    can->rx_len = 8;
    can->rx_buff[0] = state << 4;
    can->rx_buff[1] = p >> 8;
    can->rx_buff[2] = p;
    can->rx_buff[3] = 0x7f;
    can->rx_buff[4] = 0xf7;
    can->rx_buff[5] = 0xff;
    can->can_module_callback(can);
}

static void DMFeedbackTorque(DMMotorInstance *motor, uint16_t torque_code)
{
    CANInstance *can = motor->motor_can_instace;
    can->rx_len = 8;
    can->rx_buff[0] = 0x10;
    can->rx_buff[4] = (uint8_t)((can->rx_buff[4] & 0xf0) | ((torque_code >> 8) & 0x0f));
    can->rx_buff[5] = (uint8_t)torque_code;
    can->can_module_callback(can);
}

static void FrictionFeedback(void)
{
    DJIMotorInstance *motors[] = {friction_l, friction_r};
    for (unsigned i = 0; i < 2; ++i)
    {
        CANInstance *can = motors[i]->motor_can_instance;
        can->rx_len = 8;
        can->can_module_callback(can);
    }
}

static void PublishShootCommand(Shoot_Ctrl_Cmd_s command)
{
    static Publisher_t *publisher;
    if (publisher == NULL)
        publisher = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    PubPushMessage(publisher, &command);
}

static void Stopped(void)
{
    assert(friction_l->stop_flag == MOTOR_STOP && friction_r->stop_flag == MOTOR_STOP);
    assert(loader->stop_flag == MOTOR_STOP);
    DMMotorControl();
    assert(loader->motor_send_mailbox.Kp == 0 && loader->motor_send_mailbox.Kd == 0);
    assert(loader->motor_send_mailbox.velocity_des == 2047);
    assert(loader->motor_send_mailbox.torque_des == 2047);
}

static void Velocity(float expected)
{
    assert(loader->stop_flag == MOTOR_ENALBED);
    assert(loader->mit_config.position_des == 0 && loader->mit_config.kp == 0);
    assert(loader->mit_config.kd == LOADER_MIT_KD && loader->mit_config.torque_des == 0);
    assert(fabsf(loader->mit_config.velocity_des-expected) < 0.0001f);
    DMMotorControl();
    float encoded_velocity = (float)loader->motor_send_mailbox.velocity_des / 4095 *
                             (DM_V_MAX-DM_V_MIN) + DM_V_MIN;
    assert(fabsf(encoded_velocity-expected) < 0.011f);
    assert(loader->motor_send_mailbox.Kp == 0 && loader->motor_send_mailbox.Kd > 0);
}

int main(void)
{
    // Three gimbal motors and this loader fill the production four-motor DM table.
    const Motor_Type_e gimbal_motor_types[] = {DM4340, DM4310, DM4310};
    DMMotorInstance *gimbal_motors[3];
    for (unsigned i = 0; i < 3; ++i)
    {
        Motor_Init_Config_s gimbal = {
            .can_init_config = {.can_handle = &hcan3, .tx_id = 10+i, .rx_id = 0x610+i},
            .motor_type = gimbal_motor_types[i],
        };
        const unsigned zero_commands_before = zero_position_commands;
        gimbal_motors[i] = DMMotorInit(&gimbal);
        assert(gimbal_motors[i]); DMMotorStop(gimbal_motors[i]);
        assert(zero_position_commands == zero_commands_before);
    }
    DMMotorCaliEncoder(gimbal_motors[0]);
    assert(zero_position_commands == 1);
    ShootInit();
    assert(loader && dji_motors == 2);
    assert(loader->control_mode == DMMOTOR_CONTROL_MIT);
    assert(loader->motor_can_instace->can_handle == &hcan2);
    assert(loader->motor_can_instace->tx_id == 0x06);
    assert(loader->motor_can_instace->rx_id == 0x302);
    assert(REDUCTION_RATIO_LOADER == 1.0f);

    DMMotorEnable(loader);
    DMMotorSetMITRef(loader, 0, 0, 0, 0, 10.0f);
    DMMotorControl();
    assert(loader->motor_send_mailbox.torque_des == 4095);
    DMMotorSetMITRef(loader, 0, 0, 0, 0, -10.0f);
    DMMotorControl();
    assert(loader->motor_send_mailbox.torque_des == 0);
    DMMotorStop(loader);
    DMMotorControl();

    DMMotorEnable(gimbal_motors[0]);
    DMMotorSetMITRef(gimbal_motors[0], 0, 0, 0, 0, 12.5f);
    DMMotorControl();
    assert(gimbal_motors[0]->motor_send_mailbox.torque_des == 4095);
    DMMotorStop(gimbal_motors[0]);
    DMMotorControl();

    ShootTask();
    assert(shoot_feedback_data.fault.comm_fault_flags & APP_COMM_FAULT_COMMAND_OFFLINE);
    assert(shoot_feedback_data.fault.motor_offline_flags == 7);
    Stopped();

    Shoot_Ctrl_Cmd_s command = {
        .shoot_mode = SHOOT_OFF, .load_mode = LOAD_STOP, .lid_mode = LID_OPEN,
        .friction_mode = FRICTION_OFF, .bullet_speed = BULLET_SPEED_NONE, .shoot_rate = 8,
    };
    FrictionFeedback(); DMFeedback(0, 1);
    DMFeedbackTorque(loader, 4095);
    assert(fabsf(loader->measure.torque - DM4310_T_MAX) < 0.001f);
    DMFeedbackTorque(gimbal_motors[0], 4095);
    assert(fabsf(gimbal_motors[0]->measure.torque - DM_T_MAX) < 0.001f);
    DMFeedback(0, 1);
    PublishShootCommand(command); ShootTask();
    assert(DaemonGetFaultFlags(shoot_cmd_daemon) == 0); Stopped();
    command.shoot_mode = SHOOT_ON; command.load_mode = LOAD_BURSTFIRE;
    command.friction_mode = FRICTION_ON;
    PublishShootCommand(command); ShootTask(); Velocity(2.0f);
    assert(friction_l->stop_flag == MOTOR_STOP && friction_r->stop_flag == MOTOR_STOP);
    command.shoot_rate = 1;
    PublishShootCommand(command); ShootTask(); Velocity(PI2 / NUM_PER_CIRCLE);
    command.load_mode = LOAD_REVERSE;
    PublishShootCommand(command); ShootTask(); Velocity(-PI2 / NUM_PER_CIRCLE);
    command.shoot_rate = 100000;
    PublishShootCommand(command); ShootTask(); Velocity(-2.0f);
    command.load_mode = LOAD_STOP;
    PublishShootCommand(command); ShootTask();
    assert(loader->stop_flag == MOTOR_STOP);
    DMMotorControl(); assert(loader->motor_send_mailbox.Kd == 0);
    command.load_mode = LOAD_BURSTFIRE;
    command.shoot_rate = 8;
    PublishShootCommand(command); ShootTask(); Velocity(2.0f);
    command.shoot_mode = SHOOT_OFF;
    PublishShootCommand(command); ShootTask(); Stopped();

    command.shoot_mode = SHOOT_ON;
    const loader_mode_e invalid_modes[] = {LOAD_1_BULLET, LOAD_3_BULLET, (loader_mode_e)99};
    for (unsigned i = 0; i < sizeof(invalid_modes)/sizeof(invalid_modes[0]); ++i)
    {
        command.load_mode = invalid_modes[i];
        shoot_cmd_daemon->temp_count = 1;
        PublishShootCommand(command); ShootTask();
        assert(shoot_cmd_daemon->temp_count == 1);
        assert(shoot_feedback_data.fault.comm_fault_flags & APP_COMM_FAULT_INVALID_COMMAND);
        Stopped();
    }
    command.load_mode = LOAD_BURSTFIRE; command.shoot_rate = NAN;
    PublishShootCommand(command); ShootTask();
    assert(shoot_feedback_data.fault.comm_fault_flags & APP_COMM_FAULT_INVALID_COMMAND); Stopped();
    command.shoot_rate = 8;
    PublishShootCommand(command); ShootTask(); Velocity(2.0f);
    shoot_cmd_daemon->temp_count = 0; ShootTask();
    assert(shoot_feedback_data.fault.comm_fault_flags & APP_COMM_FAULT_COMMAND_OFFLINE); Stopped();
    PublishShootCommand(command); ShootTask(); Velocity(2.0f);

    friction_r->daemon->temp_count = 0; ShootTask();
    assert(shoot_feedback_data.fault.motor_offline_flags == MOTOR_2_OFFLINE); Stopped();
    FrictionFeedback(); ShootTask(); Velocity(2.0f);
    loader->motor_daemon->temp_count = 0; ShootTask();
    assert(shoot_feedback_data.fault.motor_offline_flags == MOTOR_3_OFFLINE); Stopped();
    DMFeedback(1, 1); ShootTask(); Velocity(2.0f);
    DMFeedback(1, 8); ShootTask();
    assert(shoot_feedback_data.fault.motor_offline_flags == MOTOR_3_OFFLINE); Stopped();
    DMFeedback(1, 1); ShootTask(); Velocity(2.0f);
    ++bus.bus_off_count; ShootTask();
    assert(shoot_feedback_data.fault.motor_offline_flags == MOTOR_3_OFFLINE); Stopped();
    DMFeedback(1, 1); ShootTask(); Velocity(2.0f);
    bus_ready = 0; ShootTask();
    assert(shoot_feedback_data.fault.motor_offline_flags == 7); Stopped();
    bus_ready = 1; FrictionFeedback(); DMFeedback(1, 1); ShootTask(); Velocity(2.0f);
    puts("PASS production shooter + real DJI/DM drivers: four DM motors, CAN IDs, direct drive, MIT velocity packet, stop, invalid modes, faults and recovery");
}
