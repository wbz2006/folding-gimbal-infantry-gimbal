#include <assert.h>
#include <stdio.h>
#include <string.h>
#include DJI_SOURCE

CAN_HandleTypeDef hcan1, hcan2, hcan3;
static uint8_t sent[8];
static unsigned transmissions;
uint32_t __get_PRIMASK(void) { return 0; }
void __disable_irq(void) {}
void __set_PRIMASK(uint32_t value) { (void)value; }
void CANServiceTask(void) {}
uint32_t HAL_GetTick(void) { return 0; }
float DWT_GetDeltaT(uint32_t *count) { ++*count; return 0.001f; }
uint8_t CANIsReady(const CANInstance *can) { return can != NULL; }
CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    CANInstance *can = calloc(1, sizeof(*can));
    can->id = config->id;
    can->can_handle = config->can_handle;
    can->tx_id = config->tx_id;
    can->rx_id = config->rx_id;
    return can;
}
uint8_t CANTransmit(CANInstance *can, float timeout)
{
    (void)timeout;
    ++transmissions;
    memcpy(sent, can->tx_buff, sizeof(sent));
    return 1;
}

int main(void)
{
    Motor_Init_Config_s config = {
        .can_init_config = {.can_handle = &hcan2, .tx_id = 1},
        .motor_type = M3508,
        .controller_setting_init_config = {
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP | CURRENT_LOOP,
        },
        .controller_param_init_config = {
            .angle_PID = {.Kp = 2, .Ki = 10, .MaxOut = 1000},
            .speed_PID = {.Kp = 3, .Ki = 10, .MaxOut = 1000},
            .current_PID = {.Kp = 4, .Ki = 10, .MaxOut = 10000},
        },
    };
    DJIMotorInstance *motor = DJIMotorInit(&config);
    config.can_init_config.tx_id = 2;
    DJIMotorInstance *neighbor = DJIMotorInit(&config);
    CANInstance *can = motor->motor_can_instance;
    assert(!DaemonIsOnline(motor->daemon));
    can->rx_len = 7;
    DecodeDJIMotor(can);
    assert(!DaemonHasReloaded(motor->daemon));
    can->rx_len = 8; can->rx_buff[0] = 0x20;
    DecodeDJIMotor(can); // Encoder 8192 is outside the DJI protocol range.
    assert(!DaemonHasReloaded(motor->daemon));
    can->rx_buff[0] = 0x0f; can->rx_buff[1] = 0xff;
    can->rx_buff[2] = 0xff; can->rx_buff[3] = 0x9c; // -100 rpm
    DecodeDJIMotor(can);
    assert(DaemonIsOnline(motor->daemon));
    assert(motor->measure.ecd == 4095 && motor->measure.speed_aps < 0);
    DJI_Motor_Measure_s saved = motor->measure;
    motor->daemon->temp_count = 1;
    can->rx_len = 6; DecodeDJIMotor(can);
    assert(motor->daemon->temp_count == 1);
    assert(memcmp(&saved, &motor->measure, sizeof(saved)) == 0);
    can->rx_len = 8; can->rx_buff[0] = 0x20; DecodeDJIMotor(can);
    assert(motor->daemon->temp_count == 1);
    assert(memcmp(&saved, &motor->measure, sizeof(saved)) == 0);
    can->rx_buff[0] = 0x1f; can->rx_buff[1] = 0xff; DecodeDJIMotor(can);
    assert(motor->measure.ecd == 8191);
    can->rx_buff[0] = can->rx_buff[1] = 0; DecodeDJIMotor(can);
    assert(motor->measure.ecd == 0 && motor->measure.total_round == 1);

    DJIMotorSetRef(motor, 50);
    DJIMotorSetRef(neighbor, 20);
    DJIMotorControl();
    assert(sent[0] || sent[1]);
    assert(sent[2] || sent[3]);
    Motor_Controller_s before_stop = motor->motor_controller;
    DJIMotorStop(motor);
    unsigned before_count = transmissions;
    for (int i = 0; i < 20; ++i) DJIMotorControl();
    assert(transmissions == before_count + 20);
    assert(sent[0] == 0 && sent[1] == 0);
    assert(sent[2] || sent[3]);
    assert(memcmp(&before_stop, &motor->motor_controller, sizeof(before_stop)) == 0);

    DJIMotorEnable(motor);
    assert(motor->motor_controller.angle_PID.Iout == 0);
    assert(motor->motor_controller.speed_PID.Iout == 0);
    assert(motor->motor_controller.current_PID.Iout == 0);
    assert(motor->motor_controller.angle_PID.Kp == 2);
    assert(motor->motor_controller.speed_PID.Kp == 3);
    assert(motor->motor_controller.current_PID.Kp == 4);
    DJIMotorControl();
    assert(sent[0] || sent[1]);
    Motor_Controller_s running = motor->motor_controller;
    DJIMotorEnable(motor);
    assert(memcmp(&running, &motor->motor_controller, sizeof(running)) == 0);
    puts("PASS production DJI: frame/encoder validation, stopped PID freeze, zero slot, neighbor output, reset only on reenable");
}
