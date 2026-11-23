/* Run production CMD, Gimbal, DM, DBUS, CAN communication and PID code. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bsp_usart.h"
#include "robot_def.h"
#ifdef TEST_MECHANICAL_CENTER
#undef GIMBAL_YAW_MECHANICAL_CENTER_DEG
#define GIMBAL_YAW_MECHANICAL_CENTER_DEG 123.0f
#endif
#include "../application/cmd/robot_cmd.c"
#include "../application/gimbal/gimbal.c"
#include "../modules/motor/DMmotor/dmmotor.c"

UART_HandleTypeDef huart1, huart5, huart9;
CAN_HandleTypeDef hcan1, hcan2, hcan3;
static uint32_t now_ms;
static CANBusStatus bus;
static attitude_t imu;
static USARTInstance receiver;
static void (*remote_callback)(void);
static uint8_t remote_active, imu_online = 1;
static uint8_t left_switch = RC_SW_DOWN, right_switch = RC_SW_UP;
static int16_t sticks[4], dial_input;
static float positions[3], velocities[3];
static uint8_t states[3] = {1, 1, 1}, fresh[3] = {1, 1, 1};
static unsigned zero_position_commands, enable_commands[16];
static uint32_t last_enable_time[16];
static uint8_t motor_frames[16][8];
static bool check_enable_stopped;
static Publisher_t *shoot_test_pub;

#define NEAR(actual, expected) assert(fabsf((actual) - (expected)) < 0.001f)

uint32_t HAL_GetTick(void) { return now_ms; }
uint32_t __get_PRIMASK(void) { return 0; }
void __disable_irq(void) {}
void __set_PRIMASK(uint32_t value) { (void)value; }
float DWT_GetDeltaT(uint32_t *count) { *count = now_ms; return 0.005f; }
float DWT_GetTimeline_ms(void) { return now_ms; }
void DWT_Delay(float seconds) { (void)seconds; }
attitude_t *INS_GetData(void) { return &imu; }
uint8_t INS_IsFeedbackValid(void) { return imu_online; }
void CANServiceTask(void) {}
uint8_t CANIsReady(const CANInstance *can) { return can != NULL; }
const CANBusStatus *CANGetBusStatus(CANInstance *can) { return &bus; }
void CANSetDLC(CANInstance *can, uint8_t length) { can->txconf.DLC = length; }
void CANSetLinkOnline(const CANInstance *can, uint8_t online) {}

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    CANInstance *can = calloc(1, sizeof(*can));
    can->id = config->id;
    can->can_handle = config->can_handle;
    can->tx_id = config->tx_id;
    can->rx_id = config->rx_id;
    can->can_module_callback = config->can_module_callback;
    can->txconf.DLC = 8;
    return can;
}

uint8_t CANTransmit(CANInstance *can, float timeout)
{
    const uint8_t enable[8] = {255, 255, 255, 255, 255, 255, 255, DM_CMD_MOTOR_MODE};
    const uint8_t zero_position[8] = {255, 255, 255, 255, 255, 255, 255, DM_CMD_ZERO_POSITION};
    if (memcmp(can->tx_buff, zero_position, sizeof(zero_position)) == 0)
        ++zero_position_commands;
    if (memcmp(can->tx_buff, enable, sizeof(enable)) == 0)
    {
        if (check_enable_stopped)
        {
            assert(yaw_motor->stop_flag == MOTOR_STOP);
            assert(upper_pitch_motor->stop_flag == MOTOR_STOP);
            assert(lower_pitch_motor->stop_flag == MOTOR_STOP);
            assert(now_ms - last_enable_time[can->tx_id] >= GIMBAL_MOTOR_ENABLE_RETRY_MS);
        }
        ++enable_commands[can->tx_id];
        last_enable_time[can->tx_id] = now_ms;
    }
    if (can->tx_id < 16)
        memcpy(motor_frames[can->tx_id], can->tx_buff, sizeof(can->tx_buff));
    return 1;
}

USARTInstance *USARTRegister(USART_Init_Config_s *config)
{
    remote_callback = config->module_callback;
    return &receiver;
}
void USARTServiceInit(USARTInstance *instance) { (void)instance; }

static void remote_frame(void)
{
    memset(receiver.recv_buff, 0, sizeof(receiver.recv_buff));
    for (unsigned channel = 0; channel < 4; ++channel)
    {
        const unsigned value = 1024 + sticks[channel];
        for (unsigned bit = 0; bit < 11; ++bit)
            if (value & (1u << bit))
                receiver.recv_buff[(channel * 11 + bit) / 8] |= 1u << ((channel * 11 + bit) % 8);
    }
    receiver.recv_buff[5] |= (left_switch << 6) | (right_switch << 4);
    const unsigned dial = 1024 + dial_input;
    receiver.recv_buff[16] = dial;
    receiver.recv_buff[17] = dial >> 8;
    receiver.recv_len = 18;
    remote_callback();
}

static void motor_feedback(DMMotorInstance *motor, float position, float velocity, uint8_t state)
{
    CANInstance *can = motor->motor_can_instace;
    const uint16_t p = float_to_uint(position, DM_P_MIN, DM_P_MAX, 16);
    const uint16_t v = float_to_uint(velocity, DM_V_MIN, DM_V_MAX, 12);
    const uint8_t frame[8] = {(state << 4) | can->tx_id, p >> 8, p, v >> 4,
                              (v << 4) | 0x07, 0xff, 25, 25};
    memcpy(can->rx_buff, frame, sizeof(frame));
    can->rx_len = sizeof(frame);
    can->can_module_callback(can);
}

static void step(void)
{
    now_ms += 5;
    DMMotorInstance *motors[] = {yaw_motor, upper_pitch_motor, lower_pitch_motor};
    for (unsigned i = 0; i < 3; ++i)
        if (fresh[i])
            motor_feedback(motors[i], positions[i], velocities[i], states[i]);
    if (remote_active)
        remote_frame();
    if (now_ms % DAEMON_TASK_PERIOD_MS == 0)
        DaemonTask();
    Shoot_Upload_Data_s shoot_feed = {0};
    PubPushMessage(shoot_test_pub, &shoot_feed);
    RobotCMDTask();
    GimbalTask();
    DMMotorControl();
}

static void live_translation(int phase)
{
    sticks[0] = phase % 2 ? 120 : -80;
    sticks[1] = phase % 3 ? -70 : 0;
    step();
    NEAR(chassis_cmd_send.vx, 10.0f * sticks[0]);
    NEAR(chassis_cmd_send.vy, 10.0f * sticks[1]);
    NEAR(chassis_cmd_send.offset_angle, theta_format(
        gimbal_fetch_data.yaw_motor_single_round_angle - GIMBAL_YAW_MECHANICAL_CENTER_DEG));
}

static void stopped(void)
{
    assert(!fold_ctrl.active);
    assert(yaw_motor->stop_flag == MOTOR_STOP);
    assert(upper_pitch_motor->stop_flag == MOTOR_STOP);
    assert(lower_pitch_motor->stop_flag == MOTOR_STOP);
}

static void startup_and_enable(void)
{
    fresh[0] = fresh[1] = fresh[2] = 0;
    step();
    stopped();
    remote_active = 1;
    step();
    stopped();
    assert(gimbal_feedback_data.fault.motor_offline_flags == 7);
    fresh[0] = fresh[1] = fresh[2] = 1;
    states[0] = states[1] = states[2] = 0;
    check_enable_stopped = true;
    for (unsigned i = 0; i < 60; ++i)
    {
        step();
        stopped();
    }
    assert(enable_commands[2] >= 3 && enable_commands[3] >= 3 && enable_commands[4] >= 3);
    check_enable_stopped = false;
    states[0] = states[1] = states[2] = 1;
    imu.Pitch = 8;
    positions[0] = (GIMBAL_YAW_MECHANICAL_CENTER_DEG + 45) * DEGREE_2_RAD;
    positions[1] = -3.3f;
    positions[2] = 0.4f;
    step();
    assert(fold_ctrl.active && fold_ctrl.step == FOLD_STEP_LEVEL_UPPER_PITCH);
    NEAR(fold_ctrl.lower_ref, lower_pitch_motor->measure.position);
    puts("PASS startup waits for three real motor feedbacks; enable retries are stopped and rate limited");
}

static void folding_sequence(void)
{
    const float held_lower = fold_ctrl.lower_ref;
    const uint32_t start = fold_ctrl.action_start;
    imu.YawTotalAngle = 1376.25f;
    sticks[2] = 660;
    sticks[3] = 660;
    for (int i = 0; i < 30; ++i)
    {
        right_switch = i % 3 == 0 ? RC_SW_UP : i % 3 == 1 ? RC_SW_MID : RC_SW_DOWN;
        live_translation(i);
        assert(fold_ctrl.step == FOLD_STEP_LEVEL_UPPER_PITCH);
        NEAR(lower_pitch_motor->pid_ref, held_lower);
        NEAR(upper_pitch_motor->pid_ref, 0);
        assert(yaw_motor->stop_flag == MOTOR_STOP);
        assert(chassis_cmd_send.chassis_mode == CHASSIS_NO_FOLLOW);
        NEAR(chassis_cmd_send.wz, 0);
        NEAR(gimbal_cmd_send.pitch, 0);
        assert(fold_ctrl.action_start == start);
    }
    imu.Pitch = 0;
    imu.Gyro[0] = 0.6f;
    for (int i = 0; i < 25; ++i) live_translation(i);
    assert(fold_ctrl.step == FOLD_STEP_LEVEL_UPPER_PITCH);
    imu.Gyro[0] = 0;
    for (int i = 0; i < 25; ++i) live_translation(i);
    assert(fold_ctrl.step == FOLD_STEP_CENTER_YAW);
    assert(yaw_motor->stop_flag == MOTOR_ENALBED);
    NEAR(yaw_motor->pid_ref, imu.YawTotalAngle - theta_format(
        yaw_motor->measure.angle_single_round - GIMBAL_YAW_MECHANICAL_CENTER_DEG));
    for (int i = 0; i < 25; ++i)
    {
        live_translation(i);
        NEAR(lower_pitch_motor->pid_ref, held_lower);
        assert(fold_ctrl.step == FOLD_STEP_CENTER_YAW);
    }
    // IMU zero alone cannot satisfy mechanical centering.
    imu.YawTotalAngle = 0;
    step();
    assert(fold_ctrl.step == FOLD_STEP_CENTER_YAW);
    imu.YawTotalAngle = -720.5f;
    positions[0] = GIMBAL_YAW_MECHANICAL_CENTER_DEG * DEGREE_2_RAD;
    velocities[0] = 0.5f;
    for (int i = 0; i < 25; ++i) live_translation(i);
    assert(fold_ctrl.step == FOLD_STEP_CENTER_YAW);
    velocities[0] = 0;
    for (int i = 0; i < 25; ++i) live_translation(i);
    assert(fold_ctrl.step == FOLD_STEP_MOVE_LOWER_PITCH);
    assert(yaw_motor->stop_flag == MOTOR_STOP);

    imu.Pitch = 5;
    step();
    assert(fold_ctrl.step == FOLD_STEP_LEVEL_UPPER_PITCH);
    const float paused_lower = fold_ctrl.lower_ref;
    positions[2] += 0.01f;
    step();
    NEAR(fold_ctrl.lower_ref, paused_lower);
    imu.Pitch = 0;
    for (int i = 0; i < 1500 && fold_ctrl.state != GIMBAL_FOLDED; ++i)
    {
        positions[2] = lower_pitch_motor->pid_ref;
        live_translation(i);
        assert(shoot_cmd_send.shoot_mode == SHOOT_OFF);
    }
    assert(fold_ctrl.state == GIMBAL_FOLDED);
    puts("PASS level -> mechanical Yaw center -> lower fold, with stable gates, pause and live translation");
}

static void folded_controls(void)
{
    for (int i = 0; i < 3; ++i)
    {
        right_switch = i == 0 ? RC_SW_UP : i == 1 ? RC_SW_MID : RC_SW_DOWN;
        sticks[2] = i == 0 ? -660 : i == 1 ? 0 : 660;
        live_translation(i);
        assert(chassis_cmd_send.chassis_mode == CHASSIS_FOLDED_ROTATE);
        NEAR(chassis_cmd_send.wz, i == 0 ? -4500 : i == 1 ? 0 : 4500);
        assert(yaw_motor->stop_flag == MOTOR_STOP);
        NEAR(upper_pitch_motor->pid_ref, 0);
    }
    const float target = gimbal_cmd_send.yaw;
    for (int i = 0; i < 12; ++i) live_translation(i);
    NEAR(gimbal_cmd_send.yaw, target);
    sticks[0] = sticks[1] = sticks[2] = sticks[3] = 0;
    step();
    NEAR(chassis_cmd_send.vx, 0);
    NEAR(chassis_cmd_send.vy, 0);
    NEAR(chassis_cmd_send.wz, 0);
    puts("PASS folded mode ignores right switch, turns chassis with left stick and never turns Yaw");
}

static void unfolding_and_deployed(void)
{
    left_switch = RC_SW_MID;
    right_switch = RC_SW_DOWN;
    sticks[2] = 660;
    positions[2] = 1.1f;
    live_translation(1);
    assert(fold_ctrl.state == GIMBAL_UNFOLDING);
    for (int i = 0; i < 1100; ++i)
    {
        live_translation(i);
        assert(fold_ctrl.state == GIMBAL_UNFOLDING);
        assert(chassis_cmd_send.chassis_mode == CHASSIS_NO_FOLLOW);
        assert(yaw_motor->stop_flag == MOTOR_STOP);
    }
    // The reference is at deploy, but actual lower feedback must reach deploy too.
    NEAR(lower_pitch_motor->pid_ref, 0);
    positions[2] = 0;
    for (int i = 0; i < 100 && fold_ctrl.state != GIMBAL_DEPLOYED; ++i) live_translation(i);
    assert(fold_ctrl.state == GIMBAL_DEPLOYED);
    sticks[2] = sticks[3] = 0;
    live_translation(1);
    NEAR(gimbal_cmd_send.yaw, imu.YawTotalAngle);
    assert(yaw_motor->stop_flag == MOTOR_ENALBED);
    NEAR(yaw_motor->pid_ref, imu.YawTotalAngle);
    const chassis_mode_e modes[] = {CHASSIS_NO_FOLLOW, CHASSIS_FOLLOW_GIMBAL_YAW, CHASSIS_ROTATE};
    for (int i = 0; i < 3; ++i)
    {
        right_switch = i == 0 ? RC_SW_UP : i == 1 ? RC_SW_MID : RC_SW_DOWN;
        live_translation(i);
        assert(chassis_cmd_send.chassis_mode == modes[i]);
    }
    sticks[2] = 100;
    sticks[3] = -100;
    const float yaw = gimbal_cmd_send.yaw;
    step();
    NEAR(gimbal_cmd_send.yaw, yaw - 0.1f);
    NEAR(yaw_motor->pid_ref, gimbal_cmd_send.yaw);
    NEAR(upper_pitch_motor->pid_ref, gimbal_cmd_send.pitch);
    for (int i = 0; i < 650; ++i) step();
    NEAR(gimbal_cmd_send.pitch, PITCH_MAX_ANGLE);
    sticks[3] = 660;
    for (int i = 0; i < 220; ++i) step();
    NEAR(gimbal_cmd_send.pitch, PITCH_MIN_ANGLE);
    puts("PASS unfold translates throughout, checks deploy feedback, then restores IMU control and pitch limits");
}

static void reversals_faults_and_stop(void)
{
    for (int i = 0; i < 20; ++i)
    {
        left_switch = i % 2 ? RC_SW_MID : RC_SW_DOWN;
        live_translation(i);
        assert(fold_ctrl.state == (i % 2 ? GIMBAL_UNFOLDING : GIMBAL_FOLDING));
    }
    left_switch = RC_SW_DOWN;
    imu.Pitch = 8;
    live_translation(0);
    const uint32_t start = fold_ctrl.action_start;
    for (int i = 0; i < 10; ++i) live_translation(i);
    assert(fold_ctrl.action_start == start);
    imu_online = 0;
    step();
    assert(gimbal_feedback_data.fault.imu_fault_flags & APP_IMU_FAULT_OFFLINE);
    assert(gimbal_feedback_data.action_fault_flags == GIMBAL_ACTION_FAULT_NONE);
    assert(fold_ctrl.active);
    imu_online = 1;
    now_ms = start + GIMBAL_ACTION_TIMEOUT_MS;
    step();
    stopped();
    assert(gimbal_feedback_data.action_fault_flags & GIMBAL_ACTION_FAULT_TIMEOUT);
    right_switch = RC_SW_MID;
    for (int i = 0; i < 3; ++i) step();
    assert(gimbal_feedback_data.action_fault_flags & GIMBAL_ACTION_FAULT_TIMEOUT);
    assert(chassis_cmd_send.chassis_mode == CHASSIS_ZERO_FORCE);
    right_switch = RC_SW_UP;
    step();
    assert(gimbal_feedback_data.action_fault_flags == GIMBAL_ACTION_FAULT_NONE);
    step();
    assert(fold_ctrl.active);

    states[2] = 0;
    step();
    stopped();
    positions[2] = 0.7f;
    states[2] = 1;
    step();
    assert(fold_ctrl.active);
    NEAR(fold_ctrl.lower_ref, lower_pitch_motor->measure.position);
    dial_input = 400;
    step();
    stopped();
    assert(chassis_cmd_send.chassis_mode == CHASSIS_ZERO_FORCE);
    step();
    stopped();
    dial_input = 0;
    step();
    assert(fold_ctrl.active);
    left_switch = RC_SW_UP;
    step();
    NEAR(chassis_cmd_send.vx, 0);
    NEAR(chassis_cmd_send.vy, 0);
    remote_active = 0;
    for (int i = 0; i < 160; ++i) step();
    stopped();
    assert(RobotCMDGetFault()->control_fault_flags & APP_CONTROL_FAULT_REMOTE_OFFLINE);
    remote_active = 1;
    left_switch = RC_SW_DOWN;
    step();
    assert(fold_ctrl.active);

    imu.Pitch = NAN;
    step();
    stopped();
    assert(gimbal_feedback_data.fault.imu_fault_flags & APP_IMU_FAULT_DATA_INVALID);
    imu.Pitch = 0;
    step();
    assert(fold_ctrl.active);
    Gimbal_Ctrl_Cmd_s invalid = gimbal_cmd_recv;
    invalid.yaw = NAN;
    PubPushMessage(gimbal_cmd_pub, &invalid);
    GimbalTask();
    stopped();
    assert(gimbal_feedback_data.fault.comm_fault_flags & APP_COMM_FAULT_INVALID_COMMAND);
    puts("PASS reversals, repeats, fault reports, timeout acknowledgment, enable recovery, emergency and invalid input");
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    RobotCMDInit();
    GimbalInit();
    shoot_test_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    assert(zero_position_commands == 0);
    assert(yaw_motor->motor_can_instace->can_handle == &hcan2);
    assert(upper_pitch_motor->motor_can_instace->can_handle == &hcan1);
    assert(lower_pitch_motor->motor_can_instace->can_handle == &hcan1);
    assert(yaw_motor->motor_settings.angle_feedback_source == OTHER_FEED);
    assert(yaw_motor->motor_settings.speed_feedback_source == OTHER_FEED);
    assert(yaw_motor->other_angle_feedback_ptr == &imu.YawTotalAngle);
    assert(yaw_motor->other_speed_feedback_ptr == &imu.Gyro[2]);
    assert(upper_pitch_motor->other_angle_feedback_ptr == &imu.Pitch);
    assert(upper_pitch_motor->other_speed_feedback_ptr == &imu.Gyro[0]);
    assert(lower_pitch_motor->motor_settings.angle_feedback_source == MOTOR_FEED);
    assert(yaw_motor->motor_type == DM4310 && upper_pitch_motor->motor_type == DM4310 &&
           lower_pitch_motor->motor_type == DM4310);
    startup_and_enable();
    folding_sequence();
    folded_controls();
    unfolding_and_deployed();
    reversals_faults_and_stop();
    puts("PASS all restored folding integration tests");
}
