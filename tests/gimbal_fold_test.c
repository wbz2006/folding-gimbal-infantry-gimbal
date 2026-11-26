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
static uint8_t remote_active, imu_online = 1, imu_ready;
static uint8_t left_switch = RC_SW_DOWN, right_switch = RC_SW_UP;
static int16_t sticks[4], dial_input;
static float positions[3], velocities[3];
static uint8_t states[3] = {1, 1, 1}, fresh[3] = {1, 1, 1};
static unsigned zero_position_commands, enable_commands[16];
static unsigned disable_commands[16];
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
uint8_t INS_IsReady(void) { return imu_ready; }
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
    const uint8_t disable[8] = {255, 255, 255, 255, 255, 255, 255, DM_CMD_RESET_MODE};
    const uint8_t zero_position[8] = {255, 255, 255, 255, 255, 255, 255, DM_CMD_ZERO_POSITION};
    if (memcmp(can->tx_buff, zero_position, sizeof(zero_position)) == 0)
        ++zero_position_commands;
    if (memcmp(can->tx_buff, disable, sizeof(disable)) == 0)
        ++disable_commands[can->tx_id];
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
    NEAR(chassis_cmd_send.vx, -1.0f * sticks[1]);
    NEAR(chassis_cmd_send.vy, 1.0f * sticks[0]);
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

static void calibration_keeps_motors_disabled(void)
{
    DMMotorInstance *motors[] = {yaw_motor, upper_pitch_motor, lower_pitch_motor};
    unsigned initial_enable_commands[3];
    for (unsigned i = 0; i < 3; ++i)
    {
        const unsigned id = motors[i]->motor_can_instace->tx_id;
        assert(disable_commands[id] == 1);
        assert(motor_frames[id][7] == DM_CMD_RESET_MODE);
        initial_enable_commands[i] = enable_commands[id];
    }
    remote_active = 1;
    sticks[0] = 200;
    sticks[1] = -100;
    sticks[2] = sticks[3] = 660;
    for (unsigned i = 0; i < 6200; ++i)
    {
        states[0] = states[1] = states[2] = i % 2;
        step();
        stopped();
        assert(gimbal_cmd_send.gimbal_mode == GIMBAL_ZERO_FORCE);
        assert(chassis_cmd_send.chassis_mode == CHASSIS_ZERO_FORCE);
        NEAR(chassis_cmd_send.vx, 0);
        NEAR(chassis_cmd_send.vy, 0);
        NEAR(chassis_cmd_send.wz, 0);
        assert(shoot_cmd_send.shoot_mode == SHOOT_OFF);
        assert(shoot_cmd_send.friction_mode == FRICTION_OFF);
        assert(shoot_cmd_send.load_mode == LOAD_STOP);
        assert(fold_ctrl.step == FOLD_STEP_LEVEL_UPPER_PITCH);
        assert(gimbal_feedback_data.action_fault_flags == GIMBAL_ACTION_FAULT_NONE);
        for (unsigned j = 0; j < 3; ++j)
        {
            const unsigned id = motors[j]->motor_can_instace->tx_id;
            assert(enable_commands[id] == initial_enable_commands[j]);
            assert(motors[j]->motor_send_mailbox.Kp == 0);
            assert(motors[j]->motor_send_mailbox.Kd == 0);
            assert(abs((int)motors[j]->motor_send_mailbox.torque_des - 2048) <= 1);
        }
    }
    remote_active = 0;
    memset(sticks, 0, sizeof(sticks));
    for (unsigned i = 0; i < 3; ++i)
        motors[i]->motor_daemon->temp_count = 0;
    imu_ready = 1;
    puts("PASS IMU startup: three driver disable commands, 31 s with no motion or enable retries");
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
    imu.Pitch = GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG + GIMBAL_UPPER_PITCH_TOLERANCE_DEG + 2.0f;
    positions[0] = (GIMBAL_YAW_MECHANICAL_CENTER_DEG + 45) * DEGREE_2_RAD;
    positions[1] = -3.3f;
    positions[2] = 0.5f * (GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD + GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD);
    step();
    assert(fold_ctrl.active && fold_ctrl.step == FOLD_STEP_LEVEL_UPPER_PITCH);
    NEAR(fold_ctrl.lower_ref, lower_pitch_motor->measure.control_position);
    puts("PASS startup waits for three real motor feedbacks; enable retries are stopped and rate limited");
}

#ifdef TEST_UPPER_PITCH_TUNING
static void upper_pitch_tuning(void)
{
    const unsigned yaw_id = yaw_motor->motor_can_instace->tx_id;
    const unsigned upper_id = upper_pitch_motor->motor_can_instace->tx_id;
    const unsigned lower_id = lower_pitch_motor->motor_can_instace->tx_id;
    const unsigned initial_yaw_enables = enable_commands[yaw_id];
    const unsigned initial_lower_enables = enable_commands[lower_id];
    const unsigned initial_upper_enables = enable_commands[upper_id];
    remote_active = 1;
    states[0] = states[1] = states[2] = 0;
    check_enable_stopped = true;
    for (unsigned i = 0; i < 60; ++i)
    {
        step();
        stopped();
        assert(enable_commands[yaw_id] == initial_yaw_enables);
        assert(enable_commands[lower_id] == initial_lower_enables);
    }
    check_enable_stopped = false;
    assert(enable_commands[upper_id] >= initial_upper_enables + 3);

    states[1] = 1;
    for (unsigned i = 0; i < 4000; ++i)
    {
        left_switch = i % 2 ? RC_SW_DOWN : RC_SW_MID;
        imu.Pitch = i % 3 ? 15.0f : 0.0f;
        positions[2] = -2.0f - (i % 100) * 0.01f;
        velocities[2] = i % 2 ? 0.5f : -0.5f;
        fresh[0] = fresh[2] = i < 2000;
        step();
        assert(upper_pitch_motor->stop_flag == MOTOR_ENALBED);
        NEAR(upper_pitch_motor->pid_ref, GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
        assert(yaw_motor->stop_flag == MOTOR_STOP);
        assert(lower_pitch_motor->stop_flag == MOTOR_STOP);
        assert(enable_commands[yaw_id] == initial_yaw_enables);
        assert(enable_commands[lower_id] == initial_lower_enables);
        assert(lower_pitch_motor->motor_send_mailbox.Kp == 0);
        assert(lower_pitch_motor->motor_send_mailbox.Kd == 0);
        assert(abs((int)lower_pitch_motor->motor_send_mailbox.torque_des - 2048) <= 1);
        assert(!fold_ctrl.active && fold_ctrl.step == FOLD_STEP_LEVEL_UPPER_PITCH);
        assert(gimbal_feedback_data.action_fault_flags == GIMBAL_ACTION_FAULT_NONE);
    }
    dial_input = 400;
    step();
    stopped();
    dial_input = 0;
    step();
    assert(upper_pitch_motor->stop_flag == MOTOR_ENALBED);
    imu.Pitch = NAN;
    step();
    stopped();
    imu.Pitch = 0;
    step();
    assert(upper_pitch_motor->stop_flag == MOTOR_ENALBED);
    remote_active = 0;
    for (unsigned i = 0; i < 160; ++i) step();
    stopped();
    assert(enable_commands[yaw_id] == initial_yaw_enables);
    assert(enable_commands[lower_id] == initial_lower_enables);
    puts("PASS upper Pitch tuning: upper-only enable, manual lower motion, missing idle-axis feedback, 20 s hold, emergency and remote stop");
}
#endif

#ifdef TEST_YAW_TUNING
static void yaw_tuning_deployed(void)
{
    remote_active = 1;
    fresh[0] = fresh[1] = fresh[2] = 1;
    states[0] = states[1] = states[2] = 1;
    left_switch = RC_SW_DOWN;
    right_switch = RC_SW_UP;
    positions[2] = GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD;
    velocities[2] = 0;
    imu.Pitch = 12.0f;
    imu.YawTotalAngle = 725.0f;
    sticks[2] = 200;
    sticks[3] = -660;
    for (unsigned i = 0; i < 30; ++i)
    {
        step();
        assert(gimbal_cmd_send.request_mode == GIMBAL_REQUEST_DEPLOY);
        assert(fold_ctrl.state == GIMBAL_UNFOLDING);
        assert(fold_ctrl.step == FOLD_STEP_LEVEL_UPPER_PITCH);
        assert(yaw_motor->stop_flag == MOTOR_STOP);
        NEAR(lower_pitch_motor->pid_ref, positions[2]);
        NEAR(upper_pitch_motor->pid_ref, GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
        NEAR(gimbal_cmd_send.pitch, 0);
    }
    imu.Pitch = 0;
    for (unsigned i = 0; i < 2000; ++i)
    {
        step();
        assert(fold_ctrl.state == GIMBAL_UNFOLDING);
        assert(yaw_motor->stop_flag == MOTOR_STOP);
    }
    NEAR(lower_pitch_motor->pid_ref, GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD);
    positions[2] = GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD + PI2;
    velocities[2] = 0.1f;
    for (unsigned i = 0; i < 30; ++i)
    {
        step();
        assert(fold_ctrl.state == GIMBAL_UNFOLDING);
        assert(yaw_motor->stop_flag == MOTOR_STOP);
    }
    velocities[2] = 0;
    sticks[2] = 0;
    for (unsigned i = 0; i < 100 && fold_ctrl.state != GIMBAL_DEPLOYED; ++i) step();
    assert(fold_ctrl.state == GIMBAL_DEPLOYED);
    step();
    assert(yaw_motor->stop_flag == MOTOR_ENALBED);
    NEAR(yaw_motor->pid_ref, imu.YawTotalAngle);
    NEAR(yaw_motor->angle_PID.Measure, imu.YawTotalAngle);

    sticks[2] = 200;
    step();
    NEAR(yaw_motor->pid_ref, imu.YawTotalAngle - 0.2f);
    assert(yaw_motor->angle_PID.Output < 0);
    assert(yaw_motor->motor_send_mailbox.torque_des < 2047);
    sticks[2] = -200;
    step();
    step();
    NEAR(yaw_motor->pid_ref, imu.YawTotalAngle + 0.2f);
    assert(yaw_motor->angle_PID.Output > 0);
    assert(yaw_motor->motor_send_mailbox.torque_des > 2047);
    sticks[2] = 0;
    for (unsigned i = 0; i < 4000; ++i)
    {
        left_switch = i % 3 == 0 ? RC_SW_DOWN : i % 3 == 1 ? RC_SW_MID : RC_SW_UP;
        sticks[3] = i % 2 ? 660 : -660;
        step();
        assert(gimbal_cmd_send.request_mode == GIMBAL_REQUEST_DEPLOY);
        assert(fold_ctrl.state == GIMBAL_DEPLOYED);
        assert(yaw_motor->stop_flag == MOTOR_ENALBED);
        assert(upper_pitch_motor->stop_flag == MOTOR_ENALBED);
        assert(lower_pitch_motor->stop_flag == MOTOR_ENALBED);
        NEAR(gimbal_cmd_send.pitch, 0);
        NEAR(upper_pitch_motor->pid_ref, GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
        NEAR(lower_pitch_motor->pid_ref, GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD);
    }
    assert(gimbal_feedback_data.action_fault_flags == GIMBAL_ACTION_FAULT_NONE);

    fresh[0] = 0;
    for (unsigned i = 0; i < 40; ++i) step();
    stopped();
    assert(gimbal_feedback_data.fault.motor_offline_flags & MOTOR_1_OFFLINE);
    fresh[0] = 1;
    imu.YawTotalAngle = 815.0f;
    for (unsigned i = 0; i < 100; ++i) step();
    assert(fold_ctrl.state == GIMBAL_DEPLOYED);
    assert(yaw_motor->stop_flag == MOTOR_ENALBED);
    NEAR(yaw_motor->pid_ref, imu.YawTotalAngle);

    dial_input = 400;
    step();
    stopped();
    dial_input = 0;
    for (unsigned i = 0; i < 100; ++i) step();
    assert(fold_ctrl.state == GIMBAL_DEPLOYED);
    assert(yaw_motor->stop_flag == MOTOR_ENALBED);
    remote_active = 0;
    for (unsigned i = 0; i < 160; ++i) step();
    stopped();
    puts("PASS Yaw tuning: level and deploy before Yaw enable, both IMU control directions, fixed Pitch targets, no folding on switch changes, 20 s hold, feedback loss/recovery, emergency and remote stop");
}
#endif

#ifdef TEST_LOWER_PITCH_TUNING
static void lower_pitch_integral_unwind(void)
{
    PIDInstance accelerated = lower_pitch_motor->speed_PID;
    PIDInstance ordinary = accelerated;
    ordinary.IntegralUnwindGain = 1.0f;
    const float errors[] = {0.4f, -0.4f, -0.4f, 0.02f};
    const float initial_integrals[] = {-0.4f, 0.4f, -0.4f, -0.4f};
    assert(accelerated.IntegralUnwindGain > 1.0f);
    assert(upper_pitch_motor->speed_PID.IntegralUnwindGain == 0);
    assert(yaw_motor->speed_PID.IntegralUnwindGain == 0);

    for (unsigned test = 0; test < 4; ++test)
    {
        PIDResetState(&accelerated);
        PIDResetState(&ordinary);
        accelerated.Iout = ordinary.Iout = initial_integrals[test];
        accelerated.Last_Err = ordinary.Last_Err = errors[test];
        for (unsigned i = 0; i < 200; ++i)
        {
            PIDCalculate(&accelerated, 0, errors[test]);
            PIDCalculate(&ordinary, 0, errors[test]);
            assert(fabsf(accelerated.Iout) <= accelerated.IntegralLimit);
            assert(fabsf(accelerated.Output) <= accelerated.MaxOut);
        }
        if (test < 2)
        {
            assert(accelerated.Iout * initial_integrals[test] < 0);
            assert(ordinary.Iout * initial_integrals[test] > 0);
        }
        else
            NEAR(accelerated.Iout, ordinary.Iout);
    }
    NEAR(accelerated.IntegralUnwindGain, lower_pitch_motor->speed_PID.IntegralUnwindGain);
    puts("PASS lower Pitch integral: faster decay in both directions, normal buildup, small-error hold and preserved limits");
}

static void lower_pitch_tuning(void)
{
    const unsigned yaw_id = yaw_motor->motor_can_instace->tx_id;
    const unsigned upper_id = upper_pitch_motor->motor_can_instace->tx_id;
    const unsigned lower_id = lower_pitch_motor->motor_can_instace->tx_id;
    const unsigned initial_yaw_enables = enable_commands[yaw_id];
    const unsigned initial_upper_enables = enable_commands[upper_id];
    const unsigned initial_lower_enables = enable_commands[lower_id];
    remote_active = 1;
    fresh[0] = 0;
    states[0] = states[1] = states[2] = 0;
    check_enable_stopped = true;
    for (unsigned i = 0; i < 60; ++i)
    {
        step();
        stopped();
        assert(enable_commands[yaw_id] == initial_yaw_enables);
    }
    check_enable_stopped = false;
    assert(enable_commands[upper_id] >= initial_upper_enables + 3);
    assert(enable_commands[lower_id] >= initial_lower_enables + 3);
    states[1] = states[2] = 1;
    positions[2] = -2.6f;
    step();
    NEAR(fold_ctrl.lower_ref, lower_pitch_motor->measure.position);
    assert(fold_ctrl.step == FOLD_STEP_LEVEL_UPPER_PITCH);
    for (unsigned i = 0; i < 25; ++i) step();
    assert(fold_ctrl.step == FOLD_STEP_MOVE_LOWER_PITCH);

    imu.Pitch = 15.0f;
    for (unsigned i = 0; i < 4000; ++i)
    {
        step();
        assert(fold_ctrl.state == GIMBAL_FOLDING);
        assert(fold_ctrl.step == FOLD_STEP_MOVE_LOWER_PITCH);
        assert(upper_pitch_motor->stop_flag == MOTOR_ENALBED);
        assert(lower_pitch_motor->stop_flag == MOTOR_ENALBED);
        assert(gimbal_feedback_data.action_fault_flags == GIMBAL_ACTION_FAULT_NONE);
    }

    for (unsigned motion = 0; motion < 2; ++motion)
    {
        left_switch = motion == 0 ? RC_SW_DOWN : RC_SW_MID;
        imu.Pitch = 0;
        for (unsigned i = 0; i < 25; ++i) step();
        const gimbal_fold_state_e expected = motion == 0 ? GIMBAL_FOLDED : GIMBAL_DEPLOYED;
        const float target = motion == 0 ? GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD :
                                          GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD;
        for (unsigned i = 0; i < 1200 && fold_ctrl.state != expected; ++i)
        {
            const float previous_ref = fold_ctrl.lower_ref;
            positions[2] = lower_pitch_motor->pid_ref;
            imu.Pitch = i % 2 ? 15.0f : -15.0f;
            step();
            assert(fold_ctrl.step != FOLD_STEP_LEVEL_UPPER_PITCH);
            assert(fabsf(fold_ctrl.lower_ref - previous_ref) <=
                   GIMBAL_LOWER_REF_RATE_RAD_S * 0.005f + 0.00001f);
            assert(yaw_motor->stop_flag == MOTOR_STOP);
            assert(upper_pitch_motor->stop_flag == MOTOR_ENALBED);
            assert(lower_pitch_motor->stop_flag == MOTOR_ENALBED);
            NEAR(upper_pitch_motor->pid_ref, GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
        }
        assert(fold_ctrl.state == expected);
        NEAR(lower_pitch_motor->pid_ref, target);
    }
    positions[2] = GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD;
    sticks[2] = sticks[3] = 660;
    for (unsigned i = 0; i < 4000; ++i)
    {
        step();
        assert(yaw_motor->stop_flag == MOTOR_STOP);
        assert(upper_pitch_motor->stop_flag == MOTOR_ENALBED);
        assert(lower_pitch_motor->stop_flag == MOTOR_ENALBED);
        NEAR(upper_pitch_motor->pid_ref, GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
        NEAR(lower_pitch_motor->pid_ref, GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD);
        assert(gimbal_feedback_data.action_fault_flags == GIMBAL_ACTION_FAULT_NONE);
    }
    dial_input = 400;
    step();
    stopped();
    positions[2] = -2.7f;
    imu.Pitch = 0;
    dial_input = 0;
    step();
    NEAR(fold_ctrl.lower_ref, lower_pitch_motor->measure.position);
    imu.Pitch = NAN;
    step();
    stopped();
    imu.Pitch = 0;
    step();
    remote_active = 0;
    for (unsigned i = 0; i < 160; ++i) step();
    stopped();
    assert(enable_commands[yaw_id] == initial_yaw_enables);
    puts("PASS lower Pitch tuning: Pitch-only enable, initial leveling, fold/deploy ramps, no upper-error rollback, Yaw disabled, 20 s hold and stop recovery");
}

static void lower_pitch_feedback_recovery(void)
{
    remote_active = 1;
    left_switch = RC_SW_MID;
    imu.Pitch = 0;
    positions[2] = -2.6f;
    velocities[2] = 0;
    for (unsigned i = 0; i < 30; ++i) step();
    assert(fold_ctrl.state == GIMBAL_UNFOLDING);
    assert(lower_pitch_motor->angle_PID.Output < 0);

    for (unsigned pass = 0; pass < 2; ++pass)
    {
        fresh[2] = 0;
        for (unsigned i = 0; i < 80; ++i) step();
        assert(gimbal_feedback_data.fault.motor_offline_flags & MOTOR_3_OFFLINE);
        assert(!fold_ctrl.active);
        assert(lower_pitch_motor->stop_flag == MOTOR_STOP);
        assert(upper_pitch_motor->stop_flag == MOTOR_ENALBED);
        NEAR(upper_pitch_motor->pid_ref, GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
        assert(lower_pitch_motor->motor_send_mailbox.Kp == 0);
        assert(lower_pitch_motor->motor_send_mailbox.Kd == 0);
        assert(abs((int)lower_pitch_motor->motor_send_mailbox.torque_des - 2048) <= 1);

        positions[2] = pass == 0 ? -3.3f : -2.6f;
        fresh[2] = 1;
        states[2] = 0;
        const unsigned lower_id = lower_pitch_motor->motor_can_instace->tx_id;
        const unsigned previous_enables = enable_commands[lower_id];
        for (unsigned i = 0; i < 25; ++i) step();
        assert(enable_commands[lower_id] > previous_enables);
        assert(lower_pitch_motor->stop_flag == MOTOR_STOP);
        assert(!fold_ctrl.active);

        lower_pitch_motor->angle_PID.Iout = -2.0f;
        lower_pitch_motor->speed_PID.Iout = -2.0f;
        lower_pitch_motor->current_PID.Iout = -2.0f;
        states[2] = 1;
        step();
        assert(fold_ctrl.active);
        assert(!(gimbal_feedback_data.fault.motor_offline_flags & MOTOR_3_OFFLINE));
        NEAR(fold_ctrl.lower_ref, lower_pitch_motor->measure.position);
        NEAR(lower_pitch_motor->angle_PID.Output, 0);
        NEAR(lower_pitch_motor->angle_PID.Iout, 0);
        NEAR(lower_pitch_motor->speed_PID.Iout, 0);
        NEAR(lower_pitch_motor->current_PID.Iout, 0);

        const float direction = pass == 0 ? 1.0f : -1.0f;
        for (unsigned i = 0; i < 900; ++i)
        {
            const float previous_ref = fold_ctrl.lower_ref;
            step();
            assert(direction * (fold_ctrl.lower_ref - previous_ref) >= 0);
            assert(fabsf(fold_ctrl.lower_ref - previous_ref) <=
                   GIMBAL_LOWER_REF_RATE_RAD_S * 0.005f + 0.00001f);
            assert(direction * lower_pitch_motor->angle_PID.Output >= 0);
        }
        NEAR(lower_pitch_motor->pid_ref, GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD);
        assert(direction * lower_pitch_motor->angle_PID.Output > 0.5f);
        assert(direction * lower_pitch_motor->speed_PID.Output > 0.5f);
        printf("PASS lower Pitch reconnect: position=%.4f ref=%.4f angle_out=%.4f speed_out=%.4f\n",
               lower_pitch_motor->measure.position, lower_pitch_motor->pid_ref,
               lower_pitch_motor->angle_PID.Output, lower_pitch_motor->speed_PID.Output);

        positions[2] = GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD;
        for (unsigned i = 0; i < 30; ++i) step();
        assert(fold_ctrl.state == GIMBAL_DEPLOYED);
    }

    remote_active = 0;
    fresh[2] = 0;
    for (unsigned i = 0; i < 160; ++i) step();
    positions[2] = -3.3f;
    fresh[2] = 1;
    step();
    stopped();
    puts("PASS lower Pitch feedback loss pauses output, holds upper Pitch, resets PID on recovery and respects remote stop");
}

static void lower_pitch_terminal_approach(void)
{
    remote_active = 1;
    imu.Pitch = 0;
    velocities[2] = 0;
    for (unsigned motion = 0; motion < 2; ++motion)
    {
        const float target = motion == 0 ? GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD :
                                          GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD;
        const float direction = motion == 0 ? 1.0f : -1.0f;
        const gimbal_fold_state_e moving = motion == 0 ? GIMBAL_FOLDING : GIMBAL_UNFOLDING;
        const gimbal_fold_state_e completed = motion == 0 ? GIMBAL_FOLDED : GIMBAL_DEPLOYED;
        const gimbal_reach_target_type_e reach = motion == 0 ? LOWER_PITCH_FOLD : LOWER_PITCH_DEPLOY;
        left_switch = motion == 0 ? RC_SW_DOWN : RC_SW_MID;
        positions[2] = motion == 0 ? GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD :
                                   GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD;
        for (unsigned i = 0; i < 25; ++i) step();
        uint32_t slowdown_start = 0, reference_arrival = 0;
        bool saw_cruise = false, saw_slowdown = false, saw_final = false;
        for (unsigned i = 0; i < 1600 && fold_ctrl.state != completed; ++i)
        {
            const float previous_ref = fold_ctrl.lower_ref;
            const float remaining = fabsf(target - previous_ref);
            if (slowdown_start == 0 && remaining <= GIMBAL_LOWER_SLOWDOWN_DISTANCE_RAD)
                slowdown_start = now_ms;
            positions[2] = previous_ref;
            step();
            if (reference_arrival == 0 && fabsf(target - fold_ctrl.lower_ref) < 0.0001f)
                reference_arrival = now_ms;
            const float rate = direction * (fold_ctrl.lower_ref - previous_ref) / 0.005f;
            assert(rate >= 0 && rate <= GIMBAL_LOWER_REF_RATE_RAD_S + 0.0001f);
            assert(direction * (target - fold_ctrl.lower_ref) >= -0.000001f);
            if (remaining > 0.06f)
            {
                NEAR(rate, GIMBAL_LOWER_REF_RATE_RAD_S);
                saw_cruise = true;
            }
            if (remaining > 0.02f && remaining < 0.03f)
            {
                assert(rate > 0.05f && rate < 0.2f);
                saw_slowdown = true;
            }
            if (remaining > 0.001f && remaining < 0.005f)
            {
                NEAR(rate, GIMBAL_LOWER_FINAL_REF_RATE_RAD_S);
                saw_final = true;
            }
        }
        assert(saw_cruise && saw_slowdown && saw_final);
        assert(fold_ctrl.state == completed);
        NEAR(lower_pitch_motor->pid_ref, target);
        printf("lower Pitch %s: deceleration reference reaches target in %lu ms\n",
               motion == 0 ? "fold" : "deploy",
               (unsigned long)(reference_arrival - slowdown_start));
        assert(reference_arrival - slowdown_start <= 600U);

        for (int side = -1; side <= 1; side += 2)
        {
            fold_ctrl.state = moving;
            fold_ctrl.finish_stable.tracking = false;
            positions[2] = target + side * 0.012f;
            velocities[2] = 0;
            for (unsigned i = 0; i < 30; ++i) step();
            assert(!GimbalTargetReached(reach));
            assert(fold_ctrl.state == moving);
            positions[2] = target + side * 0.009f;
            velocities[2] = side * 0.1f;
            for (unsigned i = 0; i < 30; ++i) step();
            assert(!GimbalTargetReached(reach));
            assert(fold_ctrl.state == moving);
            velocities[2] = 0;
            for (unsigned i = 0; i < 30; ++i) step();
            assert(GimbalTargetReached(reach));
            assert(fold_ctrl.state == completed);
            NEAR(lower_pitch_motor->pid_ref, target);
        }
    }
    puts("PASS lower Pitch terminal approach: both directions cruise then slow, no reference overshoot, symmetric position window, low-speed arrival and target hold");
}
#endif

#if defined(TEST_GIMBAL_SWITCH) || defined(TEST_LOWER_PITCH_TUNING)
static void lower_pitch_switch_holds_load(void)
{
    remote_active = 1;
    fresh[0] = fresh[1] = fresh[2] = 1;
    states[0] = states[1] = states[2] = 1;
    imu.Pitch = 0;
    positions[0] = GIMBAL_YAW_MECHANICAL_CENTER_DEG * DEGREE_2_RAD;
    velocities[2] = 0;
    const gimbal_fold_state_e initial_states[] = {
        GIMBAL_FOLDED, GIMBAL_DEPLOYED, GIMBAL_FOLDING, GIMBAL_UNFOLDING
    };
    for (unsigned test = 0; test < 4; ++test)
    {
        const bool from_fold = test % 2 == 0;
        const float ref = test < 2 ? (from_fold ? GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD :
                                                 GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD) : -2.7f;
        left_switch = from_fold ? RC_SW_DOWN : RC_SW_MID;
        positions[2] = ref + 0.004f;
        step();
        fold_ctrl.state = initial_states[test];
        fold_ctrl.step = test < 2 ? FOLD_STEP_FINISH : FOLD_STEP_MOVE_LOWER_PITCH;
        fold_ctrl.lower_ref = ref;
        DMMotorSetRef(lower_pitch_motor, ref);
        DMMotorResetPIDState(lower_pitch_motor);
        lower_pitch_motor->speed_PID.Iout = from_fold ? -0.6f : 0.2f;
        DMMotorControl();
        const float previous_integral = lower_pitch_motor->speed_PID.Iout;
        const float previous_output = lower_pitch_motor->speed_PID.Output;
        const uint16_t previous_torque = lower_pitch_motor->motor_send_mailbox.torque_des;

        left_switch = from_fold ? RC_SW_MID : RC_SW_DOWN;
        step();
        assert(fold_ctrl.state == (from_fold ? GIMBAL_UNFOLDING : GIMBAL_FOLDING));
        assert(fold_ctrl.step == FOLD_STEP_LEVEL_UPPER_PITCH);
        assert(lower_pitch_motor->stop_flag == MOTOR_ENALBED);
        NEAR(lower_pitch_motor->pid_ref, ref);
        NEAR(lower_pitch_motor->speed_PID.Iout, previous_integral);
        NEAR(lower_pitch_motor->speed_PID.Output, previous_output);
        assert(abs((int)lower_pitch_motor->motor_send_mailbox.torque_des - previous_torque) <= 1);

        for (unsigned i = 0; i < 60 && fold_ctrl.step != FOLD_STEP_MOVE_LOWER_PITCH; ++i)
        {
            step();
            NEAR(lower_pitch_motor->pid_ref, ref);
            assert(fabsf(lower_pitch_motor->speed_PID.Output - previous_output) < 0.01f);
        }
        assert(fold_ctrl.step == FOLD_STEP_MOVE_LOWER_PITCH);
        step();
        const float direction = from_fold ? -1.0f : 1.0f;
        assert(direction * (lower_pitch_motor->pid_ref - ref) > 0);
        assert(fabsf(lower_pitch_motor->pid_ref - ref) <=
               GIMBAL_LOWER_REF_RATE_RAD_S * 0.005f + 0.00001f);
    }

    dial_input = 400;
    step();
    stopped();
    lower_pitch_motor->speed_PID.Iout = 0.6f;
    positions[2] = -2.65f;
    dial_input = 0;
    step();
    assert(fold_ctrl.active);
    NEAR(lower_pitch_motor->pid_ref, lower_pitch_motor->measure.position);
    NEAR(lower_pitch_motor->speed_PID.Iout, 0);
    puts("PASS lower Pitch switches: both directions and mid-motion reversals retain reference, load integral and CAN torque; stopped restart resets state");
}
#endif

#if defined(TEST_LOWER_PITCH_COORDINATE) || defined(TEST_LOWER_PITCH_TUNING)
static void lower_pitch_power_on_coordinate(void)
{
    Motor_Control_Setting_s motor_feed = {.angle_feedback_source = MOTOR_FEED};
    const float raw_positions[] = {-9.0f, -3.3f, 2.61635f, 9.0f};
    for (unsigned i = 0; i < sizeof(raw_positions) / sizeof(raw_positions[0]); ++i)
    {
        motor_feedback(yaw_motor, raw_positions[i], 0, 1);
        NEAR(yaw_motor->measure.position, raw_positions[i]);
        NEAR(yaw_motor->measure.control_position, raw_positions[i]);
        NEAR(DMMotorGetAngleMeasure(yaw_motor, &motor_feed), raw_positions[i]);
    }

    motor_feedback(lower_pitch_motor, 2.61635f, 0, 1);
    DMMotorSetPositionCenter(lower_pitch_motor, 3.1f);
    NEAR(lower_pitch_motor->measure.control_position, 2.61635f);
    DMMotorSetPositionCenter(lower_pitch_motor,
        0.5f * (GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD + GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD));
    NEAR(lower_pitch_motor->measure.position, 2.61635f);
    NEAR(lower_pitch_motor->measure.control_position, -3.666835f);
    NEAR(DMMotorGetAngleMeasure(lower_pitch_motor, &lower_pitch_motor->motor_settings), -3.666835f);

    remote_active = 1;
    fresh[0] = fresh[1] = fresh[2] = 1;
    states[0] = states[1] = states[2] = 1;
    imu.Pitch = 0;
    positions[0] = GIMBAL_YAW_MECHANICAL_CENTER_DEG * DEGREE_2_RAD;
    velocities[2] = 0;
    const float mechanical_positions[] = {-3.666835f, -3.35f, -3.05f, -2.35f, -2.05f};
    for (unsigned test = 0; test < sizeof(mechanical_positions) / sizeof(mechanical_positions[0]); ++test)
    {
        const float position = mechanical_positions[test];
        const bool deploying = test < 3;
        const float target = deploying ? GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD :
                                         GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD;
        left_switch = deploying ? RC_SW_MID : RC_SW_DOWN;
        for (int turn = 0; turn <= 2; ++turn)
        {
            const float raw_position = position + (turn == 1 ? PI2 : turn == 2 ? -PI2 : 0);
            positions[2] = raw_position;
            dial_input = 400;
            step();
            stopped();
            NEAR(lower_pitch_motor->measure.position, raw_position);
            NEAR(lower_pitch_motor->measure.control_position, position);
            NEAR(DMMotorGetAngleMeasure(lower_pitch_motor, &lower_pitch_motor->motor_settings), position);

            lower_pitch_motor->speed_PID.Iout = -0.8f;
            dial_input = 0;
            step();
            assert(fold_ctrl.active);
            NEAR(fold_ctrl.lower_ref, position);
            NEAR(lower_pitch_motor->angle_PID.Measure, position);
            NEAR(lower_pitch_motor->speed_PID.Iout, 0);
            for (unsigned i = 0; i < 80; ++i) step();
            const float direction = target > position ? 1.0f : -1.0f;
            assert(direction * (lower_pitch_motor->pid_ref - position) > 0);
            assert(direction * lower_pitch_motor->angle_PID.Output > 0.1f);
            assert(direction * lower_pitch_motor->speed_PID.Output > 0.1f);
            const float torque = uint_to_float(lower_pitch_motor->motor_send_mailbox.torque_des,
                                               -DM4310_T_MAX, DM4310_T_MAX, 12);
            assert(direction * torque > 0.1f);
        }
    }

    left_switch = RC_SW_MID;
    for (unsigned test = 0; test < 2; ++test)
    {
        positions[2] = GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD + (test == 0 ? 0 : PI2);
        dial_input = 400;
        step();
        dial_input = 0;
        for (unsigned i = 0; i < 100; ++i) step();
        assert(GimbalTargetReached(LOWER_PITCH_DEPLOY));
        assert(fold_ctrl.state == GIMBAL_DEPLOYED);
    }
    puts("PASS DM position coordinates: opt-in per motor, immediate center update, raw feedback retained, unconfigured multi-turn feedback unchanged; lower Pitch startup, angle/torque direction and arrival");
}
#endif

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
    assert(yaw_motor->stop_flag == MOTOR_ENALBED);

    imu.Pitch = GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG + GIMBAL_UPPER_PITCH_TOLERANCE_DEG + 2.0f;
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
        if (fold_ctrl.step == FOLD_STEP_MOVE_LOWER_PITCH || fold_ctrl.state == GIMBAL_FOLDED)
            assert(yaw_motor->stop_flag == MOTOR_ENALBED);
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
        NEAR(chassis_cmd_send.wz, i == 0 ? -26.4f : i == 1 ? 0 : 26.4f);
        assert(yaw_motor->stop_flag == MOTOR_ENALBED);
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

static void folded_yaw_holds_center(void)
{
    const float heading = imu.YawTotalAngle;
    const float remote_yaw = gimbal_cmd_send.yaw;
    const unsigned yaw_id = yaw_motor->motor_can_instace->tx_id;
    for (unsigned i = 0; i < 4000; ++i)
    {
        const float offset = i < 3200 ? 0.0f : i < 3600 ? 3.0f : -3.0f;
        positions[0] = (GIMBAL_YAW_MECHANICAL_CENTER_DEG + offset +
                        (i % 2 ? 360.0f : -360.0f)) * DEGREE_2_RAD;
        imu.YawTotalAngle += 0.25f;
        sticks[2] = i % 2 ? 660 : -660;
        step();
        assert(fold_ctrl.state == GIMBAL_FOLDED);
        assert(yaw_motor->stop_flag == MOTOR_ENALBED);
        assert(upper_pitch_motor->stop_flag == MOTOR_ENALBED);
        assert(lower_pitch_motor->stop_flag == MOTOR_ENALBED);
        assert(gimbal_feedback_data.action_fault_flags == GIMBAL_ACTION_FAULT_NONE);
        NEAR(gimbal_cmd_send.yaw, remote_yaw);
        assert(fabsf(yaw_motor->angle_PID.Err + offset) < 0.03f);
        const uint16_t torque = ((motor_frames[yaw_id][6] & 0x0f) << 8) |
                                motor_frames[yaw_id][7];
        if (offset == 0)
            assert(abs((int)torque - 2047) <= 1);
        else
        {
            assert(offset * yaw_motor->angle_PID.Output < 0);
            assert(offset * yaw_motor->speed_PID.Output < 0);
            assert(offset * ((int)torque - 2047) < 0);
        }
    }
    positions[0] = GIMBAL_YAW_MECHANICAL_CENTER_DEG * DEGREE_2_RAD;
    imu.YawTotalAngle = heading;
    sticks[2] = 0;
    step();
    puts("PASS folded Yaw: 20 s hold, restoring CAN torque in both directions after 16 s, heading changes, encoder wraps and stick isolation");
}

static void unfolding_and_deployed(void)
{
    left_switch = RC_SW_MID;
    right_switch = RC_SW_DOWN;
    sticks[2] = 660;
    positions[2] = GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD;
    live_translation(1);
    assert(fold_ctrl.state == GIMBAL_UNFOLDING);
    for (int i = 0; i < 2000 &&
         fabsf(lower_pitch_motor->pid_ref - GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD) > 0.0001f; ++i)
    {
        live_translation(i);
        assert(fold_ctrl.state == GIMBAL_UNFOLDING);
        assert(chassis_cmd_send.chassis_mode == CHASSIS_NO_FOLLOW);
        assert(yaw_motor->stop_flag == MOTOR_STOP);
    }
    // The reference is at deploy, but actual lower feedback must reach deploy too.
    NEAR(lower_pitch_motor->pid_ref, GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD);
    positions[2] = GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD + PI2;
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
    imu.Pitch = GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG + GIMBAL_UPPER_PITCH_TOLERANCE_DEG + 2.0f;
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
    positions[2] = 0.5f * (GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD + GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD) + PI2;
    states[2] = 1;
    step();
    assert(fold_ctrl.active);
    NEAR(fold_ctrl.lower_ref, lower_pitch_motor->measure.control_position);
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
    assert(lower_pitch_motor->other_angle_feedback_ptr == NULL);
    assert(lower_pitch_motor->motor_can_instace->can_module_callback == DMMotorDecode);
    assert(lower_pitch_motor->motor_settings.speed_feedback_source == MOTOR_FEED);
    assert(yaw_motor->motor_type == DM4310 && upper_pitch_motor->motor_type == DM4310 &&
           lower_pitch_motor->motor_type == DM4310);
    calibration_keeps_motors_disabled();
#ifdef TEST_YAW_TUNING
    yaw_tuning_deployed();
    return 0;
#endif
#ifdef TEST_LOWER_PITCH_COORDINATE
    lower_pitch_power_on_coordinate();
    return 0;
#endif
#ifdef TEST_GIMBAL_SWITCH
    lower_pitch_switch_holds_load();
    return 0;
#endif
#ifdef TEST_UPPER_PITCH_TUNING
    upper_pitch_tuning();
    return 0;
#endif
#ifdef TEST_LOWER_PITCH_TUNING
    lower_pitch_integral_unwind();
    lower_pitch_tuning();
    lower_pitch_feedback_recovery();
    lower_pitch_terminal_approach();
    lower_pitch_switch_holds_load();
    lower_pitch_power_on_coordinate();
    return 0;
#endif
    startup_and_enable();
#ifdef TEST_IMU_STARTUP
    return 0;
#endif
    folding_sequence();
    folded_controls();
    folded_yaw_holds_center();
    unfolding_and_deployed();
    reversals_faults_and_stop();
    puts("PASS all restored folding integration tests");
}
