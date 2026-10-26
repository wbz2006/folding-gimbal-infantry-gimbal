/* Execute production C directly; only the clock, sensors, CAN and remote transport are simulated. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "../application/cmd/robot_cmd.c"
#include "../application/gimbal/gimbal.c"
#include "../modules/motor/DMmotor/dmmotor.c"

UART_HandleTypeDef huart1, huart5, huart9;
CAN_HandleTypeDef hcan1, hcan2, hcan3;
static uint32_t now_ms;
static uint8_t remote_online = 1, imu_online = 1, link_online = 1, bus_ready = 1;
static CANBusStatus bus;
static attitude_t imu;
static RC_ctrl_t remote[2];
static Chassis_Ctrl_Cmd_s sent_chassis;
static int checks;
static uint8_t test_chassis_fault_flags;
#define CHECK(x) do { ++checks; assert(x); } while (0)
#define NEAR(a,b) CHECK(fabsf((a)-(b)) < 0.001f)

uint32_t HAL_GetTick(void) { return now_ms; }
float DWT_GetDeltaT(uint32_t *count) { *count = now_ms; return 0.005f; }
void DWT_Delay(float seconds) { (void)seconds; }
attitude_t *INS_Init(void) { return &imu; }
uint8_t INS_IsFeedbackValid(void) { return imu_online; }
RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *uart) { (void)uart; return remote; }
uint8_t RemoteControlIsOnline(void) { return remote_online; }
Vision_Recv_s *VisionInit(UART_HandleTypeDef *uart) { (void)uart; static Vision_Recv_s v; return &v; }
void VisionSend(void) {}
CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    CANInstance *can = calloc(1, sizeof(*can)); can->id = config->id; return can;
}
uint8_t CANTransmit(CANInstance *can, float timeout) { (void)can; (void)timeout; return bus_ready; }
uint8_t CANIsReady(CANInstance *can) { return can != NULL && bus_ready; }
const CANBusStatus *CANGetBusStatus(CANInstance *can) { return can ? &bus : NULL; }
DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config)
{
    DaemonInstance *d = calloc(1, sizeof(*d));
    d->reload_count = config->reload_count ? config->reload_count : 100;
    d->temp_count = config->init_count ? config->init_count : d->reload_count;
    d->has_reloaded = !config->require_first_reload;
    return d;
}
void DaemonReload(DaemonInstance *d)
{ if (d) { d->temp_count = d->reload_count; d->has_reloaded = 1; d->offline_reported = 0; } }
uint8_t DaemonIsOnline(DaemonInstance *d) { return d && d->has_reloaded && d->temp_count; }
uint8_t DaemonHasReloaded(DaemonInstance *d) { return d && d->has_reloaded; }
void DaemonSetFeedbackValid(DaemonInstance *d, uint8_t valid) { if (d) d->feedback_valid = valid != 0; }
uint8_t DaemonIsFeedbackValid(DaemonInstance *d) { return d && d->feedback_valid; }
void DaemonSetFaultFlags(DaemonInstance *d, uint32_t flags) { if (d) d->fault_flags = flags; }
void DaemonAddFaultFlags(DaemonInstance *d, uint32_t flags) { if (d) d->fault_flags |= flags; }
void DaemonClearFaultFlags(DaemonInstance *d, uint32_t flags) { if (d) d->fault_flags &= ~flags; }
uint32_t DaemonGetFaultFlags(DaemonInstance *d) { return d ? d->fault_flags : UINT32_MAX; }
CANCommInstance *CANCommInit(CANComm_Init_Config_s *config)
{
    CANCommInstance *comm = calloc(1, sizeof(*comm));
    comm->recv_data_len = config->recv_data_len; comm->send_data_len = config->send_data_len;
    return comm;
}
uint8_t CANCommReceive(CANCommInstance *comm, void *data)
{
    if (!link_online) return 0;
    memset(data, 0, comm->recv_data_len);
    if (comm->recv_data_len == sizeof(Chassis_Upload_Data_s))
        ((Chassis_Upload_Data_s *)data)->fault_flags = test_chassis_fault_flags;
    return 1;
}
void CANCommSend(CANCommInstance *comm, uint8_t *data)
{
    CHECK(comm->send_data_len == sizeof(sent_chassis)); memcpy(&sent_chassis, data, sizeof(sent_chassis));
}

static void refresh(void)
{
    DMMotorInstance *motors[] = {yaw_motor, upper_pitch_motor, lower_pitch_motor};
    for (int i = 0; i < 3; ++i)
    {
        motors[i]->feedback_bus_off_count = bus.bus_off_count;
        motors[i]->measure.state = 1; DaemonReload(motors[i]->motor_daemon);
    }
}

static uint8_t test_shoot_fault_flags;
static Publisher_t *test_shoot_feed_pub;

static void tick(void)
{
    now_ms += 5;
    refresh();
    if (test_shoot_feed_pub == NULL)
        test_shoot_feed_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    Shoot_Upload_Data_s shoot_feed = {.fault_flags = test_shoot_fault_flags};
    PubPushMessage(test_shoot_feed_pub, &shoot_feed);
    RobotCMDTask(); GimbalTask();
}

static void publish_state(void)
{
    gimbal_feedback_data.gimbal_imu_data = imu;
    gimbal_feedback_data.fold_state = fold_ctrl.state;
    gimbal_feedback_data.feedback_valid = 1;
    gimbal_feedback_data.fault_flags = (uint8_t)DaemonGetFaultFlags(gimbal_cmd_daemon);
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;
    PubPushMessage(gimbal_pub, &gimbal_feedback_data);
}

static void baseline(gimbal_fold_state_e state)
{
    remote_online = imu_online = link_online = bus_ready = 1;
    test_shoot_fault_flags = test_chassis_fault_flags = APP_FAULT_NONE;
    memset(remote, 0, sizeof(remote)); memset(&imu, 0, sizeof(imu));
    imu.YawTotalAngle = 1376.25f;
    remote[0].rc.switch_left = state == GIMBAL_DEPLOYED ? RC_SW_DOWN : RC_SW_MID;
    remote[0].rc.switch_right = RC_SW_MID;
    remote[0].rc.rocker_r_ = 120; remote[0].rc.rocker_r1 = -50;
    robot_state = ROBOT_READY; fold_ctrl.state = state; fold_ctrl.active = true;
    DaemonSetFaultFlags(gimbal_cmd_daemon, APP_FAULT_NONE);
    DaemonSetFeedbackValid(gimbal_cmd_daemon, 1);
    yaw_input_last = false; gimbal_request_mode = state == GIMBAL_DEPLOYED ? GIMBAL_REQUEST_DEPLOY : GIMBAL_REQUEST_FOLD;
    gimbal_cmd_send.yaw = -500; gimbal_cmd_send.pitch = 0;
    lower_pitch_motor->measure.position = state == GIMBAL_DEPLOYED ? 0 : -1;
    lower_pitch_motor->measure.velocity = 0; fold_ctrl.lower_ref = lower_pitch_motor->measure.position;
    yaw_motor->measure.angle_single_round = 310.75f;
    fold_ctrl.upper_stable.tracking = fold_ctrl.finish_stable.tracking = false;
    publish_state();
}

static void follow_lower(void) { lower_pitch_motor->measure.position = lower_pitch_motor->mit_config.position_des; }

static void six_modes(void)
{
    const uint8_t rights[] = {RC_SW_UP, RC_SW_MID, RC_SW_DOWN};
    const chassis_mode_e modes[] = {CHASSIS_NO_FOLLOW, CHASSIS_FOLLOW_GIMBAL_YAW, CHASSIS_ROTATE};
    for (int shape = 0; shape < 2; ++shape)
        for (int right = 0; right < 3; ++right)
        {
            baseline(shape ? GIMBAL_DEPLOYED : GIMBAL_FOLDED);
            remote[0].rc.switch_right = rights[right]; remote[0].rc.rocker_l_ = 100;
            remote[0].rc.rocker_l1 = 500; tick();
            CHECK(sent_chassis.chassis_mode == modes[right]);
            NEAR(sent_chassis.vx, 1200); NEAR(sent_chassis.vy, -500); NEAR(sent_chassis.wz, 0);
            NEAR(gimbal_cmd_send.yaw, imu.YawTotalAngle + 0.5f);
            NEAR(upper_pitch_motor->pid_ref, 0); // actual conservative deployed limits remain 0..0
            NEAR(sent_chassis.offset_angle, theta_format(310.75f - YAW_ALIGN_ANGLE));
            float target = gimbal_cmd_send.yaw; tick(); NEAR(gimbal_cmd_send.yaw, target + 0.5f);
            CHECK(yaw_motor->stop_flag == MOTOR_ENALBED);
            if (!shape) CHECK(shoot_cmd_send.shoot_mode == SHOOT_OFF);
        }
    puts("PASS six switch combinations, stable Yaw accumulation, offset, conservative Pitch limits");
}

static void folding_continuity(void)
{
    baseline(GIMBAL_DEPLOYED); remote[0].rc.switch_left = RC_SW_MID;
    remote[0].rc.rocker_l_ = 600; remote[0].rc.rocker_l1 = 600;
    gimbal_cmd_send.pitch = 12; imu.Pitch = 8;
    lower_pitch_motor->measure.position = -0.2f;
    tick(); // feedback still DEPLOYED at CMD entry
    CHECK(fold_ctrl.state == GIMBAL_FOLDING); NEAR(upper_pitch_motor->pid_ref, 0);
    NEAR(gimbal_cmd_send.pitch, 0); NEAR(fold_ctrl.hold_yaw, 1376.25f);
    NEAR(yaw_motor->pid_ref, fold_ctrl.hold_yaw); NEAR(sent_chassis.vx, 1200);
    float latched = fold_ctrl.lower_ref; uint32_t start = fold_ctrl.action_start;
    for (int i = 0; i < 30; ++i)
    {
        lower_pitch_motor->measure.position -= 0.0005f; tick();
        NEAR(lower_pitch_motor->mit_config.position_des, latched);
        CHECK(fold_ctrl.action_start == start); CHECK(fold_ctrl.step == FOLD_STEP_LEVEL_UPPER_PITCH);
        NEAR(yaw_motor->pid_ref, fold_ctrl.hold_yaw);
    }
    imu.Pitch = 0; imu.Gyro[0] = 0.2f;
    for (int i = 0; i < 25; ++i) tick(); // low angle error alone is insufficient
    CHECK(fold_ctrl.step == FOLD_STEP_LEVEL_UPPER_PITCH);
    imu.Gyro[0] = 0;
    for (int i = 0; i < 20; ++i) tick();
    CHECK(fold_ctrl.step == FOLD_STEP_LEVEL_UPPER_PITCH); tick();
    CHECK(fold_ctrl.step == FOLD_STEP_MOVE_LOWER_PITCH); tick();
    CHECK(fold_ctrl.lower_ref < latched);
    float before = fold_ctrl.lower_ref;
    imu.Pitch = 3; tick();
    CHECK(fold_ctrl.step == FOLD_STEP_LEVEL_UPPER_PITCH);
    float paused = fold_ctrl.lower_ref;
    CHECK(paused != before);
    for (int i = 0; i < 25; ++i) { lower_pitch_motor->measure.position += 0.0001f; tick(); NEAR(fold_ctrl.lower_ref, paused); }
    imu.Pitch = 0;
    for (int i = 0; i < 1100 && fold_ctrl.state != GIMBAL_FOLDED; ++i)
    {
        follow_lower(); remote[0].rc.switch_right = (i % 3 == 0 ? RC_SW_UP : i % 3 == 1 ? RC_SW_MID : RC_SW_DOWN);
        remote[0].rc.rocker_r_ = i % 2 ? 80 : 0;
        yaw_motor->measure.angle_single_round = fmodf(350.0f + i, 360.0f);
        imu.YawTotalAngle += 0.001f;
        tick();
        NEAR(sent_chassis.vx, 10.0f * remote[0].rc.rocker_r_);
        CHECK(sent_chassis.chassis_mode == (i % 3 == 0 ? CHASSIS_NO_FOLLOW : i % 3 == 1 ? CHASSIS_FOLLOW_GIMBAL_YAW : CHASSIS_ROTATE));
        NEAR(sent_chassis.offset_angle, theta_format(gimbal_fetch_data.yaw_motor_single_round_angle - YAW_ALIGN_ANGLE));
        NEAR(upper_pitch_motor->pid_ref, 0);
        if (fold_ctrl.state == GIMBAL_FOLDING) NEAR(yaw_motor->pid_ref, fold_ctrl.hold_yaw);
    }
    CHECK(fold_ctrl.state == GIMBAL_FOLDED); CHECK(DaemonGetFaultFlags(gimbal_cmd_daemon) == APP_FAULT_NONE);
    remote[0].rc.rocker_l_ = 0; remote[0].rc.rocker_r_ = 100; tick();
    NEAR(gimbal_cmd_send.yaw, imu.YawTotalAngle); NEAR(sent_chassis.vx, 1000);
    NEAR(lower_pitch_motor->mit_config.position_des, -1); NEAR(gimbal_cmd_send.pitch, 0);
    remote[0].rc.rocker_l_ = 100; tick(); NEAR(gimbal_cmd_send.yaw, imu.YawTotalAngle + 0.5f);
    puts("PASS folding order, 100ms stability, speed gate, latched lower hold, pause, live modes/input and all folding boundaries");
}

static void unfolding_and_reversals(void)
{
    baseline(GIMBAL_FOLDED); remote[0].rc.switch_left = RC_SW_DOWN;
    remote[0].rc.rocker_l_ = remote[0].rc.rocker_l1 = 600;
    tick(); CHECK(fold_ctrl.state == GIMBAL_UNFOLDING);
    NEAR(sent_chassis.vx, 0); NEAR(sent_chassis.vy, 0); NEAR(sent_chassis.wz, 0);
    CHECK(sent_chassis.chassis_mode == CHASSIS_NO_FOLLOW);
    uint32_t start = fold_ctrl.action_start;
    for (int i = 0; i < 60; ++i) tick();
    CHECK(fold_ctrl.state != GIMBAL_DEPLOYED); CHECK(fold_ctrl.action_start == start); // still at -1, not DEPLOY
    float position = lower_pitch_motor->measure.position;
    remote[0].rc.switch_left = RC_SW_MID; tick();
    CHECK(fold_ctrl.state == GIMBAL_FOLDING); CHECK(fold_ctrl.action_start != start);
    NEAR(fold_ctrl.lower_ref, position); NEAR(sent_chassis.vx, 1200);
    remote[0].rc.switch_left = RC_SW_DOWN; tick(); CHECK(fold_ctrl.state == GIMBAL_UNFOLDING);
    NEAR(sent_chassis.vx, 0);
    for (int i = 0; i < 20; ++i)
    {
        remote[0].rc.switch_left = i % 2 ? RC_SW_MID : RC_SW_DOWN; tick();
        CHECK(fold_ctrl.state == (i % 2 ? GIMBAL_FOLDING : GIMBAL_UNFOLDING));
        NEAR(lower_pitch_motor->mit_config.position_des, position);
    }
    remote[0].rc.switch_left = RC_SW_DOWN; tick();
    remote[0].rc.switch_right = RC_SW_DOWN; remote[0].rc.rocker_l_ = remote[0].rc.rocker_l1 = 0;
    for (int i = 0; i < 1100 && fold_ctrl.state != GIMBAL_DEPLOYED; ++i)
    {
        follow_lower(); tick(); NEAR(sent_chassis.vx, 0); NEAR(gimbal_cmd_send.pitch, 0);
        CHECK(sent_chassis.chassis_mode == CHASSIS_NO_FOLLOW);
    }
    CHECK(fold_ctrl.state == GIMBAL_DEPLOYED); tick();
    CHECK(sent_chassis.chassis_mode == CHASSIS_ROTATE); NEAR(sent_chassis.vx, 1200);
    NEAR(gimbal_cmd_send.pitch, 0); NEAR(gimbal_cmd_send.yaw, imu.YawTotalAngle);
    puts("PASS unfold uses DEPLOY position, pause until completion, latest mode restored, reversals and repeated requests");
}

static void stops_and_angles(void)
{
    baseline(GIMBAL_DEPLOYED); tick();
    remote[0].rc.switch_left = RC_SW_UP; gimbal_request_mode_e old = gimbal_request_mode;
    float yaw = gimbal_cmd_send.yaw; remote[0].rc.rocker_l_ = 660;
    tick(); NEAR(sent_chassis.vx, 0); NEAR(sent_chassis.vy, 0); NEAR(sent_chassis.wz, 0);
    NEAR(gimbal_cmd_send.yaw, yaw); CHECK(gimbal_request_mode == old);
    baseline(GIMBAL_FOLDED); remote[0].rc.switch_right = RC_SW_UP; remote[0].rc.dial = 301; tick();
    CHECK(robot_state == ROBOT_STOP); CHECK(gimbal_cmd_send.gimbal_mode == GIMBAL_ZERO_FORCE);
    CHECK(yaw_motor->stop_flag == MOTOR_STOP); CHECK(sent_chassis.chassis_mode == CHASSIS_ZERO_FORCE);
    tick(); CHECK(robot_state == ROBOT_STOP); // up switch cannot override fold_ctrl.active emergency
    remote[0].rc.dial = 0; remote_online = 0; tick(); CHECK(robot_state == ROBOT_STOP);
    remote_online = 1; link_online = 0; tick(); CHECK(robot_state == ROBOT_STOP);
    link_online = 1; imu_online = 0; tick(); CHECK(yaw_motor->stop_flag == MOTOR_STOP);
    tick(); CHECK(robot_state == ROBOT_STOP); // CMD consumes the prior GimbalTask feedback
    imu_online = 1;
    for (int i = 0; i < 4; ++i) tick();
    CHECK(robot_state == ROBOT_READY);
    NEAR(fold_ctrl.hold_yaw, imu.YawTotalAngle); CHECK(fold_ctrl.state == GIMBAL_FOLDING);
    baseline(GIMBAL_DEPLOYED); imu.YawTotalAngle = 359.8f; remote[0].rc.rocker_l_ = 100;
    publish_state(); tick(); NEAR(gimbal_cmd_send.yaw, 360.3f);
    imu.YawTotalAngle = 360.1f; tick(); NEAR(gimbal_cmd_send.yaw, 360.8f);
    imu.YawTotalAngle = -720.3f; yaw_input_last = false; publish_state(); tick();
    NEAR(gimbal_cmd_send.yaw, -719.8f);
    CHECK(theta_format(359.9f) < 0); CHECK(theta_format(-359.9f) > 0);
    baseline(GIMBAL_DEPLOYED); tick();
    gimbal_feedback_daemon->temp_count = 0;
    CHECK(!FeedbackReady()); EmergencyHandler(); CHECK(robot_state == ROBOT_STOP);
    puts("PASS left-up no residual motion, emergency precedence, remote/CAN/IMU offline, recovery, stale feedback and continuous angles");
}

static void application_fault_aggregation(void)
{
    baseline(GIMBAL_DEPLOYED);
    test_shoot_fault_flags = APP_FAULT_FEEDBACK_STALE;
    tick();
    CHECK(robot_state == ROBOT_STOP);
    CHECK(gimbal_cmd_send.gimbal_mode == GIMBAL_ZERO_FORCE);
    CHECK(sent_chassis.chassis_mode == CHASSIS_ZERO_FORCE);
    CHECK(shoot_cmd_send.shoot_mode == SHOOT_OFF);

    test_shoot_fault_flags = APP_FAULT_NONE;
    test_chassis_fault_flags = APP_FAULT_INVALID_COMMAND;
    tick();
    CHECK(robot_state == ROBOT_STOP);
    CHECK(sent_chassis.chassis_mode == CHASSIS_ZERO_FORCE);

    test_chassis_fault_flags = APP_FAULT_NONE;
    remote[0].rc.switch_right = RC_SW_UP;
    tick();
    CHECK(robot_state == ROBOT_READY);
    puts("PASS CMD aggregates Gimbal/Shoot/Chassis fault reports and only recovers after faults clear");
}

static void faults_and_driver(void)
{
    baseline(GIMBAL_DEPLOYED); remote[0].rc.switch_left = RC_SW_MID; imu.Pitch = 10; tick();
    now_ms = fold_ctrl.action_start + GIMBAL_ACTION_TIMEOUT_MS - 5; tick();
    CHECK(DaemonGetFaultFlags(gimbal_cmd_daemon) & APP_FAULT_ACTION_TIMEOUT); CHECK(fold_ctrl.state == GIMBAL_FOLDING); CHECK(yaw_motor->stop_flag == MOTOR_STOP);
    tick(); CHECK(robot_state == ROBOT_STOP); CHECK(DaemonGetFaultFlags(gimbal_cmd_daemon) == APP_FAULT_NONE); // 停机确认后清除故障锁存
    remote[0].rc.switch_right = RC_SW_MID; tick(); CHECK(robot_state == ROBOT_STOP);
    remote[0].rc.switch_right = RC_SW_UP; tick(); tick(); CHECK(robot_state == ROBOT_READY);
    baseline(GIMBAL_DEPLOYED); tick();
    yaw_motor->motor_daemon->temp_count = 0; GimbalTask();
    CHECK(DaemonGetFaultFlags(gimbal_cmd_daemon) & APP_FAULT_FEEDBACK_STALE); CHECK(!gimbal_feedback_data.feedback_valid);
    refresh(); fold_ctrl.active = true; gimbal_cmd_daemon->temp_count = 0; GimbalTask();
    CHECK(DaemonGetFaultFlags(gimbal_cmd_daemon) & APP_FAULT_COMMAND_STALE);
    // Test the actual DM decoder, fractional angle, fault status and bus-off generation.
    CANInstance *can = yaw_motor->motor_can_instace;
    can->rx_len = 8; memset(can->rx_buff, 0, 8); can->rx_buff[0] = 0x11;
    uint16_t encoded = float_to_uint(-0.713f, DM_P_MIN, DM_P_MAX, 16);
    can->rx_buff[1] = encoded >> 8; can->rx_buff[2] = encoded;
    can->rx_buff[3] = 0x7f; can->rx_buff[4] = 0xf0;
    DMMotorDecode(can); CHECK(DMMotorIsFeedbackValid(yaw_motor));
    float angle = yaw_motor->measure.angle_single_round;
    CHECK(angle > 319 && angle < 320); CHECK(fabsf(angle - floorf(angle)) > 0.01f);
    can->rx_buff[0] = 0x81; DMMotorDecode(can); CHECK(!DMMotorIsFeedbackValid(yaw_motor));
    can->rx_buff[0] = 0x11; DMMotorDecode(can); ++bus.bus_off_count;
    CHECK(!DMMotorIsFeedbackValid(yaw_motor)); DMMotorDecode(can);
    CHECK(DMMotorIsFeedbackValid(yaw_motor)); yaw_motor->motor_daemon->temp_count = 0;
    CHECK(!DMMotorIsFeedbackValid(yaw_motor));
    yaw_motor->speed_PID.Iout = 91; DMMotorStop(yaw_motor); DMMotorControl();
    NEAR(yaw_motor->speed_PID.Iout, 91); CHECK(yaw_motor->motor_send_mailbox.Kp == 0);
    fold_ctrl.state = GIMBAL_DEPLOYED;
    fold_ctrl.active = true;
    gimbal_cmd_recv.request_mode = GIMBAL_REQUEST_FOLD;
    yaw_motor->speed_PID.Iout = 91;
    upper_pitch_motor->speed_PID.Iout = 73;
    UpdateFoldState(now_ms);
    NEAR(yaw_motor->speed_PID.Iout, 0); NEAR(yaw_motor->speed_PID.Kp, 50);
    NEAR(upper_pitch_motor->speed_PID.Iout, 0); NEAR(upper_pitch_motor->speed_PID.Kp, 50);
    puts("PASS timeout latch/recovery, missing motor/command feedback, actual DM decode/health, stopped PID and boundary reset");
}

static void completion_and_time_boundaries(void)
{
    baseline(GIMBAL_DEPLOYED); remote[0].rc.switch_left = RC_SW_MID; tick();
    remote[0].rc.switch_left = RC_SW_UP;
    // Complete under left-up: feedback drift must not restore the old transition Yaw target.
    for (int i = 0; i < 1100 && fold_ctrl.state != GIMBAL_FOLDED; ++i)
    {
        imu.YawTotalAngle += 0.01f; follow_lower(); tick();
    }
    CHECK(fold_ctrl.state == GIMBAL_FOLDED); tick();
    NEAR(gimbal_cmd_send.yaw, imu.YawTotalAngle);
    float held = gimbal_cmd_send.yaw; imu.YawTotalAngle += 1;
    tick(); tick(); NEAR(gimbal_cmd_send.yaw, held); // steady left-up still holds a position
    // Down-axis velocity must stay low for the entire finishing interval.
    baseline(GIMBAL_FOLDED); remote[0].rc.switch_left = RC_SW_DOWN; tick();
    lower_pitch_motor->measure.position = 0;
    lower_pitch_motor->measure.velocity = 0.1f;
    fold_ctrl.lower_ref = 0;
    for (int i = 0; i < 50; ++i) tick();
    CHECK(fold_ctrl.state == GIMBAL_UNFOLDING);
    lower_pitch_motor->measure.velocity = 0;
    for (int i = 0; i < 20; ++i) tick();
    CHECK(fold_ctrl.state == GIMBAL_UNFOLDING); tick(); CHECK(fold_ctrl.state == GIMBAL_DEPLOYED);
    // Tick wraparound must not defeat the non-blocking stability/timeout comparisons.
    baseline(GIMBAL_DEPLOYED); now_ms = UINT32_MAX - 50;
    remote[0].rc.switch_left = RC_SW_MID; publish_state(); tick();
    for (int i = 0; i < 25; ++i) tick();
    CHECK(fold_ctrl.step == FOLD_STEP_MOVE_LOWER_PITCH); CHECK(DaemonGetFaultFlags(gimbal_cmd_daemon) == APP_FAULT_NONE);
    // Exact timeout boundary wins even if both targets became stable that cycle.
    fold_ctrl.action_start = now_ms - GIMBAL_ACTION_TIMEOUT_MS + 5;
    fold_ctrl.lower_ref = -1; lower_pitch_motor->measure.position = -1;
    fold_ctrl.finish_stable.tracking = true; fold_ctrl.finish_stable.since = now_ms - 100;
    tick(); CHECK(DaemonGetFaultFlags(gimbal_cmd_daemon) & APP_FAULT_ACTION_TIMEOUT); CHECK(fold_ctrl.state != GIMBAL_FOLDED);
    puts("PASS completion under left-up, no steady feedback resync, lower speed stability, tick wraparound and timeout precedence");
}

int main(void)
{
    RobotCMDInit(); GimbalInit();
    remote[0].rc.switch_left = RC_SW_MID; remote[0].rc.switch_right = RC_SW_UP;
    RobotCMDTask(); CHECK(robot_state == ROBOT_STOP); CHECK(!DaemonIsOnline(gimbal_feedback_daemon));
    GimbalTask(); CHECK(!gimbal_feedback_data.feedback_valid); CHECK(fold_ctrl.state != GIMBAL_FOLDED);
    six_modes(); folding_continuity(); unfolding_and_reversals(); stops_and_angles();
    application_fault_aggregation(); faults_and_driver();
    completion_and_time_boundaries();
    printf("PASS control regression: %d assertions\n", checks);
}
