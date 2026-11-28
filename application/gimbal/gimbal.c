#include "gimbal.h"
#include "robot_def.h"
#include "dmmotor.h"
#include "ins_task.h"
#include "message_center.h"
#include "daemon.h"
#include "user_lib.h"
#include "general_def.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

// 目标位置：上 pitch 使用 IMU 角度(deg)，下 pitch 使用电机位置(rad)。
#define GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG     (0.0f)
#define GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD     (-2.14f)
#define GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD   (-3.2f)
#define GIMBAL_LOWER_PITCH_MIT_KP              (100.0f)
#define GIMBAL_LOWER_PITCH_MIT_KD              (2.5f)
// MIT 力矩前馈，单位 N*m；需按实际保持力矩标定正负和大小。
#define GIMBAL_LOWER_PITCH_MIT_TORQUE_FF_NM    (0.3f)

// 到位位置误差限值。
#define GIMBAL_UPPER_PITCH_TOLERANCE_DEG      2.0f
#define GIMBAL_LOWER_PITCH_TOLERANCE_RAD      0.01f
#define GIMBAL_YAW_TOLERANCE_DEG              2.0f

// 到位判定要求对应速度低于该限值(rad/s)。
#define GIMBAL_UPPER_STABLE_SPEED_RAD_S       0.5f
#define GIMBAL_YAW_STABLE_SPEED_RAD_S         0.1f
#define GIMBAL_LOWER_STABLE_SPEED_RAD_S       0.03f

// 下 pitch 参考位置变化率及单周期最大计时步长。
#define GIMBAL_LOWER_REF_RATE_RAD_S           1.2f
#define GIMBAL_LOWER_SLOWDOWN_DISTANCE_RAD    0.01f
#define GIMBAL_LOWER_FINAL_REF_RATE_RAD_S     0.75f
#define GIMBAL_REF_MAX_DT_S                   0.02f

// 到位条件持续时间、动作超时时间及电机使能重试间隔。
#define GIMBAL_STABLE_TIME_MS                 100U
#define GIMBAL_ACTION_TIMEOUT_MS              15000U
#define GIMBAL_MOTOR_ENABLE_RETRY_MS          100U

typedef struct
{
    uint32_t since;
    bool tracking;
} StableTimer;

typedef struct
{
    gimbal_fold_state_e state;
    fold_step_e step;
    bool active;
    float lower_ref;
    uint32_t action_start;
    uint32_t last_run;
    StableTimer upper_stable;
    StableTimer yaw_stable;
    StableTimer finish_stable;
} GimbalFoldCtrl_s;

static attitude_t *gimbal_IMU_data;
static DMMotorInstance *yaw_motor;
static DMMotorInstance *upper_pitch_motor;
static DMMotorInstance *lower_pitch_motor;
static Publisher_t *gimbal_pub;
static Subscriber_t *gimbal_sub;
static DaemonInstance *gimbal_cmd_daemon;
static Gimbal_Upload_Data_s gimbal_feedback_data;
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;
static uint32_t motor_enable_last_request;

static GimbalFoldCtrl_s fold_ctrl = {
    .state = GIMBAL_FOLDING,
    .step = FOLD_STEP_LEVEL_UPPER_PITCH,
};

static void SetLowerPitchRef(float ref)
{
    DMMotorSetRef(lower_pitch_motor, ref);
    // MIT 位置目标使用原始电机坐标，需要补回中心映射移除的整圈偏移。
    const float motor_position_offset = lower_pitch_motor->measure.position -
                                        lower_pitch_motor->measure.control_position;
    DMMotorSetMITRef(lower_pitch_motor, ref + motor_position_offset, 0.0f,
                     GIMBAL_LOWER_PITCH_MIT_KP,
                     GIMBAL_LOWER_PITCH_MIT_KD,
                     GIMBAL_LOWER_PITCH_MIT_TORQUE_FF_NM);
}

static bool GimbalTargetReached(gimbal_reach_target_type_e target_type)
{
    switch (target_type)
    {
        case LOWER_PITCH_FOLD:
            return fabsf(lower_pitch_motor->measure.control_position - GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD) <=
                       GIMBAL_LOWER_PITCH_TOLERANCE_RAD &&
                   fabsf(lower_pitch_motor->measure.velocity) <= GIMBAL_LOWER_STABLE_SPEED_RAD_S;
        case LOWER_PITCH_DEPLOY:
            return fabsf(lower_pitch_motor->measure.control_position - GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD) <=
                       GIMBAL_LOWER_PITCH_TOLERANCE_RAD &&
                   fabsf(lower_pitch_motor->measure.velocity) <= GIMBAL_LOWER_STABLE_SPEED_RAD_S;
        case UPPER_PITCH_FOLD:
            return fabsf(gimbal_IMU_data->Pitch - GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG) <=
                       GIMBAL_UPPER_PITCH_TOLERANCE_DEG &&
                   fabsf(gimbal_IMU_data->Gyro[0]) <= GIMBAL_UPPER_STABLE_SPEED_RAD_S;
        case YAW_FOLD:
            return fabsf(theta_format(yaw_motor->measure.angle_single_round -
                                      GIMBAL_YAW_MECHANICAL_CENTER_DEG)) <=
                       GIMBAL_YAW_TOLERANCE_DEG &&
                   fabsf(yaw_motor->measure.velocity) <= GIMBAL_YAW_STABLE_SPEED_RAD_S;
        default:
            return false;
    }
}

static void UpdateFoldState(uint32_t now)
{
    gimbal_fold_state_e next_state = fold_ctrl.state;
    if (!fold_ctrl.active)
        next_state = gimbal_cmd_recv.request_mode == GIMBAL_REQUEST_FOLD ?
            GIMBAL_FOLDING : GIMBAL_UNFOLDING;
    else if (gimbal_cmd_recv.request_mode == GIMBAL_REQUEST_FOLD &&
             (fold_ctrl.state == GIMBAL_UNFOLDING || fold_ctrl.state == GIMBAL_DEPLOYED))
        next_state = GIMBAL_FOLDING;
    else if (gimbal_cmd_recv.request_mode == GIMBAL_REQUEST_DEPLOY &&
             (fold_ctrl.state == GIMBAL_FOLDING || fold_ctrl.state == GIMBAL_FOLDED))
        next_state = GIMBAL_UNFOLDING;

    if (next_state != fold_ctrl.state || !fold_ctrl.active)
    {
        fold_ctrl.state = next_state;
        fold_ctrl.step = FOLD_STEP_LEVEL_UPPER_PITCH;
        fold_ctrl.lower_ref = fold_ctrl.active ?
            lower_pitch_motor->pid_ref : lower_pitch_motor->measure.control_position;
        fold_ctrl.upper_stable.tracking = false;
        fold_ctrl.yaw_stable.tracking = false;
        fold_ctrl.finish_stable.tracking = false;
        fold_ctrl.action_start = now;
        fold_ctrl.last_run = now;
        DMMotorResetPIDState(yaw_motor);
        DMMotorResetPIDState(upper_pitch_motor);
    }
    fold_ctrl.active = true;
}

static void HoldYawCenter(void)
{
    // 编码器只用于机械回中误差；角度/速度闭环仍取 IMU。
    const float offset = theta_format(yaw_motor->measure.angle_single_round -
                                      GIMBAL_YAW_MECHANICAL_CENTER_DEG);
    DMMotorSetRef(yaw_motor, gimbal_IMU_data->YawTotalAngle - offset);
    DMMotorEnable(yaw_motor);
}

static void RunTransition(uint32_t now, float lower_target,
                          gimbal_reach_target_type_e lower_position,
                          gimbal_fold_state_e completed, bool center_yaw)
{
    const bool upper_reached = GimbalTargetReached(UPPER_PITCH_FOLD);
    DMMotorSetRef(upper_pitch_motor, GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);

    // 调平偏离时锁存下轴位置，并从调平步骤重新检查回中顺序。
    if (!GIMBAL_LOWER_PITCH_TUNING && fold_ctrl.step != FOLD_STEP_LEVEL_UPPER_PITCH &&
        fabsf(gimbal_IMU_data->Pitch - GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG) >
            GIMBAL_UPPER_PITCH_TOLERANCE_DEG)
    {
        fold_ctrl.lower_ref = lower_pitch_motor->measure.control_position;
        fold_ctrl.step = FOLD_STEP_LEVEL_UPPER_PITCH;
        fold_ctrl.upper_stable.tracking = false;
        fold_ctrl.yaw_stable.tracking = false;
        fold_ctrl.finish_stable.tracking = false;
    }

    if (center_yaw && fold_ctrl.step != FOLD_STEP_LEVEL_UPPER_PITCH)
        HoldYawCenter();
    else
        DMMotorStop(yaw_motor);

    if (fold_ctrl.step == FOLD_STEP_LEVEL_UPPER_PITCH)
    {
        if (!upper_reached)
            fold_ctrl.upper_stable.tracking = false;
        else if (!fold_ctrl.upper_stable.tracking)
        {
            fold_ctrl.upper_stable.tracking = true;
            fold_ctrl.upper_stable.since = now;
        }

        if (fold_ctrl.upper_stable.tracking &&
            (uint32_t)(now - fold_ctrl.upper_stable.since) >= GIMBAL_STABLE_TIME_MS)
            fold_ctrl.step = center_yaw ? FOLD_STEP_CENTER_YAW : FOLD_STEP_MOVE_LOWER_PITCH;
    }
    else if (fold_ctrl.step == FOLD_STEP_CENTER_YAW)
    {
        if (!GimbalTargetReached(YAW_FOLD))
            fold_ctrl.yaw_stable.tracking = false;
        else if (!fold_ctrl.yaw_stable.tracking)
        {
            fold_ctrl.yaw_stable.tracking = true;
            fold_ctrl.yaw_stable.since = now;
        }
        if (fold_ctrl.yaw_stable.tracking &&
            (uint32_t)(now - fold_ctrl.yaw_stable.since) >= GIMBAL_STABLE_TIME_MS)
            fold_ctrl.step = FOLD_STEP_MOVE_LOWER_PITCH;
    }
    else
    {
        const float dt = float_constrain(
            (float)(uint32_t)(now - fold_ctrl.last_run) * 0.001f,
            0.0f, GIMBAL_REF_MAX_DT_S);
        // 提前降低参考速度，避免参考到达终点时仍以全速推进。
        const float ref_rate = float_constrain(
            GIMBAL_LOWER_REF_RATE_RAD_S *
                fabsf(lower_target - fold_ctrl.lower_ref) / GIMBAL_LOWER_SLOWDOWN_DISTANCE_RAD,
            GIMBAL_LOWER_FINAL_REF_RATE_RAD_S, GIMBAL_LOWER_REF_RATE_RAD_S);
        fold_ctrl.lower_ref += float_constrain(
            lower_target - fold_ctrl.lower_ref,
            -ref_rate * dt, ref_rate * dt);

        const bool reached = (GIMBAL_LOWER_PITCH_TUNING || upper_reached) &&
                             GimbalTargetReached(lower_position) &&
                             fabsf(fold_ctrl.lower_ref - lower_target) < 0.0001f;
        if (!reached)
            fold_ctrl.finish_stable.tracking = false;
        else if (!fold_ctrl.finish_stable.tracking)
        {
            fold_ctrl.finish_stable.tracking = true;
            fold_ctrl.finish_stable.since = now;
        }

        if (fold_ctrl.finish_stable.tracking &&
            (uint32_t)(now - fold_ctrl.finish_stable.since) >= GIMBAL_STABLE_TIME_MS)
        {
            fold_ctrl.step = FOLD_STEP_FINISH;
            fold_ctrl.state = completed;
        }
    }

    SetLowerPitchRef(fold_ctrl.lower_ref);
    DMMotorEnable(lower_pitch_motor);
}

void GimbalInit(void)
{
    gimbal_IMU_data = INS_GetData();
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 0x03,
            .rx_id = 0x603,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.2f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .DeadBand = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .IntegralLimit = 100.0f,
                .MaxOut = 5.0f,
            },
            .speed_PID = {
                .Kp = 0.5f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .IntegralLimit = 150.0f,
                .MaxOut = 5.0f,
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->YawTotalAngle,
            .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[2],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = DM4310,
    };

    Motor_Init_Config_s upper_pitch_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x04,
            .rx_id = 0x604,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.8f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .IntegralLimit = 100.0f,
                .MaxOut = 1.0f,
            },
            .speed_PID = {
                .Kp = 1.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .IntegralLimit = 100.0f,
                .MaxOut = 1.0f,
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->Pitch,
            .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[0],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = DM4310,
    };

    Motor_Init_Config_s lower_pitch_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x02,
            .rx_id = 0x602,
        },
        .motor_type = DM4310,
    };

    yaw_motor = DMMotorInit(&yaw_config);
    DMMotorDisable(yaw_motor);
    upper_pitch_motor = DMMotorInit(&upper_pitch_config);
    DMMotorDisable(upper_pitch_motor);
    lower_pitch_motor = DMMotorInit(&lower_pitch_config);
    DMMotorSetControlMode(lower_pitch_motor, DMMOTOR_CONTROL_MIT);
    DMMotorSetPositionCenter(lower_pitch_motor,
        0.5f * (GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD + GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD));
    DMMotorDisable(lower_pitch_motor);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));

    Daemon_Init_Config_s cmd_daemon_config = {
        .reload_count = DAEMON_COUNT_FROM_MS(APP_COMMAND_TIMEOUT_MS),
        .require_first_reload = 1,
    };
    gimbal_cmd_daemon = DaemonRegister(&cmd_daemon_config);

    gimbal_cmd_recv.request_mode = GIMBAL_REQUEST_FOLD;
    gimbal_feedback_data.fold_state = GIMBAL_FOLDING;
}

void GimbalTask(void)
{
    const uint32_t now = HAL_GetTick();
    const bool command_received = SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    const bool command_fields_valid =
        isfinite(gimbal_cmd_recv.yaw) && isfinite(gimbal_cmd_recv.pitch) &&
        (gimbal_cmd_recv.gimbal_mode == GIMBAL_IMU_MODE ||
         gimbal_cmd_recv.gimbal_mode == GIMBAL_ZERO_FORCE) &&
        (gimbal_cmd_recv.request_mode == GIMBAL_REQUEST_FOLD ||
         gimbal_cmd_recv.request_mode == GIMBAL_REQUEST_DEPLOY);
    if (command_received && command_fields_valid)
        DaemonReload(gimbal_cmd_daemon);

    App_Fault_s *fault = &gimbal_feedback_data.fault;
    memset(fault, 0, sizeof(*fault));
    if (!DaemonIsOnline(gimbal_cmd_daemon))
        fault->comm_fault_flags |= APP_COMM_FAULT_COMMAND_OFFLINE;
    if (!command_fields_valid)
        fault->comm_fault_flags |= APP_COMM_FAULT_INVALID_COMMAND;

    DMMotorInstance *const motors[] = {yaw_motor, upper_pitch_motor, lower_pitch_motor};
    const uint8_t motor_flags[] = {
        MOTOR_1_OFFLINE, MOTOR_2_OFFLINE, MOTOR_3_OFFLINE
    };
    const uint8_t required_motor_flags = GIMBAL_UPPER_PITCH_TUNING ?
        MOTOR_2_OFFLINE : GIMBAL_LOWER_PITCH_TUNING ?
        MOTOR_2_OFFLINE | MOTOR_3_OFFLINE : MOTOR_1_OFFLINE | MOTOR_2_OFFLINE | MOTOR_3_OFFLINE;
    uint8_t disabled_motor_flags = 0;
    for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); ++i)
    {
        if (!DMMotorIsFeedbackValid(motors[i]))
            fault->motor_offline_flags |= motor_flags[i];
        if ((required_motor_flags & motor_flags[i]) &&
            DaemonIsOnline(motors[i]->motor_daemon) && motors[i]->measure.state == 0)
            disabled_motor_flags |= motor_flags[i];
    }

    if (!INS_IsFeedbackValid())
        fault->imu_fault_flags |= APP_IMU_FAULT_OFFLINE;
    if (!isfinite(gimbal_IMU_data->YawTotalAngle) ||
        !isfinite(gimbal_IMU_data->Pitch) ||
        !isfinite(gimbal_IMU_data->Gyro[0]) ||
        !isfinite(gimbal_IMU_data->Gyro[2]))
        fault->imu_fault_flags |= APP_IMU_FAULT_DATA_INVALID;
    AppFaultUpdate(gimbal_cmd_daemon, fault);

    const bool command_valid = DaemonIsOnline(gimbal_cmd_daemon) && command_fields_valid;
    // 沿用临时联调策略：反馈健康故障上报，不新增失反馈动作锁存。
    if (command_valid && gimbal_cmd_recv.gimbal_mode == GIMBAL_ZERO_FORCE)
        gimbal_feedback_data.action_fault_flags = GIMBAL_ACTION_FAULT_NONE;

    const bool enable_requested = command_valid &&
        gimbal_feedback_data.action_fault_flags == GIMBAL_ACTION_FAULT_NONE &&
        gimbal_cmd_recv.gimbal_mode == GIMBAL_IMU_MODE;
    bool control_allowed = enable_requested &&
                           disabled_motor_flags == 0 &&
                           (fault->motor_offline_flags & required_motor_flags) == 0 &&
                           (fault->imu_fault_flags & APP_IMU_FAULT_DATA_INVALID) == 0;

    if (control_allowed && GIMBAL_UPPER_PITCH_TUNING)
    {
        DMMotorStop(yaw_motor);
        DMMotorStop(lower_pitch_motor);
        DMMotorSetRef(upper_pitch_motor, GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
        if (upper_pitch_motor->stop_flag == MOTOR_STOP)
            DMMotorResetPIDState(upper_pitch_motor);
        DMMotorEnable(upper_pitch_motor);
        fold_ctrl.active = false;
        fold_ctrl.state = GIMBAL_FOLDING;
        fold_ctrl.step = FOLD_STEP_LEVEL_UPPER_PITCH;
    }
    else if (control_allowed)
    {
        UpdateFoldState(now);
        const bool transitioning = fold_ctrl.state == GIMBAL_FOLDING ||
                                   fold_ctrl.state == GIMBAL_UNFOLDING;
        DMMotorEnable(upper_pitch_motor);

        if (fold_ctrl.state == GIMBAL_FOLDING)
            RunTransition(now, GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD,
                          LOWER_PITCH_FOLD, GIMBAL_FOLDED, !GIMBAL_LOWER_PITCH_TUNING);
        else if (fold_ctrl.state == GIMBAL_UNFOLDING)
            RunTransition(now, GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD,
                          LOWER_PITCH_DEPLOY, GIMBAL_DEPLOYED, false);
        else if (fold_ctrl.state == GIMBAL_FOLDED || GIMBAL_LOWER_PITCH_TUNING)
        {
            if (GIMBAL_LOWER_PITCH_TUNING)
                DMMotorStop(yaw_motor);
            else
                HoldYawCenter();
            DMMotorEnable(upper_pitch_motor);
            DMMotorSetRef(upper_pitch_motor, GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
            SetLowerPitchRef(fold_ctrl.state == GIMBAL_FOLDED ?
                GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD : GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD);
            DMMotorEnable(lower_pitch_motor);
        }
        else
        {
            DMMotorEnable(yaw_motor);
            DMMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);
            DMMotorEnable(upper_pitch_motor);
            DMMotorSetRef(upper_pitch_motor, gimbal_cmd_recv.pitch);
            SetLowerPitchRef(GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD);
            DMMotorEnable(lower_pitch_motor);
        }
        if (!GIMBAL_LOWER_PITCH_TUNING && transitioning &&
            (uint32_t)(now - fold_ctrl.action_start) >= GIMBAL_ACTION_TIMEOUT_MS)
        {
            gimbal_feedback_data.action_fault_flags |= GIMBAL_ACTION_FAULT_TIMEOUT;
            control_allowed = false;
        }
        fold_ctrl.last_run = now;
    }

    if (!control_allowed)
    {
        DMMotorStop(yaw_motor);
        DMMotorStop(lower_pitch_motor);
        // 下轴失去反馈时，仍用健康的上轴维持调平，避免机构因重力下落。
        if (GIMBAL_LOWER_PITCH_TUNING && enable_requested &&
            (fault->motor_offline_flags & MOTOR_2_OFFLINE) == 0 &&
            (fault->imu_fault_flags & APP_IMU_FAULT_DATA_INVALID) == 0)
        {
            DMMotorSetRef(upper_pitch_motor, GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
            if (upper_pitch_motor->stop_flag == MOTOR_STOP)
                DMMotorResetPIDState(upper_pitch_motor);
            DMMotorEnable(upper_pitch_motor);
        }
        else
            DMMotorStop(upper_pitch_motor);
        fold_ctrl.active = false;
        fold_ctrl.state = gimbal_cmd_recv.request_mode == GIMBAL_REQUEST_DEPLOY ?
            GIMBAL_UNFOLDING : GIMBAL_FOLDING;

        if (enable_requested && disabled_motor_flags != 0 &&
            (uint32_t)(now - motor_enable_last_request) >= GIMBAL_MOTOR_ENABLE_RETRY_MS)
        {
            motor_enable_last_request = now;
            for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); ++i)
                if ((disabled_motor_flags & motor_flags[i]) &&
                    CANIsReady(motors[i]->motor_can_instace))
                    DMMotorRequestEnable(motors[i]);
        }
    }

    if (gimbal_IMU_data != NULL)
        gimbal_feedback_data.gimbal_imu_data = *gimbal_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;
    gimbal_feedback_data.yaw_motor_position_rad = yaw_motor->measure.position;
    gimbal_feedback_data.fold_state = fold_ctrl.state;
    gimbal_feedback_data.feedback_valid = DaemonIsFeedbackValid(gimbal_cmd_daemon);
    PubPushMessage(gimbal_pub, &gimbal_feedback_data);
}
