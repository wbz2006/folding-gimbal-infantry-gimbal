#include "gimbal.h"
#include "robot_def.h"
#include "dmmotor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"

#include <math.h>
#include <stdbool.h>

/* These values are mechanical calibration points in motor/IMU radians. */
#define GIMBAL_FOLD_YAW_TARGET_DEG           0.0f
#define GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG   0.0f
#define GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD  (-1.0f)
#define GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD 0.0f
#define GIMBAL_POSITION_TOLERANCE_RAD        0.05f
#define GIMBAL_PITCH_TOLERANCE_DEG           1.0f
#define GIMBAL_LOWER_KP                      20.0f
#define GIMBAL_LOWER_KD                      1.0f

static attitude_t *gimbal_IMU_data;
static DMMotorInstance *yaw_motor;
static DMMotorInstance *upper_pitch_motor;
static DMMotorInstance *lower_pitch_motor;

static Publisher_t *gimbal_pub;
static Subscriber_t *gimbal_sub;
static Gimbal_Upload_Data_s gimbal_feedback_data;
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;

static gimbal_fold_state_e fold_state = GIMBAL_FOLDING;
static fold_step_e fold_step = FOLD_STEP_LEVEL_UPPER_PITCH;

static bool PositionReached(float actual, float target)
{
    return fabsf(actual - target) <= GIMBAL_POSITION_TOLERANCE_RAD;
}

static bool PitchReached(float target)
{
    return fabsf(gimbal_IMU_data->Pitch - target) <=
           GIMBAL_PITCH_TOLERANCE_DEG;
}

static void SetLowerPitchMIT(float position)
{
    DMMotorSetMITRef(lower_pitch_motor, position, 0.0f,
                     GIMBAL_LOWER_KP, GIMBAL_LOWER_KD, 0.0f);
    DMMotorEnable(lower_pitch_motor);
}

static void EnterFolding(void)
{
    fold_state = GIMBAL_FOLDING;
    fold_step = FOLD_STEP_LEVEL_UPPER_PITCH;
}

static void EnterUnfolding(void)
{
    fold_state = GIMBAL_UNFOLDING;
}

static void UpdateFoldState(void)
{
    const bool fold_requested =
        gimbal_cmd_recv.request_mode == GIMBAL_REQUEST_FOLD;

    if (fold_requested &&
        (fold_state == GIMBAL_DEPLOYED || fold_state == GIMBAL_UNFOLDING))
    {
        EnterFolding();
    }
    else if (!fold_requested &&
             (fold_state == GIMBAL_FOLDED || fold_state == GIMBAL_FOLDING))
    {
        EnterUnfolding();
    }
}

static void RunFolding(void)
{
    DMMotorStop(yaw_motor);
    DMMotorEnable(upper_pitch_motor);

    switch (fold_step)
    {
    case FOLD_STEP_LEVEL_UPPER_PITCH:
        DMMotorSetRef(upper_pitch_motor,
                      GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
        SetLowerPitchMIT(lower_pitch_motor->measure.position);
        if (PitchReached(GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG))
        {
            fold_step = FOLD_STEP_CENTER_YAW;
        }
        break;

    case FOLD_STEP_CENTER_YAW:
        DMMotorSetRef(upper_pitch_motor,
                      GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
        DMMotorEnable(yaw_motor);
        DMMotorSetRef(yaw_motor, GIMBAL_FOLD_YAW_TARGET_DEG);
        SetLowerPitchMIT(lower_pitch_motor->measure.position);
        if (PositionReached(yaw_motor->measure.position,
                            0.0f))
        {
            fold_step = FOLD_STEP_MOVE_LOWER_PITCH;
        }
        break;

    case FOLD_STEP_MOVE_LOWER_PITCH:
        DMMotorStop(yaw_motor);
        DMMotorSetRef(upper_pitch_motor,
                      GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
        SetLowerPitchMIT(GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD);
        if (PositionReached(lower_pitch_motor->measure.position,
                            GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD))
        {
            fold_step = FOLD_STEP_FINISH;
        }
        break;

    case FOLD_STEP_FINISH:
    default:
        DMMotorStop(yaw_motor);
        DMMotorSetRef(upper_pitch_motor,
                      GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
        SetLowerPitchMIT(GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD);
        fold_state = GIMBAL_FOLDED;
        break;
    }
}

static void RunUnfolding(void)
{
    /* Lower pitch changes the height first; yaw and upper pitch stay parked. */
    DMMotorStop(yaw_motor);
    DMMotorEnable(upper_pitch_motor);
    DMMotorSetRef(upper_pitch_motor,
                  GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
    SetLowerPitchMIT(GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD);

    if (PositionReached(lower_pitch_motor->measure.position,
                        GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD))
    {
        fold_state = GIMBAL_DEPLOYED;
    }
}

void GimbalInit(void)
{
    gimbal_IMU_data = INS_Init();

    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan3,
            .tx_id = 0x01,
            .rx_id = 0x02,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 8.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .DeadBand = 0.1f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .IntegralLimit = 100.0f,
                .MaxOut = 500.0f,
            },
            .speed_PID = {
                .Kp = 50.0f,
                .Ki = 200.0f,
                .Kd = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .IntegralLimit = 3000.0f,
                .MaxOut = 20000.0f,
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
        .motor_type = DM4340,
    };

    Motor_Init_Config_s upper_pitch_config = {
        .can_init_config = {
            .can_handle = &hcan3,
            .tx_id = 0x04,
            .rx_id = 0x03,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 10.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .IntegralLimit = 100.0f,
                .MaxOut = 500.0f,
            },
            .speed_PID = {
                .Kp = 50.0f,
                .Ki = 350.0f,
                .Kd = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .IntegralLimit = 2500.0f,
                .MaxOut = 20000.0f,
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
        .motor_type = DM4340,
    };

    Motor_Init_Config_s lower_pitch_config = {
        .can_init_config = {
            .can_handle = &hcan3,
            .tx_id = 0x05,
            .rx_id = 0x06,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = DM4340,
    };

    yaw_motor = DMMotorInit(&yaw_config);
    upper_pitch_motor = DMMotorInit(&upper_pitch_config);
    lower_pitch_motor = DMMotorInit(&lower_pitch_config);
    DMMotorSetControlMode(lower_pitch_motor, DMMOTOR_CONTROL_MIT);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));

    gimbal_cmd_recv.request_mode = GIMBAL_REQUEST_FOLD;
    gimbal_feedback_data.fold_state = GIMBAL_FOLDING;
}

void GimbalTask(void)
{
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    UpdateFoldState();

    if (gimbal_cmd_recv.gimbal_mode == GIMBAL_ZERO_FORCE)
    {
        DMMotorStop(yaw_motor);
        DMMotorStop(upper_pitch_motor);
        DMMotorStop(lower_pitch_motor);
    }
    else
    {
        switch (fold_state)
        {
        case GIMBAL_FOLDING:
            RunFolding();
            break;
        case GIMBAL_UNFOLDING:
            RunUnfolding();
            break;
        case GIMBAL_FOLDED:
            DMMotorStop(yaw_motor);
            DMMotorEnable(upper_pitch_motor);
            DMMotorSetRef(upper_pitch_motor,
                          GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
            SetLowerPitchMIT(GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD);
            break;
        case GIMBAL_DEPLOYED:
        default:
            DMMotorEnable(yaw_motor);
            DMMotorEnable(upper_pitch_motor);
            DMMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);
            DMMotorSetRef(upper_pitch_motor, gimbal_cmd_recv.pitch);
            SetLowerPitchMIT(GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD);
            break;
        }
    }

    gimbal_feedback_data.gimbal_imu_data = *gimbal_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle =
        (uint16_t)yaw_motor->measure.angle_single_round;
    gimbal_feedback_data.fold_state = fold_state;
    PubPushMessage(gimbal_pub, &gimbal_feedback_data);
}
