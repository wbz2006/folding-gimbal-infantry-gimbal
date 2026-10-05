#include "gimbal.h"
#include "robot_def.h"
#include "dmmotor.h"
#include "ins_task.h"
#include "message_center.h"
#include "bmi088.h"
#include <math.h>
#include <stdbool.h>

#include "user_lib.h"

/* ===================== 机械校准点与控制参数 ===================== */
// 折叠目标位置：yaw 回正到 0°，上 pitch 调平到 0°，下 pitch 收到 （收纳位）
#define GIMBAL_FOLD_YAW_TARGET_DEG           0.0f    // 折叠时 yaw 目标角度（度，电机位置环参考）
#define GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG   0.0f    // 折叠时上 pitch 目标角度（度，IMU Pitch 反馈）
#define GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD  (-1.0f)  // 折叠时下 pitch 目标位置（弧度，MIT 位置控制，收纳位）
#define GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD 0.0f    // 展开时下 pitch 目标位置（弧度，工作位）

// 到位判断容差
#define GIMBAL_LOWER_PITCH_TOLERANCE_RAD        0.05f   // 电机位置到位容差（弧度），用于 lower_pitch 到位判断
#define GIMBAL_UPPER_PITCH_TOLERANCE_DEG        1.0f    // 上 pitch 到位容差（度），基于 IMU Pitch 反馈
#define GIMBAL_YAW_TOLERANCE_DEG                1.0f    // yaw 到位容差（度），基于IMU Yaw 反馈

// 下 pitch 电机 MIT 控制参数（直接位置-速度-力矩前馈控制，不经过 PID 模块）
#define GIMBAL_LOWER_KP                      20.0f   // 下 pitch MIT 位置增益
#define GIMBAL_LOWER_KD                      1.0f    // 下 pitch MIT 速度阻尼增益

static attitude_t *gimbal_IMU_data; // 云台IMU数据
static DMMotorInstance *yaw_motor, *upper_pitch_motor , *lower_pitch_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static BMI088Instance *bmi088; // 云台IMU

static gimbal_fold_state_e fold_state = GIMBAL_FOLDING;  // 当前折叠状态，启动默认进入折叠流程
static fold_step_e fold_step = FOLD_STEP_LEVEL_UPPER_PITCH; // 折叠子步骤（仅 FOLDING 状态有效，4 步顺序执行）

/**
 * @brief 电机位置到位判断：比较实际位置与目标位置的差值是否在容差范围内。
 * @param target_type 目标电机位置的种类
 * @return true  已到位（差值 ≤ TOLERANCE）
 * @return false 未到位
 */
static bool GimbalTargetReached(const gimbal_reach_target_type_e target_type)
{
    if (yaw_motor == NULL || upper_pitch_motor == NULL ||
        lower_pitch_motor == NULL || gimbal_IMU_data == NULL)
        return false;
    switch (target_type)
    {
        case LOWER_PITCH_FOLD:
            return fabsf(lower_pitch_motor->measure.position - GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD)
                                    <= GIMBAL_LOWER_PITCH_TOLERANCE_RAD;
        case LOWER_PITCH_DEPLOY:
            return fabsf(lower_pitch_motor->measure.position - GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD)
                                    <= GIMBAL_LOWER_PITCH_TOLERANCE_RAD;
        case UPPER_PITCH_FOLD:
            return fabsf(gimbal_IMU_data->Pitch - GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG)
                                    <= GIMBAL_UPPER_PITCH_TOLERANCE_DEG;
        case YAW_FOLD:
            return fabsf(theta_format(gimbal_IMU_data->Yaw - GIMBAL_FOLD_YAW_TARGET_DEG))
                                    <= GIMBAL_YAW_TOLERANCE_DEG;
    default:
        return false;
    }
}

/**
 * @brief 以 MIT 模式设置下 pitch 电机的目标位置，并使能电机。
 *        不经过标准 PID 模块，直接下发位置/速度/Kp/Kd/前馈力矩，响应更快，适合折叠机构的定点控制。
 * @param position 目标位置（弧度）
 * @note  速度目标固定为 0，前馈力矩固定为 0，仅用位置环 + 阻尼控制。
 */
static void SetLowerPitchMIT(float position)
{
    if (lower_pitch_motor == NULL)
        return;
    DMMotorSetMITRef(lower_pitch_motor, position, 0.0f, GIMBAL_LOWER_KP, GIMBAL_LOWER_KD, 0.0f);
    DMMotorEnable(lower_pitch_motor);
}

/**
 * @brief 进入折叠状态：重置 fold_state 为 FOLDING，并将折叠子步骤复位到第一步（调平上 pitch）。
 *        每次从展开/展开中切换到折叠时调用，确保折叠流程从头开始。
 */
static void EnterFolding(void)
{
    fold_state = GIMBAL_FOLDING;
    fold_step = FOLD_STEP_LEVEL_UPPER_PITCH;
}

/**
 * @brief 进入展开状态：将 fold_state 置为 UNFOLDING。
 *        展开流程较简单（仅需下 pitch 升到工作位），不需要子步骤状态机。
 */
static void EnterUnfolding(void)
{
    fold_state = GIMBAL_UNFOLDING;
}

/**
 * @brief 根据 CMD 应用下发的折叠请求（gimbal_cmd_recv.request_mode）更新 fold_state。
 *
 * 状态转换规则：
 *   - 请求折叠（GIMBAL_REQUEST_FOLD） 且当前处于 DEPLOYED 或 UNFOLDING → 进入 FOLDING
 *   - 请求不折叠（其他模式）且当前处于 FOLDED 或 FOLDING → 进入 UNFOLDING
 *   - 其他情况保持当前状态（例如折叠进行中再次请求折叠，不重置流程）
 *
 * @note  此函数只负责状态转换的"入口判断"，具体的折叠/展开动作由 RunFolding/RunUnfolding 在 GimbalTask 中执行。
 */
static void UpdateFoldState(void)
{
    switch (gimbal_cmd_recv.request_mode)
    {
        case GIMBAL_REQUEST_FOLD:
            if (fold_state == GIMBAL_DEPLOYED || fold_state == GIMBAL_UNFOLDING)
            {
                EnterFolding();
            }
            break;

        case GIMBAL_REQUEST_DEPLOY:
            if (fold_state == GIMBAL_FOLDED || fold_state == GIMBAL_FOLDING)
            {
                EnterUnfolding();
            }
            break;

        default:
            break;
    }
}

/**
 * @brief 执行折叠流程（4 步顺序状态机）。在 fold_state == GIMBAL_FOLDING 时由 GimbalTask 调用。
 *
 * 折叠步骤设计思路（按机械安全优先级排序）：
 *   1. FOLD_STEP_LEVEL_UPPER_PITCH ：先将上 pitch 调平（枪管水平），防止后续 yaw 旋转时枪管碰底盘
 *   2. FOLD_STEP_CENTER_YAW         ：yaw 回正到 0°，使云台与底盘正方向对齐，为收纳做准备
 *   3. FOLD_STEP_MOVE_LOWER_PITCH   ：下 pitch 收到收纳位（-1.0 rad），降低整体高度完成折叠
 *   4. FOLD_STEP_FINISH              ：保持最终折叠姿态，置 fold_state = GIMBAL_FOLDED
 *
 * 每步的通用操作：
 *   - yaw_motor 在不需要旋转的步骤中停止（DMMotorStop），减少功耗和抖动
 *   - upper_pitch_motor 始终使能并保持水平目标（折叠全过程枪管保持水平，避免碰撞）
 *   - lower_pitch 在步骤 1/2 保持当前位置（SetLowerPitchMIT(当前位置)），步骤 3 才移动到收纳位
 *
 * 到位判断：
 *   - 上 pitch 用 PitchReached（IMU 反馈）
 *   - yaw 用 PositionReached（电机编码器位置，目标硬编码为 0.0f）
 *   - 下 pitch 用 PositionReached（电机编码器位置）
 */
static void RunFolding(void)
{
    // 折叠通用前置：yaw 默认停止，上 pitch 使能（具体目标在各 step 中设置）
    DMMotorStop(yaw_motor);
    DMMotorEnable(upper_pitch_motor);

    switch (fold_step)
    {
    /* ---------- 步骤1：调平上 pitch ---------- */
    case FOLD_STEP_LEVEL_UPPER_PITCH:
        // 上 pitch 转到水平目标（0°），下 pitch 保持当前位置不动
        DMMotorSetRef(upper_pitch_motor, GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
        SetLowerPitchMIT(lower_pitch_motor->measure.position);
        if (GimbalTargetReached(UPPER_PITCH_FOLD))
        {
            // 上 pitch 已水平，进入下一步：回正 yaw
            fold_step = FOLD_STEP_CENTER_YAW;
        }
        break;

    /* ---------- 步骤2：回正 yaw ---------- */
    case FOLD_STEP_CENTER_YAW:
        // 上 pitch 继续保持水平；yaw 使能并转到 0°；下 pitch 保持当前位置
        DMMotorSetRef(upper_pitch_motor, GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
        DMMotorEnable(yaw_motor);
        DMMotorSetRef(yaw_motor, GIMBAL_FOLD_YAW_TARGET_DEG);
        SetLowerPitchMIT(lower_pitch_motor->measure.position);
        if (GimbalTargetReached(YAW_FOLD))
        {
            // yaw 已回正，进入下一步：收下 pitch
            fold_step = FOLD_STEP_MOVE_LOWER_PITCH;
        }
        break;

    /* ---------- 步骤3：收下 pitch 到收纳位 ---------- */
    case FOLD_STEP_MOVE_LOWER_PITCH:
        // yaw 停止（已回正无需保持）；上 pitch 保持水平；下 pitch 转到收纳位 -1.0 rad
        DMMotorStop(yaw_motor);
        DMMotorSetRef(upper_pitch_motor, GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
        SetLowerPitchMIT(GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD);
        if (GimbalTargetReached(LOWER_PITCH_FOLD))
        {
            // 下 pitch 已到收纳位，进入完成步骤
            fold_step = FOLD_STEP_FINISH;
        }
        break;

    /* ---------- 步骤4：折叠完成，保持姿态并切换状态 ---------- */
    case FOLD_STEP_FINISH:
    default:
        // 保持最终折叠姿态：yaw 停止，上 pitch 水平，下 pitch 收纳位
        DMMotorStop(yaw_motor);
        DMMotorSetRef(upper_pitch_motor,GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
        SetLowerPitchMIT(GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD);
        // 切换到已折叠状态（后续 GimbalTask 会走 GIMBAL_FOLDED 分支保持姿态）
        fold_state = GIMBAL_FOLDED;
        break;
    }
}

/**
 * @brief 执行展开流程。在 fold_state == GIMBAL_UNFOLDING 时由 GimbalTask 调用。
 *
 * 展开设计思路：
 *   - 展开比折叠简单，因为折叠时 yaw 已回正、上 pitch 已水平，展开只需将下 pitch 从收纳位升到工作位。
 *   - yaw 保持停止（无需动作，已在 0°），上 pitch 保持水平（展开后由 DEPLOYED 分支接管目标角度）。
 *   - 下 pitch 升到工作位（0.0 rad）后，直接置 fold_state = GIMBAL_DEPLOYED，进入正常工作模式。
 *
 * @note 展开过程中 yaw 和 upper_pitch 不接收外部目标角度，确保展开动作不受遥控器干扰。
 */
static void RunUnfolding(void)
{
    /* Lower pitch changes the height first; yaw and upper pitch stay parked. */
    // yaw 停止，上 pitch 使能并保持水平，下 pitch 升到工作位
    DMMotorStop(yaw_motor);
    DMMotorEnable(upper_pitch_motor);
    DMMotorSetRef(upper_pitch_motor, GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
    SetLowerPitchMIT(GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD);

    if (GimbalTargetReached(LOWER_PITCH_FOLD))
    {
        // 下 pitch 已到工作位，展开完成，进入正常工作模式
        fold_state = GIMBAL_DEPLOYED;
    }
}

void GimbalInit()
{   
    gimbal_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan3,
            .tx_id = 0x01,
            .rx_id = 0x02,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 8, // 8
                .Ki = 0,
                .Kd = 0,
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 50,  // 50
                .Ki = 200, // 200
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 20000,
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
        .motor_type = DM4340};
    // GIMBAL_PITCH
    Motor_Init_Config_s upper_pitch_config = {
        .can_init_config = {
            .can_handle = &hcan3,
            .tx_id = 0x04,
            .rx_id = 0x03
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 10, // 10
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 50,  // 50
                .Ki = 350, // 350
                .Kd = 0,   // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2500,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->Pitch,
            .other_speed_feedback_ptr = (&gimbal_IMU_data->Gyro[0]),
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = DM4310,
    };
    // FOLDING_PITCH
    Motor_Init_Config_s lower_pitch_config = {
        .can_init_config = {
            .can_handle = &hcan3,
            .tx_id = 0x05,
            .rx_id = 0x06
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 10, // 10
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 50,  // 50
                .Ki = 350, // 350
                .Kd = 0,   // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2500,
                .MaxOut = 20000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = DM4340,
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DMMotorInit(&yaw_config);
    upper_pitch_motor = DMMotorInit(&upper_pitch_config);
    lower_pitch_motor = DMMotorInit(&lower_pitch_config);
    if (lower_pitch_motor != NULL)
        lower_pitch_motor->control_mode = DMMOTOR_CONTROL_MIT;

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));

    gimbal_cmd_recv.request_mode = GIMBAL_REQUEST_FOLD;
    gimbal_feedback_data.fold_state = GIMBAL_FOLDING;
}


void GimbalTask()
{
    if (yaw_motor == NULL || upper_pitch_motor == NULL ||
        lower_pitch_motor == NULL || gimbal_IMU_data == NULL)
        return;

    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    // ---------- 根据折叠请求更新状态机 ----------
    UpdateFoldState();


    if (gimbal_cmd_recv.gimbal_mode == GIMBAL_ZERO_FORCE)
    {
        // 急停：所有电机停止输出，力矩为零
        DMMotorStop(yaw_motor);
        DMMotorStop(upper_pitch_motor);
        DMMotorStop(lower_pitch_motor);
    }
    else
    {
        switch (fold_state)
        {
            case GIMBAL_FOLDING:
                // 折叠进行中：执行 4 步折叠状态机
                RunFolding();
                break;
            case GIMBAL_UNFOLDING:
                // 展开进行中：下 pitch 升工作位
                RunUnfolding();
                break;
            case GIMBAL_FOLDED:
                // 已折叠：保持折叠姿态（yaw 停止，上 pitch 水平，下 pitch 收纳位）
                DMMotorStop(yaw_motor);
                DMMotorEnable(upper_pitch_motor);
                DMMotorSetRef(upper_pitch_motor, GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
                SetLowerPitchMIT(GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD);
                break;
            case GIMBAL_DEPLOYED:
            default:
                // 已展开：正常工作模式，yaw/upper_pitch 跟踪外部目标角度，下 pitch 保持工作位
                DMMotorEnable(yaw_motor);
                DMMotorEnable(upper_pitch_motor);
                DMMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);           // yaw 目标角度来自 CMD 应用
                DMMotorSetRef(upper_pitch_motor, gimbal_cmd_recv.pitch); // 上 pitch 目标角度来自 CMD 应用
                SetLowerPitchMIT(GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD);  // 下 pitch 保持工作位
                break;
        }
    }

    // ---------- 4. 填充并上报反馈数据 ----------
    gimbal_feedback_data.gimbal_imu_data = *gimbal_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = (uint16_t)yaw_motor->measure.angle_single_round;
    gimbal_feedback_data.fold_state = fold_state;           // 当前折叠状态（供 CMD 应用做底盘联动）

    PubPushMessage(gimbal_pub, &gimbal_feedback_data);

}
