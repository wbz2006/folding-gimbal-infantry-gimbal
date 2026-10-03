/**
 * @file gimbal.c
 * @brief 折叠云台应用层：控制三轴云台（yaw 偏航 / upper_pitch 上俯仰 / lower_pitch 下俯仰），
 *        实现折叠-展开状态机、电机闭环控制、IMU 数据融合与反馈上报。
 *
 * 机械结构说明：
 *   - yaw_motor         ：云台水平旋转轴，使用 IMU YawTotalAngle 作为角度反馈（陀螺仪环）
 *   - upper_pitch_motor ：上俯仰轴（直接负载枪管/相机），使用 IMU Pitch 作为角度反馈
 *   - lower_pitch_motor ：下俯仰轴（折叠升降机构），使用电机自身编码器反馈，MIT 模式直接控制位置
 *
 * 折叠状态机（fold_state）：
 *   GIMBAL_FOLDING   → 折叠进行中（4 步顺序执行：调平上pitch → 回正yaw → 收下pitch → 完成）
 *   GIMBAL_FOLDED    → 已折叠（保持折叠姿态，yaw 停止，上pitch保持水平，下pitch保持收纳位）
 *   GIMBAL_UNFOLDING → 展开进行中（下pitch升到工作位即完成）
 *   GIMBAL_DEPLOYED  → 已展开（正常工作，yaw/upper_pitch 接收外部目标角度，下pitch保持工作位）
 *
 * 控制模式：
 *   GIMBAL_ZERO_FORCE → 所有电机停止输出（急停）
 *   GIMBAL_GYRO_MODE  → 陀螺仪环模式，yaw/upper_pitch 用 IMU 反馈做角度闭环
 *
 * 通信：通过消息中心 "gimbal_cmd" 订阅控制指令，"gimbal_feed" 发布反馈数据（IMU、电机角度、折叠状态）。
 */
#include "gimbal.h"
#include "robot_def.h"
#include "dmmotor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"

#include <math.h>
#include <stdbool.h>

/* ===================== 机械校准点与控制参数 ===================== */
/* These values are mechanical calibration points in motor/IMU radians. */
// 折叠目标位置：yaw 回正到 0°，上 pitch 调平到 0°，下 pitch 收到 -1.0 rad（收纳位）
#define GIMBAL_FOLD_YAW_TARGET_DEG           0.0f    // 折叠时 yaw 目标角度（度，电机位置环参考）
#define GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG   0.0f    // 折叠时上 pitch 目标角度（度，IMU Pitch 反馈）
#define GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD  (-1.0f)  // 折叠时下 pitch 目标位置（弧度，MIT 位置控制，收纳位）
#define GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD 0.0f    // 展开时下 pitch 目标位置（弧度，工作位）

// 到位判断容差
#define GIMBAL_POSITION_TOLERANCE_RAD        0.05f   // 电机位置到位容差（弧度），用于 yaw/lower_pitch 到位判断
#define GIMBAL_PITCH_TOLERANCE_DEG           1.0f    // 上 pitch 到位容差（度），基于 IMU Pitch 反馈

// 下 pitch 电机 MIT 控制参数（直接位置-速度-力矩前馈控制，不经过 PID 模块）
#define GIMBAL_LOWER_KP                      20.0f   // 下 pitch MIT 位置增益
#define GIMBAL_LOWER_KD                      1.0f    // 下 pitch MIT 速度阻尼增益

/* ===================== 模块实例与交互数据 ===================== */
static attitude_t *gimbal_IMU_data;           // 云台 IMU 姿态数据指针（INS_Init 返回，含 Yaw/Pitch/Roll/Gyro）
static DMMotorInstance *yaw_motor;             // yaw 轴电机（DM4340，CAN ID tx=0x01/rx=0x02）
static DMMotorInstance *upper_pitch_motor;     // 上 pitch 轴电机（DM4340，CAN ID tx=0x04/rx=0x03）
static DMMotorInstance *lower_pitch_motor;     // 下 pitch 轴电机（DM4340，CAN ID tx=0x05/rx=0x06，MIT 模式）

static Publisher_t *gimbal_pub;                // 云台反馈消息发布者（"gimbal_feed"）
static Subscriber_t *gimbal_sub;                // 云台控制消息订阅者（"gimbal_cmd"）
static Gimbal_Upload_Data_s gimbal_feedback_data;  // 上报给 CMD 应用的反馈数据（IMU、电机单圈角度、折叠状态）
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;          // 从 CMD 应用接收的控制指令（yaw/pitch 目标、模式、折叠请求）

/* ===================== 折叠状态机变量 ===================== */
static gimbal_fold_state_e fold_state = GIMBAL_FOLDING;  // 当前折叠状态，启动默认进入折叠流程
static fold_step_e fold_step = FOLD_STEP_LEVEL_UPPER_PITCH; // 折叠子步骤（仅 FOLDING 状态有效，4 步顺序执行）

/**
 * @brief 电机位置到位判断：比较实际位置与目标位置的差值是否在容差范围内。
 *        用于 yaw 电机和 lower_pitch 电机的到位检测（基于电机编码器位置，单位弧度）。
 * @param actual 实际位置（弧度）
 * @param target 目标位置（弧度）
 * @return true  已到位（差值 ≤ GIMBAL_POSITION_TOLERANCE_RAD）
 * @return false 未到位
 */
static bool PositionReached(float actual, float target)
{
    return fabsf(actual - target) <= GIMBAL_POSITION_TOLERANCE_RAD;
}

/**
 * @brief 上 pitch 轴到位判断：基于 IMU Pitch 角度反馈（非电机编码器），
 *        因为上 pitch 负载直接由 IMU 测量，电机编码器存在传动间隙。
 * @param target 目标 pitch 角度（度）
 * @return true  已到位（差值 ≤ GIMBAL_PITCH_TOLERANCE_DEG）
 * @return false 未到位
 */
static bool PitchReached(float target)
{
    return fabsf(gimbal_IMU_data->Pitch - target) <=
           GIMBAL_PITCH_TOLERANCE_DEG;
}

/**
 * @brief 以 MIT 模式设置下 pitch 电机的目标位置，并使能电机。
 *        MIT（Motor Initialization Technology）模式是达妙电机的直接位置-速度-力矩控制模式，
 *        不经过标准 PID 模块，直接下发位置/速度/Kp/Kd/前馈力矩，响应更快，适合折叠机构的定点控制。
 * @param position 目标位置（弧度）
 * @note  速度目标固定为 0，前馈力矩固定为 0，仅用位置环 + 阻尼控制。
 */
static void SetLowerPitchMIT(float position)
{
    DMMotorSetMITRef(lower_pitch_motor, position, 0.0f,
                     GIMBAL_LOWER_KP, GIMBAL_LOWER_KD, 0.0f);
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
    const bool fold_requested =
        gimbal_cmd_recv.request_mode == GIMBAL_REQUEST_FOLD;

    if (fold_requested &&
        (fold_state == GIMBAL_DEPLOYED || fold_state == GIMBAL_UNFOLDING))
    {
        // 从展开态/展开中请求折叠 → 进入折叠流程
        EnterFolding();
    }
    else if (!fold_requested &&
             (fold_state == GIMBAL_FOLDED || fold_state == GIMBAL_FOLDING))
    {
        // 从折叠态/折叠中请求展开 → 进入展开流程
        EnterUnfolding();
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
        DMMotorSetRef(upper_pitch_motor,
                      GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
        SetLowerPitchMIT(lower_pitch_motor->measure.position);
        if (PitchReached(GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG))
        {
            // 上 pitch 已水平，进入下一步：回正 yaw
            fold_step = FOLD_STEP_CENTER_YAW;
        }
        break;

    /* ---------- 步骤2：回正 yaw ---------- */
    case FOLD_STEP_CENTER_YAW:
        // 上 pitch 继续保持水平；yaw 使能并转到 0°；下 pitch 保持当前位置
        DMMotorSetRef(upper_pitch_motor,
                      GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
        DMMotorEnable(yaw_motor);
        DMMotorSetRef(yaw_motor, GIMBAL_FOLD_YAW_TARGET_DEG);
        SetLowerPitchMIT(lower_pitch_motor->measure.position);
        if (PositionReached(yaw_motor->measure.position,
                            0.0f))
        {
            // yaw 已回正，进入下一步：收下 pitch
            fold_step = FOLD_STEP_MOVE_LOWER_PITCH;
        }
        break;

    /* ---------- 步骤3：收下 pitch 到收纳位 ---------- */
    case FOLD_STEP_MOVE_LOWER_PITCH:
        // yaw 停止（已回正无需保持）；上 pitch 保持水平；下 pitch 转到收纳位 -1.0 rad
        DMMotorStop(yaw_motor);
        DMMotorSetRef(upper_pitch_motor,
                      GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
        SetLowerPitchMIT(GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD);
        if (PositionReached(lower_pitch_motor->measure.position,
                            GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD))
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
        DMMotorSetRef(upper_pitch_motor,
                      GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
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
    DMMotorSetRef(upper_pitch_motor,
                  GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
    SetLowerPitchMIT(GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD);

    if (PositionReached(lower_pitch_motor->measure.position,
                        GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD))
    {
        // 下 pitch 已到工作位，展开完成，进入正常工作模式
        fold_state = GIMBAL_DEPLOYED;
    }
}

/**
 * @brief 云台应用初始化：初始化 IMU、三个达妙电机（yaw / upper_pitch / lower_pitch）、
 *        消息中心发布/订阅者，并设置初始状态为折叠请求。
 *
 * 电机配置要点：
 *   - yaw 和 upper_pitch 使用"角度环+速度环"双闭环，角度/速度反馈均来自 IMU（OTHER_FEED），
 *     而非电机编码器，这样可以消除传动间隙对云台精度的影响。
 *   - lower_pitch 使用电机自身编码器反馈（MOTOR_FEED），并切换到 MIT 控制模式，
 *     适合折叠机构的大扭矩定点控制。
 *   - 三个电机均为 DM4340，挂在 hcan3 上，通过不同 CAN ID 区分。
 *
 * PID 参数说明（角度环 / 速度环）：
 *   - 角度环：Kp 较小（8~10），负责将 IMU 角度收敛到目标，输出作为速度环目标
 *   - 速度环：Kp/Ki 较大（50/200~350），负责快速跟踪角速度目标，输出电机力矩
 *   - 均启用梯形积分、积分限幅、微分先行（微分基于测量值而非误差，避免目标阶跃时微分冲击）
 */
void GimbalInit(void)
{
    // ---------- IMU 初始化 ----------
    gimbal_IMU_data = INS_Init();

    // ===================== yaw 轴电机配置 =====================
    // yaw 轴：水平旋转，角度反馈用 IMU YawTotalAngle（累计角度，不受 0/360 跳变影响），
    //         速度反馈用 IMU Gyro[2]（z 轴角速度）
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan3,
            .tx_id = 0x01,  // 主控发往 yaw 电机的 CAN ID
            .rx_id = 0x02,  // yaw 电机发回主控的 CAN ID
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
                .MaxOut = 500.0f,  // 角度环输出限幅（即速度环目标最大值）
            },
            .speed_PID = {
                .Kp = 50.0f,
                .Ki = 200.0f,
                .Kd = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .IntegralLimit = 3000.0f,
                .MaxOut = 20000.0f,  // 速度环输出限幅（即电机力矩最大值）
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->YawTotalAngle, // 外部角度反馈：IMU 累计 yaw
            .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[2],        // 外部速度反馈：IMU z 轴陀螺
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,  // 角度环用外部反馈（IMU）
            .speed_feedback_source = OTHER_FEED,  // 速度环用外部反馈（IMU 陀螺）
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,  // 双闭环：角度外环 + 速度内环
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = DM4340,
    };

    // ===================== 上 pitch 轴电机配置 =====================
    // 上 pitch 轴：直接负载枪管/相机，角度反馈用 IMU Pitch，速度反馈用 IMU Gyro[0]（x 轴角速度）
    Motor_Init_Config_s upper_pitch_config = {
        .can_init_config = {
            .can_handle = &hcan3,
            .tx_id = 0x04,
            .rx_id = 0x03,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 10.0f,  // 比 yaw 稍大，pitch 轴负载惯量较小
                .Ki = 0.0f,
                .Kd = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .IntegralLimit = 100.0f,
                .MaxOut = 500.0f,
            },
            .speed_PID = {
                .Kp = 50.0f,
                .Ki = 350.0f,  // 比 yaw 大，pitch 需要更快的速度响应抵抗重力
                .Kd = 0.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .IntegralLimit = 2500.0f,
                .MaxOut = 20000.0f,
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->Pitch,   // 外部角度反馈：IMU pitch
            .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[0], // 外部速度反馈：IMU x 轴陀螺
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

    // ===================== 下 pitch 轴电机配置 =====================
    // 下 pitch 轴：折叠升降机构，使用电机自身编码器反馈（MOTOR_FEED），
    //             初始化后通过 DMMotorSetControlMode 切换到 MIT 模式直接控制位置
    Motor_Init_Config_s lower_pitch_config = {
        .can_init_config = {
            .can_handle = &hcan3,
            .tx_id = 0x05,
            .rx_id = 0x06,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,  // 角度环用电机编码器
            .speed_feedback_source = MOTOR_FEED,  // 速度环用电机编码器
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = DM4340,
        // 注意：lower_pitch 不配置 PID 参数，因为使用 MIT 模式直接控制，
        // PID 参数在 SetLowerPitchMIT 中以 Kp/Kd 形式直接下发给电机。
    };

    // ---------- 电机实例化 ----------
    yaw_motor = DMMotorInit(&yaw_config);
    upper_pitch_motor = DMMotorInit(&upper_pitch_config);
    lower_pitch_motor = DMMotorInit(&lower_pitch_config);
    DMMotorSetControlMode(lower_pitch_motor, DMMOTOR_CONTROL_MIT); // 下 pitch 切换到 MIT 控制模式

    // ---------- 消息中心注册 ----------
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));

    // ---------- 初始状态：默认请求折叠，反馈状态标记为折叠中 ----------
    gimbal_cmd_recv.request_mode = GIMBAL_REQUEST_FOLD;
    gimbal_feedback_data.fold_state = GIMBAL_FOLDING;
}

/**
 * @brief 云台核心控制任务，按系统调度频率运行（通常 200Hz~1000Hz，由主循环决定）。
 *
 * 执行流程（每周期固定顺序）：
 *   1. 接收控制指令：从消息中心获取 CMD 应用下发的 gimbal_cmd_recv（目标角度、模式、折叠请求）
 *   2. 更新折叠状态：UpdateFoldState — 根据请求模式在 FOLDING/UNFOLDING 之间切换
 *   3. 电机控制分支：
 *      - GIMBAL_ZERO_FORCE → 所有电机停止输出（急停，最高优先级）
 *      - GIMBAL_FOLDING    → RunFolding（4 步折叠状态机）
 *      - GIMBAL_UNFOLDING  → RunUnfolding（下 pitch 升工作位）
 *      - GIMBAL_FOLDED     → 保持折叠姿态（yaw 停，上 pitch 水平，下 pitch 收纳位）
 *      - GIMBAL_DEPLOYED   → 正常工作（yaw/upper_pitch 跟踪外部目标角度，下 pitch 保持工作位）
 *   4. 上报反馈数据：填充 IMU 姿态、yaw 电机单圈角度、当前折叠状态，发布到 "gimbal_feed"
 *
 * @note  GIMBAL_ZERO_FORCE 分支在最前面判断，会覆盖所有折叠/工作状态，确保急停最高优先级。
 *        DEPLOYED 状态下 yaw/upper_pitch 的目标角度直接来自 gimbal_cmd_recv.yaw/pitch（由 CMD 应用的摇杆/鼠标累计生成），
 *        lower_pitch 始终保持工作位（0.0 rad），不随遥控器变化。
 */
void GimbalTask(void)
{
    // ---------- 1. 接收 CMD 应用下发的控制指令 ----------
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    // ---------- 2. 根据折叠请求更新状态机 ----------
    UpdateFoldState();

    // ---------- 3. 电机控制：按模式/状态分支 ----------
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
            DMMotorSetRef(upper_pitch_motor,
                          GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG);
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
    gimbal_feedback_data.gimbal_imu_data = *gimbal_IMU_data;  // 完整 IMU 姿态（Yaw/Pitch/Roll/Gyro/Accel）
    gimbal_feedback_data.yaw_motor_single_round_angle =
        (uint16_t)yaw_motor->measure.angle_single_round;  // yaw 电机单圈角度（0~360°，供 CMD 计算 offset_angle）
    gimbal_feedback_data.fold_state = fold_state;           // 当前折叠状态（供 CMD 应用做底盘联动）
    PubPushMessage(gimbal_pub, &gimbal_feedback_data);
}
