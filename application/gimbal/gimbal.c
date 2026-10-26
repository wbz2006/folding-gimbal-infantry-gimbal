/**
 * @file gimbal.c
 * @brief 折叠云台应用：Yaw / 上Pitch / 下Pitch 三台达妙电机的闭环控制，
 *        以及折叠/展开过渡状态机。
 *
 * @details
 * 电机配置(均挂载 CAN3)：
 * - Yaw(DM4340)：角度/速度反馈取 IMU 的 YawTotalAngle / Gyro[Z]，对连续角闭环；
 * - 上 Pitch(DM4310)：反馈取 IMU 的 Pitch / Gyro[X]，即俯仰工作轴；
 * - 下 Pitch(DM4340)：折叠轴，使用电机自身反馈，由 DM 内部 MIT 位置环控制。
 *
 * 与 CMD 的交互：
 * - 订阅 "gimbal_cmd"(Gimbal_Ctrl_Cmd_s)：控制模式 + 折叠/展开请求；
 * - 发布 "gimbal_feed"(Gimbal_Upload_Data_s)：IMU 姿态、Yaw 单圈角、
 *   折叠状态、反馈有效性与故障标志。
 *
 * 折叠与展开共用一条"先调平上轴、再限速收纳/展开下轴"的过渡流程；
 * 命令新鲜度由 daemon 检查，反馈有效性由 IMU/电机驱动接口检查，
 * 任一异常都会进入统一停机覆盖并锁存故障，直到 CMD 下发零力矩确认后才恢复。
 */
#include "gimbal.h"
#include "robot_def.h"
#include "dmmotor.h"
#include "ins_task.h"
#include "message_center.h"
#include "daemon.h"
#include "user_lib.h"
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/* 折叠/展开过渡参数。位置沿用当前配置；速度、稳定时长、超时均为待实机确认的调试初值。 */
#define GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD    (-1.0f) // 下 Pitch 折叠收纳位，rad
#define GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD   0.0f   // 下 Pitch 展开(工作)位，rad
#define GIMBAL_UPPER_PITCH_TOLERANCE_DEG       1.0f   // 上 Pitch 视为"已调平"的角度误差门限，deg
#define GIMBAL_LOWER_PITCH_TOLERANCE_RAD       0.05f  // 下 Pitch 视为到位的位置误差门限，rad
#define GIMBAL_UPPER_STABLE_SPEED_RAD_S        0.05f  /* 上 Pitch 静止判据：IMU Gyro[X]，rad/s */
#define GIMBAL_LOWER_STABLE_SPEED_RAD_S        0.05f  /* 下 Pitch 静止判据：DM 反馈，rad/s */
#define GIMBAL_LOWER_REF_RATE_RAD_S            0.25f  /* 下 Pitch MIT 位置参考推进速率(限速)，rad/s */
#define GIMBAL_REF_MAX_DT_S                    0.02f  // 参考推进用 dt 上限，防止任务间隔异常时步进过大
#define GIMBAL_STABLE_TIME_MS                  100U   // 稳定保持时长：到位条件需连续满足该时长才转移状态
#define GIMBAL_ACTION_TIMEOUT_MS               15000U // 单次折叠/展开过渡的总超时
#define GIMBAL_LOWER_KP                        20.0f  // 下 Pitch MIT 位置刚度
#define GIMBAL_LOWER_KD                        1.0f   // 下 Pitch MIT 速度阻尼

/* "连续稳定"计时器：条件开始满足时置 tracking 并记录起始时刻，条件中断即复位。 */
typedef struct
{
    uint32_t since;   // 本次连续满足的起始时刻，ms
    bool tracking;    // 是否处于连续满足状态
} StableTimer;

/* 折叠/展开过渡过程的全部可变状态。 */
typedef struct
{
    gimbal_fold_state_e state;              // 形态状态机：FOLDED/FOLDING/UNFOLDING/DEPLOYED
    fold_step_e step;                       // 过渡内部步骤：调平上轴 -> 移动下轴 -> 完成
    bool active;                            // 过渡控制是否在运行(停机覆盖后置false，恢复时重新进入)
    float hold_yaw, lower_ref;              // 过渡期间保持的 Yaw 连续角 / 下 Pitch 位置参考
    uint32_t action_start, last_run;        // 本次过渡的起始时刻(超时计时) / 上次执行时刻(推参考用dt)
    StableTimer upper_stable, finish_stable; // 上轴调平稳定计时 / 整体到位稳定计时
} GimbalFoldCtrl_s;

/* 硬件句柄和通信对象保持独立；只有折叠/展开过程状态放入聚合结构。 */
static attitude_t *gimbal_IMU_data;
static DMMotorInstance *yaw_motor, *upper_pitch_motor, *lower_pitch_motor;
static Publisher_t *gimbal_pub;
static Subscriber_t *gimbal_sub;
static DaemonInstance *gimbal_cmd_daemon;
static Gimbal_Upload_Data_s gimbal_feedback_data;
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;

static GimbalFoldCtrl_s fold_ctrl = {
    .state = GIMBAL_FOLDING,
    .step = FOLD_STEP_LEVEL_UPPER_PITCH,
};

/**
 * @brief 判断指定目标是否"到位且静止"：位置误差与运动速度同时低于门限。
 *
 * @param target 目标类型。上 Pitch 调平位用 IMU 反馈判断；
 *               下 Pitch 折叠/展开位用电机自身反馈判断。
 * @return true 已到位
 */
static bool GimbalTargetReached(gimbal_reach_target_type_e target)
{
    if (target == UPPER_PITCH_FOLD) // 上 Pitch：IMU 角度进入水平目标容差且角速度足够小
        return fabsf(gimbal_IMU_data->Pitch - GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG) <=
                   GIMBAL_UPPER_PITCH_TOLERANCE_DEG &&
               fabsf(gimbal_IMU_data->Gyro[0]) <= GIMBAL_UPPER_STABLE_SPEED_RAD_S;
    float position = target == LOWER_PITCH_DEPLOY ? GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD : // 展开工作位
                                                  GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD;     // 折叠收纳位
    return fabsf(lower_pitch_motor->measure.position - position) <= GIMBAL_LOWER_PITCH_TOLERANCE_RAD &&
           fabsf(lower_pitch_motor->measure.velocity) <= GIMBAL_LOWER_STABLE_SPEED_RAD_S;
}

/**
 * @brief 根据CMD请求更新形态状态机，并在首次进入或方向切换时重置过渡上下文。
 *
 * 首次进入(停机恢复后 active==false)按当前请求决定方向；此后只有反向请求
 * 才切换状态，同向重复请求不重置目标与计时。切换入口锁存当前 Yaw 连续角
 * 和下 Pitch 实际位置作为保持目标，并清空 Yaw/上 Pitch 各 PID 环的运行时
 * 历史(保留参数)，避免旧误差/积分作用在新目标上造成输出冲击。
 */
static void UpdateFoldState(uint32_t now)
{
    gimbal_fold_state_e next_state = fold_ctrl.state;
    if (!fold_ctrl.active) // 停机恢复后的首次进入：直接按当前请求方向起步
        next_state = gimbal_cmd_recv.request_mode == GIMBAL_REQUEST_FOLD ? GIMBAL_FOLDING : GIMBAL_UNFOLDING;
    else if (gimbal_cmd_recv.request_mode == GIMBAL_REQUEST_FOLD &&
             (fold_ctrl.state == GIMBAL_UNFOLDING || fold_ctrl.state == GIMBAL_DEPLOYED))
        next_state = GIMBAL_FOLDING; // 反向：展开中/已展开 -> 折叠
    else if (gimbal_cmd_recv.request_mode == GIMBAL_REQUEST_DEPLOY &&
             (fold_ctrl.state == GIMBAL_FOLDING || fold_ctrl.state == GIMBAL_FOLDED))
        next_state = GIMBAL_UNFOLDING; // 反向：折叠中/已折叠 -> 展开

    if (next_state != fold_ctrl.state || !fold_ctrl.active) // 方向切换或首次进入：重置过渡上下文
    {
        fold_ctrl.state = next_state;
        fold_ctrl.step = FOLD_STEP_LEVEL_UPPER_PITCH;
        fold_ctrl.hold_yaw = gimbal_IMU_data->YawTotalAngle; // 连续角只在方向切换入口锁存，不回中
        fold_ctrl.lower_ref = lower_pitch_motor->measure.position; // 参考从实际位置出发，起步无阶跃
        fold_ctrl.upper_stable.tracking = fold_ctrl.finish_stable.tracking = false;
        fold_ctrl.action_start = fold_ctrl.last_run = now;
        DMMotorInstance *motors[] = {yaw_motor, upper_pitch_motor};
        for (size_t i = 0; i < sizeof(motors) / sizeof(motors[0]); ++i)
        {
            PIDInstance *pids[] = {&motors[i]->angle_PID, &motors[i]->speed_PID, &motors[i]->current_PID};
            for (size_t j = 0; j < sizeof(pids) / sizeof(pids[0]); ++j)
            {
                // 清空 Measure 起的全部运行时状态(测量/误差/积分/输出)，保留其前的 PID 参数
                memset((char *)pids[j] + offsetof(PIDInstance, Measure), 0,
                       sizeof(*pids[j]) - offsetof(PIDInstance, Measure));
                DWT_GetDeltaT(&pids[j]->DWT_CNT); // 重置计时基准，恢复后首周期不会用到过大的 dt
            }
        }
    }
    fold_ctrl.active = true; // 标记过渡控制已激活，此后仅反向请求会重置上下文
}

/**
 * @brief 执行一次折叠/展开过渡：上 Pitch 持续闭环调平，下 Pitch 限速推向目标。
 *
 * @param now 当前时刻，ms
 * @param lower_target 下 Pitch 目标位置，rad(折叠/展开各不同)
 * @param lower_position 下 Pitch 到位判定使用的目标类型
 * @param completed 过渡全部完成后进入的形态状态(FOLDED/DEPLOYED)
 *
 * 步骤流程：LEVEL_UPPER_PITCH(上轴调平并稳定) -> MOVE_LOWER_PITCH(限速推进
 * 下轴直到全部到位并稳定) -> FINISH。移动过程中若上 Pitch 偏出调平窗口，
 * 暂停推进并回到调平步骤重新调平，稳定后继续。
 */
/* 折叠和展开共用步骤；仅目标与完成状态不同。 */
static void RunTransition(uint32_t now, float lower_target, gimbal_reach_target_type_e lower_position,
                          gimbal_fold_state_e completed)
{
    DMMotorSetRef(yaw_motor, fold_ctrl.hold_yaw); // Yaw 闭环保持入口锁存的连续角，过渡中不响应新指令
    DMMotorSetRef(upper_pitch_motor, GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG); // 上 Pitch 始终闭环到水平目标
    const bool upper_reached = GimbalTargetReached(UPPER_PITCH_FOLD);
    // 下轴移动中若上 Pitch 偏出调平窗口(外扰/底盘运动耦合)：锁存当前位置，
    // 回到调平步骤，重新调平稳定后再继续推进。
    if (fold_ctrl.step == FOLD_STEP_MOVE_LOWER_PITCH &&
        fabsf(gimbal_IMU_data->Pitch) > GIMBAL_UPPER_PITCH_TOLERANCE_DEG)
    {
        fold_ctrl.lower_ref = lower_pitch_motor->measure.position; // 暂停时只锁存一次
        fold_ctrl.step = FOLD_STEP_LEVEL_UPPER_PITCH;
        fold_ctrl.upper_stable.tracking = fold_ctrl.finish_stable.tracking = false;
    }

    if (fold_ctrl.step == FOLD_STEP_LEVEL_UPPER_PITCH) // 步骤1：等待上 Pitch 调平并稳定
    {
        if (!upper_reached)
            fold_ctrl.upper_stable.tracking = false;
        else if (!fold_ctrl.upper_stable.tracking)
        {
            fold_ctrl.upper_stable.tracking = true;
            fold_ctrl.upper_stable.since = now;
        }
        if (fold_ctrl.upper_stable.tracking &&
            (uint32_t)(now - fold_ctrl.upper_stable.since) >= GIMBAL_STABLE_TIME_MS) // 调平稳定后进入下一步
            fold_ctrl.step = FOLD_STEP_MOVE_LOWER_PITCH;
    }
    else // 步骤2：限速推进下 Pitch，直到全部到位并稳定
    {
        // 按周期 dt 限速推进参考，dt 封顶防止任务间隔异常时单周期步进过大
        float dt = float_constrain((float)(uint32_t)(now - fold_ctrl.last_run) * 0.001f,
                                   0.0f, GIMBAL_REF_MAX_DT_S);
        fold_ctrl.lower_ref += float_constrain(lower_target - fold_ctrl.lower_ref,
            -GIMBAL_LOWER_REF_RATE_RAD_S * dt, GIMBAL_LOWER_REF_RATE_RAD_S * dt);
        // 完成条件：上轴保持调平 + 下轴实际到位 + 参考也已推到目标
        const bool reached = upper_reached && GimbalTargetReached(lower_position) &&
                             fabsf(fold_ctrl.lower_ref - lower_target) < 0.0001f;
        if (!reached)
            fold_ctrl.finish_stable.tracking = false;
        else if (!fold_ctrl.finish_stable.tracking)
        {
            fold_ctrl.finish_stable.tracking = true;
            fold_ctrl.finish_stable.since = now;
        }
        if (fold_ctrl.finish_stable.tracking &&
            (uint32_t)(now - fold_ctrl.finish_stable.since) >= GIMBAL_STABLE_TIME_MS) // 连续稳定后判定完成
        {
            fold_ctrl.step = FOLD_STEP_FINISH;
            fold_ctrl.state = completed;
            // 完成时把 Yaw 参考同步到当前实际朝向，交还控制权瞬间不产生跳变
            DMMotorSetRef(yaw_motor, gimbal_IMU_data->YawTotalAngle);
        }
    }
    // 下 Pitch 用 MIT 位置环跟踪限速参考(速度目标0+刚度/阻尼)，过渡期间持续使能
    DMMotorSetMITRef(lower_pitch_motor, fold_ctrl.lower_ref, 0.0f,
                     GIMBAL_LOWER_KP, GIMBAL_LOWER_KD, 0.0f);
    DMMotorEnable(lower_pitch_motor);
}

/**
 * @brief 初始化云台：IMU、三台达妙电机、消息收发与命令新鲜度守护。
 *
 * Yaw 与上 Pitch 的角度/速度反馈均取自 IMU(OTHER_FEED)，Yaw 闭环对象是上电
 * 清零的连续角 YawTotalAngle；下 Pitch 使用电机自身反馈，并切换为 MIT 位置
 * 模式。初始化末尾三轴全部 Stop，收到 CMD 有效命令后才会使能输出。
 */
void GimbalInit()
{   
    gimbal_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW 电机：对 IMU 连续角闭环，上电目标即为零，保持静止直到收到指令
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
    // 上 PITCH 电机(俯仰工作轴)：反馈取 IMU 的 Pitch/Gyro[X]，角度环以水平为零位
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
    // 下 PITCH 电机(折叠轴)：使用电机自身反馈，实际由下方 MIT 位置模式控制
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
        lower_pitch_motor->control_mode = DMMOTOR_CONTROL_MIT; // 下 Pitch 切换为 DM 内部 MIT 位置环
    DMMotorStop(yaw_motor);
    DMMotorStop(upper_pitch_motor);
    DMMotorStop(lower_pitch_motor);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    Daemon_Init_Config_s cmd_daemon_config = {
        .reload_count = DAEMON_COUNT_FROM_MS(APP_COMMAND_TIMEOUT_MS),
        .require_first_reload = 1,
    };
    gimbal_cmd_daemon = DaemonRegister(&cmd_daemon_config);

    gimbal_cmd_recv.request_mode = GIMBAL_REQUEST_FOLD; // 上电默认请求折叠，与 CMD 初始请求一致
    gimbal_feedback_data.fold_state = GIMBAL_FOLDING;   // 反馈初始为"折叠中"，首帧有效反馈前 CMD 不放开控制
}


void GimbalTask()
{
    const uint32_t now = HAL_GetTick();

    // 1. 接收命令，检查命令与反馈有效性。
    const bool command_received = SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    // 命令字段全部合法才喂狗：异常命令不允许刷新新鲜度
    if (command_received &&
        isfinite(gimbal_cmd_recv.yaw) && isfinite(gimbal_cmd_recv.pitch) &&
        (gimbal_cmd_recv.gimbal_mode == GIMBAL_IMU_MODE ||
         gimbal_cmd_recv.gimbal_mode == GIMBAL_ZERO_FORCE) &&
        (gimbal_cmd_recv.request_mode == GIMBAL_REQUEST_FOLD ||
         gimbal_cmd_recv.request_mode == GIMBAL_REQUEST_DEPLOY))
        DaemonReload(gimbal_cmd_daemon);
    // 本地反馈有效：IMU 解算与三台电机反馈均在线、新鲜且数值为有限值
    const bool feedback_valid = gimbal_IMU_data != NULL && INS_IsFeedbackValid() &&
        DMMotorIsFeedbackValid(yaw_motor) &&
        DMMotorIsFeedbackValid(upper_pitch_motor) &&
        DMMotorIsFeedbackValid(lower_pitch_motor) &&
        isfinite(gimbal_IMU_data->YawTotalAngle) && isfinite(gimbal_IMU_data->Pitch) &&
        isfinite(gimbal_IMU_data->Gyro[0]) && isfinite(gimbal_IMU_data->Gyro[2]);
    DaemonSetFeedbackValid(gimbal_cmd_daemon, feedback_valid);
    // 字段合法性(与喂狗条件一致)，对最后一次收到的命令逐周期持续检查
    const bool command_fields_valid = isfinite(gimbal_cmd_recv.yaw) && isfinite(gimbal_cmd_recv.pitch) &&
        (gimbal_cmd_recv.gimbal_mode == GIMBAL_IMU_MODE ||
         gimbal_cmd_recv.gimbal_mode == GIMBAL_ZERO_FORCE) &&
        (gimbal_cmd_recv.request_mode == GIMBAL_REQUEST_FOLD ||
         gimbal_cmd_recv.request_mode == GIMBAL_REQUEST_DEPLOY);
    const bool command_fresh = DaemonIsOnline(gimbal_cmd_daemon);
    const bool command_valid = command_fresh && command_fields_valid;

    // 2. 锁存应用故障；有效反馈下收到 CMD 停机确认后才清除。
    // CMD 仍锁存 STOP，只有故障清除、输入与反馈有效且右上档才恢复。
    const bool command_was_seen = DaemonHasReloaded(gimbal_cmd_daemon);
    if (command_was_seen && !feedback_valid)
        DaemonAddFaultFlags(gimbal_cmd_daemon, APP_FAULT_FEEDBACK_STALE);
    if (command_was_seen && !command_fresh)
        DaemonAddFaultFlags(gimbal_cmd_daemon, APP_FAULT_COMMAND_STALE);
    if ((command_was_seen || command_received) && !command_fields_valid)
        DaemonAddFaultFlags(gimbal_cmd_daemon, APP_FAULT_INVALID_COMMAND);
    if (command_valid && feedback_valid && gimbal_cmd_recv.gimbal_mode == GIMBAL_ZERO_FORCE)
        DaemonSetFaultFlags(gimbal_cmd_daemon, APP_FAULT_NONE);

    // 放行条件：反馈与命令均有效、无锁存故障、且 CMD 要求 IMU 闭环模式
    bool control_allowed = DaemonIsFeedbackValid(gimbal_cmd_daemon) && command_valid &&
                           DaemonGetFaultFlags(gimbal_cmd_daemon) == APP_FAULT_NONE &&
                           gimbal_cmd_recv.gimbal_mode == GIMBAL_IMU_MODE;

    // 3. 执行过渡动作，或保持已完成形态。
    if (control_allowed)
    {
        UpdateFoldState(now);
        const bool transitioning = fold_ctrl.state == GIMBAL_FOLDING ||
                                   fold_ctrl.state == GIMBAL_UNFOLDING; // 是否仍处于过渡过程
        DMMotorEnable(yaw_motor);
        DMMotorEnable(upper_pitch_motor);

        if (transitioning) // 过渡中：按方向选用目标，执行共用过渡流程
        {
            const bool folding = fold_ctrl.state == GIMBAL_FOLDING;
            RunTransition(now,
                folding ? GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD : GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD,
                folding ? LOWER_PITCH_FOLD : LOWER_PITCH_DEPLOY,
                folding ? GIMBAL_FOLDED : GIMBAL_DEPLOYED);
        }
        else // 完成态：Yaw/上Pitch 跟随 CMD 目标，下 Pitch 保持对应形态的位置
        {
            const bool folded = fold_ctrl.state == GIMBAL_FOLDED;
            DMMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);
            DMMotorSetRef(upper_pitch_motor, folded ?
                          GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG : gimbal_cmd_recv.pitch);
            DMMotorSetMITRef(lower_pitch_motor,
                folded ? GIMBAL_FOLD_LOWER_PITCH_TARGET_RAD : GIMBAL_DEPLOY_LOWER_PITCH_TARGET_RAD,
                0.0f, GIMBAL_LOWER_KP, GIMBAL_LOWER_KD, 0.0f);
            DMMotorEnable(lower_pitch_motor);
        }

        // 使用本周期动作前的过渡标志，确保超时优先于同周期到位。
        if (transitioning &&
            (uint32_t)(now - fold_ctrl.action_start) >= GIMBAL_ACTION_TIMEOUT_MS)
        {
            DaemonAddFaultFlags(gimbal_cmd_daemon, APP_FAULT_ACTION_TIMEOUT);
            control_allowed = false;
        }
        fold_ctrl.last_run = now;
    }

    // 4. 统一停机覆盖；此后不再放行电机或上报完成形态。
    if (!control_allowed)
    {
        DMMotorStop(yaw_motor);
        DMMotorStop(upper_pitch_motor);
        DMMotorStop(lower_pitch_motor);
        fold_ctrl.active = false; // 恢复后由 UpdateFoldState 重新进入并重置上下文
        // 状态复位为请求方向的过程态，避免上报与请求矛盾的"完成"形态
        fold_ctrl.state = gimbal_cmd_recv.request_mode == GIMBAL_REQUEST_DEPLOY ?
                          GIMBAL_UNFOLDING : GIMBAL_FOLDING;
    }

    // 5. 发布本周期反馈。
    if (gimbal_IMU_data != NULL)
        gimbal_feedback_data.gimbal_imu_data = *gimbal_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor != NULL ?
        yaw_motor->measure.angle_single_round : 0.0f;
    gimbal_feedback_data.fold_state = fold_ctrl.state;
    gimbal_feedback_data.feedback_valid = DaemonIsFeedbackValid(gimbal_cmd_daemon);
    gimbal_feedback_data.fault_flags = (uint8_t)DaemonGetFaultFlags(gimbal_cmd_daemon);
    PubPushMessage(gimbal_pub, &gimbal_feedback_data);
}
