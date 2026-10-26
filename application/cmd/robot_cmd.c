/**
 * @file robot_cmd.c
 * @brief 机器人控制中枢(CMD)应用：采集遥控输入、汇总各应用反馈，
 *        仲裁底盘/云台/发射的控制权限，并统一处理急停与恢复。
 *
 * @details
 * 云台板数据流：
 * - 输入：遥控器(DBUS 串口)、云台/发射反馈(消息中心)、底盘反馈(板间 CAN)；
 * - 输出："gimbal_cmd" / "shoot_cmd" 消息、底盘命令(板间 CAN)、视觉数据(串口)。
 *
 * 折叠云台仲裁要点：本地形态请求(gimbal_request_mode)与云台反馈的形态
 * 共同决定底盘/Yaw/Pitch/发射权限(反馈滞后一周期，故两侧同时检查)；
 * 所有停机条件集中在 StopRequired()，由 EmergencyHandler() 在普通控制
 * 之后统一覆盖输出，急停条件消失且右上档才恢复 READY。
 */
// app
#include "robot_def.h"
#include "robot_cmd.h"
// module
#include "remote_control.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "general_def.h"
#include "dji_motor.h"
#include "bmi088.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "user_lib.h"
#include "daemon.h"
#include <stdbool.h>
#include <math.h>
#include <string.h>

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 沿用 DJI 等效零位；DM 上电置零后尚无标定依据，待机械校准
#define PITCH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch水平时电机的角度,0-360

/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信
#endif
#ifdef ONE_BOARD
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data;  // 视觉发送数据

static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static DaemonInstance *gimbal_feedback_daemon, *shoot_feedback_daemon;
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

#ifdef GIMBAL_BOARD
static bool chassis_link_valid; // 底盘板 CAN 链路(含数据新鲜度检查)是否有效
#endif
#ifdef ONE_BOARD
static DaemonInstance *chassis_feedback_daemon;
#endif
static bool yaw_input_last;         // 上一周期是否允许 Yaw 输入，用于检测权限边沿
static bool yaw_target_initialized; // 是否已完成首次 Yaw 目标与反馈同步
static bool shape_complete_last;    // 上一周期形态是否为完成态，用于检测完成边沿
static Robot_Status_e robot_state; // 机器人整体工作状态

static chassis_mode_e normal_chassis_mode;       // 用户选择的正常底盘模式
static gimbal_request_mode_e gimbal_request_mode; // 云台形态请求，在最终仲裁阶段写入发送结构体

BMI088Instance *bmi088_test; // 云台 IMU 裸驱动句柄，仅供调试采集(任务中未启用)
BMI088_Data_t bmi088_data;   // BMI088 原始数据缓存

/**
 * @brief CMD 应用初始化：遥控器/视觉串口、消息收发、反馈守护与板间通信。
 *
 * 初始形态请求为折叠、底盘不跟随、机器人状态 STOP；待输入与各应用反馈
 * 就绪后，由右拨杆上档恢复 READY(见 EmergencyHandler)。
 */
void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart5);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    vision_recv_data = VisionInit(&huart9); // 视觉通信串口

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    Daemon_Init_Config_s feedback_daemon_config = {
        .reload_count = DAEMON_COUNT_FROM_MS(APP_FEEDBACK_TIMEOUT_MS),
        .require_first_reload = 1,
    };
    gimbal_feedback_daemon = DaemonRegister(&feedback_daemon_config);
    shoot_feedback_daemon = DaemonRegister(&feedback_daemon_config);

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    chassis_feedback_daemon = DaemonRegister(&feedback_daemon_config);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    // 板间通信：云台发 0x312 / 收 0x311；收发长度与两侧共用结构体严格一致
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .daemon_count = DAEMON_COUNT_FROM_MS(APP_FEEDBACK_TIMEOUT_MS),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    gimbal_cmd_send.pitch = 0;
    gimbal_cmd_send.yaw = 0;
    gimbal_cmd_send.gimbal_mode = GIMBAL_IMU_MODE;  // 陀螺仪环模式（角度环用IMU反馈）
    gimbal_cmd_send.request_mode = GIMBAL_REQUEST_FOLD; // 启动默认请求折叠
    chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
    chassis_cmd_send.wz = 0;

    normal_chassis_mode = CHASSIS_NO_FOLLOW;
    gimbal_request_mode = GIMBAL_REQUEST_FOLD;
    robot_state = ROBOT_STOP; // 首次有效反馈和遥控输入就绪后由右上档恢复
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    static float angle;
    angle = gimbal_fetch_data.yaw_motor_single_round_angle; // 从云台获取的当前yaw电机单圈角度
    // 取云台相对底盘的最近角度，避免跨越0/360度时得到接近360度的误差
    chassis_cmd_send.offset_angle = theta_format(angle - YAW_ALIGN_ANGLE);
}

/**
 * @brief 对云台角度进行软件限位，防止超出机械边界(硬截断)
 *        目前仅对 pitch 做限幅，yaw 可无限旋转
 */
static void GimbalPitchAngleLimit()
{
    gimbal_cmd_send.pitch = float_constrain(gimbal_cmd_send.pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置。
 *
 */
static void RemoteControlSet(void)
{
    const uint8_t left_switch = rc_data[TEMP].rc.switch_left;
    const uint8_t right_switch = rc_data[TEMP].rc.switch_right;
    if (switch_is_mid(left_switch))
        gimbal_request_mode = GIMBAL_REQUEST_FOLD;
    else if (switch_is_down(left_switch))
        gimbal_request_mode = GIMBAL_REQUEST_DEPLOY;
    // 左上档保留最后的形态请求，不产生运动增量，也不新增键鼠控制。
    if (switch_is_up(right_switch))
        normal_chassis_mode = CHASSIS_NO_FOLLOW;
    else if (switch_is_mid(right_switch))
        normal_chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    else if (switch_is_down(right_switch))
        normal_chassis_mode = CHASSIS_ROTATE;

    // 发射参数
    // 摩擦轮控制,拨轮向上打为负,向下为正
    if (rc_data[TEMP].rc.dial < -100) // 向上超过100,打开摩擦轮
        shoot_cmd_send.friction_mode = FRICTION_ON;
    else
        shoot_cmd_send.friction_mode = FRICTION_OFF;
    // 拨弹控制,遥控器固定为一种拨弹模式,可自行选择
    if (rc_data[TEMP].rc.dial < -500)
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    else
        shoot_cmd_send.load_mode = LOAD_STOP;
    // 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
    shoot_cmd_send.shoot_rate = 8;
}

// TODO: 输入为图传(键鼠)时模式和控制量设置，预留；图传链路暂未接入，当前仅支持遥控器输入

/**
 * @brief 拨杆值是否为三个有效档位(上/中/下)之一，用于过滤遥控器异常值。
 */
static bool SwitchValid(uint8_t value)
{
    return switch_is_up(value) || switch_is_mid(value) || switch_is_down(value);
}

/**
 * @brief 遥控器输入是否有效：在线且左右拨杆均为合法档位。
 */
static bool InputValid(void)
{
    return RemoteControlIsOnline() && SwitchValid(rc_data[TEMP].rc.switch_left) &&
           SwitchValid(rc_data[TEMP].rc.switch_right);
}

/**
 * @brief 云台反馈是否就绪：消息新鲜、有效性标志置位、角度为有限值且形态枚举在合法范围。
 */
static bool FeedbackReady(void)
{
    return DaemonIsOnline(gimbal_feedback_daemon) && gimbal_fetch_data.feedback_valid &&
           isfinite(gimbal_fetch_data.gimbal_imu_data.YawTotalAngle) &&
           isfinite(gimbal_fetch_data.yaw_motor_single_round_angle) &&
           gimbal_fetch_data.fold_state >= GIMBAL_FOLDED && gimbal_fetch_data.fold_state <= GIMBAL_DEPLOYED;
}

/**
 * @brief 底盘反馈是否就绪。单板检查本地消息守护；双板检查板间 CAN 链路状态。
 */
static bool ChassisFeedbackReady(void)
{
#ifdef ONE_BOARD
    return DaemonIsOnline(chassis_feedback_daemon);
#else
    return chassis_link_valid;
#endif
}

/**
 * @brief 发射反馈是否就绪(本地消息守护在线)。
 */
static bool ShootFeedbackReady(void)
{
    return DaemonIsOnline(shoot_feedback_daemon);
}

/**
 * @brief 是否必须停机：任一输入/反馈链路失效、急停拨轮(>300)，
 *        或底盘/云台/发射任一应用上报故障标志。
 */
static bool StopRequired(void)
{
    return !InputValid() || !FeedbackReady() || !ChassisFeedbackReady() || !ShootFeedbackReady() ||
           rc_data[TEMP].rc.dial > 300 || gimbal_fetch_data.fault_flags != APP_FAULT_NONE ||
           chassis_fetch_data.fault_flags != APP_FAULT_NONE ||
           shoot_fetch_data.fault_flags != APP_FAULT_NONE;
}

/**
 * @brief 控制权限仲裁：综合遥控输入、就绪状态与折叠形态，生成本周期的
 *        云台/底盘/发射指令；未放行的控制量保持安全值。
 *
 * 形态判定同时使用本地请求与云台反馈，抵消 CMD 与 Gimbal 之间一周期的
 * 反馈滞后。底盘在"请求折叠(含折叠中)"或部署完成时放行；Yaw 仅在完成态
 * (折叠或部署)接受增量；Pitch 与发射仅在部署完成时放行。
 */
static void ArbitrateControlOutput(void)
{
    // 左拨杆处于中(折叠)/下(展开)档，即用户正在发出运动指令
    const bool remote_motion = InputValid() &&
        (switch_is_mid(rc_data[TEMP].rc.switch_left) || switch_is_down(rc_data[TEMP].rc.switch_left));
    const bool enabled = robot_state == ROBOT_READY && !StopRequired();
    // 请求与反馈共同仲裁，处理 CMD 先执行造成的一周期形态反馈滞后。
    const bool deployed = gimbal_request_mode == GIMBAL_REQUEST_DEPLOY &&
                          gimbal_fetch_data.fold_state == GIMBAL_DEPLOYED;
    const bool folded = gimbal_request_mode == GIMBAL_REQUEST_FOLD &&
                        gimbal_fetch_data.fold_state == GIMBAL_FOLDED;
    const bool chassis_allowed = remote_motion && enabled && // 折叠请求/折叠中/折叠完成/部署完成均可平移
        (gimbal_request_mode == GIMBAL_REQUEST_FOLD || deployed);
    const bool yaw_allowed = remote_motion && enabled && (folded || deployed); // 仅完成态接受 Yaw 增量
    const bool pitch_allowed = remote_motion && enabled && deployed;           // 仅部署完成接受 Pitch 增量
    gimbal_cmd_send.request_mode = gimbal_request_mode; // 本地形态请求直接下发
    gimbal_cmd_send.gimbal_mode = GIMBAL_IMU_MODE;
    chassis_cmd_send.chassis_mode = chassis_allowed ? normal_chassis_mode : CHASSIS_NO_FOLLOW; // 未放行退回自由模式(速度为零)
    if (chassis_allowed)
    {
        chassis_cmd_send.vx = 10.0f * (float)rc_data[TEMP].rc.rocker_r_;
        chassis_cmd_send.vy = 10.0f * (float)rc_data[TEMP].rc.rocker_r1;
        // wz 由底盘既有自由/随动/小陀螺策略实时生成，左摇杆不直接控制底盘旋转。
    }
    // Yaw 目标重同步(满足任一)：首次运行 / Yaw 权限边沿 / 系统禁用 / 形态完成边沿。
    // 重同步后目标等于当前实际朝向，重新放开控制的瞬间不产生角度跳变。
    if (FeedbackReady() && (!yaw_target_initialized || yaw_allowed != yaw_input_last || !enabled ||
                           ((folded || deployed) && !shape_complete_last)))
    {
        gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        yaw_target_initialized = true;
    }
    if (yaw_allowed) // 左摇杆水平逐周期叠加增量；目标为连续角，不做取模
        gimbal_cmd_send.yaw += 0.005f * (float)rc_data[TEMP].rc.rocker_l_;
    yaw_input_last = yaw_allowed;
    shape_complete_last = FeedbackReady() && (folded || deployed);
    if (pitch_allowed) // 左摇杆竖直叠加俯仰增量(仅部署态)
        gimbal_cmd_send.pitch += 0.001f * (float)rc_data[TEMP].rc.rocker_l1;
    else
        gimbal_cmd_send.pitch = GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG; // 非部署态强制水平目标
    GimbalPitchAngleLimit(); // 仅部署目标的机械限幅，与强制调平零位分离
    if (!pitch_allowed) // 强制调平目标不受部署限幅配置影响，再次覆盖
        gimbal_cmd_send.pitch = GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG;
    // 沿用拨轮映射；保持折叠及过渡期间禁用发射，不新增折叠态射击。
    if (!deployed || !remote_motion || !enabled)
    {
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
    else
        shoot_cmd_send.shoot_mode = SHOOT_ON;
}

/**
 * @brief 急停与恢复处理。必须在普通控制(ArbitrateControlOutput)之后执行；
 *        急停条件与右上档同周期冲突时停机优先。
 *
 * STOP 态下所有应用切零力矩/停止并清速度，Yaw 输入历史复位以保证恢复后
 * 重新同步目标；急停条件全部消失且右拨杆打到上档才回到 READY。
 */
static void EmergencyHandler(void)
{
    if (StopRequired()) // 急停条件仍存在，持续锁定 STOP
        robot_state = ROBOT_STOP;
    else if (robot_state == ROBOT_STOP && switch_is_up(rc_data[TEMP].rc.switch_right)) // 右上档恢复
        robot_state = ROBOT_READY;
    if (robot_state == ROBOT_STOP)
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        chassis_cmd_send.vx = chassis_cmd_send.vy = chassis_cmd_send.wz = 0.0f;
        gimbal_cmd_send.pitch = GIMBAL_FOLD_UPPER_PITCH_TARGET_DEG;
        yaw_input_last = false;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
}

/**
 * @brief 机器人核心控制任务，由 RobotTask() 以 200Hz 调用(必须高于视觉发送频率)。
 *
 * 每周期流程：拉取各应用反馈(底盘走板间 CAN) -> 遥控输入解析 -> 权限仲裁
 * -> 急停覆盖 -> 发布云台/发射命令与底盘命令(消息或板间 CAN)、发送视觉数据。
 */
void RobotCMDTask()
{
   // BMI088Acquire(bmi088_test,&bmi088_data) ;
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    if (SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data))
        DaemonReload(chassis_feedback_daemon);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    // 返回值同时表示链路与数据有效性，失联时清零反馈，避免继续使用陈旧数据
    chassis_link_valid = CANCommReceive(cmd_can_comm, &chassis_fetch_data);
    if (!chassis_link_valid)
    {
        memset(&chassis_fetch_data, 0, sizeof(chassis_fetch_data));
    }
#endif // GIMBAL_BOARD
    if (SubGetMessage(shoot_feed_sub, &shoot_fetch_data))
        DaemonReload(shoot_feedback_daemon);
    if (SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data))
        DaemonReload(gimbal_feedback_daemon);
    // 瞬时速度每周期从零开始生成，形态边界不覆盖当周期正常折叠输入。
    chassis_cmd_send.vx = chassis_cmd_send.vy = chassis_cmd_send.wz = 0.0f;

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    if (InputValid())
        RemoteControlSet();

    ArbitrateControlOutput();
    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionSetFlag(chassis_fetch_data.enemy_color,,chassis_fetch_data.bullet_speed)

    // 推送消息,双板通信,视觉通信等
    // 其他应用所需的控制数据在remotecontrolsetmode和mousekeysetmode中完成设置
#ifdef ONE_BOARD
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
#endif // GIMBAL_BOARD
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    VisionSend(&vision_send_data);
}
