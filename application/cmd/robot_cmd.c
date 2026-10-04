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

// 私有宏,自动将编码器转换成角度值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 对齐时的角度,0-360
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
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

static Robot_Status_e robot_state; // 机器人整体工作状态

static chassis_mode_e normal_chassis_mode;       // 用户选择的正常底盘模式
static gimbal_request_mode_e gimbal_request_mode;

BMI088Instance *bmi088_test; // 云台IMU
BMI088_Data_t bmi088_data;
void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart5);   // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    vision_recv_data = VisionInit(&huart9); // 视觉通信串口

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
#endif // GIMBAL_BOARD
    gimbal_cmd_send.pitch = 0;
    gimbal_cmd_send.yaw = 0;
    gimbal_cmd_send.gimbal_mode = GIMBAL_IMU_MODE;  // 陀螺仪环模式（角度环用IMU反馈）
    gimbal_cmd_send.request_mode = GIMBAL_REQUEST_FOLD; // 启动默认请求折叠
    chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
    chassis_cmd_send.wz = 0;

    normal_chassis_mode = CHASSIS_FOLDED_ROTATE;
    gimbal_request_mode = GIMBAL_REQUEST_FOLD;
    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
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
    {
        // 左侧中档只更新请求状态。 最终是否进入 CHASSIS_FOLDED_ROTATE，交给仲裁函数根据 fold_state 决定。
        gimbal_request_mode = GIMBAL_REQUEST_FOLD;
        normal_chassis_mode = CHASSIS_NO_FOLLOW;

        // 折叠完成后，yaw 拨杆作为底盘 wz。
        chassis_cmd_send.wz = 8.0f * (float)rc_data[TEMP].rc.rocker_l_;
    }
    else if (switch_is_down(left_switch))
    {
        gimbal_request_mode = GIMBAL_REQUEST_DEPLOY;

        if (switch_is_down(right_switch))
        {
            normal_chassis_mode = CHASSIS_ROTATE;               // 小陀螺模式
        }
        else if (switch_is_mid(right_switch))
        {
            normal_chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;    // 部署模式的随动
        }
        else
        {
            normal_chassis_mode = CHASSIS_NO_FOLLOW;            // 自由模式
        }
        // 普通控制模式
        gimbal_cmd_send.yaw += 0.005f * (float)rc_data[TEMP].rc.rocker_l_;
        gimbal_cmd_send.pitch += 0.001f * (float)rc_data[TEMP].rc.rocker_l1;

        chassis_cmd_send.vx = 10.0f * (float)rc_data[TEMP].rc.rocker_r_;
        chassis_cmd_send.vy = 10.0f * (float)rc_data[TEMP].rc.rocker_r1;
    }

    gimbal_cmd_send.gimbal_mode = GIMBAL_IMU_MODE;

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

/**
 * @brief 输入为图传时模式和控制量设置
 *
 */

/**
 * @brief 云台请求是本地请求状态，在最终仲裁阶段写入发送结构体。
 */
static void ArbitrateControlOutput(void)
{

    gimbal_cmd_send.request_mode = gimbal_request_mode;

    switch (gimbal_fetch_data.fold_state)
    {
        case GIMBAL_FOLDED:
            chassis_cmd_send.chassis_mode = CHASSIS_FOLDED_ROTATE;
            chassis_cmd_send.wz = float_constrain(chassis_cmd_send.wz, -4500.0f, 4500.0f);
            break;
        case GIMBAL_FOLDING:
        case GIMBAL_UNFOLDING:
            chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
            chassis_cmd_send.wz = 0.0f;
            break;
        case GIMBAL_DEPLOYED:
            chassis_cmd_send.chassis_mode = normal_chassis_mode;
            chassis_cmd_send.wz = 0.0f;
            break;

        default:
            chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
            chassis_cmd_send.wz = 0.0f;
            break;
    }
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    if (rc_data[TEMP].rc.dial > 300 || robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
    {
        robot_state = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        LOGERROR("[CMD] emergency stop!");
    }
    // 遥控器右侧开关为[上],恢复正常运行
    if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        robot_state = ROBOT_READY;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        LOGINFO("[CMD] reinstate, robot ready");
    }
    GimbalPitchAngleLimit();
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
   // BMI088Acquire(bmi088_test,&bmi088_data) ;
    // 从其他应用获取回传数据
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    if (switch_is_down(rc_data[TEMP].rc.switch_left)  || switch_is_mid(rc_data[TEMP].rc.switch_left))
        RemoteControlSet();
    else if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[上],键盘控制


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
