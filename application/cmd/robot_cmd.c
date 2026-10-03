/**
 * @file robot_cmd.c
 * @brief 机器人指令应用层（CMD）：接收遥控器/键鼠输入，解析为底盘、云台、发射三大子系统的控制指令，
 *        并通过消息中心（单板）或 CAN 通信（双板）下发。同时处理折叠/展开模式联动与紧急停止。
 *
 * 运行频率：200Hz（RobotCMDTask），必须高于视觉发送频率以保证控制延迟可控。
 *
 * 控制模式切换逻辑（遥控器左侧开关）：
 *   - 左开关 [下] / [中]  → 遥控器控制（RemoteControlSet）
 *   - 左开关 [上]          → 键鼠控制（MouseKeySet）
 *
 * 折叠联动：左开关 [中] 强制进入折叠模式；云台折叠状态（gimbal_fetch_data.fold_state）
 *           会反向覆盖底盘模式，确保折叠过程中底盘不跟随云台。
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
#include "user_lib.h"
#include "dji_motor.h"
#include "bmi088.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

// 私有宏,自动将编码器转换成角度值
// YAW_CHASSIS_ALIGN_ECD / PITCH_HORIZON_ECD 在 robot_def.h 中定义，为机械零位对应的编码器原始值
#define YAW_ALIGN_ANGLE (YAW_CHASSIS_ALIGN_ECD * ECD_ANGLE_COEF_DJI) // 云台与底盘正方向对齐时的yaw电机角度,0-360°
#define PTICH_HORIZON_ANGLE (PITCH_HORIZON_ECD * ECD_ANGLE_COEF_DJI) // pitch轴水平时电机的角度,0-360°（目前未在代码中使用，保留供后续软限位参考）

/* ===================== 双板通信兼容（条件编译） ===================== */
/* cmd应用包含的模块实例指针和交互信息存储*/
#ifdef GIMBAL_BOARD // 对双板的兼容,条件编译：云台板通过CAN与底盘板通信
#include "can_comm.h"
static CANCommInstance *cmd_can_comm; // 双板通信实例，替代单板的消息中心pub/sub
#endif
#ifdef ONE_BOARD // 单板：所有应用在同一MCU上，通过消息中心（发布/订阅）交互
static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
#endif                                 // ONE_BOARD

/* ===================== 底盘子系统交互数据 ===================== */
static Chassis_Ctrl_Cmd_s chassis_cmd_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

/* ===================== 输入设备与视觉 ===================== */
static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回（含遥控器+键鼠合并数据，TEMP索引取最新一帧）
static Vision_Recv_s *vision_recv_data; // 视觉接收数据指针,初始化时返回
static Vision_Send_s vision_send_data;  // 视觉发送数据（自瞄目标信息等，目前未填充，待完善）

/* ===================== 云台子系统交互数据 ===================== */
static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;          // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;      // 传递给云台的控制信息（yaw/pitch目标角度、模式、折叠请求）
static Gimbal_Upload_Data_s gimbal_fetch_data; // 从云台获取的反馈信息（IMU、电机单圈角度、折叠状态）

/* ===================== 发射子系统交互数据 ===================== */
static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;         // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;      // 传递给发射的控制信息（摩擦轮、拨弹、弹舱、弹速、射频）
static Shoot_Upload_Data_s shoot_fetch_data; // 从发射获取的反馈信息

/* ===================== 整机状态与模式记忆 ===================== */
static Robot_Status_e robot_state; // 机器人整体工作状态（READY / STOP），急停时置STOP

// normal_* 记录用户通过遥控器/键鼠选择的"正常工作模式"，折叠过程中会被临时覆盖，
// 展开完成后由 ApplyFoldChassisMode 恢复为这些值。
static chassis_mode_e normal_chassis_mode;       // 用户选择的正常底盘模式
static gimbal_request_mode_e normal_gimbal_request; // 用户选择的正常云台请求模式

BMI088Instance *bmi088_test; // 云台IMU（目前仅声明，未在任务中使用，预留调试）
BMI088_Data_t bmi088_data;
/**
 * @brief CMD应用初始化：注册输入设备（遥控器/视觉）、消息中心发布/订阅者、双板CAN通信，
 *        并设置各子系统控制指令的初始默认值。
 * @note  启动时默认进入折叠状态（gimbal_mode=GYRO, request_mode=FOLD, chassis_mode=NO_FOLLOW），
 *        后续由遥控器左开关切换到展开/正常模式。
 */
void RobotCMDInit()
{
    // ---------- 输入设备初始化 ----------
    rc_data = RemoteControlInit(&huart5);   // 遥控器DBus串口，注意自研板需选用接了反相器的串口
    vision_recv_data = VisionInit(&huart9); // 视觉通信串口（与上位机/NUC通信）

    // ---------- 消息中心：云台 & 发射（单板/双板均在云台板上，直接注册） ----------
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

    // ---------- 消息中心：底盘（仅单板） / CAN通信（仅双板云台板） ----------
#ifdef ONE_BOARD // 双板兼容
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    // 双板方案：云台板作为CAN主节点，tx_id=0x312发往底盘板，rx_id=0x311接收底盘反馈
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

    // ---------- 控制指令初始默认值 ----------
    gimbal_cmd_send.pitch = 0;
    gimbal_cmd_send.yaw = 0;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;  // 陀螺仪环模式（角度环用IMU反馈）
    gimbal_cmd_send.request_mode = GIMBAL_REQUEST_FOLD; // 启动默认请求折叠
    chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
    chassis_cmd_send.wz = 0;
    // normal_* 记录用户选择的正常模式，初始与折叠模式一致
    normal_chassis_mode = CHASSIS_FOLDED_ROTATE;
    normal_gimbal_request = GIMBAL_REQUEST_FOLD;

    robot_state = ROBOT_READY; // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 根据gimbal app传回的当前yaw电机单圈角度，计算云台与底盘正方向的夹角（offset_angle）。
 *        该值用于底盘"跟随云台"模式下，让底盘知道云台相对自己转了多少度。
 *        单圈绝对角度范围 0~360°，theta_format 将差值归一化到 [-180, 180]，
 *        避免跨越 0/360° 边界时得到接近 360° 的错误大误差。
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
 * @brief 遥控器控制模式下的模式选择与控制量设置。
 *
 * 开关逻辑（左侧开关决定是否折叠，右侧开关在展开时选择三种底盘跟随模式）：
 *   左开关 [中]            → 强制折叠模式（CHASSIS_FOLDED_ROTATE + GIMBAL_REQUEST_FOLD），忽略右开关
 *   左开关 [下] + 右开关 [下] → 小陀螺模式（CHASSIS_ROTATE + GIMBAL_REQUEST_GYRO）
 *   左开关 [下] + 右开关 [中] → 云台跟随模式（CHASSIS_FOLLOW_GIMBAL_YAW + GIMBAL_REQUEST_FOLLOW）
 *   左开关 [下] + 右开关 [上] → 不跟随模式（CHASSIS_NO_FOLLOW + GIMBAL_REQUEST_FREE）
 *
 * 摇杆映射：
 *   左摇杆水平(rocker_l_)  → yaw 角度增量（仅左开关[下]时有效，折叠时该摇杆被 ApplyFoldChassisMode 挪用为底盘wz）
 *   左摇杆竖直(rocker_l1)  → pitch 角度增量
 *   右摇杆水平(rocker_r_)  → 底盘 vx（前后）
 *   右摇杆竖直(rocker_r1)  → 底盘 vy（左右）
 *   拨轮(dial)             → 上拨打开摩擦轮，上拨超过500触发连发
 */
static void RemoteControlSet()
{
    const uint8_t left_switch = rc_data[TEMP].rc.switch_left;
    const uint8_t right_switch = rc_data[TEMP].rc.switch_right;

    // ---------- 模式选择：左开关优先判断折叠 ----------
    if (switch_is_mid(left_switch))
    {
        /* Left switch middle: force folded mode; right switch is ignored. */
        normal_chassis_mode = CHASSIS_FOLDED_ROTATE;
        normal_gimbal_request = GIMBAL_REQUEST_FOLD;
    }
    else
    {
        /* Left switch down: deploy and use the three right-switch modes. */
        if (switch_is_down(right_switch))
        {
            // 右开关下：小陀螺，底盘自转，云台保持陀螺仪环
            normal_chassis_mode = CHASSIS_ROTATE;
            normal_gimbal_request = GIMBAL_REQUEST_GYRO;
        }
        else if (switch_is_mid(right_switch))
        {
            // 右开关中：底盘跟随云台yaw，云台进入跟随模式
            normal_chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
            normal_gimbal_request = GIMBAL_REQUEST_FOLLOW;
        }
        else
        {
            // 右开关上：底盘不跟随，云台自由模式
            normal_chassis_mode = CHASSIS_NO_FOLLOW;
            normal_gimbal_request = GIMBAL_REQUEST_FREE;
        }
    }

    // 将用户选择的正常模式写入发送结构体（折叠状态下会被 ApplyFoldChassisMode 覆盖）
    chassis_cmd_send.chassis_mode = normal_chassis_mode;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    gimbal_cmd_send.request_mode = normal_gimbal_request;

    // ---------- 云台角度控制：仅左开关[下]（展开状态）时摇杆才控制云台角度 ----------
    // 云台参数,确定云台控制数据
    if (switch_is_down(left_switch))
    { // 按照摇杆的输出大小进行角度增量,增益系数需调整
        gimbal_cmd_send.yaw += 0.005f * (float)rc_data[TEMP].rc.rocker_l_;
        gimbal_cmd_send.pitch += 0.001f * (float)rc_data[TEMP].rc.rocker_l1;
    }
    // 云台软件限位（待添加：限制 yaw/pitch 的累计角度范围，防止机械超限）

    // ---------- 底盘速度控制 ----------
    // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
    chassis_cmd_send.vx = 10.0f * (float)rc_data[TEMP].rc.rocker_r_; // _水平方向（前后）
    chassis_cmd_send.vy = 10.0f * (float)rc_data[TEMP].rc.rocker_r1; // 1数值方向（左右）

    // ---------- 发射控制 ----------
    // 弹舱开关：右开关[上]打开弹舱，否则关闭（目前舵机模块待添加，空实现）
    if (switch_is_up(rc_data[TEMP].rc.switch_right)) // 右侧开关状态[上],弹舱打开
        ;                                            // 弹舱舵机控制,待添加servo_motor模块,开启
    else
        ; // 弹舱舵机控制,待添加servo_motor模块,关闭

    // 摩擦轮控制,拨轮向上打为负,向下为正
    if (rc_data[TEMP].rc.dial < -100) // 向上超过100,打开摩擦轮
        shoot_cmd_send.friction_mode = FRICTION_ON;
    else
        shoot_cmd_send.friction_mode = FRICTION_OFF;
    // 拨弹控制,遥控器固定为一种拨弹模式,可自行选择
    if (rc_data[TEMP].rc.dial < -500)
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE; // 拨轮上拨超过500：连发
    else
        shoot_cmd_send.load_mode = LOAD_STOP;       // 否则停止拨弹
    // 射频控制,固定每秒1发,后续可以根据左侧拨轮的值大小切换射频,
    shoot_cmd_send.shoot_rate = 8;
}

/**
 * @brief 键鼠控制模式下的模式选择与控制量设置（左开关[上]时进入）。
 *
 * 键鼠模式下固定为底盘不跟随 + 云台自由模式，所有控制通过鼠标/键盘映射：
 *   鼠标 x 轴 → yaw 角度增量
 *   鼠标 y 轴 → pitch 角度增量
 *   W/S      → 底盘前后（vx）
 *   A/D      → 底盘左右（vy）（注：代码中 vy 用 s-d 计算，方向需实测确认）
 *
 * 多功能按键（按奇数次切换，用 key_count % N 实现循环档位）：
 *   Z 键 → 弹速三档循环：15 / 18 / 30 m/s
 *   E 键 → 发射模式四档循环：停止 / 单发 / 三连发 / 连发
 *   R 键 → 弹舱开/关
 *   F 键 → 摩擦轮开/关
 *   C 键 → 底盘速度增益四档循环：40 / 60 / 80 / 100
 *   Shift → 超功率模式（待实现：消耗缓冲能量突破功率限制）
 */
static void MouseKeySet()
{
    // 键鼠模式固定使用不跟随 + 自由模式
    normal_chassis_mode = CHASSIS_NO_FOLLOW;
    normal_gimbal_request = GIMBAL_REQUEST_FREE;
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;
    gimbal_cmd_send.request_mode = GIMBAL_REQUEST_FREE;
    chassis_cmd_send.chassis_mode = normal_chassis_mode;

    // ---------- 底盘速度：WASD 映射，系数待测 ----------
    chassis_cmd_send.vx = rc_data[TEMP].key[KEY_PRESS].w * 300 - rc_data[TEMP].key[KEY_PRESS].s * 300; // 系数待测
    chassis_cmd_send.vy = rc_data[TEMP].key[KEY_PRESS].s * 300 - rc_data[TEMP].key[KEY_PRESS].d * 300;

    // ---------- 云台角度：鼠标移动映射为角度增量，系数待测 ----------
    gimbal_cmd_send.yaw += (float)rc_data[TEMP].mouse.x / 660 * 10; // 系数待测
    gimbal_cmd_send.pitch += (float)rc_data[TEMP].mouse.y / 660 * 10;

    // ---------- Z键：弹速三档循环 ----------
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_Z] % 3) // Z键设置弹速
    {
    case 0:
        shoot_cmd_send.bullet_speed = 15;
        break;
    case 1:
        shoot_cmd_send.bullet_speed = 18;
        break;
    default:
        shoot_cmd_send.bullet_speed = 30;
        break;
    }
    // ---------- E键：发射模式四档循环 ----------
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_E] % 4) // E键设置发射模式
    {
    case 0:
        shoot_cmd_send.load_mode = LOAD_STOP;
        break;
    case 1:
        shoot_cmd_send.load_mode = LOAD_1_BULLET;
        break;
    case 2:
        shoot_cmd_send.load_mode = LOAD_3_BULLET;
        break;
    default:
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        break;
    }
    // ---------- R键：弹舱开/关 ----------
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2) // R键开关弹舱
    {
    case 0:
        shoot_cmd_send.lid_mode = LID_OPEN;
        break;
    default:
        shoot_cmd_send.lid_mode = LID_CLOSE;
        break;
    }
    // ---------- F键：摩擦轮开/关 ----------
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2) // F键开关摩擦轮
    {
    case 0:
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        break;
    default:
        shoot_cmd_send.friction_mode = FRICTION_ON;
        break;
    }
    // ---------- C键：底盘速度增益四档循环 ----------
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 4) // C键设置底盘速度
    {
    case 0:
        chassis_cmd_send.chassis_speed_buff = 40;
        break;
    case 1:
        chassis_cmd_send.chassis_speed_buff = 60;
        break;
    case 2:
        chassis_cmd_send.chassis_speed_buff = 80;
        break;
    default:
        chassis_cmd_send.chassis_speed_buff = 100;
        break;
    }
    // ---------- Shift键：超功率模式（待实现） ----------
    switch (rc_data[TEMP].key[KEY_PRESS].shift) // 待添加 按shift允许超功率 消耗缓冲能量
    {
    case 1:

        break;

    default:

        break;
    }
}

/**
 * @brief 根据云台当前折叠状态（gimbal_fetch_data.fold_state）覆盖底盘模式，
 *        确保折叠/展开过程中底盘行为与云台机械状态安全联动。
 *
 * 状态映射：
 *   GIMBAL_FOLDED    → 底盘进入折叠旋转模式（CHASSIS_FOLDED_ROTATE），
 *                       左摇杆水平(rocker_l_)直接作为底盘 wz（自转速度），限幅 ±4500
 *   GIMBAL_FOLDING   → 折叠进行中，底盘冻结（CHASSIS_NO_FOLLOW, wz=0），防止运动干涉
 *   GIMBAL_UNFOLDING → 展开进行中，同上冻结
 *   GIMBAL_DEPLOYED  → 展开完成，恢复为用户选择的正常底盘模式（normal_chassis_mode）
 *
 * @note  此函数在 RemoteControlSet/MouseKeySet 之后调用，会覆盖 chassis_cmd_send.chassis_mode
 *        和 chassis_cmd_send.wz，因此折叠状态下用户对底盘模式的选择暂时不生效。
 */
static void ApplyFoldChassisMode(void)
{
    switch (gimbal_fetch_data.fold_state)
    {
    case GIMBAL_FOLDED:
        // 已折叠：底盘可原地旋转，左摇杆水平直接控制 wz
        chassis_cmd_send.chassis_mode = CHASSIS_FOLDED_ROTATE;
        /* In folded mode the yaw stick is the only source of chassis wz. */
        chassis_cmd_send.wz = 8.0f * (float)rc_data[TEMP].rc.rocker_l_;
        chassis_cmd_send.wz = float_constrain(chassis_cmd_send.wz,
                                               -4500.0f, 4500.0f); // 限幅防止超速
        break;
    case GIMBAL_FOLDING:
    case GIMBAL_UNFOLDING:
        // 折叠/展开进行中：冻结底盘运动，避免与云台机构干涉
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
        chassis_cmd_send.wz = 0.0f;
        break;
    case GIMBAL_DEPLOYED:
    default:
        // 展开完成：恢复用户选择的正常底盘模式
        chassis_cmd_send.chassis_mode = normal_chassis_mode;
        break;
    }
}

/**
 * @brief  紧急停止处理：检测急停触发条件并将整机切入零力/停止状态，或从急停恢复。
 *
 * 触发条件（满足任一即急停）：
 *   - 遥控器拨轮向下拨超过阈值 300（注意：向下拨 dial 为正值）
 *   - robot_state 已经是 ROBOT_STOP（保持急停，防止自行恢复）
 *   - （待添加）重要应用/模块离线、双板通信失效
 *
 * 急停动作：
 *   - 云台 → GIMBAL_ZERO_FORCE（力矩输出为零，自由下垂）
 *   - 底盘 → CHASSIS_ZERO_FORCE
 *   - 发射 → SHOOT_OFF + 摩擦轮关 + 拨弹停
 *
 * 恢复条件：左开关不在[中]（即非强制折叠） 且 右开关[上]，此时 robot_state 恢复为 READY，
 *           发射模式置为 SHOOT_ON。云台/底盘模式由后续 RemoteControlSet/MouseKeySet 正常刷新。
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 * @todo   急停阈值'300'待修改成合适的值,或改为开关控制
 */
static void EmergencyHandler()
{
    // 拨轮的向下拨超过一半进入急停模式.注意向打时下拨轮是正
    if (rc_data[TEMP].rc.dial > 300 || robot_state == ROBOT_STOP) // 还需添加重要应用和模块离线的判断
    {
        // 进入/保持急停：所有子系统零力或停止
        robot_state = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
        LOGERROR("[CMD] emergency stop!");
    }
    // 遥控器右侧开关为[上],恢复正常运行（需左开关不在[中]，避免与强制折叠冲突）
    if (!switch_is_mid(rc_data[TEMP].rc.switch_left) &&
        switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        robot_state = ROBOT_READY;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
        LOGINFO("[CMD] reinstate, robot ready");
    }
}

/**
 * @brief 机器人核心控制任务，200Hz 频率运行（必须高于视觉发送频率以保证控制延迟可控）。
 *
 * 执行流程（每周期固定顺序）：
 *   1. 采集反馈：从消息中心/CAN 获取底盘、发射、云台的回传数据
 *   2. 计算偏移：CalcOffsetAngle — 云台相对底盘正方向的夹角
 *   3. 解析输入：根据左开关位置选择 RemoteControlSet 或 MouseKeySet，填充各子系统控制量
 *   4. 折叠联动：ApplyFoldChassisMode — 根据云台折叠状态覆盖底盘模式
 *   5. 急停处理：EmergencyHandler — 检测急停/恢复，最高优先级覆盖所有控制
 *   6. 下发指令：通过消息中心（单板）或 CAN（双板）将控制指令推送给各子系统，并发送视觉数据
 *
 * @note  急停(EmergencyHandler)在折叠联动之后执行，因此急停的 ZERO_FORCE 会覆盖一切模式设置，
 *        确保最高优先级。视觉发送数据目前未填充（VisionSetFlag 被注释），待完善自瞄功能。
 */
void RobotCMDTask()
{
   // BMI088Acquire(bmi088_test,&bmi088_data) ; // 预留：直接读取云台IMU（目前通过云台反馈间接获取）

    // ---------- 1. 从其他应用获取回传数据 ----------
#ifdef ONE_BOARD
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
#endif // ONE_BOARD
#ifdef GIMBAL_BOARD
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
#endif // GIMBAL_BOARD
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // ---------- 2. 计算云台与底盘正方向夹角 ----------
    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();

    // ---------- 3. 根据遥控器左侧开关选择输入源并解析控制量 ----------
    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    if (switch_is_down(rc_data[TEMP].rc.switch_left) ||
        switch_is_mid(rc_data[TEMP].rc.switch_left)) // 左侧下/中档为遥控器控制
        RemoteControlSet();
    else if (switch_is_up(rc_data[TEMP].rc.switch_left)) // 遥控器左侧开关状态为[上],键盘控制
        MouseKeySet();

    // ---------- 4. 折叠状态联动覆盖底盘模式 ----------
    ApplyFoldChassisMode();

    // ---------- 5. 紧急停止处理（最高优先级，会覆盖上述所有设置） ----------
    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况

    // ---------- 6. 视觉数据设置（待完善：需填充敌方颜色、弹速等自瞄信息） ----------
    // 设置视觉发送数据,还需增加加速度和角速度数据
    // VisionSetFlag(chassis_fetch_data.enemy_color,,chassis_fetch_data.bullet_speed)

    // ---------- 7. 下发所有控制指令 ----------
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
