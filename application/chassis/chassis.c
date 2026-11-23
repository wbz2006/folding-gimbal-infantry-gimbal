#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "daemon.h"
#include "referee_task.h"

#include "general_def.h"
#include "user_lib.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"
#include <math.h>
#include <float.h>

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长
#define LINEAR_TO_MOTOR (360.0f * REDUCTION_RATIO_WHEEL / PERIMETER_WHEEL)
#define MOTOR_TO_LINEAR (1.0f / LINEAR_TO_MOTOR)

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static DaemonInstance *chassis_cmd_daemon;
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static referee_info_t* referee_data; // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static SuperCapInstance *cap;                                       // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
// 在线判断函数声明
static uint8_t ChassisMotorOfflineFlags(void);
static uint8_t ChassisCommandValid(void);
static void SetChassisEnabled(Motor_Working_Type_e state);

/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅

void ChassisInit()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 10, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 12000,
            },
            .current_PID = {
                .Kp = 0.5, // 0.4
                .Ki = 0,   // 0
                .Kd = 0,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508,
    };

    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lb = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb = DJIMotorInit(&chassis_motor_config);

    SetChassisEnabled(MOTOR_STOP);

    referee_data = UITaskInit(&huart1,&ui_data); // 裁判系统初始化,会同时初始化UI

    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x302, // 超级电容默认接收id
            .rx_id = 0x301, // 超级电容默认发送id,注意tx和rx在其他人看来是反的
        }};
    cap = SuperCapInit(&cap_conf); // 超级电容初始化

    // 发布订阅初始化,如果为双板,则需要can comm来传递消息
#ifdef CHASSIS_BOARD
    Chassis_IMU_data = INS_GetData(); // 硬件初始化由 INS 任务在中断开启后执行

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan3,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
        .daemon_count = DAEMON_COUNT_FROM_MS(APP_COMMAND_TIMEOUT_MS),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
    // 双板复用通信 daemon 的计时与状态存储，不再额外注册一层命令超时。
    chassis_cmd_daemon = chasiss_can_comm->comm_daemon;
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    Daemon_Init_Config_s cmd_daemon_config = {
        .reload_count = DAEMON_COUNT_FROM_MS(APP_COMMAND_TIMEOUT_MS),
        .require_first_reload = 1,
    };
    chassis_cmd_daemon = DaemonRegister(&cmd_daemon_config);
#endif // ONE_BOARD
}

#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
#ifdef CHASSIS_DRIVE_MECANUM
static void MecanumCalculate()
{
    // vt_* 是轮面线速度；正负号由当前底盘坐标约定及麦轮滚子方向决定。
    vt_lf = -chassis_vx - chassis_vy - chassis_cmd_recv.wz * LF_CENTER;
    vt_rf = -chassis_vx + chassis_vy - chassis_cmd_recv.wz * RF_CENTER;
    vt_lb = chassis_vx - chassis_vy - chassis_cmd_recv.wz * LB_CENTER;
    vt_rb = chassis_vx + chassis_vy - chassis_cmd_recv.wz * RB_CENTER;
}
#endif

#ifdef CHASSIS_DRIVE_OMNI_X
#define COS45 0.70710678f
#define OMNI_L (LF_CENTER)
static void OmniCalculate()
{
    // X 型布置下，各轮沿 45° 方向滚动，平移和旋转分量投影到对应轮轴。
    vt_rf =  chassis_vx * COS45 + chassis_vy * COS45 + chassis_cmd_recv.wz * OMNI_L;
    vt_lf = -chassis_vx * COS45 + chassis_vy * COS45 + chassis_cmd_recv.wz * OMNI_L;
    vt_lb = -chassis_vx * COS45 - chassis_vy * COS45 + chassis_cmd_recv.wz * OMNI_L;
    vt_rb =  chassis_vx * COS45 - chassis_vy * COS45 + chassis_cmd_recv.wz * OMNI_L;
}
#endif


/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput()
{
    // 功率限制待添加
    // referee_data->PowerHeatData.chassis_power;
    // referee_data->PowerHeatData.chassis_power_buffer;

    // vt_* 已经是轮面 mm/s；先按实际轮面速度限幅，再换算为 DJI 电机的 degree/s 参考值。
    float wheel_speed_lf = vt_lf;
    float wheel_speed_rf = vt_rf;
    float wheel_speed_lb = vt_lb;
    float wheel_speed_rb = vt_rb;
    float max_speed = fmaxf(fmaxf(fabsf(wheel_speed_lf), fabsf(wheel_speed_rf)),
                            fmaxf(fabsf(wheel_speed_lb), fabsf(wheel_speed_rb)));
    // 四个轮向电机按同一比例限幅，保留底盘解算得到的各轮速度比例。
    float scale = max_speed > CHASSIS_MAX_WHEEL_SPEED ? CHASSIS_MAX_WHEEL_SPEED / max_speed : 1.0f;
    wheel_speed_lf *= scale;
    wheel_speed_rf *= scale;
    wheel_speed_lb *= scale;
    wheel_speed_rb *= scale;
    // 设置轮子速度
    DJIMotorSetRef(motor_lf, wheel_speed_lf * LINEAR_TO_MOTOR);
    DJIMotorSetRef(motor_rf, wheel_speed_rf * LINEAR_TO_MOTOR);
    DJIMotorSetRef(motor_lb, wheel_speed_lb * LINEAR_TO_MOTOR);
    DJIMotorSetRef(motor_rb, wheel_speed_rb * LINEAR_TO_MOTOR);
}

/**
 * @brief 根据每个轮子的速度反馈，计算底盘几何中心的实际运动速度。
 *        当前不融合 IMU；若后续用 IMU 加速度估速，需要补偿 IMU 相对几何中心的安装偏移。
 *
 */
static void EstimateSpeed()
{
    // 故障时先清零，正常时再用本周期反馈填入 mm/s 与 degree/s。
    chassis_feedback_data.real_vx = 0;
    chassis_feedback_data.real_vy = 0;
    chassis_feedback_data.real_wz = 0;
    // 任一轮组反馈离线时，反馈值保持为零，避免把过期数据继续上传。
    if (!DaemonIsFeedbackValid(chassis_cmd_daemon))
        return;
    float speed_lf = motor_lf->measure.speed_aps * MOTOR_TO_LINEAR;
    float speed_rf = motor_rf->measure.speed_aps * MOTOR_TO_LINEAR;
    float speed_lb = motor_lb->measure.speed_aps * MOTOR_TO_LINEAR;
    float speed_rb = motor_rb->measure.speed_aps * MOTOR_TO_LINEAR;
    if (motor_lf->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE) speed_lf = -speed_lf;
    if (motor_rf->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE) speed_rf = -speed_rf;
    if (motor_lb->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE) speed_lb = -speed_lb;
    if (motor_rb->motor_settings.motor_reverse_flag == MOTOR_DIRECTION_REVERSE) speed_rb = -speed_rb;
#ifdef CHASSIS_DRIVE_MECANUM
    // 按麦轮滚子方向反解平移和旋转速度；输入轮速已经还原为逻辑方向的 mm/s。
    chassis_feedback_data.real_vx = (-speed_lf - speed_rf + speed_lb + speed_rb) * 0.25f;
    chassis_feedback_data.real_vy = (-speed_lf + speed_rf - speed_lb + speed_rb) * 0.25f;
    chassis_feedback_data.real_wz = -(speed_lf + speed_rf + speed_lb + speed_rb) /
        (4.0f * (HALF_WHEEL_BASE + HALF_TRACK_WIDTH) * DEGREE_2_RAD);
#else
    // X 型全向轮按各滚动方向的速度投影反解底盘平移及旋转速度。
    chassis_feedback_data.real_vx = (-speed_lf + speed_rf - speed_lb + speed_rb) / (4.0f * COS45);
    chassis_feedback_data.real_vy = (speed_lf + speed_rf - speed_lb - speed_rb) / (4.0f * COS45);
    chassis_feedback_data.real_wz = (speed_lf + speed_rf + speed_lb + speed_rb) / (4.0f * OMNI_L);
#endif
}

/* 机器人底盘控制核心任务 */
/**
 * @brief 接收控制命令，检查命令与电机反馈，运行所选底盘解算并发布反馈。
 */
void ChassisTask()
{
    uint8_t command_available = 0;
#ifdef ONE_BOARD
    // 消息中心没有接收超时管理：只在收到新命令时喂狗，周期之间沿用未超时的命令。
    if (SubGetMessage(chassis_sub, &chassis_cmd_recv))
        DaemonReload(chassis_cmd_daemon);
    command_available = DaemonIsOnline(chassis_cmd_daemon);
#endif
#ifdef CHASSIS_BOARD
    // 通信模块已检查完整包、CRC、接收超时及总线状态；在线时可返回最近的有效命令。
    // 返回 0 时不再使用接收缓存，也不在底盘中增加第二层超时等待。
    command_available = CANCommReceive(chasiss_can_comm, &chassis_cmd_recv);
#endif

    // 每周期覆盖已接入的分类状态，通信和电机分别记录，不再另存一份重复的汇总位。
    chassis_feedback_data.fault.comm_fault_flags = command_available ?
        APP_COMM_FAULT_NONE : APP_COMM_FAULT_COMMAND_OFFLINE;

    // 命令可用不代表电机在线，四个驱动轮分别记录离线位。
    // 公共故障结构保存本应用的电机离线位；反馈是否完整则供 EstimateSpeed 判断能否估速。
    chassis_feedback_data.fault.motor_offline_flags = ChassisMotorOfflineFlags();
#ifdef CHASSIS_BOARD
    chassis_feedback_data.fault.imu_fault_flags = INS_GetFaultFlags();
#endif
    // 通信校验只保证报文完整，速度中的 NaN/Inf 或不支持的模式仍需由底盘拒绝。
    if (ChassisCommandValid() == 0)
        chassis_feedback_data.fault.comm_fault_flags |= APP_COMM_FAULT_INVALID_COMMAND;

    // 通用函数同步故障快照和设备反馈有效性；底盘自身负责故障后的命令清除与停机。
    if (AppFaultUpdate(chassis_cmd_daemon, &chassis_feedback_data.fault) != 0)
    {
        // 清除故障前的速度、偏角和可能的非有限值，避免继续参与后面的坐标变换与解算。
        // 下一周期重新接收并检查命令，全部检查通过后才允许恢复运动。
        memset(&chassis_cmd_recv, 0, sizeof(chassis_cmd_recv));
        chassis_cmd_recv.chassis_mode = CHASSIS_ZERO_FORCE;
    }

    chassis_cmd_recv.offset_angle = theta_format(chassis_cmd_recv.offset_angle);
    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_ZERO_FORCE:
        SetChassisEnabled(MOTOR_STOP);
        chassis_cmd_recv.vx = 0;
        chassis_cmd_recv.vy = 0;
        chassis_cmd_recv.wz = 0;
        break;
    case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
        chassis_cmd_recv.wz = 0;
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,不单独设置pid,以误差角度平方为速度输出
    {
        // 用云台与底盘的角度误差生成旋转速度，并限制跟随速度。
        float offset = theta_format(chassis_cmd_recv.offset_angle);
        chassis_cmd_recv.wz = -50.0f * offset;
        chassis_cmd_recv.wz = float_constrain(chassis_cmd_recv.wz, -4500.0f, 4500.0f);
        break;
    }
    case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
        chassis_cmd_recv.wz = 4000;
        break;
    case CHASSIS_FOLDED_ROTATE:
        chassis_cmd_recv.wz = float_constrain(chassis_cmd_recv.wz, -4500.0f, 4500.0f);
        break;
    default:
        break;
    }

    // 把云台坐标系下的平移命令旋转到车体坐标系；x 向前、y 向右。
    static float sin_theta, cos_theta;
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    // 按编译时选择的底盘类型，将车体速度解算成各轮参考速度。
#if defined(CHASSIS_DRIVE_MECANUM)
    MecanumCalculate();
#elif defined(CHASSIS_DRIVE_OMNI_X)
    OmniCalculate();
#endif

    // 等比例限速后写入四个驱动轮的闭环参考值。
    LimitChassisOutput();
    if (chassis_cmd_recv.chassis_mode != CHASSIS_ZERO_FORCE)
    {
        SetChassisEnabled(MOTOR_ENALBED);
    }

    // 根据电机反馈速度计算真实速度；当前不融合 IMU。
    EstimateSpeed();

    // // 获取裁判系统数据   建议将裁判系统与底盘分离，所以此处数据应使用消息中心发送
    // // 我方颜色id小于7是红色,大于7是蓝色,注意这里发送的是对方的颜色, 0:blue , 1:red
    // chassis_feedback_data.enemy_color = referee_data->GameRobotState.robot_id > 7 ? 1 : 0;
    // // 当前只做了17mm热量的数据获取,后续根据robot_def中的宏切换双枪管和英雄42mm的情况
    // chassis_feedback_data.bullet_speed = referee_data->GameRobotState.shooter_id1_17mm_speed_limit;
    // chassis_feedback_data.rest_heat = referee_data->PowerHeatData.shooter_heat0;

    // 将速度与故障反馈发给本板订阅者或云台板。
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    // 回传底盘 IMU 的三轴角速度、加速度和姿态；轮速反解仍独立使用电机反馈。
    chassis_feedback_data.chassis_imu_data = *Chassis_IMU_data;
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}

/**
 * @brief 检查当前底盘构型的全部电机，生成公共故障结构中的电机离线位。
 * @return uint8_t 需要的电机全部在线时返回 MOTOR_OFFLINE_NONE，否则返回各离线电机对应位的组合。
 */
static uint8_t ChassisMotorOfflineFlags(void)
{
    // 每周期清零重算，恢复在线后自动清除对应位；逐个检查，不能发现一个离线就提前返回。
    uint8_t offline_flags = MOTOR_OFFLINE_NONE;
    // 电机 1～4：各底盘构型共用的 LF/RF/LB/RB 轮向电机。
    if (MotorIsOffline(motor_lf->daemon, motor_lf->motor_can_instance)) offline_flags |= MOTOR_1_OFFLINE;
    if (MotorIsOffline(motor_rf->daemon, motor_rf->motor_can_instance)) offline_flags |= MOTOR_2_OFFLINE;
    if (MotorIsOffline(motor_lb->daemon, motor_lb->motor_can_instance)) offline_flags |= MOTOR_3_OFFLINE;
    if (MotorIsOffline(motor_rb->daemon, motor_rb->motor_can_instance)) offline_flags |= MOTOR_4_OFFLINE;
    return offline_flags;
}

/**
 * @brief 检查底盘命令中的速度、角度和运行模式是否受支持。
 * @return uint8_t 所有浮点字段均为有限值且模式有效时返回 1，否则返回 0。
 */
static uint8_t ChassisCommandValid(void)
{
    // 命令来自消息或 CAN 边界；拒绝非有限速度值及当前不支持的底盘模式。
    if (!isfinite(chassis_cmd_recv.vx) || !isfinite(chassis_cmd_recv.vy) ||
        !isfinite(chassis_cmd_recv.wz) || !isfinite(chassis_cmd_recv.offset_angle))
        return 0;
    // 沿用参考工程的输入范围，避免坐标变换和轮速分量叠加溢出；实际轮速在输出端限幅。
    if (fabsf(chassis_cmd_recv.vx) > sqrtf(FLT_MAX) / 8.0f ||
        fabsf(chassis_cmd_recv.vy) > sqrtf(FLT_MAX) / 8.0f)
        return 0;
    return chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE ||
           chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE ||
           chassis_cmd_recv.chassis_mode == CHASSIS_NO_FOLLOW ||
           chassis_cmd_recv.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW ||
           chassis_cmd_recv.chassis_mode == CHASSIS_FOLDED_ROTATE;
}

/**
 * @brief 统一使能或停止底盘驱动电机。
 * @param state MOTOR_ENALBED 使能电机；MOTOR_STOP 清除 PID 状态并停止输出。
 */
static void SetChassisEnabled(Motor_Working_Type_e state)
{
    if (state == MOTOR_ENALBED)
    {
        // 四个驱动轮使用相同的使能门控。
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
        return;
    }

    // 停机前清除闭环历史，避免重新使能时沿用故障前的积分量。
    if (motor_lf->stop_flag == MOTOR_ENALBED) DJIMotorResetPIDState(motor_lf);
    if (motor_rf->stop_flag == MOTOR_ENALBED) DJIMotorResetPIDState(motor_rf);
    if (motor_lb->stop_flag == MOTOR_ENALBED) DJIMotorResetPIDState(motor_lb);
    if (motor_rb->stop_flag == MOTOR_ENALBED) DJIMotorResetPIDState(motor_rb);
    DJIMotorStop(motor_lf);
    DJIMotorStop(motor_rf);
    DJIMotorStop(motor_lb);
    DJIMotorStop(motor_rb);
}
