#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "dmmotor.h"
#include "message_center.h"
#include "daemon.h"
#include "general_def.h"
#include "user_lib.h"
#include <math.h>
#include <stdbool.h>

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r;
static DMMotorInstance *loader;
#define LOADER_MAX_SPEED_RAD_S 2.0f
#define LOADER_MIT_KD 0.5f
// static servo_instance *lid; 需要增加弹舱盖

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息
static DaemonInstance *shoot_cmd_daemon;

void ShootInit()
{
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan1,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 20, // 20
                .Ki = 1, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0.1, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508
    };
    friction_config.can_init_config.tx_id = 1,
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 2; // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_r = DJIMotorInit(&friction_config);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 0x06,
            .rx_id = 0x302,
        },
        .controller_setting_init_config = {
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = DM4310,
    };
    loader = DMMotorInit(&loader_config);
    DMMotorSetControlMode(loader, DMMOTOR_CONTROL_MIT);
    DMMotorStop(loader);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    Daemon_Init_Config_s cmd_daemon_config = {
        .reload_count = DAEMON_COUNT_FROM_MS(APP_COMMAND_TIMEOUT_MS),
        .require_first_reload = 1,
    };
    shoot_cmd_daemon = DaemonRegister(&cmd_daemon_config);
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    bool message_received = SubGetMessage(shoot_sub, &shoot_cmd_recv);
    bool command_valid = (shoot_cmd_recv.shoot_mode == SHOOT_OFF || shoot_cmd_recv.shoot_mode == SHOOT_ON) &&
                         (shoot_cmd_recv.load_mode == LOAD_STOP || shoot_cmd_recv.load_mode == LOAD_REVERSE ||
                          shoot_cmd_recv.load_mode == LOAD_BURSTFIRE) &&
                         (shoot_cmd_recv.lid_mode == LID_OPEN || shoot_cmd_recv.lid_mode == LID_CLOSE) &&
                         (shoot_cmd_recv.friction_mode == FRICTION_OFF || shoot_cmd_recv.friction_mode == FRICTION_ON) &&
                         (shoot_cmd_recv.bullet_speed == BULLET_SPEED_NONE || shoot_cmd_recv.bullet_speed == BIG_AMU_10 ||
                          shoot_cmd_recv.bullet_speed == SMALL_AMU_15 || shoot_cmd_recv.bullet_speed == BIG_AMU_16 ||
                          shoot_cmd_recv.bullet_speed == SMALL_AMU_18 || shoot_cmd_recv.bullet_speed == SMALL_AMU_30) &&
                         isfinite(shoot_cmd_recv.shoot_rate) && shoot_cmd_recv.shoot_rate >= 0.0f;
    if (message_received && command_valid)
        DaemonReload(shoot_cmd_daemon);

    App_Fault_s *fault = &shoot_feedback_data.fault;
    memset(fault, 0, sizeof(*fault));
    if (!DaemonIsOnline(shoot_cmd_daemon))
        fault->comm_fault_flags |= APP_COMM_FAULT_COMMAND_OFFLINE;
    if (!command_valid)
        fault->comm_fault_flags |= APP_COMM_FAULT_INVALID_COMMAND;
    // 电机位 1/2/3 分别对应左摩擦轮、右摩擦轮、拨弹轮。
    if (MotorIsOffline(friction_l->daemon, friction_l->motor_can_instance))
        fault->motor_offline_flags |= MOTOR_1_OFFLINE;
    if (MotorIsOffline(friction_r->daemon, friction_r->motor_can_instance))
        fault->motor_offline_flags |= MOTOR_2_OFFLINE;
    if (!DMMotorIsFeedbackValid(loader))
        fault->motor_offline_flags |= MOTOR_3_OFFLINE;
    if (AppFaultUpdate(shoot_cmd_daemon, fault) != 0)
    {
        if (friction_l != NULL) DJIMotorStop(friction_l);
        if (friction_r != NULL) DJIMotorStop(friction_r);
        DMMotorStop(loader);
        PubPushMessage(shoot_pub, &shoot_feedback_data);
        return;
    }

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DMMotorStop(loader);
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
    }

    // MIT 速度控制不使用位置反馈保持或累计转角。
    if (shoot_cmd_recv.load_mode == LOAD_STOP)
        DMMotorStop(loader);
    else
    {
        const float direction = shoot_cmd_recv.load_mode == LOAD_REVERSE ? -1.0f : 1.0f;
        const float speed = float_constrain(shoot_cmd_recv.shoot_rate * PI2 *
            REDUCTION_RATIO_LOADER / NUM_PER_CIRCLE, 0.0f, LOADER_MAX_SPEED_RAD_S);
        DMMotorSetMITRef(loader, 0.0f, direction * speed, 0.0f, LOADER_MIT_KD, 0.0f);
        if (shoot_cmd_recv.shoot_mode == SHOOT_ON)
            DMMotorEnable(loader);
    }

    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
        switch (shoot_cmd_recv.bullet_speed)
        {
        case SMALL_AMU_15:
            DJIMotorSetRef(friction_l, 4000);
            DJIMotorSetRef(friction_r, 4000);
            break;
        case SMALL_AMU_18:
            DJIMotorSetRef(friction_l, 4000);
            DJIMotorSetRef(friction_r, 4000);
            break;
        case SMALL_AMU_30:
            DJIMotorSetRef(friction_l, 4000);
            DJIMotorSetRef(friction_r, 4000);
            break;
        default: // 弹速未配置时保持关闭
            DJIMotorSetRef(friction_l, 4000);
            DJIMotorSetRef(friction_r, 4000);
            break;
        }
    }
    else // 关闭摩擦轮
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    }
    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}
