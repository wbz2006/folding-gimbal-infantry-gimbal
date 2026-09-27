/**
 * @file dmmotor.c
 * @brief 达妙 DM4310 电机驱动实现。
 *
 * @details
 * 本文件实现 DM4310 的 CAN 反馈解析、MIT 报文打包、外部多环 PID 控制和
 * 直接 MIT 控制两种周期控制入口。
 */
#include "dmmotor.h"

#include "bsp_log.h"
#include "daemon.h"
#include "general_def.h"
#include "memory.h"
#include "stdlib.h"
#include "string.h"
#include "user_lib.h"

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];

/**
 * @brief 将浮点物理量映射为无符号整数编码。
 *
 * @param x 待编码物理量。
 * @param x_min 物理量下限。
 * @param x_max 物理量上限。
 * @param bits 编码位宽。
 * @return 编码后的无符号整数。
 */
static uint16_t float_to_uint(
    float x,
    float x_min,
    float x_max,
    uint8_t bits)
{
    const float span = x_max - x_min;
    const float offset = x_min;

    return (uint16_t)((x - offset) * ((float)((1U << bits) - 1U)) / span);
}

/**
 * @brief 将无符号整数编码还原为浮点物理量。
 *
 * @param x_int 编码值。
 * @param x_min 物理量下限。
 * @param x_max 物理量上限。
 * @param bits 编码位宽。
 * @return 解码后的物理量。
 */
static float uint_to_float(
    int x_int,
    float x_min,
    float x_max,
    int bits)
{
    const float span = x_max - x_min;
    const float offset = x_min;

    return ((float)x_int) * span / ((float)((1U << bits) - 1U)) + offset;
}

/**
 * @brief 发送达妙电机模式命令。
 *
 * @param cmd 模式命令字。
 * @param motor 电机实例。
 *
 * @note 模式命令格式为前 7 字节 0xff，第 8 字节为命令字。
 */
static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd;
    CANTransmit(motor->motor_can_instace, 1);
}

/**
 * @brief 解析达妙电机 CAN 反馈。
 *
 * @param motor_can 触发回调的 CAN 实例。
 *
 * @note 收到反馈后会刷新 daemon 离线计数。
 */
static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp;
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &motor->measure;

    DaemonReload(motor->motor_daemon);

    measure->last_position = measure->position;

    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    /*
     * 达妙反馈没有 DJI 电机的原始 ECD 字段，单圈角度由位置反馈换算得到。
     * 这里统一输出为 [0, 360) degree，便于与 DJI 电机库的
     * measure.angle_single_round 保持相同的使用方式。
     */
    measure->angle_single_round = measure->position;
    while (measure->angle_single_round < 0.0f)
    {
        measure->angle_single_round += PI2;
    }
    while (measure->angle_single_round >= PI2)
    {
        measure->angle_single_round -= PI2;
    }
    measure->angle_single_round *= RAD_2_DEGREE;

    tmp = (uint16_t)((rxbuff[3] << 4) | (rxbuff[4] >> 4));
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
}

/**
 * @brief 达妙电机反馈超时回调。
 *
 * @param motor_ptr 电机实例指针，类型为 DMMotorInstance*。
 *
 * @note 回调只清零参考并置停止标志，实际全零 MIT 报文由周期控制函数发送。
 */
static void DMMotorLostCallback(void *motor_ptr)
{
    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;
    motor->pid_ref = 0.0f;
    motor->stop_flag = MOTOR_STOP;
}

/**
 * @brief 获取角度环反馈值。
 *
 * @param motor 电机实例。
 * @param setting 电机控制设置。
 * @return 当前角度反馈值。
 *
 * @note 当角度反馈源为 OTHER_FEED 且外部指针有效时使用外部反馈，否则使用电机位置反馈。
 */
static float DMMotorGetAngleMeasure(
    DMMotorInstance *motor,
    Motor_Control_Setting_s *setting)
{
    if (setting->angle_feedback_source == OTHER_FEED &&
        motor->other_angle_feedback_ptr != NULL)
    {
        return *motor->other_angle_feedback_ptr;
    }

    return motor->measure.position;
}

/**
 * @brief 获取速度环反馈值。
 *
 * @param motor 电机实例。
 * @param setting 电机控制设置。
 * @return 当前速度反馈值。
 *
 * @note 当速度反馈源为 OTHER_FEED 且外部指针有效时使用外部反馈，否则使用电机速度反馈。
 */
static float DMMotorGetSpeedMeasure(
    DMMotorInstance *motor,
    Motor_Control_Setting_s *setting)
{
    if (setting->speed_feedback_source == OTHER_FEED &&
        motor->other_speed_feedback_ptr != NULL)
    {
        return *motor->other_speed_feedback_ptr;
    }

    return motor->measure.velocity;
}

/**
 * @brief 计算外部多环 PID 的最终输出。
 *
 * @param motor 电机实例。
 * @return 外部角度/速度/力矩链路计算后的力矩目标。
 *
 * @details
 * pid_ref 作为外环目标，按配置依次经过角度环、速度环和力矩/电流环。
 * 该输出在 DMMotorControl() 中作为 MIT 报文 torque 字段的主体。
 */
static float DMMotorCalculateExternalOutput(DMMotorInstance *motor)
{
    float pid_measure;
    float pid_ref = motor->pid_ref;
    Motor_Control_Setting_s *setting = &motor->motor_settings;

    if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
    {
        pid_ref *= -1.0f;
    }

    if ((setting->close_loop_type & ANGLE_LOOP) &&
        setting->outer_loop_type == ANGLE_LOOP)
    {
        pid_measure = DMMotorGetAngleMeasure(motor, setting);
        pid_ref = PIDCalculate(&motor->angle_PID, pid_measure, pid_ref);
    }

    if ((setting->close_loop_type & SPEED_LOOP) &&
        (setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
    {
        if ((setting->feedforward_flag & SPEED_FEEDFORWARD) &&
            motor->speed_feedforward_ptr != NULL)
        {
            pid_ref += *motor->speed_feedforward_ptr;
        }

        pid_measure = DMMotorGetSpeedMeasure(motor, setting);
        pid_ref = PIDCalculate(&motor->speed_PID, pid_measure, pid_ref);
    }

    if ((setting->feedforward_flag & CURRENT_FEEDFORWARD) &&
        motor->current_feedforward_ptr != NULL)
    {
        pid_ref += *motor->current_feedforward_ptr;
    }

    if (setting->close_loop_type & CURRENT_LOOP)
    {
        pid_ref = PIDCalculate(
            &motor->current_PID,
            motor->measure.torque,
            pid_ref);
    }

    if (setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
    {
        pid_ref *= -1.0f;
    }

    return pid_ref;
}

/**
 * @brief 发送 MIT 控制报文。
 *
 * @param motor 电机实例。
 * @param position_des 电机轴目标位置，单位 rad。
 * @param velocity_des 电机轴目标速度，单位 rad/s。
 * @param kp MIT 位置刚度。
 * @param kd MIT 速度阻尼。
 * @param torque_des 力矩目标/前馈，单位 N*m。
 *
 * @note 两种控制模式共用该底层发送函数。停止状态下会强制发送 p/v/Kp/Kd/tau 全零。
 */
static void DMMotorSendMIT(
    DMMotorInstance *motor,
    float position_des,
    float velocity_des,
    float kp,
    float kd,
    float torque_des)
{
    if (motor->stop_flag == MOTOR_STOP)
    {
        position_des = 0.0f;
        velocity_des = 0.0f;
        kp = 0.0f;
        kd = 0.0f;
        torque_des = 0.0f;
    }

    LIMIT_MIN_MAX(position_des, DM_P_MIN, DM_P_MAX);
    LIMIT_MIN_MAX(velocity_des, DM_V_MIN, DM_V_MAX);
    LIMIT_MIN_MAX(kp, DM_KP_MIN, DM_KP_MAX);
    LIMIT_MIN_MAX(kd, DM_KD_MIN, DM_KD_MAX);
    LIMIT_MIN_MAX(torque_des, DM_T_MIN, DM_T_MAX);

    motor->motor_send_mailbox.position_des =
        float_to_uint(position_des, DM_P_MIN, DM_P_MAX, 16);
    motor->motor_send_mailbox.velocity_des =
        float_to_uint(velocity_des, DM_V_MIN, DM_V_MAX, 12);
    motor->motor_send_mailbox.torque_des =
        float_to_uint(torque_des, DM_T_MIN, DM_T_MAX, 12);
    motor->motor_send_mailbox.Kp =
        float_to_uint(kp, DM_KP_MIN, DM_KP_MAX, 12);
    motor->motor_send_mailbox.Kd =
        float_to_uint(kd, DM_KD_MIN, DM_KD_MAX, 12);

    motor->motor_can_instace->tx_buff[0] =
        (uint8_t)(motor->motor_send_mailbox.position_des >> 8);
    motor->motor_can_instace->tx_buff[1] =
        (uint8_t)motor->motor_send_mailbox.position_des;
    motor->motor_can_instace->tx_buff[2] =
        (uint8_t)(motor->motor_send_mailbox.velocity_des >> 4);
    motor->motor_can_instace->tx_buff[3] =
        (uint8_t)(((motor->motor_send_mailbox.velocity_des & 0x0f) << 4) |
                  (motor->motor_send_mailbox.Kp >> 8));
    motor->motor_can_instace->tx_buff[4] =
        (uint8_t)motor->motor_send_mailbox.Kp;
    motor->motor_can_instace->tx_buff[5] =
        (uint8_t)(motor->motor_send_mailbox.Kd >> 4);
    motor->motor_can_instace->tx_buff[6] =
        (uint8_t)(((motor->motor_send_mailbox.Kd & 0x0f) << 4) |
                  (motor->motor_send_mailbox.torque_des >> 8));
    motor->motor_can_instace->tx_buff[7] =
        (uint8_t)motor->motor_send_mailbox.torque_des;

    CANTransmit(motor->motor_can_instace, 1);
}

/**
 * @brief 将当前电机位置设为编码器零位。
 *
 * @param motor 电机实例。
 */
void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}

/**
 * @brief 注册并初始化达妙电机实例。
 *
 * @param config 通用电机初始化配置。
 * @return 初始化成功返回电机实例指针，失败返回 NULL。
 */
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor =
        (DMMotorInstance *)malloc(sizeof(DMMotorInstance));

    if (motor == NULL || config == NULL || idx >= DM_MOTOR_CNT)
    {
        return NULL;
    }

    memset(motor, 0, sizeof(DMMotorInstance));

    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->current_PID,
            &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID,
            &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID,
            &config->controller_param_init_config.angle_PID);

    motor->other_angle_feedback_ptr =
        config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr =
        config->controller_param_init_config.other_speed_feedback_ptr;
    motor->speed_feedforward_ptr =
        config->controller_param_init_config.speed_feedforward_ptr;
    motor->current_feedforward_ptr =
        config->controller_param_init_config.current_feedforward_ptr;

    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s daemon_config = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&daemon_config);

    DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1);
    DMMotorCaliEncoder(motor);
    DWT_Delay(0.1);

    dm_motor_instance[idx++] = motor;
    return motor;
}

/**
 * @brief 设置外部多环 PID 的参考输入。
 *
 * @param motor 电机实例。
 * @param ref 参考值，含义由外层闭环类型决定。
 */
void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}

/**
 * @brief 使能达妙电机周期控制输出。
 *
 * @param motor 电机实例。
 */
void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

/**
 * @brief 停止达妙电机周期控制输出。
 *
 * @param motor 电机实例。
 *
 * @note 底层发送函数会在停止状态下发送全零 MIT 报文。
 */
void DMMotorStop(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

/**
 * @brief 设置外层闭环类型。
 *
 * @param motor 电机实例。
 * @param close_loop_type 外层闭环类型。
 */
void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e close_loop_type)
{
    motor->motor_settings.outer_loop_type = close_loop_type;
}

/**
 * @brief 切换外部 PID 闭环使用的反馈来源。
 *
 * @param motor 电机实例。
 * @param loop 需要切换反馈源的闭环，支持 ANGLE_LOOP 或 SPEED_LOOP。
 * @param type 反馈来源类型。
 */
void DMMotorChangeFeed(
    DMMotorInstance *motor,
    Closeloop_Type_e loop,
    Feedback_Source_e type)
{
    if (loop == ANGLE_LOOP)
    {
        motor->motor_settings.angle_feedback_source = type;
    }
    else if (loop == SPEED_LOOP)
    {
        motor->motor_settings.speed_feedback_source = type;
    }
    else
    {
        LOGERROR("[dm_motor] loop type error, check memory access and func param");
    }
}

/**
 * @brief 配置完整 MIT 控制参数。
 *
 * @param motor 电机实例。
 * @param config MIT 控制参数。
 */
void DMMotorSetMITConfig(
    DMMotorInstance *motor,
    const DMMotor_MIT_Config_s *config)
{
    if (motor == NULL || config == NULL)
    {
        return;
    }

    motor->mit_config = *config;
}

/**
 * @brief 逐参数配置 MIT 控制参数。
 *
 * @param motor 电机实例。
 * @param position_des 电机轴目标位置，单位 rad。
 * @param velocity_des 电机轴目标速度，单位 rad/s。
 * @param kp MIT 位置刚度。
 * @param kd MIT 速度阻尼。
 * @param torque_ff 力矩前馈，单位 N*m。
 */
void DMMotorSetMITRef(
    DMMotorInstance *motor,
    float position_des,
    float velocity_des,
    float kp,
    float kd,
    float torque_ff)
{
    const DMMotor_MIT_Config_s config = {
        .position_des = position_des,
        .velocity_des = velocity_des,
        .kp = kp,
        .kd = kd,
        .torque_des = torque_ff,
    };

    DMMotorSetMITConfig(motor, &config);
}

/**
 * @brief IMU 外部阻抗控制入口。
 *
 * @details
 * 该函数执行外部角度/速度/力矩多环 PID，并强制以
 * p=0, v=0, Kp=0, Kd=0, torque=PID输出+力矩前馈 的形式发送 MIT 报文。
 */
void DMMotorControl(void)
{
    for (size_t i = 0; i < idx; ++i)
    {
        DMMotorInstance *motor = dm_motor_instance[i];
        const float pid_output = DMMotorCalculateExternalOutput(motor);
        const float torque_des = pid_output + motor->mit_config.torque_des;

        DMMotorSendMIT(motor, 0.0f, 0.0f, 0.0f, 0.0f, torque_des);
    }
}

/**
 * @brief MIT 内部位置速度环控制入口。
 *
 * @details
 * 该函数直接发送 DMMotorSetMITConfig() 配置的 p/v/Kp/Kd/tau_ff，
 * 不计算外部 PID，适合应用层已完成 IMU 外环或轨迹规划的场景。
 */
void DMMotorMITControl(void)
{
    for (size_t i = 0; i < idx; ++i)
    {
        DMMotorInstance *motor = dm_motor_instance[i];

        DMMotorSendMIT(
            motor,
            motor->mit_config.position_des,
            motor->mit_config.velocity_des,
            motor->mit_config.kp,
            motor->mit_config.kd,
            motor->mit_config.torque_des);
    }
}
