/**
 * @file dmmotor.h
 * @brief 达妙 DM4310 电机驱动接口。
 *
 * @details
 * 本驱动提供两种互斥的周期控制入口：
 * - DMMotorControl()：外部 IMU 阻抗控制，驱动内部执行外部多环 PID，只发送 MIT 力矩项。
 * - DMMotorMITControl()：直接发送应用层配置的 MIT 位置、速度、刚度、阻尼和力矩前馈。
 *
 * @note 同一个控制周期内不要同时调用 DMMotorControl() 和 DMMotorMITControl()。
 */
#ifndef DMMOTOR_H
#define DMMOTOR_H

#include <stdint.h>

#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

/** @brief 最多注册的达妙电机数量。 */
#define DM_MOTOR_CNT 4

/** @brief MIT 位置映射下限，单位 rad。 */
#define DM_P_MIN (-12.5f)
/** @brief MIT 位置映射上限，单位 rad。 */
#define DM_P_MAX (12.5f)
/** @brief MIT 速度映射下限，单位 rad/s。 */
#define DM_V_MIN (-20.94f)
/** @brief MIT 速度映射上限，单位 rad/s。 */
#define DM_V_MAX (20.94f)
/** @brief MIT 力矩映射下限，单位 N*m。 */
#define DM_T_MIN (-12.5f)
/** @brief MIT 力矩映射上限，单位 N*m。 */
#define DM_T_MAX (12.5f)
/** @brief MIT 位置刚度映射下限。 */
#define DM_KP_MIN (0.0f)
/** @brief MIT 位置刚度映射上限。 */
#define DM_KP_MAX (500.0f)
/** @brief MIT 速度阻尼映射下限。 */
#define DM_KD_MIN (0.0f)
/** @brief MIT 速度阻尼映射上限。 */
#define DM_KD_MAX (5.0f)

/**
 * @brief 达妙电机反馈测量值。
 */
typedef struct
{
    uint8_t id;             /**< 电机反馈 ID。 */
    uint8_t state;          /**< 电机状态字。 */
    float velocity;         /**< 电机速度，单位 rad/s。 */
    float last_position;    /**< 上一次位置反馈，单位 rad。 */
    float position;         /**< 当前电机位置，单位 rad。 */
    float angle_single_round; /**< 当前单圈角度，单位 degree，范围为 [0, 360)。 */
    float torque;           /**< 当前反馈力矩，单位 N*m。 */
    float T_Mos;            /**< MOS 温度。 */
    float T_Rotor;          /**< 转子温度。 */
    int32_t total_round;    /**< 预留总圈数计数。 */
} DM_Motor_Measure_s;

/**
 * @brief MIT 控制报文的编码后字段。
 *
 * @note 该结构体只应由驱动内部填充，应用层应使用 DMMotor_MIT_Config_s。
 */
typedef struct
{
    uint16_t position_des;  /**< 16 bit 目标位置编码。 */
    uint16_t velocity_des;  /**< 12 bit 目标速度编码。 */
    uint16_t torque_des;    /**< 12 bit 力矩编码。 */
    uint16_t Kp;            /**< 12 bit 位置刚度编码。 */
    uint16_t Kd;            /**< 12 bit 速度阻尼编码。 */
} DMMotor_Send_s;

/**
 * @brief MIT 控制参数，应用层使用物理量配置。
 */
typedef struct
{
    float position_des;     /**< 电机轴目标位置，单位 rad。 */
    float velocity_des;     /**< 电机轴目标速度，单位 rad/s。 */
    float torque_des;       /**< 力矩前馈，单位 N*m。 */
    float kp;               /**< MIT 位置刚度。 */
    float kd;               /**< MIT 速度阻尼。 */
} DMMotor_MIT_Config_s;

/**
 * @brief 达妙电机实例。
 */
typedef struct
{
    DM_Motor_Measure_s measure;                 /**< 电机反馈测量值。 */
    Motor_Control_Setting_s motor_settings;     /**< 通用电机控制设置。 */

    PIDInstance current_PID;                    /**< 力矩/电流环 PID。 */
    PIDInstance speed_PID;                      /**< 速度环 PID。 */
    PIDInstance angle_PID;                      /**< 角度环 PID。 */

    float *other_angle_feedback_ptr;            /**< 外部角度反馈指针，常用于 IMU 角度。 */
    float *other_speed_feedback_ptr;            /**< 外部速度反馈指针，常用于 IMU 陀螺仪。 */
    float *speed_feedforward_ptr;               /**< 速度前馈指针。 */
    float *current_feedforward_ptr;             /**< 力矩/电流前馈指针。 */
    float pid_ref;                              /**< 外部多环 PID 参考输入。 */

    DMMotor_MIT_Config_s mit_config;            /**< MIT 控制参数配置。 */
    DMMotor_Send_s motor_send_mailbox;          /**< MIT 报文编码缓存。 */

    Motor_Working_Type_e stop_flag;             /**< 电机启停状态。 */
    CANInstance *motor_can_instace;             /**< 绑定的 CAN 实例。 */
    DaemonInstance *motor_daemon;               /**< 反馈离线监测实例。 */
    uint32_t lost_cnt;                          /**< 预留离线计数。 */
} DMMotorInstance;

/**
 * @brief 达妙电机模式命令。
 */
typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,     /**< 使能电机，响应控制报文。 */
    DM_CMD_RESET_MODE = 0xfd,     /**< 失能/停止。 */
    DM_CMD_ZERO_POSITION = 0xfe,  /**< 将当前位置设为编码器零位。 */
    DM_CMD_CLEAR_ERROR = 0xfb     /**< 清除电机错误。 */
} DMMotor_Mode_e;

/**
 * @brief 注册并初始化达妙电机实例。
 *
 * @param config 通用电机初始化配置。
 * @return 初始化成功返回电机实例指针，失败返回 NULL。
 */
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);

/**
 * @brief 设置外部多环 PID 的参考输入。
 *
 * @param motor 电机实例。
 * @param ref 参考值，含义由 outer_loop_type 决定。
 */
void DMMotorSetRef(DMMotorInstance *motor, float ref);

/**
 * @brief 修改外层闭环类型。
 *
 * @param motor 电机实例。
 * @param close_loop_type 外层闭环类型。
 */
void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e close_loop_type);

/**
 * @brief 使能电机周期控制输出。
 *
 * @param motor 电机实例。
 */
void DMMotorEnable(DMMotorInstance *motor);

/**
 * @brief 停止电机周期控制输出。
 *
 * @param motor 电机实例。
 *
 * @note 停止状态下底层发送函数会发送 p/v/Kp/Kd/tau 全零报文。
 */
void DMMotorStop(DMMotorInstance *motor);

/**
 * @brief 将当前位置设为电机编码器零位。
 *
 * @param motor 电机实例。
 */
void DMMotorCaliEncoder(DMMotorInstance *motor);

/**
 * @brief 切换外部PID环使用的反馈来源。
 *
 * @param motor 电机实例。
 * @param loop 需要切换反馈来源的闭环，支持 ANGLE_LOOP 或 SPEED_LOOP。
 * @param type 反馈来源类型。
 */
void DMMotorChangeFeed(
    DMMotorInstance *motor,
    Closeloop_Type_e loop,
    Feedback_Source_e type);

/**
 * @brief 配置物理量形式的MIT控制参数。
 *
 * @param motor 电机实例。
 * @param config MIT 控制参数配置。
 *
 * @note DMMotorMITControl()会完整使用该配置。
 *       DMMotorControl()只使用torque_des作为力矩前馈，p/v/Kp/Kd会被置零发送。
 */
void DMMotorSetMITConfig(
    DMMotorInstance *motor,
    const DMMotor_MIT_Config_s *config);

/**
 * @brief 逐参数设置MIT控制参数，等价于构造DMMotor_MIT_Config_s后调用DMMotorSetMITConfig()。
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
    float torque_ff);

/**
 * @brief IMU外部阻抗控制。
 *
 * @note 函数内部执行外部角度/速度/力矩串级PID。
 *       最终发送p=0, v=0, Kp=0, Kd=0, torque=PID输出+力矩前馈。
 */
void DMMotorControl(void);

/**
 * @brief IMU外环 + DM内部MIT位置速度环控制。
 *
 * @note 直接发送DMMotorSetMITConfig()配置的MIT参数，不计算外部PID。
 */
void DMMotorMITControl(void);

#endif /* DMMOTOR_H */
