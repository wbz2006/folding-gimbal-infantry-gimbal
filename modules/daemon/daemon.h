#ifndef MONITOR_H
#define MONITOR_H

#include "stdint.h"
#include "string.h"

#define DAEMON_MX_CNT 64
#define DAEMON_TASK_PERIOD_MS 10U
#define DAEMON_COUNT_FROM_MS(timeout_ms) \
    (((timeout_ms) + DAEMON_TASK_PERIOD_MS - 1U) / DAEMON_TASK_PERIOD_MS)

/**
 * @brief 通用电机离线位，供 App_Fault_s.motor_offline_flags 按位组合。
 * @note 各应用自行约定电机编号与设备的对应关系，未使用的电机位保持为 0。
 *       置位表示反馈不可用，也可能由 CAN 总线故障引起，不代表电机本体损坏。
 */
typedef enum
{
    MOTOR_OFFLINE_NONE = 0,        // 本应用使用的电机全部在线
    MOTOR_1_OFFLINE = 1U << 0,     // 本应用电机 1 离线
    MOTOR_2_OFFLINE = 1U << 1,     // 本应用电机 2 离线
    MOTOR_3_OFFLINE = 1U << 2,     // 本应用电机 3 离线
    MOTOR_4_OFFLINE = 1U << 3,     // 本应用电机 4 离线
    MOTOR_5_OFFLINE = 1U << 4,     // 本应用电机 5 离线
    MOTOR_6_OFFLINE = 1U << 5,     // 本应用电机 6 离线
    MOTOR_7_OFFLINE = 1U << 6,     // 本应用电机 7 离线
    MOTOR_8_OFFLINE = 1U << 7,     // 本应用电机 8 离线
} Motor_Offline_Flag_e;

/**
 * @brief 通用应用通信故障位，供 App_Fault_s.comm_fault_flags 按位组合。
 * @note 表示本应用依赖的命令或反馈链路状态，不绑定 CAN、串口或消息中心。
 */
typedef enum
{
    APP_COMM_FAULT_NONE = 0,
    APP_COMM_FAULT_COMMAND_OFFLINE = 1U << 0,  // 命令链路不可用：尚未收到、接收超时或底层通信故障
    APP_COMM_FAULT_FEEDBACK_OFFLINE = 1U << 1, // 所依赖的其他应用反馈链路不可用，不表示本地电机离线
    APP_COMM_FAULT_INVALID_COMMAND = 1U << 2,  // 收到的应用命令内容无效，如 NaN/Inf 或不支持的控制模式
} App_Comm_Fault_Flag_e;

/**
 * @brief 通用 IMU 故障位，供 App_Fault_s.imu_fault_flags 按位组合。
 */
typedef enum
{
    APP_IMU_FAULT_NONE = 0,
    APP_IMU_FAULT_OFFLINE = 1U << 0,      // 未收到有效采样或采样更新超时
    APP_IMU_FAULT_DATA_INVALID = 1U << 1, // 已有采样或姿态数据无效，由 IMU 驱动或所属应用判定
} App_IMU_Fault_Flag_e;

/**
 * @brief 通用控制输入故障位，供 App_Fault_s.control_fault_flags 按位组合。
 * @note 由接收遥控器或图传控制数据的应用填写；底盘未直接使用这些输入时保持为 0。
 */
typedef enum
{
    APP_CONTROL_FAULT_NONE = 0,
    APP_CONTROL_FAULT_REMOTE_OFFLINE = 1U << 0, // 遥控器控制数据尚未收到或更新超时
    APP_CONTROL_FAULT_VIDEO_OFFLINE = 1U << 1,  // 图传控制数据尚未收到或更新超时
} App_Control_Fault_Flag_e;

/**
 * @brief CMD、底盘、云台、发射共用的故障状态结构，每个应用各持有一份。
 * @note 各字段按位组合，支持同时反馈多个故障；未使用的字段和位保持为 0。
 *       电机编号由所属应用约定，不绑定机构或厂家；检测、故障汇总及处置由所属应用负责。
 *       本结构不执行在线检测，未接入检测的字段为 0 也不表示对应设备已确认正常。
 *       使用 uint8_t 固定各字段的通信宽度，共 4 字节；枚举仅定义位掩码。
 */
typedef struct
{
    uint8_t motor_offline_flags; // 本应用电机 1～8 离线位，见 Motor_Offline_Flag_e
    uint8_t comm_fault_flags;    // 本应用依赖的通信链路故障位，见 App_Comm_Fault_Flag_e
    uint8_t imu_fault_flags;     // 本应用使用的 IMU 故障位，见 App_IMU_Fault_Flag_e
    uint8_t control_fault_flags; // 遥控器/图传控制输入故障位，见 App_Control_Fault_Flag_e
} App_Fault_s;

/* 模块离线处理函数指针 */
typedef void (*offline_callback)(void *);

/* daemon结构体定义 */
typedef struct daemon_ins
{
    uint16_t reload_count;     // 重载值
    offline_callback callback; // 异常处理函数,当模块发生异常时会被调用

    volatile uint16_t temp_count; // 接收中断喂狗、守护任务递减
    void *owner_id;      // daemon实例的地址,初始化的时候填入
    volatile uint8_t offline_reported; // 防止离线期间每个周期重复触发回调
    volatile uint8_t has_reloaded;
    volatile uint8_t feedback_valid;  // application报告的复合反馈有效性
    volatile uint32_t fault_flags;    // application报告的故障位集合
} DaemonInstance;

/* daemon初始化配置 */
typedef struct
{
    uint16_t reload_count;     // 实际上这是app唯一需要设置的值?
    uint16_t init_count;       // 上线等待时间,有些模块需要收到主控的指令才会反馈报文,或pc等需要开机时间
    offline_callback callback; // 异常处理函数,当模块发生异常时会被调用

    void *owner_id;            // id取拥有daemon的实例的地址,如DJIMotorInstance*,cast成void*类型
    uint8_t require_first_reload; // 首次有效数据到达前保持离线
} Daemon_Init_Config_s;

/**
 * @brief 注册一个daemon实例
 *
 * @param config 初始化配置
 * @return DaemonInstance* 返回实例指针
 */
DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config);

/**
 * @brief 收到新的有效数据时刷新新鲜度计数。
 *
 * @param instance daemon实例指针
 */
void DaemonReload(DaemonInstance *instance);

/**
 * @brief 确认模块是否在线且数据仍在新鲜度窗口内
 *
 * @param instance daemon实例指针
 * @return uint8_t 若在线且工作正常,返回1;否则返回零. 后续根据异常类型和离线状态等进行优化.
 */
uint8_t DaemonIsOnline(DaemonInstance *instance);
uint8_t DaemonHasReloaded(DaemonInstance *instance);

/** 设置/读取由application判断的复合反馈有效状态。 */
void DaemonSetFeedbackValid(DaemonInstance *instance, uint8_t valid);
uint8_t DaemonIsFeedbackValid(DaemonInstance *instance);

/** 设置、累加、清除和读取application定义的故障位集合。 */
void DaemonSetFaultFlags(DaemonInstance *instance, uint32_t fault_flags);
void DaemonAddFaultFlags(DaemonInstance *instance, uint32_t fault_flags);
void DaemonClearFaultFlags(DaemonInstance *instance, uint32_t fault_flags);
uint32_t DaemonGetFaultFlags(DaemonInstance *instance);

/**
 * @brief 将应用的分类故障写入 daemon，并更新设备反馈有效状态。
 * @param daemon 本应用使用的 daemon 实例。
 * @param fault 本周期已采集的故障；未使用的分类字段保持为 0。
 * @return uint32_t 故障快照，0 表示无故障；从低到高依次为电机、通信、IMU、控制输入。
 * @note 电机、IMU 或所依赖的应用反馈链路故障会使设备反馈无效。
 *       命令链路和控制输入故障仍计入快照，但不改变设备反馈的有效性。
 *       本函数不采集设备状态、不喂狗、不清除命令，停机策略由所属应用执行。
 */
uint32_t AppFaultUpdate(DaemonInstance *daemon, const App_Fault_s *fault);

/**
 * @brief 放入rtos中,会给每个daemon实例的temp_count按频率进行递减操作.
 *        模块成功接受数据或成功操作则会重载temp_count的值为reload_count.
 *
 */
void DaemonTask();

#endif // !MONITOR_H
