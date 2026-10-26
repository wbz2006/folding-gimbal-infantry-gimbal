#ifndef MONITOR_H
#define MONITOR_H

#include "stdint.h"
#include "string.h"

#define DAEMON_MX_CNT 64
#define DAEMON_TASK_PERIOD_MS 10U
#define DAEMON_COUNT_FROM_MS(timeout_ms) \
    (((timeout_ms) + DAEMON_TASK_PERIOD_MS - 1U) / DAEMON_TASK_PERIOD_MS)

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
 * @brief 放入rtos中,会给每个daemon实例的temp_count按频率进行递减操作.
 *        模块成功接受数据或成功操作则会重载temp_count的值为reload_count.
 *
 */
void DaemonTask();

#endif // !MONITOR_H
