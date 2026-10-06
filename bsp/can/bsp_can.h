#ifndef BSP_CAN_H
#define BSP_CAN_H

//在此选择CAN类型，两者只能选择一个！！！
#define FDCAN //G系列和H7系列使用FDCAN
//#define BXCAN //F系列使用BxCAN

//CAN类型宏定义检查，有错误停止编译
#if !defined(FDCAN) && !defined(BXCAN)
    #error "Neither FDCAN nor BXCAN is defined. Please define one of them."
#elif defined(FDCAN) && defined(BXCAN)
    #error "Both FDCAN and BXCAN are defined. Please define only one."
#endif


#include <stdint.h>
#ifdef FDCAN
#include "fdcan.h"
#define hcan1  hfdcan1
#define hcan2  hfdcan2
#define hcan3  hfdcan3

#define CAN_MX_REGISTER_CNT 16     // 这个数量取决于CAN总线的负载
#define MX_CAN_FILTER_CNT (3 * 14) // 最多可以使用的CAN过滤器数量,目前远不会用到这么多
#define DEVICE_CAN_CNT 3           //H723VG有3个FDCAN

#endif
#ifdef BXCAN
#include "can.h"
// 最多能够支持的CAN设备数
#define CAN_MX_REGISTER_CNT 16     // 这个数量取决于CAN总线的负载
#define MX_CAN_FILTER_CNT (2 * 14) // 最多可以使用的CAN过滤器数量,目前远不会用到这么多
#define DEVICE_CAN_CNT 2           // 根据板子设定,F407IG有CAN1,CAN2,因此为2;F334只有一个,则设为1
// 如果只有1个CAN,还需要把bsp_can.c中所有的hcan2变量改为hcan1(别担心,主要是总线和FIFO的负载均衡,不影响功能)
#endif

#ifdef FDCAN
// 定义查找表
static const uint32_t DLC_LookUp_Table[9] = {
    FDCAN_DLC_BYTES_0,
    FDCAN_DLC_BYTES_1,  
    FDCAN_DLC_BYTES_2,  
    FDCAN_DLC_BYTES_3,
    FDCAN_DLC_BYTES_4,
    FDCAN_DLC_BYTES_5,
    FDCAN_DLC_BYTES_6,
    FDCAN_DLC_BYTES_7,
    FDCAN_DLC_BYTES_8
};
#endif



/* can instance typedef, every module registered to CAN should have this variable */
typedef struct _
{
#ifdef FDCAN
    FDCAN_HandleTypeDef  *can_handle; // can句柄
    FDCAN_TxHeaderTypeDef txconf;    // CAN报文发送配置
#else
    CAN_HandleTypeDef *can_handle; // can句柄
    CAN_TxHeaderTypeDef txconf;    // CAN报文发送配置
#endif
    uint32_t tx_id;                // 发送id
    uint32_t tx_mailbox;           // CAN消息填入的邮箱号
    uint8_t tx_buff[8];            // 发送缓存,发送消息长度可以通过CANSetDLC()设定,最大为8
    uint8_t rx_buff[8];            // 接收缓存,最大消息长度为8
    uint32_t rx_id;                // 接收id
    uint8_t rx_len;                // 接收长度,可能为0-8
    volatile uint32_t rx_count;
    volatile uint32_t tx_error_count;
    // 接收的回调函数,用于解析接收到的数据
    void (*can_module_callback)(struct _ *); // callback needs an instance to tell among registered ones
    void *id;                                // 使用can外设的模块指针(即id指向的模块拥有此can实例,是父子关系)
} CANInstance;

/* 内部实例使用自然对齐；只有线上payload结构需要双方约定布局。 */
typedef enum
{
    CAN_BUS_RUNNING = 0,
    CAN_BUS_WAIT_RETRY,
    CAN_BUS_RECOVERING,
} CANBusState;

typedef struct
{
    volatile CANBusState state;
    volatile uint32_t error_count;
    volatile uint32_t bus_off_count;
    volatile uint32_t recovery_attempt_count;
    volatile uint32_t recovery_success_count; // 控制器退出Bus-Off，不代表对端在线
    volatile uint32_t recovery_timeout_count;
    volatile uint32_t recovery_error_count; // 恢复请求/启动/通知接口返回失败
    volatile uint32_t rx_drop_count;
    volatile uint32_t tec; // ECR.TEC发送错误计数,无ACK时持续上升(>255触发Bus-Off);0=总线被正常应答
    volatile uint32_t rec; // ECR.REC接收错误计数
    volatile uint8_t online; // 该总线上双板通信对端在线标志(由can_comm发送周期刷新);0=对端离线/尚未上线
    uint32_t last_recovery_tick;
} CANBusStatus;

/* CAN实例初始化结构体,将此结构体指针传入注册函数 */
typedef struct
{
#ifdef FDCAN
    FDCAN_HandleTypeDef  *can_handle;           // can句柄
#else
    CAN_HandleTypeDef *can_handle;              // can句柄
#endif
    uint32_t tx_id;                             // 发送id
    uint32_t rx_id;                             // 接收id
    void (*can_module_callback)(CANInstance *); // 处理接收数据的回调函数
    void *id;                                   // 拥有can实例的模块地址,用于区分不同的模块(如果有需要的话),如果不需要可以不传入
} CAN_Init_Config_s;

/**
 * @brief Register a module to CAN service,remember to call this before using a CAN device
 *        注册(初始化)一个can实例,需要传入初始化配置的指针.
 * @param config init config
 * @return CANInstance* can instance owned by module
 */
CANInstance *CANRegister(CAN_Init_Config_s *config);

extern volatile uint32_t can_error_count;
extern volatile uint32_t can_bus_off_count;
extern CANBusStatus can_bus_status[DEVICE_CAN_CNT]; // 下标0/1/2对应CAN1/2/3

/* 由DaemonTask周期调用，任务上下文维护各总线；中断只记录错误状态。 */
void CANServiceTask(void);
uint8_t CANIsReady(const CANInstance *instance);
const CANBusStatus *CANGetBusStatus(const CANInstance *instance);

/* 由can_comm在其发送周期调用,把实例所在总线的对端在线标志写入can_bus_status[bus].online */
void CANSetLinkOnline(const CANInstance *instance, uint8_t online);

/**
 * @brief 修改CAN发送报文的数据帧长度;注意最大长度为8,在没有进行修改的时候,默认长度为8
 *
 * @param _instance 要修改长度的can实例
 * @param length    设定长度
 */
void CANSetDLC(CANInstance *_instance, uint8_t length);

/**
 * @brief transmit mesg through CAN device,通过can实例发送消息
 *        发送前需要向CAN实例的tx_buff写入发送数据
 * 
 * @attention 超时时间不应该超过调用此函数的任务的周期,否则会导致任务阻塞
 * 
 * @param timeout 超时时间,单位为ms;后续改为us,获得更精确的控制
 * @param _instance* can instance owned by module
 * @return 1仅表示提交到硬件发送队列，0表示失败或正在恢复。
 */
uint8_t CANTransmit(CANInstance *_instance,float timeout);

#endif
