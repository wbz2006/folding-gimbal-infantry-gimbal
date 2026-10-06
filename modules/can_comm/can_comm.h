/**
 * @file can_comm.h
 * @author Neo neozng1@hnu.edu.cn
 * @brief  用于多机CAN通信的收发模块
 * @version 0.1
 * @date 2022-11-27
 *
 * @copyright Copyright (c) 2022 HNUYueLu EC all rights reserved
 *
 */
#ifndef CAN_COMM_H
#define CAN_COMM_H

#include "bsp_can.h"
#include "daemon.h"

#define MX_CAN_COMM_COUNT 4 // 注意均衡负载,一条总线上不要挂载过多的外设

#define CAN_COMM_MAX_BUFFSIZE 60 // 最大发送/接收字节数,如果不够可以增加此数值
#define CAN_COMM_HEADER 's'      // 帧头
#define CAN_COMM_TAIL 'e'        // 帧尾
#define CAN_COMM_OFFSET_BYTES 4  // 's'+ datalen + 'e' + crc8
#define CAN_COMM_OFFLINE_TX_PERIOD_MS 100U // 断线时仅保留低频探测，避免持续占满发送队列
#define CAN_COMM_RX_FRAGMENT_TIMEOUT_MS 20U // 分片间隔过长时丢弃半包
#define CAN_COMM_TX_BUDGET_MS 1.0f          // 整个逻辑包共用发送资源等待预算

/* 双板通信"对端是否在线"标志已并入CANBusStatus.online(见bsp_can.h),按总线下标观测:
 * 1=对端(另一块板)在线,0=对端离线/尚未上线.可直接在调试器中观测,无需关心CAN控制器下标.
 * 例如双板通信在CAN1上,直接看can_bus_status[0].online即可,不再需要按CANComm注册顺序索引.
 * 注意:这不是物理总线故障(bus-off)指示,它只反映"对端是否在发包". */

/* CAN comm 结构体, 拥有CAN comm的app应该包含一个CAN comm指针 */
typedef struct
{
    CANInstance *can_ins;
    /* 发送部分 */
    uint8_t send_data_len; // 发送数据长度
    uint8_t send_buf_len;  // 发送缓冲区长度,为发送数据长度+帧头单包数据长度帧尾以及校验和(4)
    uint8_t raw_sendbuf[CAN_COMM_MAX_BUFFSIZE + CAN_COMM_OFFSET_BYTES]; // 额外4个bytes保存帧头帧尾和校验和
    /* 接收部分 */
    uint8_t recv_data_len; // 接收数据长度
    uint8_t recv_buf_len;  // 接收缓冲区长度,为接收数据长度+帧头单包数据长度帧尾以及校验和(4)
    uint8_t raw_recvbuf[CAN_COMM_MAX_BUFFSIZE + CAN_COMM_OFFSET_BYTES]; // 额外4个bytes保存帧头帧尾和校验和
    uint8_t unpacked_recv_data[CAN_COMM_MAX_BUFFSIZE];                  // 解包后的字节缓存，通过CANCommReceive复制到应用结构体
    /* 接收和更新标志位*/
    uint8_t recv_state;   // 接收状态,
    uint8_t cur_recv_len; // 当前已经接收到的数据长度(包括帧头帧尾datalen和校验和)
    volatile uint8_t update_flag;  // 数据更新标志位
    volatile uint32_t rx_ok_count;
    volatile uint32_t rx_error_count;
    volatile uint32_t tx_error_count;
    volatile uint8_t has_rx_data;
    uint32_t tx_attempt_count;
    uint32_t last_tx_tick;
    uint8_t tx_busy;                  // 同一实例的发送流程不可重入
    uint32_t rx_last_tick;            // 最近一个组包分片的时间
    uint32_t rx_bus_off_count;        // 完整有效包对应的总线故障代次
    uint32_t assembly_bus_off_count;  // 半包对应的总线故障代次

    DaemonInstance* comm_daemon;
} CANCommInstance;

/* CAN comm 初始化结构体 */
typedef struct
{
    CAN_Init_Config_s can_config; // CAN初始化结构体
    uint8_t send_data_len;        // 发送数据长度
    uint8_t recv_data_len;        // 接收数据长度

    uint16_t daemon_count; // 守护进程计数,用于初始化守护进程
} CANComm_Init_Config_s;

/**
 * @brief 初始化CANComm
 *
 * @param config CANComm初始化结构体
 * @return CANCommInstance*
 */
CANCommInstance *CANCommInit(CANComm_Init_Config_s *comm_config);

/**
 * @brief 通过CANComm发送数据
 *
 * @param instance cancomm实例
 * @param data 注意此地址的有效数据长度需要和初始化时传入的datalen相同
 */
void CANCommSend(CANCommInstance *instance, uint8_t *data);

/* 获取最近一次校验通过的数据；在线时即使本周期没有新帧也会返回1，断线返回0。 */
uint8_t CANCommReceive(CANCommInstance *instance, void *data);

/**
 * @brief 旧接口：离线返回NULL；在线返回内部缓存，不能保证异步读整包一致。
 *
 * @return void* 返回的数据指针
 * @attention 新调用请使用CANCommReceive复制到自身变量。payload需双方布局一致，
 *            不要将内部字节缓存直接转换为含有对齐要求的结构体指针。
 */
void *CANCommGet(CANCommInstance *instance);

/**
 * @brief 检查CANComm是否在线
 * 
 * @param instance 
 * @return uint8_t 
 */
uint8_t CANCommIsOnline(CANCommInstance *instance);

#endif // !CAN_COMM_H
