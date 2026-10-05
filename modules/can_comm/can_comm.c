#include "can_comm.h"
#include "memory.h"
#include "stdlib.h"
#include "crc8.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

/**
 * @brief 重置CAN comm的接收状态和buffer
 *
 * @param ins 需要重置的实例
 */
static void CANCommResetRx(CANCommInstance *ins)
{
    if (ins == NULL)
        return;
    // 当前已经收到的buffer清零
    memset(ins->raw_recvbuf, 0, ins->cur_recv_len);
    ins->recv_state = 0;   // 接收状态重置
    ins->cur_recv_len = 0; // 当前已经收到的长度重置
}

/**
 * @brief cancomm的接收回调函数
 *
 * @param _instance
 */
static void CANCommRxCallback(CANInstance *_instance)
{
    if (_instance == NULL || _instance->id == NULL ||
        _instance->rx_len == 0 || _instance->rx_len > 8)
        return;
    CANCommInstance *comm = (CANCommInstance *)_instance->id; // 注意写法,将can instance的id强制转换为CANCommInstance*类型

    /* 当前接收状态判断 */
    if (_instance->rx_buff[0] == CAN_COMM_HEADER && comm->recv_state == 0) // 之前尚未开始接收且此次包里第一个位置是帧头
    {
        if (_instance->rx_len >= 2 &&
            _instance->rx_buff[1] == comm->recv_data_len) // 如果这一包里的datalen也等于我们设定接收长度(这是因为暂时不支持动态包长)
        {
            comm->recv_state = 1; // 设置接收状态为1,说明已经开始接收
        }
        else
        {
            comm->rx_error_count++;
            return; // 直接跳过即可
        }
    }

    if (comm->recv_state) // 已经收到过帧头
    {
        // 如果已经接收到的长度加上当前一包的长度大于总buf len,说明接收错误
        if (comm->cur_recv_len + _instance->rx_len > comm->recv_buf_len)
        {
            CANCommResetRx(comm);
            return; // 重置状态然后返回
        }

        // 直接把当前接收到的数据接到buffer后面
        memcpy(comm->raw_recvbuf + comm->cur_recv_len, _instance->rx_buff, _instance->rx_len);
        comm->cur_recv_len += _instance->rx_len;

        // 收完这一包以后刚好等于总buf len,说明已经收完了
        if (comm->cur_recv_len == comm->recv_buf_len)
        {
            // 如果buff里本tail的位置等于CAN_COMM_TAIL
            if (comm->raw_recvbuf[comm->recv_buf_len - 1] == CAN_COMM_TAIL)
            { // 通过校验,复制数据到unpack_data中
                if (comm->raw_recvbuf[comm->recv_buf_len - 2] == crc_8(comm->raw_recvbuf + 2, comm->recv_data_len))
                { // 数据量大的话考虑使用DMA
                    memcpy(comm->unpacked_recv_data, comm->raw_recvbuf + 2, comm->recv_data_len);
                    comm->update_flag = 1;           // 数据更新flag置为1
                    comm->has_rx_data = 1;
                    comm->rx_ok_count++;
                    DaemonReload(comm->comm_daemon); // 重载daemon,避免数据更新后一直不被读取而导致数据更新不及时
                }
                else
                    comm->rx_error_count++;
            }
            else
                comm->rx_error_count++;
            CANCommResetRx(comm);
            return; // 重置状态然后返回
        }
    }
}

static void CANCommLostCallback(void *cancomm)
{
    CANCommInstance *comm = (CANCommInstance *)cancomm;
    if (comm == NULL)
        return;
    CANCommResetRx(comm);
    comm->has_rx_data = 0;
    if (comm->can_ins != NULL)
        LOGWARNING("[can_comm] can comm rx[%lu] lost, reset rx state.",
                   (unsigned long)comm->can_ins->rx_id);
}

CANCommInstance *CANCommInit(CANComm_Init_Config_s *comm_config)
{
    if (comm_config == NULL ||
        comm_config->recv_data_len == 0 || comm_config->send_data_len == 0 ||
        comm_config->recv_data_len > CAN_COMM_MAX_BUFFSIZE ||
        comm_config->send_data_len > CAN_COMM_MAX_BUFFSIZE)
        return NULL;

    CANCommInstance *ins = (CANCommInstance *)malloc(sizeof(CANCommInstance));
    if (ins == NULL)
        return NULL;
    memset(ins, 0, sizeof(CANCommInstance));

    ins->recv_data_len = comm_config->recv_data_len;
    ins->recv_buf_len = comm_config->recv_data_len + CAN_COMM_OFFSET_BYTES; // head + datalen + crc8 + tail
    ins->send_data_len = comm_config->send_data_len;
    ins->send_buf_len = comm_config->send_data_len + CAN_COMM_OFFSET_BYTES;
    ins->raw_sendbuf[0] = CAN_COMM_HEADER;            // head,直接设置避免每次发送都要重新赋值,下面的tail同理
    ins->raw_sendbuf[1] = comm_config->send_data_len; // datalen
    ins->raw_sendbuf[comm_config->send_data_len + CAN_COMM_OFFSET_BYTES - 1] = CAN_COMM_TAIL;
    // can instance的设置
    comm_config->can_config.id = ins; // CANComm的实例指针作为CANInstance的id,回调函数中会用到
    comm_config->can_config.can_module_callback = CANCommRxCallback;
    ins->can_ins = CANRegister(&comm_config->can_config);
    if (ins->can_ins == NULL)
    {
        free(ins);
        return NULL;
    }

    Daemon_Init_Config_s daemon_config = {
        .callback = CANCommLostCallback,
        .owner_id = (void *)ins,
        .reload_count = comm_config->daemon_count,
    };
    ins->comm_daemon = DaemonRegister(&daemon_config);
    if (ins->comm_daemon == NULL)
    {
        /* CAN实例已经注册，保留实例但发送/接收接口会安全地拒绝工作。 */
        return ins;
    }
    return ins;
}

void CANCommSend(CANCommInstance *instance, uint8_t *data)
{
    if (instance == NULL || instance->can_ins == NULL || data == NULL)
        return;

    uint32_t now = (uint32_t)DWT_GetTimeline_ms();
    if (!CANCommIsOnline(instance) && instance->tx_attempt_count != 0U &&
        (uint32_t)(now - instance->last_tx_tick) < CAN_COMM_OFFLINE_TX_PERIOD_MS)
        return;
    instance->last_tx_tick = now;
    instance->tx_attempt_count++;

    static uint8_t crc8;
    static uint8_t send_len;
    // 将data copy到raw_sendbuf中,计算crc8
    memcpy(instance->raw_sendbuf + 2, data, instance->send_data_len);
    crc8 = crc_8(data, instance->send_data_len);
    instance->raw_sendbuf[2 + instance->send_data_len] = crc8;

    // CAN单次发送最大为8字节,如果超过8字节,需要分包发送
    for (size_t i = 0; i < instance->send_buf_len; i += 8)
    { // 如果是最后一包,send len将会小于8,要修改CAN的txconf中的DLC位,调用bsp_can提供的接口即可
        send_len = instance->send_buf_len - i >= 8 ? 8 : instance->send_buf_len - i;
        CANSetDLC(instance->can_ins, send_len);
        memcpy(instance->can_ins->tx_buff, instance->raw_sendbuf + i, send_len);
        if (!CANTransmit(instance->can_ins, 1))
            instance->tx_error_count++;
    }
}

void *CANCommGet(CANCommInstance *instance)
{
    if (instance == NULL)
        return NULL;
    instance->update_flag = 0; // 读取后将更新flag置为0
    return instance->unpacked_recv_data;
}

uint8_t CANCommReceive(CANCommInstance *instance, void *data)
{
    if (instance == NULL || instance->can_ins == NULL ||
        instance->comm_daemon == NULL || data == NULL ||
        !instance->has_rx_data || !CANCommIsOnline(instance))
        return 0;

    memcpy(data, instance->unpacked_recv_data, instance->recv_data_len);
    instance->update_flag = 0;
    return 1;
}

uint8_t CANCommIsOnline(CANCommInstance *instance)
{
    return instance != NULL && instance->comm_daemon != NULL &&
           instance->can_ins != NULL && instance->has_rx_data &&
           DaemonIsOnline(instance->comm_daemon);
}
