#include "bsp_can.h"
#include "main.h"
#include "memory.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

/* can instance ptrs storage, used for recv callback */
// 在CAN产生接收中断会遍历数组,选出hcan和rxid与发生中断的实例相同的那个,调用其回调函数
// @todo: 后续为每个CAN总线单独添加一个can_instance指针数组,提高回调查找的性能
static CANInstance *can_instance[CAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx; // 全局CAN实例索引,每次有新的模块注册会自增
volatile uint32_t can_error_count;
volatile uint32_t can_bus_off_count;
CANBusStatus can_bus_status[DEVICE_CAN_CNT];

#ifdef FDCAN
#define CAN_RECOVERY_RETRY_MS 100U
#define CAN_RECOVERY_TIMEOUT_MS 1000U
#define CAN_ACTIVE_ITS (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST | \
                        FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_MESSAGE_LOST | \
                        FDCAN_IT_ERROR_WARNING | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_BUS_OFF)
static FDCAN_HandleTypeDef *const can_handles[DEVICE_CAN_CNT] = {&hfdcan1, &hfdcan2, &hfdcan3};
static uint8_t can_service_started;

static HAL_StatusTypeDef CANStartBus(FDCAN_HandleTypeDef *handle)
{
    // 首次启动或启动失败重试时，补齐所有配置，不能只重试Start。
    HAL_StatusTypeDef result = HAL_FDCAN_ConfigRxFifoOverwrite(handle, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE);
    if (result == HAL_OK)
        result = HAL_FDCAN_ConfigRxFifoOverwrite(handle, FDCAN_RX_FIFO1, FDCAN_RX_FIFO_OVERWRITE);
    if (result == HAL_OK)
        result = HAL_FDCAN_ConfigGlobalFilter(handle, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    if (result == HAL_OK)
        result = HAL_FDCAN_Start(handle);
    return result;
}

static void CANDiscardRx(FDCAN_HandleTypeDef *handle, CANBusStatus *status)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t discard[64];
    for (uint32_t fifo = FDCAN_RX_FIFO0; fifo <= FDCAN_RX_FIFO1; ++fifo)
    {
        uint32_t remaining = HAL_FDCAN_GetRxFifoFillLevel(handle, fifo);
        while (remaining-- != 0U)
        {
            if (HAL_FDCAN_GetRxMessage(handle, fifo, &header, discard) != HAL_OK)
                break;
            status->rx_drop_count++;
        }
    }
}

static int CANBusIndex(FDCAN_HandleTypeDef *handle)
{
    for (size_t i = 0; i < DEVICE_CAN_CNT; ++i)
        if (handle == can_handles[i])
            return (int)i;
    return -1;
}

static void CANMarkBusOff(size_t bus, uint32_t now)
{
    CANBusStatus *status = &can_bus_status[bus];
    // 同一段Bus-Off只记一次；退出Bus-Off后再次发生才重新计数。
    if (status->state == CAN_BUS_RUNNING)
    {
        status->bus_off_count++;
        can_bus_off_count++;
        status->last_recovery_tick = now;
        status->state = CAN_BUS_WAIT_RETRY;
    }
}

// 错误日志限流:错误类型变化或Bus-Off时,最快每CAN_ERROR_LOG_INTERVAL_MS打印一次
#define CAN_ERROR_LOG_INTERVAL_MS 500U
static uint32_t can_error_log_last_lec[DEVICE_CAN_CNT];
static uint32_t can_error_log_last_tick[DEVICE_CAN_CNT];

/* FDCAN末次错误码(PSR.LEC)译名;注意FDCAN的7=no-change(自上次读取以来无变化),与BxCAN的7=software不同。
 * 仅供日志(LOGWARNING)使用;DISABLE_LOG_SYSTEM会编译掉日志调用使其无引用,故一并编译掉以免-Wunused-function。 */
#if !DISABLE_LOG_SYSTEM
static const char *CANLastErrorName(uint32_t lec)
{
    static const char *const names[8] = {
        "none", "stuff", "form", "ack", "bit-recessive", "bit-dominant", "crc", "no-change"};
    return lec < 8U ? names[lec] : "unknown";
}
#endif
#endif

const CANBusStatus *CANGetBusStatus(const CANInstance *instance)
{
    if (instance == NULL)
        return NULL;
#ifdef FDCAN
    int bus = CANBusIndex(instance->can_handle);
#else
    int bus = instance->can_handle == &hcan1 ? 0 : instance->can_handle == &hcan2 ? 1 : -1;
#endif
    return bus >= 0 ? &can_bus_status[bus] : NULL;
}

void CANSetLinkOnline(const CANInstance *instance, uint8_t online)
{
    if (instance == NULL)
        return;
#ifdef FDCAN
    int bus = CANBusIndex(instance->can_handle);
#else
    int bus = instance->can_handle == &hcan1 ? 0 : instance->can_handle == &hcan2 ? 1 : -1;
#endif
    if (bus >= 0)
        can_bus_status[bus].online = online ? 1U : 0U;
}

uint8_t CANIsReady(const CANInstance *instance)
{
    const CANBusStatus *status = CANGetBusStatus(instance);
    if (status == NULL || status->state != CAN_BUS_RUNNING)
        return 0;
#ifdef FDCAN
    return HAL_FDCAN_GetState(instance->can_handle) == HAL_FDCAN_STATE_BUSY &&
           !(instance->can_handle->Instance->PSR & FDCAN_PSR_BO) &&
           !(instance->can_handle->Instance->CCCR & FDCAN_CCCR_INIT);
#else
    return HAL_CAN_GetState(instance->can_handle) == HAL_CAN_STATE_LISTENING;
#endif
}

void CANServiceTask(void)
{
#ifdef FDCAN
    if (!can_service_started)
        return;
    uint32_t now = HAL_GetTick();
    for (size_t i = 0; i < DEVICE_CAN_CNT; ++i)
    {
        FDCAN_HandleTypeDef *handle = can_handles[i];
        CANBusStatus *status = &can_bus_status[i];
        // 短临界区避免错误中断/其他任务提交发送时同时改变恢复状态。
        // 此处不Stop/DeInit，不等待硬件恢复，也不重建过滤器；仅按FIFO快照丢弃积压帧。
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        uint8_t bus_off = (handle->Instance->PSR & FDCAN_PSR_BO) != 0U;
        if (bus_off)
            CANMarkBusOff(i, now); // 轮询兜底，不依赖错误中断一定到达

        // 采样错误计数器与末次错误码,供调试观测;日志在临界区外限流打印
        uint32_t ecr = handle->Instance->ECR;
        status->tec = ecr & FDCAN_ECR_TEC_Msk;
        status->rec = (ecr & FDCAN_ECR_REC_Msk) >> FDCAN_ECR_REC_Pos;
        uint32_t lec = (handle->Instance->PSR & FDCAN_PSR_LEC_Msk) >> FDCAN_PSR_LEC_Pos;

        if (status->state == CAN_BUS_RECOVERING)
        {
            if (!bus_off && !(handle->Instance->CCCR & FDCAN_CCCR_INIT) &&
                handle->Instance->TXBRP == 0U &&
                HAL_FDCAN_GetState(handle) == HAL_FDCAN_STATE_BUSY)
            {
                status->recovery_success_count++;
                status->state = CAN_BUS_RUNNING;
            }
            else if ((uint32_t)(now - status->last_recovery_tick) >= CAN_RECOVERY_TIMEOUT_MS)
            {
                // 超时保留恢复资格；等待退避后再尝试，不能丢弃恢复请求。
                status->recovery_timeout_count++;
                status->last_recovery_tick = now;
                status->state = CAN_BUS_WAIT_RETRY;
            }
        }
        if (status->state == CAN_BUS_WAIT_RETRY &&
            (uint32_t)(now - status->last_recovery_tick) >= CAN_RECOVERY_RETRY_MS)
        {
            status->last_recovery_tick = now;
            status->recovery_attempt_count++;
            HAL_StatusTypeDef result = HAL_OK;
            if (HAL_FDCAN_GetState(handle) == HAL_FDCAN_STATE_READY)
                result = CANStartBus(handle);
            else if (HAL_FDCAN_GetState(handle) == HAL_FDCAN_STATE_BUSY)
            {
                // 丢弃故障前待发帧，避免重连后执行积压的旧命令。
                result = HAL_FDCAN_AbortTxRequest(handle, handle->Instance->TXBRP);
                // M_CAN Bus-Off自动置INIT。软件清INIT后硬件执行总线恢复序列。
                // INIT已为0时不重新置1，避免打断正在进行的恢复。
                if (result == HAL_OK)
                {
                    // 故障前积压的接收帧也不能作为恢复后的新命令。
                    CANDiscardRx(handle, status);
                    CLEAR_BIT(handle->Instance->CCCR, FDCAN_CCCR_INIT);
                }
            }
            else
                result = HAL_ERROR;

            if (result == HAL_OK)
                result = HAL_FDCAN_ActivateNotification(handle, CAN_ACTIVE_ITS, 0);
            if (result == HAL_OK)
                status->state = CAN_BUS_RECOVERING;
            else
                status->recovery_error_count++;
        }
        __set_PRIMASK(primask);

        // 限流打印(任务上下文,非中断):错误类型变化或Bus-Off时,最快每CAN_ERROR_LOG_INTERVAL_MS一次
        if ((lec != can_error_log_last_lec[i] || bus_off) &&
            (uint32_t)(now - can_error_log_last_tick[i]) >= CAN_ERROR_LOG_INTERVAL_MS)
        {
            can_error_log_last_lec[i] = lec;
            can_error_log_last_tick[i] = now;
            LOGWARNING("[bsp_can] CAN%u link err=%s tec=%lu rec=%lu state=%u",
                       (unsigned)(i + 1U), CANLastErrorName(lec),
                       (unsigned long)status->tec, (unsigned long)status->rec,
                       (unsigned)status->state);
        }
    }
#endif
}

/* ----------------two static function called by CANRegister()-------------------- */

/**
 * @brief 添加过滤器以实现对特定id的报文的接收,会被CANRegister()调用
 *        给CAN添加过滤器后,BxCAN会根据接收到的报文的id进行消息过滤,符合规则的id会被填入FIFO触发中断
 *        对于FDCAN，设置使用特定ID模式过滤。
 *
 * @note f407的bxCAN有28个过滤器,这里将其配置为前14个过滤器给CAN1使用,后14个被CAN2使用
 *       初始化时,奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
 *       注册到CAN1的模块使用过滤器0-13,CAN2使用过滤器14-27
 *       FDCAN的消息RAM是所有FDCAN外设共用的。
 *       H723系列FDCAN过滤器数量完全在CubeMX中自定义，因此先做一次检查，再添加即可。
 *
 * @attention 你不需要完全理解这个函数的作用,因为它主要是用于初始化,在开发过程中不需要关心底层的实现
 *            享受开发的乐趣吧!如果你真的想知道这个函数在干什么,请联系作者或自己查阅资料(请直接查阅官方的reference manual)
 *            FDCAN的教程较少，但是添加FDCAN的人已经发了一篇CSDN讲解了，可以参考一下
 *
 * @param _instance can instance owned by specific module
 */
static uint8_t CANAddFilter(CANInstance *_instance)
{

#ifdef FDCAN
	static uint8_t can1_filter_idx = 0, can2_filter_idx = 0 , can3_filter_idx = 0;
	uint8_t *filter_idx_p;

	if(_instance->can_handle==&hfdcan1)
	{
		filter_idx_p=&can1_filter_idx;
	}
	else if(_instance->can_handle==&hfdcan2)
	{
		filter_idx_p=&can2_filter_idx;
	}
	else if(_instance->can_handle==&hfdcan3)
	{
		filter_idx_p=&can3_filter_idx;
	}
	else
	{
		return 0;
	}
	if (*filter_idx_p >= _instance->can_handle->Init.StdFiltersNbr)
		return 0;

	FDCAN_FilterTypeDef fdcan_filter_conf;
	fdcan_filter_conf.FilterIndex=*filter_idx_p;
	//使用单个ID模式
	fdcan_filter_conf.FilterType=FDCAN_FILTER_DUAL;
	fdcan_filter_conf.FilterConfig=(_instance->tx_id & 1) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;//奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
	fdcan_filter_conf.FilterID1=_instance->rx_id;
	fdcan_filter_conf.FilterID2=_instance->rx_id;
	fdcan_filter_conf.IdType=FDCAN_STANDARD_ID;
	fdcan_filter_conf.IsCalibrationMsg=0;
	//fdcan_filter_conf.RxBufferIndex=0;

	if (HAL_FDCAN_ConfigFilter(_instance->can_handle, &fdcan_filter_conf) != HAL_OK)
		return 0;
	(*filter_idx_p)++;

#else
	CAN_FilterTypeDef can_filter_conf = {0};
	static uint8_t can1_filter_idx = 0, can2_filter_idx = 14; // 0-13给can1用,14-27给can2用

	can_filter_conf.FilterMode = CAN_FILTERMODE_IDLIST;                                                       // 使用id list模式,即只有将rxid添加到过滤器中才会接收到,其他报文会被过滤
	can_filter_conf.FilterScale = CAN_FILTERSCALE_16BIT;                                                      // 使用16位id模式,即只有低16位有效
	can_filter_conf.FilterFIFOAssignment = (_instance->tx_id & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;              // 奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
	can_filter_conf.SlaveStartFilterBank = 14;                                                                // 从第14个过滤器开始配置从机过滤器(在STM32的BxCAN控制器中CAN2是CAN1的从机)
	can_filter_conf.FilterIdLow = _instance->rx_id << 5;                                                      // 过滤器寄存器的低16位,因为使用STDID,所以只有低11位有效,高5位要填0
	uint8_t *filter_idx_p = _instance->can_handle == &hcan1 ? &can1_filter_idx : &can2_filter_idx;
	if ((*filter_idx_p >= (_instance->can_handle == &hcan1 ? 14 : 28)) ||
		(_instance->can_handle != &hcan1 && _instance->can_handle != &hcan2))
		return 0;
	can_filter_conf.FilterBank = *filter_idx_p;
	can_filter_conf.FilterIdHigh = can_filter_conf.FilterIdLow;
	can_filter_conf.FilterMaskIdLow = can_filter_conf.FilterIdLow;
	can_filter_conf.FilterMaskIdHigh = can_filter_conf.FilterIdLow;
	can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;                                                     // 启用过滤器

	if (HAL_CAN_ConfigFilter(_instance->can_handle, &can_filter_conf) != HAL_OK)
		return 0;
	(*filter_idx_p)++;
#endif
    return 1;
}

/**
 * @brief 在第一个CAN实例初始化的时候会自动调用此函数,启动CAN服务
 *
 * @note 此函数会启动CAN1并开启中断
 *       FDCAN的情况下，我们采用FIFO接收方式（而不是buffer），FIFO和buffer还有queue接收方式请自行查阅H723手册
 *       FDCAN比bxCAN多了一个全局过滤器，这里配置为全部拒绝，只接受指定ID。
 *       
 */
void CANServiceInit()
{
#ifdef FDCAN
    for (size_t i = 0; i < DEVICE_CAN_CNT; ++i)
    {
        FDCAN_HandleTypeDef *handle = can_handles[i];
        HAL_StatusTypeDef result = CANStartBus(handle);
        if (result == HAL_OK)
            result = HAL_FDCAN_ActivateNotification(handle, CAN_ACTIVE_ITS, 0);
        if (result != HAL_OK)
        {
            can_bus_status[i].error_count++;
            can_error_count++;
            can_bus_status[i].state = CAN_BUS_WAIT_RETRY;
            can_bus_status[i].last_recovery_tick = HAL_GetTick();
        }
    }
    can_service_started = 1;
#else
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
#endif

}

/* ----------------------- two extern callable function -----------------------*/

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    if (config == NULL || config->can_handle == NULL || config->tx_id > 0x7FFU || config->rx_id > 0x7FFU)
        return NULL;
#ifdef FDCAN
    if (CANBusIndex(config->can_handle) < 0)
        return NULL;
#endif
#ifdef FDCAN
    if (!can_service_started)
#else
    if (!idx)
#endif
    {
        CANServiceInit(); // 第一次注册,先进行硬件初始化
        LOGINFO("[bsp_can] CAN Service Init");
    }
    if (idx >= CAN_MX_REGISTER_CNT) // 超过最大实例数
    {
        return NULL;
    }
    for (size_t i = 0; i < idx; i++)
    { // 重复注册 | id重复
        if (can_instance[i]->rx_id == config->rx_id && can_instance[i]->can_handle == config->can_handle)
        {
            return NULL;
        }
    }

    CANInstance *instance = (CANInstance *)malloc(sizeof(CANInstance)); // 分配空间
    if (instance == NULL)
        return NULL;
    memset(instance, 0, sizeof(CANInstance));                           // 分配的空间未必是0,所以要先清空
    // 进行发送报文的配置
#ifdef FDCAN
    instance->txconf.Identifier = config->tx_id; 				// 发送id
    instance->txconf.IdType = FDCAN_STANDARD_ID;  				// 使用标准id,扩展id则使用CAN_ID_EXT(目前没有需求)
    instance->txconf.TxFrameType = FDCAN_DATA_FRAME,    		// 发送数据帧
    instance->txconf.DataLength = FDCAN_DLC_BYTES_8,    		// 数据长度为8字节
	instance->txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE,	// 兼容CAN2.0,错误状态指示器设为主动
	instance->txconf.BitRateSwitch = FDCAN_BRS_OFF,         	// 兼容CAN2.0禁用位速率切换
	instance->txconf.FDFormat = FDCAN_CLASSIC_CAN,          	// 使用经典CAN格式
	instance->txconf.TxEventFifoControl = FDCAN_NO_TX_EVENTS,	// 不需要，禁用事件FIFO
	instance->txconf.MessageMarker = 0;                     	// 不使用消息标记
#else
    instance->txconf.StdId = config->tx_id; // 发送id
    instance->txconf.IDE = CAN_ID_STD;      // 使用标准id,扩展id则使用CAN_ID_EXT(目前没有需求)
    instance->txconf.RTR = CAN_RTR_DATA;    // 发送数据帧
    instance->txconf.DLC = 0x08;            // 默认发送长度为8
#endif
    // 设置回调函数和接收发送id
    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id; // 好像没用,可以删掉
    instance->rx_id = config->rx_id;
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;

    // 先发布回调上下文，再启用过滤器，避免过滤器生效后找不到实例。
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    can_instance[idx] = instance;
    if (!CANAddFilter(instance))
    {
        can_instance[idx] = NULL;
        __set_PRIMASK(primask);
        free(instance);
        return NULL;
    }
    idx++;
    __set_PRIMASK(primask);

    return instance; // 返回can实例指针
}

/* @todo 目前似乎封装过度,应该添加一个指向tx_buff的指针,tx_buff不应该由CAN instance保存 */
/* 如果让CANinstance保存txbuff,会增加一次复制的开销 */
uint8_t CANTransmit(CANInstance *_instance, float timeout)
{
    if (_instance == NULL || _instance->can_handle == NULL)
        return 0;

#ifdef FDCAN
    int bus = CANBusIndex(_instance->can_handle);
    if (bus < 0)
        return 0;
#endif
    float dwt_start = DWT_GetTimeline_ms();
    for (;;)
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
#ifdef FDCAN
        // 恢复期间或硬件已Bus-Off时立即失败，不等待发送FIFO。
        if (can_bus_status[bus].state != CAN_BUS_RUNNING ||
            (_instance->can_handle->Instance->PSR & FDCAN_PSR_BO) ||
            (_instance->can_handle->Instance->CCCR & FDCAN_CCCR_INIT) ||
            HAL_FDCAN_GetState(_instance->can_handle) != HAL_FDCAN_STATE_BUSY)
        {
            _instance->tx_error_count++;
            __set_PRIMASK(primask);
            return 0;
        }
        uint32_t free_level = HAL_FDCAN_GetTxFifoFreeLevel(_instance->can_handle);
#else
        uint32_t free_level = HAL_CAN_GetTxMailboxesFreeLevel(_instance->can_handle);
#endif
        if (free_level != 0U)
        {
            // 资源检查和提交在同一短临界区内，避免其他发送任务抢占。
#ifdef FDCAN
            HAL_StatusTypeDef result = HAL_FDCAN_AddMessageToTxFifoQ(_instance->can_handle, &_instance->txconf, _instance->tx_buff);
#else
            HAL_StatusTypeDef result = HAL_CAN_AddTxMessage(_instance->can_handle, &_instance->txconf, _instance->tx_buff, &_instance->tx_mailbox);
#endif
            if (result != HAL_OK)
                _instance->tx_error_count++;
            __set_PRIMASK(primask);
            return result == HAL_OK; // 仅确认入队，不确认ACK或对端收到完整包
        }
        __set_PRIMASK(primask);
        // 等待时保持原中断状态；故障热路径仅计数，不反复打印日志。
        if (timeout <= 0 || DWT_GetTimeline_ms() - dwt_start >= timeout)
        {
            _instance->tx_error_count++;
            return 0;
        }
    }
}

void CANSetDLC(CANInstance *_instance, uint8_t length)
{
    if (_instance == NULL)
        return;
    // 发送长度错误!检查调用参数是否出错,或出现野指针/越界访问
    if (length > 8 || length == 0) // 安全检查
        return;

#ifdef FDCAN
    _instance->txconf.DataLength = DLC_LookUp_Table[length];
#else
    _instance->txconf.DLC = length;
#endif
}

/* -----------------------belows are callback definitions--------------------------*/

//对于FDCAN，回调函数和处理方式完全不同，因此直接用两套逻辑处理
#ifdef FDCAN
/**
 * @brief 此函数会被下面两个函数调用,用于处理FIFO0和FIFO1新消息中断
 *        所有的实例都会被遍历,找到can_handle和rx_id相等的实例时,调用该实例的回调函数
 *
 * @param _fdhcan
 * @param fifox passed to HAL_CAN_GetRxMessage() to get mesg from a specific fifo
 */
static void FDCANFIFOxCallback(FDCAN_HandleTypeDef *_hfdcan, uint32_t fifox)
{
    int bus = CANBusIndex(_hfdcan);
    if (bus < 0)
        return;
    FDCAN_RxHeaderTypeDef rxconf;
    // HAL按DLC复制，必须先提供足够大的缓存，再拒绝非经典CAN帧。
    uint8_t fdcan_rx_buff[64];
    // 处理进入回调时已存在的帧，避免持续流量让ISR无限循环。
    uint32_t remaining = HAL_FDCAN_GetRxFifoFillLevel(_hfdcan, fifox);
    while (remaining-- != 0U)
    {
        if (HAL_FDCAN_GetRxMessage(_hfdcan, fifox, &rxconf, fdcan_rx_buff) != HAL_OK)
        {
            can_bus_status[bus].rx_drop_count++;
            break;
        }
        uint32_t length = rxconf.DataLength >> 16;
        if (length > 8 || rxconf.FDFormat != FDCAN_CLASSIC_CAN ||
            rxconf.RxFrameType != FDCAN_DATA_FRAME || rxconf.IdType != FDCAN_STANDARD_ID)
        {
            can_bus_status[bus].rx_drop_count++;
            continue;
        }
        for (size_t i = 0; i < idx; ++i)
            if (_hfdcan == can_instance[i]->can_handle && rxconf.Identifier == can_instance[i]->rx_id)
            {
                CANInstance *instance = can_instance[i];
                instance->rx_len = length;
                instance->rx_count++;
                memcpy(instance->rx_buff, fdcan_rx_buff, length);
                if (instance->can_module_callback != NULL)
                    instance->can_module_callback(instance);
                break; // 继续处理FIFO中的下一帧，不提前退出整个回调
            }
    }
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	/* 检查Rx FIFO 0中是否有消息丢失 */
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0)
	{
        int bus = CANBusIndex(hfdcan);
        if (bus >= 0)
            can_bus_status[bus].rx_drop_count++;
	}
	/* 检查是否有新消息写入Rx FIFO 0或到达一定阈值 */
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)||(RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL)||(RxFifo0ITs & FDCAN_IT_RX_FIFO0_WATERMARK))
	{
		FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
	}
}
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	/* 检查Rx FIFO 1中是否有消息丢失 */
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_MESSAGE_LOST) != 0)
	{
        int bus = CANBusIndex(hfdcan);
        if (bus >= 0)
            can_bus_status[bus].rx_drop_count++;
	}
	/* 检查是否有新消息写入Rx FIFO 1或到达一定阈值 */
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)||(RxFifo1ITs & FDCAN_IT_RX_FIFO1_FULL)||(RxFifo1ITs & FDCAN_IT_RX_FIFO1_WATERMARK))
	{
		FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
	}
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    int bus = CANBusIndex(hfdcan);
    if (bus < 0)
        return;
    can_error_count++;
    can_bus_status[bus].error_count++;
    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) && (hfdcan->Instance->PSR & FDCAN_PSR_BO))
        CANMarkBusOff(bus, HAL_GetTick());
}

#else


/**
* @brief 此函数会被下面两个函数调用,用于处理FIFO0和FIFO1溢出中断(说明收到了新的数据)
*        所有的实例都会被遍历,找到can_handle和rx_id相等的实例时,调用该实例的回调函数
*
* @param _hcan
* @param fifox passed to HAL_CAN_GetRxMessage() to get mesg from a specific fifo
*/
static void CANFIFOxCallback(CAN_HandleTypeDef *_hcan, uint32_t fifox)
{
   static CAN_RxHeaderTypeDef rxconf; // 同上
   uint8_t can_rx_buff[8];
   while (HAL_CAN_GetRxFifoFillLevel(_hcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
   {
       HAL_CAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff); // 从FIFO中获取数据
       for (size_t i = 0; i < idx; ++i)
       { // 两者相等说明这是要找的实例
           if (_hcan == can_instance[i]->can_handle && rxconf.StdId == can_instance[i]->rx_id)
           {
               if (can_instance[i]->can_module_callback != NULL) // 回调函数不为空就调用
               {
                   can_instance[i]->rx_len = rxconf.DLC;                      // 保存接收到的数据长度
                   memcpy(can_instance[i]->rx_buff, can_rx_buff, rxconf.DLC); // 消息拷贝到对应实例
                   can_instance[i]->can_module_callback(can_instance[i]);     // 触发回调进行数据解析和处理
               }
               return;
           }
       }
   }
}

/**
* @brief 注意,STM32的两个CAN设备共享两个FIFO
* 下面两个函数是HAL库中的回调函数,他们被HAL声明为__weak,这里对他们进行重载(重写)
* 当FIFO0或FIFO1溢出时会调用这两个函数
*/
// 下面的函数会调用CANFIFOxCallback()来进一步处理来自特定CAN设备的消息

/**
* @brief rx fifo callback. Once FIFO_0 is full,this func would be called
*
* @param hcan CAN handle indicate which device the oddest mesg in FIFO_0 comes from
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
   CANFIFOxCallback(hcan, CAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
}

/**
* @brief rx fifo callback. Once FIFO_1 is full,this func would be called
*
* @param hcan CAN handle indicate which device the oddest mesg in FIFO_1 comes from
*/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
   CANFIFOxCallback(hcan, CAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
}


#endif
