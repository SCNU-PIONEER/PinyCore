#include "Bsp_can.hpp"
#include "Soc.hpp"
#include "StmLog.hpp"

void Can::registerCallback(canHandle *_hcan, uint32_t _stdid,
                           callback _pCallback)
{
    cbTable[_hcan][_stdid] = _pCallback;
}

void Can::unregisterCallback(canHandle *_hcan, uint32_t _stdid)
{
    auto it = cbTable.find(_hcan);
    if (it != cbTable.end()) {
        it->second.erase(_stdid);
        if (it->second.empty()) {
            cbTable.erase(it);
        }
    }
}

#if defined(SOC_FDCAN)

#define DLC(n) FDCAN_DLC_BYTES_##n

struct dlc_table {
    uint8_t dlc;
    uint32_t bytes;
};

const dlc_table dlc[] = {
    { 0, DLC(0) },   { 1, DLC(1) },   { 2, DLC(2) },   { 3, DLC(3) },
    { 4, DLC(4) },   { 5, DLC(5) },   { 6, DLC(6) },   { 7, DLC(7) },
    { 8, DLC(8) },   { 12, DLC(12) }, { 16, DLC(16) }, { 20, DLC(20) },
    { 24, DLC(24) }, { 32, DLC(32) }, { 48, DLC(48) }, { 64, DLC(64) },
};

HAL_StatusTypeDef fdcanFilterInit(FDCAN_HandleTypeDef *_hcan, uint8_t _fifo)
{
    HAL_StatusTypeDef result = HAL_OK;
    /* Configure Rx filter */
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    if (_fifo == RX_FIFO0) {
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        result = HAL_FDCAN_ActivateNotification(
                _hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    } else if (_fifo == RX_FIFO1) {
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
        result = HAL_FDCAN_ActivateNotification(
                _hcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    }
    sFilterConfig.FilterID1 = 0x00000000;
    sFilterConfig.FilterID2 = 0x00000000;
    result = HAL_FDCAN_ConfigFilter(_hcan, &sFilterConfig);
    if (_fifo == RX_FIFO0) {
        result = HAL_FDCAN_ActivateNotification(
                _hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    } else if (_fifo == RX_FIFO1) {
        result = HAL_FDCAN_ActivateNotification(
                _hcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    }
    return result;
}

HAL_StatusTypeDef Can::init(canHandle *_hcan, uint32_t _fifo)
{
    HAL_StatusTypeDef result = HAL_OK;
    result = fdcanFilterInit(_hcan, _fifo);
    result = HAL_FDCAN_ConfigGlobalFilter(
            _hcan,                //fdcan FDCAN_Handle
            FDCAN_REJECT,         //拒绝所有不匹配的标准ID数据帧
            FDCAN_REJECT,         // 拒绝所有不匹配的扩展ID数据帧
            FDCAN_FILTER_REMOTE,  //过滤掉所有标准ID远程帧
            FDCAN_FILTER_REMOTE); //过滤掉所有扩展ID远程帧
    result = HAL_FDCAN_Start(_hcan);
    return result;
}

HAL_StatusTypeDef Can::transmitData(canHandle *_hcan, uint16_t _stdid,
                                    uint8_t *_tx_data, uint32_t _len)
{
    HAL_StatusTypeDef result = HAL_OK;
    FDCAN_TxHeaderTypeDef tx_header = { 0 };
    tx_header.Identifier = _stdid;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = dlc[_len].bytes;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;
    result = HAL_FDCAN_AddMessageToTxFifoQ(_hcan, &tx_header, _tx_data);
    return result;
}

HAL_StatusTypeDef transmitBrsData(canHandle *_hcan, uint16_t _stdid,
                                  uint8_t *_tx_data, uint32_t _len)
{
    HAL_StatusTypeDef result = HAL_OK;
    FDCAN_TxHeaderTypeDef tx_header = { 0 };
    tx_header.Identifier = _stdid;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = dlc[_len].bytes;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_ON;
    tx_header.FDFormat = FDCAN_FD_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;
    result = HAL_FDCAN_AddMessageToTxFifoQ(_hcan, &tx_header, _tx_data);
    return result;
}


inline void Can::callbackFromISR(canHandle *_hcan, uint32_t _rxFifo)
{
    uint8_t rxData[8];
    canHeader rxHeader;
    if (HAL_FDCAN_GetRxMessage(_hcan, _rxFifo, &rxHeader, rxData) != HAL_OK) {
        return;
    }
    auto it = cbTable[_hcan].find(rxHeader.Identifier);
    if (it != cbTable[_hcan].end()) {
        it->second(rxData);
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    Can::instance().callbackFromISR(hfdcan, RX_FIFO0);
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    Can::instance().callbackFromISR(hfdcan, RX_FIFO1);
}

#elif defined(SOC_CAN)
HAL_StatusTypeDef canFilterInit(CAN_HandleTypeDef *_can, uint32_t _fifo)
{
    HAL_StatusTypeDef result = HAL_OK;
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterFIFOAssignment = _fifo;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    if (_can->Instance == CAN1) {
        can_filter_st.FilterBank = 0;
    } else if (_can->Instance == CAN2) {
        can_filter_st.SlaveStartFilterBank = 14;
        can_filter_st.FilterBank = 14;
    }
    result = HAL_CAN_ConfigFilter(_can, &can_filter_st);
    if (_fifo == 0) {
        result =
                HAL_CAN_ActivateNotification(_can, CAN_IT_RX_FIFO0_MSG_PENDING);
    } else if (_fifo == 1) {
        result =
                HAL_CAN_ActivateNotification(_can, CAN_IT_RX_FIFO1_MSG_PENDING);
    }
    return result;
}

HAL_StatusTypeDef Can::init(canHandle *_hcan, uint32_t _fifo)
{
    HAL_StatusTypeDef result = HAL_OK;
    result = canFilterInit(_hcan, _fifo);
    __HAL_CAN_ENABLE_IT(_hcan, CAN_IT_BUSOFF);
    result = HAL_CAN_Start(_hcan);
    return result;
}

HAL_StatusTypeDef Can::transmitData(canHandle *_hcan, uint16_t _stdid,
                                    uint8_t *_txData, uint32_t _len)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t can_mailbox;
    tx_header.DLC = _len;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.StdId = _stdid;
    tx_header.TransmitGlobalTime = DISABLE;
    return HAL_CAN_AddTxMessage(_hcan, &tx_header, _txData, &can_mailbox);
}

void Can::callbackFromISR(canHandle *_hcan, uint32_t _rxFifo)
{
    uint8_t rxData[8];
    canHeader rxHeader;
    if (HAL_CAN_GetRxMessage(_hcan, _rxFifo, &rxHeader, rxData) != HAL_OK) {
        return;
    }
    auto it = cbTable[_hcan].find(rxHeader.StdId);
    if (it != cbTable[_hcan].end()) {
        it->second(rxData);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *_hcan)
{
    Can::instance().callbackFromISR(_hcan, RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *_hcan)
{
    Can::instance().callbackFromISR(_hcan, RX_FIFO1);
}

#endif
