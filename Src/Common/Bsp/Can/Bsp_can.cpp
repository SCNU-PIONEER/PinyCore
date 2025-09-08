#include "Bsp_can.hpp"
#include "Soc.hpp"

extern canHandle HCAN1;
extern canHandle HCAN2;
#if SOC_CAN_NUM == 3
extern canHandle HCAN3;
#endif

HAL_StatusTypeDef Can::init()
{
    HAL_StatusTypeDef result = HAL_OK;
    result = initSelf(&HCAN1, RX_FIFO0);
    result = initSelf(&HCAN2, RX_FIFO1);
#ifdef HCAN3
    extern canHandle HCAN3;
    result = initSelf(&HCAN3, RX_FIFO0);
#endif
    return result;
}


HAL_StatusTypeDef Can::registerCallback(canHandle *_hcan, uint32_t _stdid,
                                        callback _pCallback)
{
    struct Handler_s temp = { .stdid = _stdid, .func = _pCallback };
    if (_hcan == &HCAN1) {
        if (can1cnt < MAX_RECV_DEVICE) {
            cbTable1[can1cnt] = temp;
            can1cnt++;
            return HAL_OK;
        } else {
            return HAL_ERROR;
        }
    } else if (_hcan == &HCAN2) {
        if (can2cnt < MAX_RECV_DEVICE) {
            cbTable2[can2cnt] = temp;
            can2cnt++;
            return HAL_OK;
        } else {
            return HAL_ERROR;
        }
#if SOC_CAN_NUM == 3
    } else if (_hcan == &HCAN3) {
        if (can3cnt < MAX_RECV_DEVICE) {
            cbTable3[can3cnt] = temp;
            can3cnt++;
            return HAL_OK;
        } else {
            return HAL_ERROR;
        }
#endif
    }
    return HAL_ERROR;
}

void Can::unregisterCallback(canHandle *_hcan, uint32_t _stdid)
{
    if (_hcan == &HCAN1) {
        for (uint8_t i = 0; i < can1cnt; i++) {
            if (cbTable1[i].stdid == _stdid) {
                cbTable1[i].func = nullptr;
                break;
            }
        }
    } else if (_hcan == &HCAN2) {
        for (uint8_t i = 0; i < can2cnt; i++) {
            if (cbTable2[i].stdid == _stdid) {
                cbTable2[i].func = nullptr;
                break;
            }
        }
#if SOC_CAN_NUM == 3
    } else if (_hcan == &HCAN3) {
        for (uint8_t i = 0; i < can3cnt; i++) {
            if (cbTable3[i].stdid == _stdid) {
                cbTable3[i].func = nullptr;
                break;
            }
        }
#endif
    }
}

#if defined(SOC_FDCAN)

#define DLC(n) FDCAN_DLC_BYTES_##n

struct DlcTable_s {
    uint8_t dlc;
    uint32_t bytes;
};

const DlcTable_s dlc[] = {
    { .dlc = 0, .bytes = DLC(0) },   { .dlc = 1, .bytes = DLC(1) },
    { .dlc = 2, .bytes = DLC(2) },   { .dlc = 3, .bytes = DLC(3) },
    { .dlc = 4, .bytes = DLC(4) },   { .dlc = 5, .bytes = DLC(5) },
    { .dlc = 6, .bytes = DLC(6) },   { .dlc = 7, .bytes = DLC(7) },
    { .dlc = 8, .bytes = DLC(8) },   { .dlc = 12, .bytes = DLC(12) },
    { .dlc = 16, .bytes = DLC(16) }, { .dlc = 20, .bytes = DLC(20) },
    { .dlc = 24, .bytes = DLC(24) }, { .dlc = 32, .bytes = DLC(32) },
    { .dlc = 48, .bytes = DLC(48) }, { .dlc = 64, .bytes = DLC(64) },
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

HAL_StatusTypeDef Can::initSelf(canHandle *_hcan, uint32_t _fifo)
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
    FDCAN_TxHeaderTypeDef txHeader = { 0 };
    txHeader.Identifier = _stdid;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = dlc[_len].bytes;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;
    result = HAL_FDCAN_AddMessageToTxFifoQ(_hcan, &txHeader, _tx_data);
    return result;
}

HAL_StatusTypeDef transmitBrsData(canHandle *_hcan, uint16_t _stdid,
                                  uint8_t *_tx_data, uint32_t _len)
{
    HAL_StatusTypeDef result = HAL_OK;
    FDCAN_TxHeaderTypeDef txHeader = { 0 };
    txHeader.Identifier = _stdid;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = dlc[_len].bytes;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_ON;
    txHeader.FDFormat = FDCAN_FD_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;
    result = HAL_FDCAN_AddMessageToTxFifoQ(_hcan, &txHeader, _tx_data);
    return result;
}


inline void Can::callbackFromISR(canHandle *_hcan, uint32_t _rxFifo)
{
    uint8_t rxData[8];
    canHeader rxHeader;
    if (HAL_FDCAN_GetRxMessage(_hcan, _rxFifo, &rxHeader, rxData) != HAL_OK) {
        return;
    }

    if (_hcan == &HCAN1) {
        for (uint8_t i = 0; i < can1cnt; i++) {
            if (cbTable1[i].stdid == rxHeader.Identifier) {
                cbTable1[i].func(rxData);
                break;
            }
        }
    } else if (_hcan == &HCAN2) {
        for (uint8_t i = 0; i < can2cnt; i++) {
            if (cbTable2[i].stdid == rxHeader.Identifier) {
                cbTable2[i].func(rxData);
                break;
            }
        }
    } else if (_hcan == &HCAN3) {
        for (uint8_t i = 0; i < can3cnt; i++) {
            if (cbTable3[i].stdid == rxHeader.Identifier) {
                cbTable3[i].func(rxData);
                break;
            }
        }
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *_hfdcan,
                               uint32_t _rxFifo0ITs)
{
    Can::instance().callbackFromISR(_hfdcan, RX_FIFO0);
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *_hfdcan,
                               uint32_t _rxFifo1ITs)
{
    Can::instance().callbackFromISR(_hfdcan, RX_FIFO1);
}

#elif defined(SOC_CAN)
HAL_StatusTypeDef canFilterInit(CAN_HandleTypeDef *_can, uint32_t _fifo)
{
    HAL_StatusTypeDef result = HAL_OK;
    CAN_FilterTypeDef canFilter;
    canFilter.FilterActivation = ENABLE;
    canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilter.FilterFIFOAssignment = _fifo;
    canFilter.FilterIdHigh = 0x0000;
    canFilter.FilterIdLow = 0x0000;
    canFilter.FilterMaskIdHigh = 0x0000;
    canFilter.FilterMaskIdLow = 0x0000;
    if (_can == &HCAN1) {
        canFilter.FilterBank = 0;
    } else if (_can == &HCAN2) {
        canFilter.SlaveStartFilterBank = 14;
        canFilter.FilterBank = 14;
    }
    result = HAL_CAN_ConfigFilter(_can, &canFilter);
    if (_fifo == 0) {
        result =
                HAL_CAN_ActivateNotification(_can, CAN_IT_RX_FIFO0_MSG_PENDING);
    } else if (_fifo == 1) {
        result =
                HAL_CAN_ActivateNotification(_can, CAN_IT_RX_FIFO1_MSG_PENDING);
    }
    return result;
}

HAL_StatusTypeDef Can::initSelf(canHandle *_hcan, uint32_t _fifo)
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
    CAN_TxHeaderTypeDef txHeader;
    uint32_t canMailbox;
    txHeader.DLC = _len;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.StdId = _stdid;
    txHeader.TransmitGlobalTime = DISABLE;
    return HAL_CAN_AddTxMessage(_hcan, &txHeader, _txData, &canMailbox);
}

void Can::callbackFromISR(canHandle *_hcan, uint32_t _rxFifo)
{
    uint8_t rxData[8];
    canHeader rxHeader;
    if (HAL_CAN_GetRxMessage(_hcan, _rxFifo, &rxHeader, rxData) != HAL_OK) {
        return;
    }
    if (_hcan == &HCAN1) {
        for (uint8_t i = 0; i < can1cnt; i++) {
            if (cbTable1[i].stdid == rxHeader.StdId) {
                cbTable1[i].func(rxData);
                break;
            }
        }
    } else if (_hcan == &HCAN2) {
        for (uint8_t i = 0; i < can2cnt; i++) {
            if (cbTable2[i].stdid == rxHeader.StdId) {
                cbTable2[i].func(rxData);
                break;
            }
        }
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
