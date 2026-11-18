/**
 *******************************************************************************
 * @file      : bsp_can.c
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V1.1.0      2026-5-23      origincy        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2026 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
 /* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"
#include "memory.h"
#include "stdlib.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t idx;
static CanInstance* pcan_instance[16] = { NULL };

/* External variables --------------------------------------------------------*/
/* Exported function declarations --------------------------------------------*/

/**
 * @brief 使用指定的初始化配置注册 CAN 实例
 * @note  此函数用于使用提供的初始化配置注册 CAN 实例
 *          仅支持11位标准帧
 *
 * @param _pconf 指向包含初始化配置的 CanInitConf 结构的指针
 * @return 指向表示已注册 CAN 实例的 CanInstance 结构的指针
 */
CanInstance* pCanRegister(const CanInitConf* _pconf) {
    //不合规的注册请求会卡在死循环
    if (idx >= 16)
        return NULL;

    for (uint8_t i = 0; i < 16; i++) {
        if (pcan_instance[i] != NULL && pcan_instance[i]->rx_id == _pconf->rx_id && pcan_instance[i]->hcan == _pconf->
            hcan) {
            return NULL;
        }
    }

    CanInstance* p_instance = (CanInstance*)malloc(sizeof(CanInstance));
    memset(p_instance, 0, sizeof(CanInstance));

    p_instance->tx_conf.StdId = _pconf->tx_id; //设置接受帧消息的ID
    p_instance->tx_conf.IDE = CAN_ID_STD; //标准ID
    p_instance->tx_conf.RTR = CAN_RTR_DATA; //数据帧
    p_instance->tx_conf.DLC = 0x08; //发送数据长度

    p_instance->hcan = _pconf->hcan;
    p_instance->rx_id = _pconf->rx_id;
    p_instance->pCanCallBack = _pconf->pCanCallBack;
    p_instance->context = _pconf->context;

    pcan_instance[idx++] = p_instance;

    return p_instance;
}

/**
 * @brief 发送CAN消息
 *
 * 此函数使用指定的 CAN 实例发送 CAN 消息并传输缓冲区
 *
 * @param _pinstance 指向 CAN 实例的指针
 * @param _ptx_buff 指向传输缓冲区的指针
 * @return HAL_StatusTypeDef 状态值
 */
HAL_StatusTypeDef CanSend(const CanInstance* _pinstance, const uint8_t* _ptx_buff) {
    if (_pinstance == NULL || _pinstance->hcan->Instance == NULL) {
        return HAL_ERROR;
    }

    uint32_t mailbox = 0;

    if (HAL_CAN_GetTxMailboxesFreeLevel(_pinstance->hcan) == 0U) {
        return HAL_BUSY;
    }

    return HAL_CAN_AddTxMessage(_pinstance->hcan, &_pinstance->tx_conf, _ptx_buff, &mailbox);
}

/**
 * @brief 设置CAN数据长度和RTR标志位
 *
 * @param _pinstance CAN实例指针
 * @param _length 数据长度
 */
void CanSetDlcAndRtr(CanInstance* _pinstance, const uint8_t _length) {
    _pinstance->tx_conf.DLC = _length;
}

/**
 * @brief CAN接收中断的回调函数
 *
 * 当CAN接收中断发生时，调用此函数
 * @param _phcan 指向CAN结构体的指针
 * @param _Fifo 与中断相关的FIFO编号
 */
static void CanRxCallBack(CAN_HandleTypeDef* _phcan, uint32_t _Fifo) {
    static CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_buff[8];
    while (HAL_CAN_GetRxFifoFillLevel(_phcan, _Fifo)) {
        HAL_CAN_GetRxMessage(_phcan, _Fifo, &rx_header, rx_buff);
        for (uint8_t i = 0; i < 16; i++) {
            if (pcan_instance[i] == NULL)
                continue;
            if (rx_header.StdId == pcan_instance[i]->rx_id && _phcan == pcan_instance[i]->hcan) {
                /* 缓存到实例，供 Update() 等周期解析函数读取 */
                pcan_instance[i]->rx_len = rx_header.DLC;
                memcpy(pcan_instance[i]->rx_buff, rx_buff, rx_header.DLC);
                if (pcan_instance[i]->pCanCallBack != NULL)
                    pcan_instance[i]->pCanCallBack(pcan_instance[i]->context,
                                                   rx_buff, rx_header.DLC);
            }
        }
    }
}

/**
 * @brief 当RxFIFO0中的消息处于挂起状态时，会调用回调函数
 * @param hcan 指向包含指定CAN的配置信息的 CAN_HandleTypeDef 结构的指针。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    CanRxCallBack(hcan, CAN_RX_FIFO0);
}
