/**
 *******************************************************************************
 * @file      : bsp_can.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_CAN_H_
#define BSP_CAN_H_

#ifdef __cplusplus
extern "C" {
#endif

    /* Includes ------------------------------------------------------------------*/
#include "can.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
 * @brief CAN结构体
 */
    typedef struct {
        CAN_HandleTypeDef* hcan;
        CAN_TxHeaderTypeDef tx_conf;
        uint32_t rx_id;
        uint8_t rx_len;
        uint8_t rx_buff[8];
        void (*pCanCallBack)(void* context, uint8_t* data, uint8_t len);
        void* context;
    } CanInstance;

    /**
     * @brief CAN初始化结构体
     *
     * 包含使用的can总线指针，发送id，接受id，特殊回调函数
     */
    typedef struct {
        CAN_HandleTypeDef* hcan;
        uint32_t tx_id;
        uint32_t rx_id;

        void (*pCanCallBack)(void* context, uint8_t* data, uint8_t len);
        void* context;
    } CanInitConf;

    /* Exported variables --------------------------------------------------------*/
    /* Exported function prototypes ----------------------------------------------*/

    CanInstance* pCanRegister(const CanInitConf* _pconf);

    HAL_StatusTypeDef CanSend(const CanInstance* _pinstance, const uint8_t* _ptx_buff);

    void CanSetDlcAndRtr(CanInstance* _pinstance, uint8_t _length);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_CAN_H_ */
