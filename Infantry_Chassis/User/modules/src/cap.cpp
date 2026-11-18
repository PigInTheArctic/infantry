
/**
 *******************************************************************************
 * @file      : Super_Cap.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-01-13      origincy        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
 /* Includes ------------------------------------------------------------------*/
#include "cap.h"
#include "string.h"  
#include "math.h"
// #include "PowerRLS.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/

CAP_t cap;
/* Private function prototypes -----------------------------------------------*/

void CAP_t::Init(CAN_HandleTypeDef* _p_hcan, uint16_t _send_id, uint16_t _recv_id) {
    CanInitConf conf;
    conf.hcan = _p_hcan;
    conf.tx_id = _send_id;
    conf.rx_id = _recv_id;
    conf.context = this;
    conf.pCanCallBack = CanCallbackEntry;
    p_instance_ = pCanRegister(&conf);
}

void CAP_t::Callback(uint8_t* data, uint8_t len) {
    memcpy(&data_.cap_energy, data, 2);
    memcpy(&data_.input_power, data + 2, 2);
    memcpy(&data_.output_power, data + 4, 2);

    data_.cap_volt = sqrtf(data_.cap_energy * 0.45454545f);
}

/**
 * @brief 控制超电控制板的期望功率和开关
 */
void CAP_t::SendData() {
    uint8_t tx[8]{};
    tx[0] = power_set_ & 0xFF;
    tx[1] = power_set_ >> 8;
    tx[2] = CapEnableFlag_ & 0xFF;
    tx[3] = CapEnableFlag_ >> 8;

    CanSend(p_instance_, tx);
}

void CAP_t::SetCapState() {
    if (data_.cap_energy < 1300) {
        data_.cap_state = 0;
    }
    else if (data_.cap_energy > 1400) {
        data_.cap_state = 1;
    }
}

void CAP_t::CanCallbackEntry(void* context, uint8_t* data, uint8_t len) {
    if (context == nullptr) return;

    reinterpret_cast<CAP_t*>(context)->Callback(data, len);
}