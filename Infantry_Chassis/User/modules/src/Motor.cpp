/**
 *******************************************************************************
 * @file      : Motor.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026/03/27        origincy        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2026 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/
 /* Includes ------------------------------------------------------------------*/
#include "Motor.h"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/

Motor::Motor(CAN_HandleTypeDef* _hcan, const uint32_t rx_id, const uint32_t tx_id, const float ratio,
             const float offset, DeviceId dev_id) :
    pmotor_instance_(nullptr),
    rx_id_(rx_id),
    tx_id_(tx_id),
    dev_id_(dev_id),
    ratio_(ratio),
    offset_(offset),
    motor_data_{} {
    CanInitConf config{};
    config.rx_id = rx_id_;
    config.tx_id = tx_id_;
    config.hcan = _hcan;
    config.pCanCallBack = &Motor::CanCallbackEntry;
    config.context = this;
    pmotor_instance_ = pCanRegister(&config);
}

void Motor::CanCallbackEntry(void* context, uint8_t* data, uint8_t len) {
    if (context == nullptr) return;

    reinterpret_cast<Motor*>(context)->Callback(data, len);
}
