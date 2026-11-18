/**
 *******************************************************************************
 * @file      : DJI_Motor.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026/04/25        origincy        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2026 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/
 /* Includes ------------------------------------------------------------------*/
#include "DJI_Motor.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

namespace dji_motor {
    DJIMotor::DJIMotor(CAN_HandleTypeDef* _hcan, uint32_t id, float ratio, float offset) :
        Motor(_hcan, id, 0, ratio, offset) {}
    DJIMotor::DJIMotor(CAN_HandleTypeDef* _hcan, uint32_t id, float ratio, float offset, DeviceId dev_id) :
        Motor(_hcan, id, 0, ratio, offset, dev_id) {}
    void DJIMotor::Callback(uint8_t* data, uint8_t len) {
        motor_data_.ecd_ang = static_cast<int16_t>(data[0] << 8 | data[1]);
        motor_data_.rotate_spd = RPM_2_RADS(static_cast<float>(static_cast<int16_t>(data[2] << 8 | data[3])));
        motor_data_.current = static_cast<int16_t>(data[4] << 8 | data[5]);
        motor_data_.temp = data[6];

        motor_data_.ang = motor_data_.ecd_ang / 4096.f * PI - PI - offset_;
        if (ecd_valid_) {
            if (motor_data_.last_ecd_ang - motor_data_.ecd_ang > 4096)
                motor_data_.round++;
            if (motor_data_.last_ecd_ang - motor_data_.ecd_ang < -4096)
                motor_data_.round--;
        }
        else
            ecd_valid_ = true;
        motor_data_.ang = motor_data_.ang + static_cast<float>(motor_data_.round * 2) * PI;

        pos_ = motor_data_.ang / ratio_;
        vel_ = motor_data_.rotate_spd / ratio_;

        motor_data_.last_ecd_ang = motor_data_.ecd_ang;
        if (dev_id_ != DeviceId::DEV_NONE)
            offline_detect.Feed(this->dev_id_);
    }
}
