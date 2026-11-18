/**
 *******************************************************************************
 * @file      : DJI_Motor.h
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
#ifndef DJI_MOTOR_H_
#define DJI_MOTOR_H_
/* Includes ------------------------------------------------------------------*/
#include "Motor.h"
#ifdef __cplusplus
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
namespace dji_motor {
    /* Exported types ------------------------------------------------------------*/
    struct MotorCfg {
        CAN_HandleTypeDef* hcan{};
        uint32_t id{};
        float ratio{};
        float offset{};
        DeviceId dev_id{};
    };

    class DJIMotor final : public Motor {
    public:
        DJIMotor(CAN_HandleTypeDef* _hcan, uint32_t id, float ratio = 1.f, float offset = 0.f);
        DJIMotor(CAN_HandleTypeDef* _hcan, uint32_t id, float ratio, float offset, DeviceId dev_id);
        void Callback(uint8_t* data, uint8_t len) override;
        void SetOffset(float offset) { offset_ = offset; }
        [[nodiscard]] float pos() const { return pos_; }
        [[nodiscard]] float vel() const { return vel_; }
        [[nodiscard]] float cur() const { return motor_data_.current; }

    private:
    };
}

#endif

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#endif //DJI_MOTOR_H_
