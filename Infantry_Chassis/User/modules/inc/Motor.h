/**
 *******************************************************************************
 * @file      : Motor.h
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
#ifndef MOTOR_H_
#define MOTOR_H_
/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"
#include "detect.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#ifndef PI
#define PI 3.14159265358979323846f
#endif

#ifdef __cplusplus
/* Exported types ------------------------------------------------------------*/
class Motor {
public:
    Motor(CAN_HandleTypeDef* _hcan, uint32_t rx_id, uint32_t tx_id, float ratio, float offset,
          DeviceId dev_id = DeviceId::DEV_NONE);
    virtual ~Motor() = default;
    virtual void Callback(uint8_t* data, uint8_t len) = 0;
    static float RPM_2_RADS(const float rpm) { return rpm * 2 * PI / 60.f; }
    static float RADS_2_RPM(const float rad_s) { return rad_s * 30.f / PI; }

    CanInstance* pmotor_instance_;
protected:
    static void CanCallbackEntry(void* context, uint8_t* data, uint8_t len);

    uint32_t rx_id_, tx_id_;
    DeviceId dev_id_;
    float ratio_;
    float pos_; //rad
    float vel_; //rad/s
    float offset_;
    bool ecd_valid_ = false;
    struct MotorData {
        float ang; //rad
        float ecd_ang;
        float last_ecd_ang; //rad
        float rotate_spd; //rad/s
        float current;
        float torque;
        float temp;
        int16_t round;
    } motor_data_{};
};

#endif

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
#endif //MOTOR_H_
