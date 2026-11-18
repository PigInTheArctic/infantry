/**
 *******************************************************************************
 * @file      : dm_motor.h
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2026-01-28      origincy        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2026 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DM_MOTOR_H_
#define __DM_MOTOR_H_

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"
/* Exported macro ------------------------------------------------------------*/
#define RPM_2_ANGLE_PER_SEC 6.0f        // ×360°/60sec
#ifndef RPM_2_RAD_PER_SEC 
#define RPM_2_RAD_PER_SEC 0.10471976f  // ×2pi/60sec
#endif
#define RAD_PER_SEC_2_RPM 9.549296f  // ×2pi/60sec
#define RAD_2_DEG 57.29578f           // 180/pi
#define DEG_2_RAD 0.017453292f           // pi/180

namespace {

#define   P_MIN     (-5.23599f)   //位置
#define   P_MAX     (5.23599f)
#define   V_MIN     (-30.0f)   //速度
#define   V_MAX     (30.0f)
#define   KP_MIN    (0.0f)   //位置环的比例系数
#define   KP_MAX    (500.0f)
#define   KD_MIN    (0.0f)    //速度环的比例系数
#define   KD_MAX    (5.0f)
#define   T_MIN     (-10.0f)   //扭矩
#define   T_MAX     (10.0f)

}
#ifndef PI 
#define PI 3.14159265358979f
#endif

/* Exported constants --------------------------------------------------------*/
namespace DM_MOTOR {
    typedef enum Ctrl_Mode {
        MIT = 0,
        POSITION,
        SPEED,
        DM_1_to_4
    }Ctrl_Mode;


    typedef enum Err_t {
        DISENABLE = 0x0,
        ENABLE = 0x1,
        OVERVOLT = 0x8,
        UNDERVOLT = 0x9,
        OVER_CURRENT = 0xA,
        MOS_OVERHEAT = 0xB,
        COIL_OVERHEAT = 0xC,
        DISCONNECT = 0xD,
        OVERLOAD = 0xE
    }Err_t;
    /* Exported types ------------------------------------------------------------*/

    class DM_Motor {
    public:
        CanInstance* pdm_motor_instance = nullptr;
        void Update();
        void Init(uint32_t _rx_id, uint32_t _tx_id, CAN_HandleTypeDef* _phcan, Ctrl_Mode _mode = Ctrl_Mode::MIT);
        float GetAngle() {
            return ang_real_;
        }
        float GetEcd() {
            return ang_;
        }
        float GetSpd_rpm() {
            return spd_rpm_;
        }
        float GetSpd_rad() {
            return spd_rad_;
        }
        void SetOffest(float _offest) {
            ang_offest_ = _offest;
        }
        void Send1to4(int16_t _current);
        void SendMIT(float _pos, float _vel, float _Kp, float _Kd, float _torq);

        void MotorEnable();
        void MotorDisable();
        void MotorClearError();
    private:
        Ctrl_Mode mode_;
        uint8_t err_;
        float spd_rpm_;
        float spd_rad_;
        int16_t cur_;
        float tor_;
        int16_t temp_;

        int16_t round_cnt_;
        float ang_;
        float last_ang_;
        float ang_real_;
        bool angle_initialized_;

        float ang_offest_ = 0;
    };
    /* Exported variables --------------------------------------------------------*/
    /* Exported function prototypes ----------------------------------------------*/
}

#endif

#endif /* __DJI_MOTOR_H_ */
