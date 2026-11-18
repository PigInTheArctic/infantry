/**
 *******************************************************************************
 * @file      : LQR.h
 * @brief     : 云台特供版
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-09-11     origincy        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2026 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LQR_H_
#define __LQR_H_

/* Includes ------------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class Lqr {
public:
    void Calc();
    void SetData(const float pitch_ang, const float pitch_vel,
                 const float yaw_ang, const float yaw_vel, const float dt) {
        dt_ = dt;
        pitch_ang_ = pitch_ang;
        pitch_vel_ = pitch_vel;
        yaw_ang_ = yaw_ang;
        yaw_vel_ = yaw_vel;
    }
    void SetPitchAngle(const float pitch_ang_ref) { pitch_ang_ref_ = pitch_ang_ref; }
    void SetPitchVel(const float pitch_vel_ref) { pitch_vel_ref_ = pitch_vel_ref; }
    void SetYawAngle(const float yaw_ang_ref) { yaw_ang_ref_ = yaw_ang_ref; }
    void SetYawVel(const float yaw_vel_ref) { yaw_vel_ref_ = yaw_vel_ref; }
    [[nodiscard]] float GetYawOutput() { return output_[0]; }
    [[nodiscard]] float GetPitchOutput() { return output_[1]; }

private:
    float dt_;
    float pitch_ang_, pitch_vel_;
    float yaw_ang_, yaw_vel_;
    float pitch_ang_ref_, pitch_vel_ref_;
    float yaw_ang_ref_, yaw_vel_ref_;
    float pitch_integral_, yaw_integral_;
    float output_[2];
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#endif /* __LQR_H_ */
