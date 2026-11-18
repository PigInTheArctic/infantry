/**
 *******************************************************************************
 * @file      : gimbal.h
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GIMBAL_H_
#define __GIMBAL_H_

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include "dji_motor.h"
#include "dm_motor.h"
#include "pid.h"
#include "LQR.h"
#include "ADRC.h"
#include "vision.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
 * @brief The Gimbal class represents a gimbal system used for controlling the pitch and yaw positions.
 */
class Gimbal {
public:
    NLESO pitch_eso_, yaw_eso_;
    Lqr gimbal_lqr_;
    Pid angle_[3];          /**< Array of Pid objects for angle control of pitch and yaw. */
    Pid speed_[3];
    Pid acc_[3];
    // DjiMotor motor_[2];   
    DM_MOTOR::DM_Motor motor_[3];  /**< Array of DjiMotor objects for controlling the motors of the gimbal. */
    Pid yaw_spin_compensation;
    /**
     * @brief Controls the gimbal system.
     */
    void Control();

    /**
     * @brief Sets the pitch position of the gimbal.
     * @param _ang The desired pitch angle in degrees.
     */
    void SetPitchPosition(float _ang);
    void SetPitchFollowPosition(float _ang);

    /**
     * @brief Sets the yaw position of the gimbal.
     * @param _ang The desired yaw angle in degrees.
     */
    void SetYawPosition(float _ang);
    void SetVisionYaw(float _ang, float _ang_vel, float _ang_acc);

    void SetVisionPitch(float _ang, float _ang_vel, float _ang_acc);
    void SyncReferenceToCurrent();
    void ResetAutoAimState();
    /**
     * @brief Initializes the PID controllers for angle and speed control.
     */
    void PidInit();
    /**
     * @brief Initializes the motors of the gimbal.
     */
    void MotorInit();

    float output_speed_[3]; /**< Array of output speeds for the motors. */
    float last_output_speed_[3]; /**< Array of output speeds for the motors. */
    float  output_torque_[3];
    float last_output_torque_[3];
    float pitch_shrink_angle = 15.0f;
    float pitch_lifting_angle = 80.0f;

private:
    float set_pitch_;
    float set_yaw_;
    float set_pitch_follow_;

    float set_vpitch_;
    float set_vpitch_follow_;
    float set_vyaw_;
    float set_yaw_acc_;
    float set_pitch_acc_;

    float yaw_torque;
    float pitch_torque;
    float pitch_follow_torque;
    uint8_t last_auto_aim_active_;

    float err_pitch_;
    float err_yaw_;
};
/* Exported variables --------------------------------------------------------*/
extern Gimbal gimbal;
/* Exported function prototypes ----------------------------------------------*/

#endif

#endif /* __GIMBAL_H_ */
