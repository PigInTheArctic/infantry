/**
 *******************************************************************************
 * @file      : shoot.h
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
#ifndef __SHOOT_H_
#define __SHOOT_H_

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include "dji_motor.h"
#include "dm_motor.h"
#include "pid.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
typedef enum {
  ANGLE_FLAG = 0,
  CONTINUE_SPEED_FLAG = 1,
  SIGNAL_SPEED_FLAG = 2,
} Shoot_Flag;

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Class representing the shooting mechanism.
 */
class Shoot {
public:
  float fric_output_[2];   /**< Array of friction motors' output values. */
  float trig_output_;      /**<  motor's output value. */
  DjiMotor fric_motor_[2]; /**< Array of friction motors. */
  DM_MOTOR::DM_Motor trigger_motor_; /**< Trigger motor. */

  // int16_t heat_remain_;
  /**
   * @brief Control function for the shooting mechanism.
   */
  void Control();

  /**
   * @brief Calculate the output for the friction motors.
   */
  void FricCalc();

  /**
   * @brief Calculate the output for the trigger motor.
   */
  void TrigCalc();

  /**
   * @brief Set the desired speed for the friction motors.
   * @param _speed The desired speed.
   */
  void SetFricSpeed(float _speed);

  /**
   * @brief Set the desired speed for the trigger motor.
   * @param _speed The desired speed.
   */
  void SetTriggerSpeed(float _speed);

  /**
   * @brief Set the desired position for the trigger motor.
   * @param _pos The desired position.
   */
  void SetTriggerPos(float _pos);

  /**
   * @brief Initialize the PID controllers.
   */
  void PidInit();

  /**
   * @brief Calculate shoot frequency through the heat limit and real-time heat
   * transmitted by the chassis referee system
   */
  void HeatClosedLoop();

  /**
   * @brief Initialize the motor objects.
   */
  void MotorInit();

  /**
   * @brief Sets the flag value.
   *
   * This function sets the flag value to the specified value.
   *
   * @param _flag The flag value to be set.
   */
  void SetShootFlag(uint8_t _flag);
  uint8_t JudgeBreakFlag();

public:
  Pid fric_speed_[2]; /**< Array of PID controllers for friction motors' speed
                         control. */
  Pid trigger_pos_,
    trigger_speed_;  /**< PID controller for trigger position control. */
  uint8_t shoot_flag_; /**< Flag indicating the shooting state. */
  float target_ang_, target_vel_;  // rad,rad/s

};

/* Exported variables --------------------------------------------------------*/
extern Shoot shoot;
/* Exported function prototypes ----------------------------------------------*/

#endif

#endif /* __SHOOT_H_ */
