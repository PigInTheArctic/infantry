/**
 *******************************************************************************
 * @file      : chassis.h
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
#ifndef __CHASSIS_H_
#define __CHASSIS_H_

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum Chassis_Mode {
  GIMBAL_DISCONNECT = 0,
  FOLLOW_GIMBAL_YAW = 0x0B,
  RUDDER_FOLLOW_GIMBAL_YAW = 0x09,
  SPIN = 0x05,
  NO_MOVE = 0x0A,
  ZERO = 0xAA,
  FOLLOW_GIMBAL_YAW_SLANT = 0x0F,   //底盘斜向跟随
  RUDDER_FOLLOW_GIMBAL_YAW_SLANT = 0x0D   //舵斜向跟随
}Chassis_Mode;

class Chassis {
public:
  void Init();
  void Control();
  void SetYawAngle(int16_t _ang) { ang_yaw_ = _ang; };
  void SetXSpeed(float _spd) { x_target_ = _spd; };
  void SetYSpeed(float _spd) { y_target_ = _spd; };
  void SetWSpeed(float _spd) { w_target_ = _spd; };
  void SetMode(Chassis_Mode _mode) { mode_ = _mode; };
private:
  Pid follow_;
  int16_t ang_yaw_;
  float x_spd_, y_spd_, w_bias;
  float x_target_, y_target_, w_target_;
  Chassis_Mode mode_;
};
/* Exported variables --------------------------------------------------------*/
extern Chassis chassis;
/* Exported function prototypes ----------------------------------------------*/

#endif

#endif /* __CHASSIS_H_ */
