/**
 *******************************************************************************
 * @file      : board_comm.h
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
#ifndef __BOARD_COMM_H_
#define __BOARD_COMM_H_

#include "stdint.h"

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#pragma pack(push,1)
typedef struct {
  int16_t x_speed;
  int16_t y_speed;
  uint8_t mode;
  uint8_t shoot_flag : 1;
  uint8_t fric_flag : 1;
  uint8_t aim_flag : 2;
  uint8_t cap_flag : 1;
  uint8_t ready_flag : 1;
  uint8_t ui_refresh_flag : 1;  //ui刷新
  uint8_t reset_flag : 1;   // 重启标志
  uint16_t reserved;
} send_pack;

typedef struct {
  uint8_t cooling_heat;
  uint8_t heat_limit;
  uint16_t current_heat;
  int16_t spin_speed;
  uint8_t robot_id;

  uint8_t shooter_output : 1;
  uint8_t cap_online_flag : 1;
  uint8_t wheel_online_flag : 1;
  uint8_t rudder_online_flag : 1;
  uint8_t chassis_referee_online_flag : 1;
  uint8_t reserved : 3;
} rece_pack;

#pragma pack(pop)

class BoardComm {
public:
  void SetXSpeed(float _x_speed) { send_.x_speed = (int16_t)_x_speed; };
  void SetYSpeed(float _y_speed) { send_.y_speed = (int16_t)_y_speed; };
  void SetMode(uint8_t _mode) { send_.mode = _mode; };
  void SetFricFlag(uint8_t _flag) { send_.fric_flag = _flag; };
  void SetAimFlag(uint8_t _flag) { send_.aim_flag = _flag; };
  void SetCapFlag(uint8_t _flag) { send_.cap_flag = _flag; };
  void SetReadyFlag(uint8_t _flag) { send_.ready_flag = _flag; };
  uint8_t GetReadyFlag(void) { return send_.ready_flag; };
  uint8_t GetFricFlag(void) { return send_.fric_flag; };
  int16_t GetCoolingHeat() { return rece_.cooling_heat; };
  int16_t GetCoolingLimit() { return rece_.heat_limit * 10; };
  int16_t GetCurrentHeat() { return rece_.current_heat; }
  float GetSpinSpeed() { return rece_.spin_speed * 0.01f; };
  uint8_t GetShooterOutput() { return rece_.shooter_output; };
  uint16_t GetRobotID() { return rece_.robot_id; };
  uint8_t GetGimbalStopFlag() { return gimbal_stop_; };
  void SetGimbalStopFlag(uint8_t _flag) { gimbal_stop_ = _flag; };
  uint8_t GetShootStop() { return shoot_stop_; };
  void SetShootStop(uint8_t _flag) { shoot_stop_ = _flag; };
  void SetResetFlag(uint8_t _flag) { send_.reset_flag = _flag; };
  void SetShootFlag(uint8_t _flag) { send_.shoot_flag = _flag; };
  send_pack* GetSendPack() { return &send_; }
  void Send();
  void Receive();
  void Init(CAN_HandleTypeDef* _phcan, uint16_t _id);

private:
  CanInstance* p_instance_;
  send_pack send_;
  rece_pack rece_;
  uint8_t gimbal_stop_, shoot_stop_;
};

/* Exported variables --------------------------------------------------------*/
extern BoardComm board_comm;
/* Exported function prototypes ----------------------------------------------*/

#endif

#ifdef __cplusplus
extern "C" {
#endif

  uint8_t BoardComm_GetGimbalStopFlag(void);
  uint8_t BoardComm_GetFricFlag(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_COMM_H_ */
