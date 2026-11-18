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
 *  Copyright (c) 2026 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BOARD_COMM_H_
#define __BOARD_COMM_H_

#include "stdint.h"

#ifdef __cplusplus

/**
 * @brief 底盘模式枚举
 */
enum class ChassisMode : uint8_t;

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#pragma pack(push, 1)
typedef struct
{
  uint8_t cooling_heat;
  uint8_t heat_limit; // 缩小10倍
  uint16_t current_heat;
  int16_t spin_speed;
  uint8_t robot_id;

  uint8_t shooter_output : 1;
  uint8_t cap_online_flag : 1;
  uint8_t wheel_online_flag : 1;
  uint8_t rudder_online_flag : 1;
  uint8_t chassis_referee_online_flag : 1;
  uint8_t reserved : 3;
} send_pack; // 与infantry里的rece_pack参数对应

typedef struct
{
  int16_t x_speed;
  int16_t y_speed;
  uint8_t mode;
  uint8_t shoot_flag : 1;
  uint8_t fric_flag : 1;
  uint8_t aim_flag : 2; // 位宽必须与云台 send_pack 一致
  uint8_t cap_flag : 1; // 0:不使用 1:请求超电（云台当前只发 0/1）
  uint8_t ready_flag : 1;
  uint8_t ui_refresh_flag : 1; // ui刷新
  uint8_t reset_flag : 1;      // 重启标志
  uint16_t reserved;
} rece_pack; // 与infantry里的send_pack参数对应,x_speed与y_speed的值由gimbal上的c板的DT16接收后发送给chassis
#pragma pack(pop)

static_assert(sizeof(send_pack) == 8, "board_comm send_pack must be 8 bytes");
static_assert(sizeof(rece_pack) == 8, "board_comm rece_pack must be 8 bytes");

class BoardComm
{
public:
  void SetCoolingHeat(uint8_t data) { send_.cooling_heat = data; };
  void SetHeatLimit(uint16_t data) { send_.heat_limit = data / 10; };
  void SetCurrentHeat(uint16_t data) { send_.current_heat = data; };
  void SetShooterOutput(uint16_t data) { send_.shooter_output = data; };
  void SetRobotID(uint16_t _data) { send_.robot_id = _data; };
  void SetSpinSpeed(int16_t _data) { send_.spin_speed = _data; };
  int16_t GetXSpeed() { return rece_.x_speed; };
  int16_t GetYSpeed() { return rece_.y_speed; };
  ChassisMode GetChassisMode() { return static_cast<ChassisMode>(rece_.mode); };
  uint8_t GetCapFlag() { return rece_.cap_flag; }
  void Send();
  void Receive(uint8_t *data, uint8_t len);
  void Init(CAN_HandleTypeDef *_phcan, uint16_t _id);
  static void Callback(void *context, uint8_t *data, uint8_t len);
  void Reset();
  bool IsOnline() const;

private:
  CanInstance *p_instance_;
  send_pack send_;
  rece_pack rece_;
  uint32_t last_rx_tick_{0};
};

/* Exported variables --------------------------------------------------------*/
extern BoardComm board_comm;
/* Exported function prototypes ----------------------------------------------*/

#endif

#endif /* __BOARD_COMM_H_ */
