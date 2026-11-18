/**
 *******************************************************************************
 * @file      : board_comm.cpp
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
 /* Includes ------------------------------------------------------------------*/
#include "board_comm.h"

#include "offline_detect.h"
#include "string.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
BoardComm board_comm;
/* Private function prototypes -----------------------------------------------*/
static void BoardCommCallback();

void BoardComm::Init(CAN_HandleTypeDef* _phcan, uint16_t _id) {
  CanInitConf conf;
  conf.hcan = _phcan;
  conf.rx_id = _id;
  conf.tx_id = _id + 1;
  p_instance_ = pCanRegister(&conf);
  p_instance_->pCanCallBack = BoardCommCallback;
}

void BoardComm::Send() {
  CanSend(p_instance_, (uint8_t*)&send_);
}

void BoardComm::Receive() {
  memcpy(&rece_, p_instance_->rx_buff, 8);
  rece_.cooling_heat = rece_.cooling_heat < 5 ? 5 : rece_.cooling_heat;
  rece_.heat_limit = rece_.heat_limit < 40 ? 40 : rece_.heat_limit;
  offline_detect.UpdateFromFlag(DeviceId::DEV_CAP, rece_.cap_online_flag);
  offline_detect.UpdateFromFlag(DeviceId::DEV_WHEEL, rece_.wheel_online_flag);
  offline_detect.UpdateFromFlag(DeviceId::DEV_RUDDER, rece_.rudder_online_flag);
  offline_detect.UpdateFromFlag(DeviceId::DEV_CHASSIS_REFEREE,
                                rece_.chassis_referee_online_flag);
}

static void BoardCommCallback() {
  offline_detect.Feed(DeviceId::DEV_BOARD_COMM);
  board_comm.Receive();
}

extern "C" uint8_t BoardComm_GetGimbalStopFlag(void) {
  return board_comm.GetGimbalStopFlag();
}//为C文件提供对外接口

extern "C" uint8_t BoardComm_GetFricFlag(void) {
  return board_comm.GetFricFlag();
}
//此文件其实类似于一个控制中心，存放了各种整体的标志位便于两块开发板都受到裁判系统控制，同时其中也存放板间通讯