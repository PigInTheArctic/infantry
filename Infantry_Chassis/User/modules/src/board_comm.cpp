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
#include "string.h"
#include "detect.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
BoardComm board_comm;
/* Private function prototypes -----------------------------------------------*/
namespace
{
constexpr uint32_t kBoardCommOfflineTimeoutMs = 100U; // 云台 20ms 一帧，连续 5 帧丢失判离线
}

void BoardComm::Init(CAN_HandleTypeDef *_phcan, uint16_t _id)
{
  CanInitConf conf{};
  conf.hcan = _phcan;
  conf.rx_id = _id + 1;
  conf.tx_id = _id;
  p_instance_ = pCanRegister(&conf);
  if (p_instance_ == nullptr)
    return;
  p_instance_->context = this;
  p_instance_->pCanCallBack = Callback;
}

void BoardComm::Send()
{
  if (p_instance_ == nullptr)
    return;
  CanSend(p_instance_, (uint8_t *)&send_);
} // 将所需的数据打包成can传输的格式发送给另一块主板

void BoardComm::Receive(uint8_t *data, uint8_t len)
{
  memset(&rece_, 0, sizeof(rece_));
  memcpy(&rece_, data, len > sizeof(rece_) ? sizeof(rece_) : len);
  last_rx_tick_ = HAL_GetTick();
  offline_detect.Feed(DeviceId::DEV_CTRL);
}

void BoardComm::Callback(void *context, uint8_t *data, uint8_t len)
{
  if (context == nullptr)
    return;

  reinterpret_cast<BoardComm *>(context)->Receive(data, len);
}

void BoardComm::Reset()
{
  memset(&rece_, 0, sizeof(rece_));
}

bool BoardComm::IsOnline() const
{
  return (HAL_GetTick() - last_rx_tick_) <= kBoardCommOfflineTimeoutMs;
}