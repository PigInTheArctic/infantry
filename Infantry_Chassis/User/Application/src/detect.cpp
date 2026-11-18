/**
 *******************************************************************************
 * @file      : detect.cpp
 * @brief     : 力控舵轮底盘 — 双舵轮（RF+LB）+ 两全向轮
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-05-26      origincy        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2026 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
/** Includes ------------------------------------------------------------------*/
#include "detect.h"
#include "bsp_dwt.h"
/** Private macro -------------------------------------------------------------*/
/** Private constants ---------------------------------------------------------*/
/** Private types -------------------------------------------------------------*/
/** Private variables ---------------------------------------------------------*/
/** External variables --------------------------------------------------------*/
/** Exported variables --------------------------------------------------------*/
Offline_Detect offline_detect;
/** Private function prototypes -----------------------------------------------*/

void Detect_Init()
{
    offline_detect.Init(DeviceId::DEV_Steer_LB, 200);
    offline_detect.Init(DeviceId::DEV_Steer_RF, 200);
    offline_detect.Init(DeviceId::DEV_Wheel_LB, 200);
    offline_detect.Init(DeviceId::DEV_Wheel_RF, 200);
    offline_detect.Init(DeviceId::DEV_CAP, 500);
    offline_detect.Init(DeviceId::DEV_Referee, 500);
    offline_detect.Init(DeviceId::DEV_YAW_MOTOR, 200);
    offline_detect.Init(DeviceId::DEV_CTRL, 100); // 云台板间通信
}

void Detect_Task()
{
    offline_detect.Update();
}

void Offline_Detect::Init(DeviceId id, uint16_t timeout)
{
    EnableList[static_cast<uint8_t>(id)] = true;
    /* 初始化时视为在线，避免任务启动立即离线 */
    offline_item[static_cast<uint8_t>(id)].state = offline_state::Online;
    offline_item[static_cast<uint8_t>(id)].last_online_ms = DWT_GetTimeline_ms();
    offline_item[static_cast<uint8_t>(id)].timeout_ms = timeout;
}

void Offline_Detect::Update(void)
{
    float now = DWT_GetTimeline_ms();
    for (uint8_t i = 0; i < DevNum; i++)
    {
        if (EnableList[i] == true)
        {
            /* 超时未喂狗则判离线，否则判在线 */
            if ((now - offline_item[i].last_online_ms) > offline_item[i].timeout_ms)
                offline_item[i].state = offline_state::Offline;
            else
                offline_item[i].state = offline_state::Online;
        }
        else
        {
            /* 未启用检测的设备保持Ignore状态，不参与判断 */
            offline_item[i].state = offline_state::Ignore;
        }
    }
}

void Offline_Detect::Feed(DeviceId id)
{
    offline_item[static_cast<uint8_t>(id)].last_online_ms = DWT_GetTimeline_ms();
    offline_item[static_cast<uint8_t>(id)].state = offline_state::Online;
}

/**
 * @brief 获取设备在线状态
 * @note 对未启用检测的设备，默认按在线处理
 * @param id 设备id
 * @retval true 在线或未启用检测
 * @retval false 已启用检测且当前离线
 */
bool Offline_Detect::IsOnline(DeviceId id)
{
    if (offline_item[static_cast<uint8_t>(id)].state != offline_state::Ignore)
        return offline_item[static_cast<uint8_t>(id)].state == offline_state::Online ? true : false;
    else
        return true;
}
