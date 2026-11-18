/**
 *******************************************************************************
 * @file      : infantry_chassis.cpp
 * @brief     : C 接口封装，供 FreeRTOS 任务调用
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2026-05-24      origincy        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2026 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
/** Includes ------------------------------------------------------------------*/
#include "infantry.h"
#include "chassis.h"
#include "can.h"
#include "cmsis_os.h"
#include "referee.h"
#include "main.h"
#include "board_comm.h"
#include "cap.h"
/** Exported variables --------------------------------------------------------*/
namespace
{
    ChassisConfig config{
        {&hcan1, 0x20A, 1.0f, 1.6609f, DeviceId::DEV_Steer_RF},              // Steer_RF;
        {&hcan1, 0x20B, 1.0f, -0.44266f + 3.14159f, DeviceId::DEV_Steer_LB}, // Steer_LB;
        {&hcan1, 0x201, 268 / 17.f, 0.0f, DeviceId::DEV_Wheel_RF},           // Wheel_RF;
        {&hcan1, 0x203, 268 / 17.f, 0.0f, DeviceId::DEV_Wheel_LB},           // Wheel_LB;
    };
}

Chassis chassis(&config);

/**
 * @brief 底盘初始化
 */
void Chassis_Init(void)
{
    chassis.Init();
    referee.Init(&huart6);
    cap.Init(&hcan1, 0x112, 0x111);
    board_comm.Init(&hcan2, 0x010);
}

/**
 * @brief 底盘控制任务
 */
void Chassis_Task(void)
{
    chassis.Control();
    cap.Send();
    board_comm.Send();
}
