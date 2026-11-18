/**
 *******************************************************************************
 * @file      : infantry.cpp
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
#include "infantry.h"

#include "board_comm.h"
#include "bsp_dwt.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "gimbal.h"
#include "ins.h"
#include "offline_detect.h"
#include "referee.h"
#include "remote.h"
#include "shoot.h"
#include "user_lib.h"
#include "vision.h"
#include "remote_keyboard.h"
#include "MotorRLS.h"
/* Private macro -------------------------------------------------------------*/
static uint16_t count;
static uint32_t dwt_cnt;
static float q_v[4] = { 0 };
static uint8_t vision_count = 0;
const GimbalMode gimbalmode = GimbalMode::AUTO_AIM;

/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
namespace {
  HAL_StatusTypeDef CanTransmit(CAN_HandleTypeDef* _phcan,
                                CAN_TxHeaderTypeDef* _ptx_conf,
                                uint8_t* _ptx_buff) {
    uint32_t tx_mailbox;

    if (HAL_CAN_GetTxMailboxesFreeLevel(_phcan) == 0U) {
      return HAL_BUSY;
    }

    return HAL_CAN_AddTxMessage(_phcan, _ptx_conf, _ptx_buff, &tx_mailbox);
  }
}  // namespace

/**
 * @brief 云台初始化，在进入rtos前调用
 */
void InfantryGimbalInit() {
  // offline_detect.Init(DeviceId::DEV_REMOTE, 120);
  // // offline_detect.Init(DeviceId::DEV_REFEREE, 120);
  // // offline_detect.Init(DeviceId::DEV_VISION, 120);
  // offline_detect.Init(DeviceId::DEV_YAW_MOTOR, 80);
  // offline_detect.Init(DeviceId::DEV_PITCH_MOTOR, 80);
  // offline_detect.Init(DeviceId::DEV_FRIC_L, 10);
  // offline_detect.Init(DeviceId::DEV_FRIC_R, 10);
  // // offline_detect.Init(DeviceId::DEV_TRIGGER, 80);
  // // offline_detect.Init(DeviceId::DEV_BOARD_COMM, 120);
  // offline_detect.Init(DeviceId::DEV_INS, 20);
  // // offline_detect.Init(DeviceId::DEV_CAP, 150);
  // // offline_detect.Init(DeviceId::DEV_WHEEL, 150);
  // // offline_detect.Init(DeviceId::DEV_RUDDER, 150);
  // // offline_detect.Init(DeviceId::DEV_CHASSIS_REFEREE, 150);
  // 遥控器初始化
  remote.Init(&huart3);
  // 裁判系统初始化
  referee.Init(&huart6);
  // 云台电机初始化
  gimbal.MotorInit();
  // 射击系统初始化
  shoot.MotorInit();
  // 板间通信初始化
  board_comm.Init(&hcan1, 0x010);
}

/**
 * @brief Initializes the gimbal.
 */
void GimbalInit() {
  gimbal.PidInit();
  shoot.PidInit();
  chassis.Init();
}

/**
 * @brief This function handles the Gimbal task.
 */
void GimbalTask() {
  // 云台控制函数
  gimbal.Control();
  // 发射机构控制函数
  shoot.Control();
  // 设置底盘速度
  chassis.Control();
  // 视觉任务发送
  // 四元数整定
  ChangeQuat(INS.q, q_v);
  // q_v为X Y Z W
  // 后期改裁判系统获取弹速与余弹量
  if (!(vision_count % 5))
    Send_Vision(gimbalmode, q_v, INS.Yaw * math::deg_2_rad, INS.Gyro[2],
                INS.Pitch * math::deg_2_rad, INS.Gyro[0], 22.1f, 0.f);
  vision_count++;
}

/**
 * @brief CAN发送任务
 * @note 控制Pitch,Yaw轴电机以及两摩擦轮(fric)和一拨弹盘(trig)
 *
 */
void CanTask() {
  // 如果云台不出再急停状态
  if (board_comm.GetGimbalStopFlag() == 0) {
    // yaw轴,pitch
    // DjiMotorSend(&hcan2, 0x1FE, gimbal.output_speed_[1], 0, 0, 0);
    gimbal.motor_[0].SendMIT(0,0,0,0,gimbal.output_torque_[0]);
    gimbal.motor_[1].SendMIT(0,0,0,0,gimbal.output_torque_[1]);

    osDelay(1);
    gimbal.motor_[2].SendMIT(0,0,0,0,gimbal.output_torque_[2]);
    //这里改成达妙电机而非6020
    // DjiMotorSend(&hcan2, 0x1FE, 0, 0, 0, 0);
    
    if (board_comm.GetFricFlag()) {
      DjiMotorSend(&hcan2, 0x200, (int16_t)shoot.fric_output_[1],
                   (int16_t)shoot.fric_output_[0], 0, 0);
    }
    else {
      if (shoot.JudgeBreakFlag()) {
        shoot.SetFricSpeed(0.0f);
        DjiMotorSend(&hcan2, 0x200, (int16_t)shoot.fric_output_[1],
                     (int16_t)shoot.fric_output_[0], 0, 0);
      }
      else {
        DjiMotorSend(&hcan2, 0x200, 0, 0, 0, 0);
      }//缓慢降速
    }
    shoot.trigger_motor_.SendMIT(0, 0, 0, 0,
                                 -(shoot.trig_output_) * 3.6667e-4f);
  }
  else {
    // Pitch,Yaw急停
    gimbal.motor_[0].SendMIT(0,0,0,0,0);
    gimbal.motor_[1].SendMIT(0,0,0,0,0);

    osDelay(1);
    
    gimbal.motor_[2].SendMIT(0,0,0,0,0);
    // 发射机构急停
    // 闭环急停
  
    if (shoot.JudgeBreakFlag()) {
      shoot.SetFricSpeed(0.0f);
      DjiMotorSend(&hcan2, 0x200, (int16_t)shoot.fric_output_[1],
                   (int16_t)shoot.fric_output_[0], 0, 0);
    }
    else {
      DjiMotorSend(&hcan2, 0x200, 0, 0, 0, 0);
    }   
    // 拨弹盘失能
    shoot.trigger_motor_.SendMIT(0, 0, 0, 0, 0);
  }
    count++;    
  if (count % 100 == 0) {
    gimbal.motor_[0].MotorEnable();  
  }
  if (count % 100 == 3)
  {
    gimbal.motor_[1].MotorEnable(); 
  }
  if(count % 100 == 5){
    shoot.trigger_motor_.MotorEnable();
  }
  if(count % 100 == 7){
    gimbal.motor_[2].MotorEnable(); 
  }
}

/**
 * @brief 视觉回调函数
 */
void VisionCallback() {
  Receive_Vision();
}

void ChassisTask() {
  board_comm.Send();
}
