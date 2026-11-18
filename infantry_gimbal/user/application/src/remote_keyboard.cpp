/**
 *******************************************************************************
 * @file      : remote_keyboard.cpp
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
#include "remote_keyboard.h"

#include "board_comm.h"
#include "bsp_dwt.h"
#include "chassis.h"
#include "gimbal.h"
#include "ins.h"
#include "main.h"
#include "offline_detect.h"
#include "referee.h"
#include "remote.h"
#include "shoot.h"
#include "user_lib.h"
#include "vision.h"


/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t signalShootFlag = 0;
uint8_t continueShootFlag = 0;
uint8_t backShootFlag = 0;
uint8_t trigBreakFlag = 0;
uint8_t trigEnableFlag = 0;
uint8_t aimingFlag = 0;
uint8_t AutoFireFlag = 0;
uint8_t ShrinkFlag = 1;
uint8_t ShootEnableFlag = 0;
uint8_t reserve_flag = 0;
uint16_t time_out = 0;
uint8_t shoot_allow_flag = 0;
uint8_t count_flag = 1;
uint8_t change_flag = 1;
uint8_t _flag;
static uint8_t shoot_timer_active = 0;
static uint8_t last_remote_shoot_mode = 0xFF;
int16_t last_ch4;
uint32_t shoot_start_time, current_time;
float SW_signal, Step_signal, Sin_signal, Cos_signal;
float last_SW_signal, last_Step_signal;
uint8_t polarity = 0;
uint8_t fric_flag = 0;


bool init = 0;

RemoteCtrlMode remotectrlmode = RemoteCtrlMode::AutoAimMode;
CtrlMode ctrlmode = CtrlMode::RemoteCtrl;
LongPressDetectState LeftButtonState = LongPressDetectState::ButtonRelease;
LongPressDetectState RightButtonState = LongPressDetectState::ButtonRelease;
uint16_t LCount = 0, RCount = 0;
static uint16_t count = 0;
static uint16_t lifting_count = 0;

float heat_limit_, heat_, heat_cooling_, heat_remain_;
float shoot_freq_;

volatile float testFricSpeed = 7000.f;
volatile float testTrigSpeed = 120.f;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

static void RemoteChassisCtrl();
static void RemoteGimbalCtrl();
static void RemoteShootCtrl();
static void KeyboardChassisCtrl();
static void KeyboardGimbalCtrl();
static void KeyboardShootCtrl();
static void HeatClosedLoop();
static void SyncVisionPacketToManual();

enum {
  REMOTE_SHOOT_MODE_SAFE = 0,
  REMOTE_SHOOT_MODE_SINGLE,
  REMOTE_SHOOT_MODE_CONTINUOUS,
  REMOTE_SHOOT_MODE_AUTO
};
/**
 * @brief 状态控制任务
 * @note 操作模式在README.md中
 */
void ModeTask() {
  switch (ctrlmode) {
  case CtrlMode::RemoteCtrl:
    if (remote.GetS2() == 1) {
      if (remote.GetS1() == 1) {
        // if (remotectrlmode == RemoteCtrlMode::AutoAimMode) {
          aimingFlag = 0;
          AutoFireFlag = 0;
          ShrinkFlag = 0;
          board_comm.SetShootStop(0);
          board_comm.SetFricFlag(1);//连发
        // }
        // else {
        //   aimingFlag = 0;
        //   AutoFireFlag = 0;
        // }
       }
       else if (remote.GetS1() == 3)
        {
          aimingFlag = 0;
          AutoFireFlag = 0;
          ShrinkFlag = 0;
          board_comm.SetShootStop(1);
          board_comm.SetFricFlag(0);//单发
        }
        else
        {
          aimingFlag = 0;
          AutoFireFlag = 0;
          ShrinkFlag = 1;
          board_comm.SetShootStop(1);
          board_comm.SetFricFlag(0);
        }
        board_comm.SetGimbalStopFlag(0);
        // if (remotectrlmode == RemoteCtrlMode::ChassisMode)
        //   board_comm.SetReadyFlag(1);
        // else
        //   board_comm.SetReadyFlag(0);
        board_comm.SetReadyFlag(0);
      }
      else if (remote.GetS2() == 2)
      {
          aimingFlag = 0;    
          AutoFireFlag = 0;
          ShrinkFlag = 1;
          board_comm.SetShootStop(1);
          board_comm.SetFricFlag(0);//连发
        // if (remotectrlmode == RemoteCtrlMode::ChassisMode)//后续手动修改remotectrlmode的值来控制开关
        //   board_comm.SetReadyFlag(1);
        // else
        //   board_comm.SetReadyFlag(0);
        board_comm.SetReadyFlag(1);
        board_comm.SetGimbalStopFlag(0);
      }
      //此处我将模式进行了修改，改为自动开火加pitch控制
    //   else if (remote.GetS1() == 3) {
    //     if (remotectrlmode == RemoteCtrlMode::AutoAimMode) {
    //       aimingFlag = 1;
    //       AutoFireFlag = 0;
    //     }
    //     else
    //       aimingFlag = 0;
    //     board_comm.SetGimbalStopFlag(0);
    //     board_comm.SetShootStop(1);
    //     board_comm.SetFricFlag(1);
    //     if (remotectrlmode == RemoteCtrlMode::ChassisMode)
    //       board_comm.SetReadyFlag(1);
    //     else
    //       board_comm.SetReadyFlag(0);
    //   }
    //   else if (remote.GetS1() == 2) {
    //     if (remotectrlmode == RemoteCtrlMode::AutoAimMode) {
    //       aimingFlag = 1;
    //       AutoFireFlag = 0;
    //     }
    //     else {
    //       aimingFlag = 0;
    //       AutoFireFlag = 0;
    //     }
    //     board_comm.SetGimbalStopFlag(0);
    //     board_comm.SetShootStop(1);
    //     board_comm.SetFricFlag(0);
    //     if (remotectrlmode == RemoteCtrlMode::ChassisMode)
    //       board_comm.SetReadyFlag(1);
    //     else
    //       board_comm.SetReadyFlag(0);
    //   }
    // }
    // else if (remote.GetS2() == 3) {
    //   if (remote.GetS1() == 1) {
    //     aimingFlag = 0;
    //     AutoFireFlag = 0;
    //     board_comm.SetGimbalStopFlag(0);
    //     board_comm.SetShootStop(0);
    //     board_comm.SetFricFlag(1);
    //     if (remotectrlmode == RemoteCtrlMode::ChassisMode)
    //       board_comm.SetReadyFlag(1);
    //     else
    //       board_comm.SetReadyFlag(0);
    //   }
    //   else if (remote.GetS1() == 3) {
    //     aimingFlag = 0;
    //     AutoFireFlag = 0;
    //     board_comm.SetGimbalStopFlag(0);
    //     board_comm.SetShootStop(1);
    //     board_comm.SetFricFlag(1);
    //     if (remotectrlmode == RemoteCtrlMode::ChassisMode)
    //       board_comm.SetReadyFlag(1);
    //     else
    //       board_comm.SetReadyFlag(0);
    //   }
    //   else if (remote.GetS1() == 2) {
    //     aimingFlag = 0;
    //     AutoFireFlag = 0;
    //     board_comm.SetGimbalStopFlag(0);
    //     board_comm.SetShootStop(1);
    //     board_comm.SetFricFlag(0);
    //     if (remotectrlmode == RemoteCtrlMode::ChassisMode)
    //       board_comm.SetReadyFlag(1);
    //     else
    //       board_comm.SetReadyFlag(0);
    //   }
    // }
    // else if (remote.GetS2() == 2) {
    //   if (remote.GetS1() == 2) {
    //     aimingFlag = 0;
    //     AutoFireFlag = 0;
    //     board_comm.SetGimbalStopFlag(1);
    //     board_comm.SetShootStop(1);
    //     board_comm.SetFricFlag(0);
    //     board_comm.SetReadyFlag(0);
    //   }
    //   else if (remote.GetS1() == 1) {
    //     aimingFlag = 0;
    //     AutoFireFlag = 0;
    //     board_comm.SetGimbalStopFlag(0);
    //     board_comm.SetShootStop(1);
    //     board_comm.SetFricFlag(0);
    //     if (remotectrlmode == RemoteCtrlMode::ChassisMode)
    //       board_comm.SetReadyFlag(1);
    //     else
    //       board_comm.SetReadyFlag(0);
    //   }
    //   else if (remote.GetS1() == 3) {
    //     aimingFlag = 0;
    //     AutoFireFlag = 0;
    //     board_comm.SetGimbalStopFlag(0);
    //     board_comm.SetShootStop(1);
    //     board_comm.SetFricFlag(0);
    //     if (remotectrlmode == RemoteCtrlMode::ChassisMode)
    //       board_comm.SetReadyFlag(1);
    //     else
    //       board_comm.SetReadyFlag(0);
    //   }

    else {
      aimingFlag = 0;
      AutoFireFlag = 0;
      board_comm.SetGimbalStopFlag(1);
      board_comm.SetShootStop(1);
      board_comm.SetFricFlag(0);
      board_comm.SetReadyFlag(0);
    }
    RemoteGimbalCtrl();
    RemoteShootCtrl();
    RemoteChassisCtrl();
    
    /* code */
  //   break;
  // case CtrlMode::KeyboardCtrl:
  //   if (referee.GetKeyTick(KEY_B)) {
  //     board_comm.SetReadyFlag(1);
  //     board_comm.SetGimbalStopFlag(0);
  //     fric_flag = 1;
  //     if (referee.GetKeyPress(MOUSE_R) && recepakge.mode != 0) {
  //       aimingFlag = 1;
  //     }
  //     else {
  //       aimingFlag = 0;
  //     }
  //   }
  //   else {
  //     board_comm.SetReadyFlag(0);
  //     board_comm.SetGimbalStopFlag(1);
  //     board_comm.SetFricFlag(0);
  //     fric_flag = 0;
  //     referee.SetKeyTick(KEY_F, 0);
  //     board_comm.SetShootStop(1);
  //   }
  //   if (referee.GetKeyPress(KEY_R)) {
  //     count++;
  //     if (count > 2000)
  //       HAL_NVIC_SystemReset();
  //     if (count > 1000)
  //       board_comm.SetResetFlag(1);
  //   }
  //   else {
  //     board_comm.SetResetFlag(0);
  //   }
  //   KeyboardGimbalCtrl();
  //   KeyboardShootCtrl();
  //   KeyboardChassisCtrl();
  //   break;
  // default:
  //   break;
  }
  HeatClosedLoop();

  board_comm.SetAimFlag(recepakge.mode != 0);

  if (!offline_detect.IsOnline(DeviceId::DEV_REMOTE)) {
    ctrlmode = CtrlMode::KeyboardCtrl;
  }
  else {
    ctrlmode = CtrlMode::RemoteCtrl;
  }
}

/**
 * @brief 遥控器底盘控制
 * @note 控制期望速度和底盘模式
 */
static void RemoteChassisCtrl() {
  if (board_comm.GetReadyFlag()) {
    chassis.SetYSpeed(remote.GetCh2() * 660);
    chassis.SetXSpeed(remote.GetCh3() * 660);
    if (remote.GetS2() == 1)
      chassis.SetMode(Chassis_Mode::SPIN);
    else if (remote.GetS2() == 3) {
      chassis.SetMode(Chassis_Mode::FOLLOW_GIMBAL_YAW);
    }
    else if (remote.GetS2() == 2) {
      chassis.SetMode(Chassis_Mode::FOLLOW_GIMBAL_YAW_SLANT);
    }
    else {
      // 默认跟随
      chassis.SetMode(Chassis_Mode::FOLLOW_GIMBAL_YAW);
    }

    if (remote.GetCh4() > 500) {
      board_comm.SetCapFlag(1);
    }
    else {
      board_comm.SetCapFlag(0);
    }
  }
  else {
    chassis.SetYSpeed(0);
    chassis.SetXSpeed(0);
    chassis.SetMode(Chassis_Mode::NO_MOVE);
  }
}//目前不会进入

/**
 * @brief 遥控器控制云台
 * @note 分视觉控制和手动控制
 */
static void RemoteGimbalCtrl() {
  // if (!board_comm.GetGimbalStopFlag())
    // if (aimingFlag) {
    //   if (recepakge.mode != 0) {
    //     gimbal.SetVisionPitch(recepakge.pitch * math::rad_2_deg,
    //                           recepakge.pitch_vel, recepakge.pitch_acc);
    //     // gimbal.SetVisionYaw(recepakge.yaw * math::rad_2_deg, recepakge.yaw_vel,
    //     //                     recepakge.yaw_acc);
    //   }//自瞄的情况
  //     else if (remote.GetCh4() > 400) {
  //       // gimbal.SetVisionPitch(Step_signal * DEG_2_RAD, (Step_signal - last_Step_signal) * DEG_2_RAD * 200.F, 0);
  //       // gimbal.SetVisionPitch(Step_signal * DEG_2_RAD, 0, 0);
  //       gimbal.SetVisionYaw(SW_signal / 50 * 24 * DEG_2_RAD, (SW_signal - last_SW_signal) / 50 * 24 * DEG_2_RAD * 500.f, 0);

  //       last_SW_signal = SW_signal;
  //     }
  //     else {
  //       // gimbal.SetVisionPitch(remote.GetCh1() * 20, 0, 0);
  //       // gimbal.SetVisionYaw(remote.GetCh0() * 50.f, 0, 0);
  //       gimbal.SetPitchPosition(remote.GetCh1() * 0.25);
  //       gimbal.SetYawPosition(remote.GetCh0() * 0.25);
  //       SyncVisionPacketToManual();
  //     }
    // }
    if (!board_comm.GetGimbalStopFlag()) {
      if (!ShrinkFlag)
      {
      // gimbal.SetPitchPosition(remote.GetCh1() * 0.08f);
        if (fabsf(gimbal.motor_[1].GetAngle() - gimbal.pitch_lifting_angle) < 10.0f && lifting_count <= 1000)
        {
          lifting_count++;
        }
        else if (fabsf(gimbal.motor_[1].GetAngle() - gimbal.pitch_lifting_angle) >= 10.0f)
        {
          lifting_count = 0;
        }
        if (lifting_count >= 1000) ShootEnableFlag = 1;
        else ShootEnableFlag = 0;//延时开启拨弹盘

      gimbal.SetPitchPosition(gimbal.pitch_lifting_angle);
      }
      else if (ShrinkFlag)
      {
      gimbal.SetPitchPosition(gimbal.pitch_shrink_angle);
      ShootEnableFlag = 0;
      }
      gimbal.SetPitchFollowPosition(0.0f);
      gimbal.SetYawPosition(remote.GetCh0() * 0.25f);

      // gimbal.SetYawPosition(remote.GetCh0() * 0.25f);
      // SyncVisionPacketToManual();
    }
    else ShootEnableFlag = 0;
}//我将模式精简化了，目前只会进入非瞄准即纯遥感控制分支

/**
 * @brief 遥控器控制发射机构(S2需在上半部)
 */
static void RemoteShootCtrl() {
  uint8_t current_remote_shoot_mode = REMOTE_SHOOT_MODE_SAFE;

  if (board_comm.GetFricFlag()) {
    if (AutoFireFlag) {
      current_remote_shoot_mode = REMOTE_SHOOT_MODE_AUTO;
    }
    else if (board_comm.GetShootStop()) {
      current_remote_shoot_mode = REMOTE_SHOOT_MODE_SINGLE;
    }
    else {
      current_remote_shoot_mode = REMOTE_SHOOT_MODE_CONTINUOUS;
    }
  }
  if (current_remote_shoot_mode != last_remote_shoot_mode) {
    // Clear latched shoot states when switching remote shoot mode.
    shoot_timer_active = 0;
    continueShootFlag = 0;
    signalShootFlag = 0;
    trigBreakFlag = 0;
    shoot_start_time = 0;
    backShootFlag = 0;
    last_remote_shoot_mode = current_remote_shoot_mode;
  }//切换模式后标志位就会重置，这样在后续设置模式的时候也会方便很多
    // if (board_comm.GetFricFlag())
    // {
    //   if (!board_comm.GetShootStop())
    //   {
    //     if (AutoFireFlag)
    //     {
    //     shoot.SetShootFlag(SPEED_FLAG);
    //     shoot.SetFricSpeed(testFricSpeed);
    //     shoot.SetTriggerSpeed(0);
    //     }
    //     else if (continueShootFlag)
    //     {
    //     shoot.SetShootFlag(SPEED_FLAG);
    //     shoot.SetFricSpeed(remote.GetCh4() * 6.0f);
    //     shoot.SetTriggerSpeed(0);
    //     }
    //   }
      
    // }//自己写的逻辑

    // if (fabsf(shoot.fric_motor_[0].GetSpd()) >= 4000.0f && fabsf(shoot.fric_motor_[1].GetSpd()) >= 4000.0f) trigEnableFlag = 1;
    // else trigEnableFlag = 0;  //判定能否开启拨弹盘

  //单发
  if (board_comm.GetFricFlag() && ShootEnableFlag) {
    // shoot.SetFricSpeed(remote.GetShootSpd());
    shoot.SetFricSpeed(testFricSpeed);
    // 没有连发的时候开单发，单发发射的逻辑依赖于状态机，只有先拨一此后松手才能发射
    if (board_comm.GetShootStop() && !AutoFireFlag)
    {
      if (remote.GetCh4() > 500) {
        trigBreakFlag = 0;
        signalShootFlag = 1;
        shoot.SetShootFlag(ANGLE_FLAG);
        shoot.SetTriggerPos(0.0f);
      }
      if (remote.GetCh4() < 500 && signalShootFlag == 1) {
        shoot.SetShootFlag(SIGNAL_SPEED_FLAG);
        shoot.SetTriggerSpeed(-65.f);
        signalShootFlag = 0;
      }
      if ((shoot.fric_output_[0] > 1800 || shoot.fric_output_[1] < -1800) &&
          !trigBreakFlag)
           {
        trigBreakFlag = 1;
        shoot.SetShootFlag(SIGNAL_SPEED_FLAG);
        shoot.SetTriggerSpeed(0.f);
        signalShootFlag = 0;
      }
      // 退弹
      if (remote.GetCh4() < -500) {
        backShootFlag = 1;
        shoot.SetShootFlag(ANGLE_FLAG);
        shoot.SetTriggerPos(0.0f);
      }
      if (remote.GetCh4() > -500 && backShootFlag == 1 ) {
        shoot.SetShootFlag(ANGLE_FLAG);
        shoot.SetTriggerPos(40.f);//设置的拨弹盘的角度
        backShootFlag = 0;
      }//在ShootStop为1的情况下进入
    }
    // if (!board_comm.GetShootStop() && !AutoFireFlag && heat_remain_ > 10) {

    //连发
    else if (!board_comm.GetShootStop() && !AutoFireFlag && ShootEnableFlag) {//
    //   if (!shoot_timer_active && !continueShootFlag) {
    //     trigBreakFlag = 0;
    //     shoot_timer_active = 1;
    //     if (remote.GetCh4() > 500) continueShootFlag = 1;
    //     else continueShootFlag = 0;
    //   }
    //   if (shoot_timer_active && continueShootFlag) {
    //     shoot_start_time = HAL_GetTick();
    //     shoot.SetShootFlag(ANGLE_FLAG);
    //     shoot.SetTriggerPos(-38.f);
    //     continueShootFlag = 0;
    //   }
    //   if (shoot_timer_active) {
    //     current_time = HAL_GetTick();
    //   }
    //   // if (((current_time - shoot_start_time > 1000.f / shoot_freq_)) &&
    //   if (((current_time - shoot_start_time > 1000.f / 20)) &&
    //       !trigBreakFlag) {
    //     trigBreakFlag = 1;//这像是一个执行阀门
    //     shoot.SetShootFlag(ANGLE_FLAG);
    //     shoot.SetTriggerPos(0.f);
    //     shoot_timer_active = 0;
    //     shoot_start_time = 0;
    //     continueShootFlag = 0;
    //   }
      // shoot.SetShootFlag(ANGLE_FLAG);
      // if (shoot_freq_>0.1){
      //   shoot.SetTriggerPos(-36.f * 0.001 * shoot_freq_,1);
      // } else {
      //   shoot.SetTriggerPos(3.f);
      // }
      // trigBreakFlag = 0;

      // shoot.SetFricSpeed(fabsf(remote.GetShootSpd()));
      // if (remote.GetCh4() > 500 && trigEnableFlag)
      // {
      //   shoot.SetShootFlag(SPEED_FLAG);
      //   shoot.SetTriggerSpeed(-60.0f);
      // }
      // else
      // {
      //   shoot.SetShootFlag(ANGLE_FLAG);
      //   shoot.SetTriggerPos(0.f);
      // }
      shoot.SetShootFlag(CONTINUE_SPEED_FLAG);
      shoot.SetTriggerSpeed(-120.0f);
    }
    // if (AutoFireFlag && heat_remain_ > 10) {
  //   if (AutoFireFlag) {
  //     shoot.SetShootFlag(ANGLE_FLAG);
  //     if ((recepakge.mode == AUTO_AIM_FIRE) && !shoot_timer_active) {
  //       trigBreakFlag = 0;
  //       shoot_timer_active = 1;
  //       continueShootFlag = 1;
  //     }
  //     if (shoot_timer_active && continueShootFlag) {
  //       shoot_start_time = HAL_GetTick();
  //       shoot.SetShootFlag(ANGLE_FLAG);
  //       shoot.SetTriggerPos(-36.f);
  //       continueShootFlag = 0;
  //     }
  //     if (shoot_timer_active) {
  //       current_time = HAL_GetTick();
  //     }
  //     // if ((current_time - shoot_start_time > 1000.f / shoot_freq_) &&
  //     if ((current_time - shoot_start_time > 1000.f / 20.f) &&
  //         !trigBreakFlag) {
  //       trigBreakFlag = 1;
  //       shoot.SetShootFlag(ANGLE_FLAG);
  //       shoot.SetTriggerPos(0.f);
  //       shoot_timer_active = 0;
  //       shoot_start_time = 0;
  //       continueShootFlag = 0;
  //     }
    
  }
  else {
    shoot.SetFricSpeed(0);
    shoot.SetShootFlag(CONTINUE_SPEED_FLAG);
    shoot.SetTriggerSpeed(0);
  }//如果到伸缩模式下还会疯传那就是电流方向反了
 }

/**
 * @brief 键盘控制底盘（当前未启用）
 *
 */
static void KeyboardChassisCtrl() {
  if (board_comm.GetReadyFlag()) {
    chassis.SetXSpeed(
      (referee.GetKeyPress(KEY_W) - referee.GetKeyPress(KEY_S)) * 660);
    chassis.SetYSpeed(
      (referee.GetKeyPress(KEY_D) - referee.GetKeyPress(KEY_A)) * 660);
    if (referee.GetKeyTick(KEY_C)) {
      chassis.SetMode(Chassis_Mode::SPIN);
    }
    else {
      if (referee.GetKeyTick(KEY_X)) {
        chassis.SetMode(Chassis_Mode::FOLLOW_GIMBAL_YAW);
      }
      else {
        chassis.SetMode(Chassis_Mode::FOLLOW_GIMBAL_YAW_SLANT);
      }
    }
    if (referee.GetKeyPress(KEY_SHIFT)) {
      board_comm.SetCapFlag(1);
    }
    else {
      board_comm.SetCapFlag(0);
    }
  }
  else {
    chassis.SetMode(Chassis_Mode::NO_MOVE);
  }
}

/**
 * @brief 键鼠控制云台
 */
// static void KeyboardGimbalCtrl() {
//   if (!board_comm.GetGimbalStopFlag()) {
//     if (referee.GetKeyPress(MOUSE_R)) {
//       if (recepakge.mode != 0) {
//         gimbal.SetVisionPitch(recepakge.pitch,
//                               recepakge.pitch_vel, recepakge.pitch_acc);
//         gimbal.SetVisionYaw(recepakge.yaw, recepakge.yaw_vel,
//                             recepakge.yaw_acc);
//       }
//       else {
//         gimbal.SetPitchPosition(-referee.comma_data.mouse_y * 0.005f);
//         gimbal.SETYawPosition(referee.comma_data.mouse_x * 0.005f);
//         SyncVisionPacketToManual();
//       }
//     }
//     else {
//       gimbal.SetPitchPosition(-referee.comma_data.mouse_y * 0.005f);
//       gimbal.SETYawPosition(referee.comma_data.mouse_x * 0.005f);
//       SyncVisionPacketToManual();
//     }
//     if (referee.GetKeyTick(KEY_Q)) {
//       gimbal.SETYawPosition(180.0f);
//       referee.SetKeyTick(KEY_Q, 0);
//     }
//   }
// }

static void SyncVisionPacketToManual() {
  recepakge.mode = 0;
  recepakge.yaw = INS.Yaw * math::deg_2_rad;
  recepakge.yaw_vel = 0.f;
  recepakge.yaw_acc = 0.0f;
  recepakge.pitch = INS.Pitch * math::deg_2_rad;
  recepakge.pitch_vel = 0.f;
  recepakge.pitch_acc = 0.0f;
}

/**
 * @brief 键鼠控制发射机构
 */
static void KeyboardShootCtrl() {
  // 检测电管发射机构输出
  // if (board_comm.GetShooterOutput() && fric_flag) {
  if (fric_flag) {
    // 手动控制开摩擦轮
    if (referee.GetKeyTick(KEY_F)) {
      board_comm.SetFricFlag(1);
      shoot.SetFricSpeed(testFricSpeed);
      // V键在左右键都松开时，打断当前拨弹状态并执行退弹。
      if (!referee.GetKeyPress(MOUSE_L) && !referee.GetKeyPress(MOUSE_R) &&
          (referee.GetKeyPress(KEY_V) || backShootFlag == 1)) {
        if (referee.GetKeyPress(KEY_V)) {
          board_comm.SetShootFlag(0);
          LeftButtonState = ButtonRelease;
          LCount = 0;
          shoot_timer_active = 0;
          shoot_start_time = 0;
          signalShootFlag = 0;
          continueShootFlag = 0;
          trigBreakFlag = 0;
          backShootFlag = 1;
          shoot.SetShootFlag(ANGLE_FLAG);
          shoot.SetTriggerPos(0.0f);
        }
        else if (backShootFlag == 1) {
          shoot.SetShootFlag(ANGLE_FLAG);
          shoot.SetTriggerPos(40.f);
          backShootFlag = 0;
        }
      }
      // 手动控制拨弹盘
      else if (!referee.GetKeyPress(MOUSE_R)) {
        // 长按状态机
        switch (LeftButtonState) {
        case ButtonRelease:
          LCount = 0;
          board_comm.SetShootFlag(0);
          if (referee.GetKeyPress(MOUSE_L))
            LeftButtonState = ButtonPressed;
          break;
        case ButtonPressed:
          LCount++;
          if (!referee.GetKeyPress(MOUSE_L)) {
            if (LCount < 50) {
              LeftButtonState = ButtonClick;
            }
            else {
              LeftButtonState = ButtonRelease;
            }
          }
          else if (LCount >= 50) {
            LeftButtonState = ButtonLongPress;
          }
          break;
        case ButtonClick:
          // 单发触发
          // if (heat_remain_ >= 10.f) {
          board_comm.SetShootFlag(0);
          trigBreakFlag = 0;
          signalShootFlag = 1;
          shoot.SetShootFlag(ANGLE_FLAG);
          shoot.SetTriggerPos(0.0f);
          // }
          // 切换状态机
          LeftButtonState = ButtonRelease;
          break;
        case ButtonLongPress:
          if (!referee.GetKeyPress(MOUSE_L)) {
            board_comm.SetShootFlag(1);
            LeftButtonState = ButtonRelease;
            shoot_timer_active = 0;
            shoot_start_time = 0;
            continueShootFlag = 0;
            trigBreakFlag = 0;
            break;
          }
          // if (!shoot_timer_active && !continueShootFlag && heat_remain_ > 10) {
          if (!shoot_timer_active && !continueShootFlag) {
            trigBreakFlag = 0;
            shoot_timer_active = 1;
            continueShootFlag = 1;
          }
          // 连发
          if (shoot_timer_active && continueShootFlag) {
            shoot_start_time = HAL_GetTick();
            shoot.SetShootFlag(ANGLE_FLAG);
            shoot.SetTriggerPos(-36.f);
            continueShootFlag = 0;
          }
          if (shoot_timer_active) {
            current_time = HAL_GetTick();
          }
          // if (((current_time - shoot_start_time > 1000.f / shoot_freq_)) &&
          if (((current_time - shoot_start_time > 1000.f / 20)) &&
              !trigBreakFlag) {
            trigBreakFlag = 1;
            shoot.SetShootFlag(ANGLE_FLAG);
            shoot.SetTriggerPos(0.f);
            shoot_timer_active = 0;
            shoot_start_time = 0;
            continueShootFlag = 0;
          }
          break;
        default:
          break;
        }
        // 单发开火
        if (signalShootFlag == 1) {
          shoot.SetShootFlag(CONTINUE_SPEED_FLAG);
          shoot.SetTriggerSpeed(-60.f);
          signalShootFlag = 0;
        }
        // 单发结束
        if (LeftButtonState != ButtonLongPress &&
            (shoot.fric_output_[0] > 1200 || shoot.fric_output_[1] < -1200) &&
            !trigBreakFlag) {
          trigBreakFlag = 1;
          shoot.SetShootFlag(ANGLE_FLAG);
          shoot.SetTriggerPos(3.f);
          signalShootFlag = 0;
        }
      }
      else if (referee.GetKeyPress(MOUSE_R)) {
        board_comm.SetShootFlag(1);
        shoot.SetShootFlag(ANGLE_FLAG);
        // if (!referee.GetKeyPress(MOUSE_L)) {
        if ((recepakge.mode == AUTO_AIM_FIRE) && !shoot_timer_active) {
          trigBreakFlag = 0;
          shoot_timer_active = 1;
          signalShootFlag = 1;
        }
        if (shoot_timer_active && signalShootFlag) {
          shoot_start_time = HAL_GetTick();
          shoot.SetShootFlag(ANGLE_FLAG);
          shoot.SetTriggerPos(-36.f);
          signalShootFlag = 0;
        }
        if (shoot_timer_active) {
          current_time = HAL_GetTick();
        } 
        // if ((current_time - shoot_start_time > 1000.f / shoot_freq_) &&
        if ((current_time - shoot_start_time > 1000.f / 20) &&
            !trigBreakFlag) {
          trigBreakFlag = 1;
          shoot.SetShootFlag(ANGLE_FLAG);
          shoot.SetTriggerPos(0.f);
          shoot_timer_active = 0;
          shoot_start_time = 0;
          signalShootFlag = 0;
        }
      }
      else {
        shoot.SetTriggerSpeed(0.0f);
      }
    }
    else {
      board_comm.SetFricFlag(0);
      shoot.SetFricSpeed(0.0f);
      shoot.SetShootFlag(CONTINUE_SPEED_FLAG);
      shoot.SetTriggerSpeed(0.0f);
      LeftButtonState = ButtonRelease;
      LCount = 0;
      shoot_timer_active = 0;
      shoot_start_time = 0;
      signalShootFlag = 0;
      continueShootFlag = 0;
      trigBreakFlag = 0;
    }
  }
  else {
    board_comm.SetFricFlag(0);
    fric_flag = 0;
    referee.SetKeyTick(KEY_F, 0);
    shoot.SetFricSpeed(0.0f);
    shoot.SetShootFlag(CONTINUE_SPEED_FLAG);
    shoot.SetTriggerSpeed(0.0f);
    LeftButtonState = ButtonRelease;
    LCount = 0;
    shoot_timer_active = 0;
    shoot_start_time = 0;
    signalShootFlag = 0;
    continueShootFlag = 0;
    trigBreakFlag = 0;
  }
}

/**
 * @brief 热量闭环
 */
void HeatClosedLoop() {
  constexpr int16_t k_heat_per_shot = 10;
  constexpr float k_max_shoot_freq = 15.0f;

  heat_limit_ = board_comm.GetCoolingLimit();
  heat_ = board_comm.GetCurrentHeat();
  heat_cooling_ = board_comm.GetCoolingHeat();

  heat_remain_ = heat_limit_ - heat_;
  if (heat_remain_ < 0) {
    heat_remain_ = 0;
  }

  // Cooling heat is converted to an equivalent sustainable firing frequency.
  const float cooling_based_freq =
    static_cast<float>(heat_cooling_) / k_heat_per_shot;
  float target_freq = 0.0f;

  // Keep at least one-shot heat budget (10) to avoid thermal overrun.
  if (heat_remain_ < 2 * k_heat_per_shot) {
    target_freq = 0.0f;
  }
  else if (heat_remain_ >= 80) {
    target_freq = k_max_shoot_freq;
  }
  else if (heat_remain_ > 25) {
    // Linearly blend from cooling-limited to max frequency in (20, 80).
    const float ratio = static_cast<float>(heat_remain_ - 25) / 55.0f;
    target_freq =
      cooling_based_freq + (k_max_shoot_freq - cooling_based_freq) * ratio;
  }
  else {
    target_freq = cooling_based_freq * 0.8f;
  }

  if (target_freq > k_max_shoot_freq) {
    target_freq = k_max_shoot_freq;
  }
  if (target_freq < 0.0f) {
    target_freq = 0.0f;
  }

  shoot_freq_ = target_freq;
}
