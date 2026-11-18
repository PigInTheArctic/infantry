/**
 *******************************************************************************
 * @file      : shoot.cpp
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
#include "shoot.h"
#include "board_comm.h"
#include "offline_detect.h"
#include "stdlib.h"
#include "user_lib.h"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
Shoot shoot;
/* Private function prototypes -----------------------------------------------*/
namespace {
  constexpr float k_speed_min = -10000.0f;
  constexpr float k_speed_max = 10000.0f;
}
/**
 * @brief This function is a callback for the left friction motor.
 * 
 * This function is called when an event related to the left friction motor
 * occurs. It is responsible for handling the event and performing the necessary
 * actions.
 *
 * @return void
 */
static void LeftFricMotorCallback();

/**
 * @brief This function is a callback for the right friction motor.
 *
 * This function is called when a specific event occurs related to the right
 * friction motor. It is responsible for handling the event and performing the
 * necessary actions.
 *
 * @return void
 */
static void RightFricMotorCallback();

/**
 * @brief This function is a callback for triggering the motor.
 */
static void TriggerMotorCallback();

void Shoot::PidInit() {
  fric_speed_[0].Init(50.0f, 1.0f, 0.0f, 15000.0f, 0.0f);
  fric_speed_[1].Init(50.0f, 1.0f, 0.0f, 15000.0f, 0.0f);
  trigger_pos_.Init(5.0f, 0.0f, 0.3f, 10.0f, 0.0f);
  trigger_speed_.Init(5.f, 0.f, 0.f, 25000.0f, 0.0f);
  // 单发调好的速度环
  // trigger_speed_.Init(5.2f, 0.1f, 0.5f, 2000.0f, 0.0f);
  // 正在测试的连发速度环(或许可以加积分分离)
  // trigger_speed_.Init(50.f, 180.f, 0.5f, 20000.0f, 0.0f);

  // 2006的量
  // trigger_speed_.Init(20.0f, 1.0f, 0.0f, 10000.0f, 0.0f);
  fric_speed_[0].Inprovement(PID_INTEGRAL_LIMIT, 2000.0f, 0.0f, 0.0f, 0.0f,
                             0.0f);
  fric_speed_[1].Inprovement(PID_INTEGRAL_LIMIT, 2000.0f, 0.0f, 0.0f, 0.0f,
                             0.0f);
  trigger_pos_.Inprovement(
    PID_DERIVATIVE_ON_MEASUREMENT | PID_DERIVATIVE_FILTER, 0.0f, 0.0f, 0.0f,
    0.0f, 0.003f);
  // trigger_speed_.Inprovement(PID_INTEGRAL_LIMIT | PID_TRAPEZOID_INTEGRAL,
  // 5000.f, 0.0f, 0.0f, 0.0f, 0.0f);
  trigger_speed_.Inprovement(PID_INTEGRAL_LIMIT | PID_TRAPEZOID_INTEGRAL,
                             8000.f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void Shoot::MotorInit() {
  fric_motor_[0].Init(0x202, &hcan2, ABSOLUTE_FLAG);
  fric_motor_[1].Init(0x201, &hcan2, ABSOLUTE_FLAG); //更改了摩擦轮的hcan口，注意左右问题
  trigger_motor_.Init(0x40, 0x04, &hcan1, DM_MOTOR::MIT);
  fric_motor_[0].pdji_motor_instance->pCanCallBack = LeftFricMotorCallback;
  fric_motor_[1].pdji_motor_instance->pCanCallBack = RightFricMotorCallback;
  trigger_motor_.pdm_motor_instance->pCanCallBack = TriggerMotorCallback;
}

void Shoot::Control() {
  FricCalc();
  TrigCalc();
}

void Shoot::FricCalc() {
  for (int i = 0; i < 2; i++) {
    fric_speed_[i].SetMeasure(fric_motor_[i].GetSpd());//用fric_motor_[0].spd_,fric_motor_[1].spd_来看测试的速度对不对
    fric_output_[i] = fric_speed_[i].Calc();
  }
}

void Shoot::TrigCalc() {
  static uint8_t last_shoot_flag = 0xFF;
  if (shoot_flag_ != last_shoot_flag) {
    // Use dedicated speed-loop gains in dual-loop (angle) mode.
    if (shoot_flag_ == CONTINUE_SPEED_FLAG) {
      // trigger_speed_.Init(80.f, 160.f, 0.5f, 20000.f, 0.0f);
      // trigger_speed_.Init(50.f, 160.f, 0.5f, 20000.f, 0.0f);
      trigger_speed_.Init(130.f, 160.f, 0.5f, 25000.f, 0.0f);
    }
    else if (shoot_flag_ == SIGNAL_SPEED_FLAG)
    {
      trigger_speed_.Init(120.f, 200.f, 0.f, 25000.f, 0.0f);
    }
    else {
      // trigger_speed_.Init(50.f, 100.f, 0.f, 20000.0f, 0.0f);
      // trigger_speed_.Init(30.f, 160.f, 0.5f, 20000.f, 0.0f);
      trigger_speed_.Init(50.f, 100.f, 0.5f, 20000.f, 0.0f);
    }
    // trigger_speed_.Inprovement(PID_INTEGRAL_LIMIT | PID_TRAPEZOID_INTEGRAL,
    //                            8000.f, 0.0f, 0.0f, 0.0f, 0.0f);
        trigger_speed_.Inprovement(PID_INTEGRAL_LIMIT | PID_TRAPEZOID_INTEGRAL,
                               8000.f, 0.0f, 0.0f, 0.0f, 0.0f);
    // trigger_speed_.Reset();
    last_shoot_flag = shoot_flag_;
  }

  if (shoot_flag_ == ANGLE_FLAG) {
    // 传动比3:5，与电机角度有系数转换关系
    trigger_pos_.SetMeasure(trigger_motor_.GetAngle() * 0.6f);
    trigger_speed_.SetRef(trigger_pos_.Calc(0.0f, trigger_motor_.GetSpd_rad() * 9.549296f * 0.6f));//给速度阻尼，但有点疑惑为什么不直接用GetSpd_rpm()
    // trigger_speed_.SetRef(trigger_pos_.Calc());
  }
  // 传动比3:5，与电机角度有系数转换关系，传入的是拨弹盘转速(rpm)
  trigger_speed_.SetMeasure(trigger_motor_.GetSpd_rad() * 9.549296f * 0.6f);
  trig_output_ = trigger_speed_.Calc();
}

void Shoot::SetFricSpeed(float _speed) {
  _speed = math::Constrain(_speed,k_speed_min,k_speed_max);
  fric_speed_[0].SetRef(_speed);
  fric_speed_[1].SetRef(-_speed);
}

void Shoot::SetTriggerPos(float _pos) {
  trigger_pos_.SetRef((trigger_motor_.GetAngle()) * 0.6f + _pos);//采用实际角度避免在限定范围两端方向不出错
  target_ang_ = math::LoopFloatConstrain(
    trigger_motor_.GetEcd() + _pos * DEG_2_RAD, -5.23599f, 5.23599f);//-300 - 300保证电机能够表示拨弹盘-180 - 180的角度 
}

void Shoot::SetTriggerSpeed(float _speed) {
  trigger_speed_.SetRef(_speed);
}

void Shoot::SetShootFlag(uint8_t _flag) {
  shoot_flag_ = _flag;
}

uint8_t Shoot::JudgeBreakFlag() {
  if (abs(fric_motor_[0].GetSpd()) > 1000 ||
      abs(fric_motor_[1].GetSpd()) > 1000)
    return 1;
  else
    return 0;
}

static void LeftFricMotorCallback() {
  offline_detect.Feed(DeviceId::DEV_FRIC_L);
  shoot.fric_motor_[0].Update();
}

static void RightFricMotorCallback() {
  offline_detect.Feed(DeviceId::DEV_FRIC_R);
  shoot.fric_motor_[1].Update();
}

static void TriggerMotorCallback() {
  offline_detect.Feed(DeviceId::DEV_TRIGGER);
  shoot.trigger_motor_.Update();
}
