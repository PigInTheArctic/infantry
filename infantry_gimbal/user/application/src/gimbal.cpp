/**
 *******************************************************************************
 * @file      : gimbal.cpp
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
#include "gimbal.h"

#include "ins.h"
#include "user_lib.h"
#include "remote_keyboard.h"
#include "board_comm.h"
#include "offline_detect.h"
#include "filter.h"
#include "arm_math.h"
#include "infantry.h"
#include <cmath>
#include "MotorRLS.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

//打符
constexpr float YAW_INERTIA = 0.021969301f;
constexpr float YAW_DAMPING_COEFF = 0.584953427f;
constexpr float YAW_COULOMB_FORCE = 0.0505f;
constexpr float PITCH_INERTIA = 0.022658515f;
constexpr float PITCH_DAMPING_COEFF = 0.228674372f;
constexpr float PITCH_COULOMB_FORCE = -0.0f;

/* External variables --------------------------------------------------------*/
Gimbal gimbal;
/* Private function prototypes -----------------------------------------------*/

static void PitchMotorCallback();
static void YawMotorCallback();
static void PitchFollowMotorCallback();
static float pitch_filter = 0;
static float chassis_wz_filter = 0;
static float pitch_gyro = 0;
static float pitch_speed_filter = 0;
static uint8_t pitch_measure_init = 0;

namespace {
  constexpr float k_pitch_min = 15.0f;
  constexpr float k_pitch_max = 100.0f;
  constexpr float k_pitch_follow_min = 3.0f;
  constexpr float k_pitch_follow_max = 40.0f;
  constexpr float k_pitch_limit_guard = 1.0f;
  constexpr float k_vel_limit = 20.0f;
  constexpr float k_acc_limit = 50.0f;

  constexpr float k_filter_sample_freq = 1000.0f;
  constexpr float k_gyro_lpf_cutoff = 80.0f;
  constexpr float k_chassis_wz_lpf_cutoff = 200.0f;
  constexpr float k_pitch_cmd_lpf_cutoff = 40.0f;
  constexpr float k_pitch_vel_lpf_cutoff = 100.0f;

  Class_Filter_Biquad pitch_gyro_lpf;
  Class_Filter_Biquad yaw_gyro_lpf;
  Class_Filter_Biquad chassis_wz_lpf;
  Class_Filter_Biquad pitch_cmd_lpf;
  Class_Filter_Biquad pitch_speed_lpf;
}  // namespace
/**
 * @brief 初始化云台Pitch,Yaw轴双环PID
 */
void Gimbal::PidInit() {
  angle_[0].Init(5.0f, 0.0f, 0.3f, 1000.0f, 0.0f);
  speed_[0].Init(60.0f, 0.5f, 0.0f, 10000.0f, 0.0f);

  angle_[1].Init(8.0f, 0.0f, 0.5f, 2000.0f, 0.0f);
  speed_[1].Init(60.0f, 0.0f, 0.0f, 10000.0f, 0.0f);

  angle_[2].Init(5.f, 0.0f, 0.0f, 2000.0f, 0.0f);
  speed_[2].Init(25.0f, 4.f, 0.0f, 10000.0f, 0.0f);    

  acc_[0].Init(1000.f, 400.f, 100.f, 200, 0.f);
  acc_[1].Init(2400.f, 1800.f, 200.f, 200, 0.f);
  yaw_spin_compensation.Init(0.126f, 0.0f, 0.30f, 0.f, 0.f);
  angle_[0].Inprovement(PID_DERIVATIVE_FILTER | PID_CHANGING_INTEGRATION_RATE,
                        0, 0, 0, 0.0f, 0.01f);
  speed_[0].Inprovement(PID_INTEGRAL_LIMIT | PID_DERIVATIVE_FILTER |
                        PID_DERIVATIVE_ON_MEASUREMENT | PID_CHANGING_INTEGRATION_RATE,
                        2000.0f, 100, 100, 0, 0.1f);
  // angle_[1].Inprovement(PID_DERIVATIVE_FILTER | PID_DERIVATIVE_ON_MEASUREMENT,
  //                       0, 0, 0, 0.0f, 0.01f);
  // speed_[1].Inprovement(PID_INTEGRAL_LIMIT | PID_DERIVATIVE_FILTER |
  //                       PID_DERIVATIVE_ON_MEASUREMENT | PID_CHANGING_INTEGRATION_RATE,
  //                       5000.0f, 100, 100, 0, 0.05f);
  angle_[2].Inprovement(PID_DERIVATIVE_FILTER | PID_CHANGING_INTEGRATION_RATE,
                        0, 0, 0, 0.0f, 0.01f);
  speed_[2].Inprovement(PID_INTEGRAL_LIMIT | PID_DERIVATIVE_FILTER |
                        PID_DERIVATIVE_ON_MEASUREMENT | PID_CHANGING_INTEGRATION_RATE,
                        3000.0f, 100, 100, 0, 0.05f);
  acc_[0].Inprovement(PID_DERIVATIVE_FILTER | PID_INTEGRAL_LIMIT | PID_CHANGING_INTEGRATION_RATE,
                      60, 0.08, 0.05, 0.00f, 0.00f);
  acc_[1].Inprovement(PID_DERIVATIVE_FILTER | PID_INTEGRAL_LIMIT | PID_CHANGING_INTEGRATION_RATE | PID_OUTPUT_FILTER,
                      100, 0.10, 0.06, 0.03f, 0.00f);
  yaw_spin_compensation.Inprovement(PID_INTEGRAL_LIMIT | PID_DERIVATIVE_FILTER |
                                    PID_DERIVATIVE_ON_MEASUREMENT,
                                    5000.0f, 0, 0, 0, 0.05f);

  pitch_gyro_lpf.Init(k_filter_sample_freq, k_gyro_lpf_cutoff, FILTER_BIQUAD_LOWPASS);
  yaw_gyro_lpf.Init(k_filter_sample_freq, k_gyro_lpf_cutoff, FILTER_BIQUAD_LOWPASS);
  chassis_wz_lpf.Init(k_filter_sample_freq, k_chassis_wz_lpf_cutoff);
  pitch_cmd_lpf.Init(k_filter_sample_freq, k_pitch_cmd_lpf_cutoff);
  pitch_speed_lpf.Init(k_filter_sample_freq, k_pitch_vel_lpf_cutoff);

  pitch_eso_.Init(44.1335189f, 100.f, 1000.f, 0.0f, 0.03f, ESOOrder::SECOND);
  yaw_eso_.Init(45.518061772f, 100.f, 1000.f, 0.0f, 0.05f, ESOOrder::SECOND);
  
}

/**
 * @brief 初始化云台电机
 */
void Gimbal::MotorInit() {
  //0号为pitch_follow，1号为pitch，2号为yaw
  motor_[0].Init(0x20, 0x02, &hcan1, DM_MOTOR::MIT);//改成达妙
  motor_[1].Init(0x30, 0x03, &hcan1, DM_MOTOR::MIT);
  motor_[2].Init(0x50, 0x05, &hcan1, DM_MOTOR::MIT);

  motor_[0].pdm_motor_instance->pCanCallBack = PitchFollowMotorCallback;
  motor_[1].pdm_motor_instance->pCanCallBack = PitchMotorCallback;
  motor_[2].pdm_motor_instance->pCanCallBack = YawMotorCallback;
}

/**
 * @brief Controls the gimbal.
 */
void Gimbal::Control() {
  if (board_comm.GetGimbalStopFlag()) {
    SyncReferenceToCurrent();
    last_auto_aim_active_ = 0;
    output_speed_[0] = 0;
    output_speed_[1] = 0;
    output_speed_[2] = 0;
    output_torque_[0] = 0;
    output_torque_[1] = 0;
    output_torque_[2] = 0;
    return;
  }//停止状态

  // const uint8_t auto_aim_active =
  //   (aimingFlag && gimbalmode == GimbalMode::AUTO_AIM) ? 1U : 0U;
  // if (auto_aim_active && !last_auto_aim_active_) {
  //   ResetAutoAimState();
  // // }
  // last_auto_aim_active_ = auto_aim_active;自动瞄准模式

  if (!board_comm.GetGimbalStopFlag()) {  //双环pid手动控制
    //0号为pitch_follow，1号为pitch，2号为yaw
    angle_[0].SetMeasure(INS.Pitch);
    angle_[1].SetMeasure(gimbal.motor_[1].GetAngle());
    angle_[2].SetMeasure(INS.YawTotalAngle);

    speed_[0].SetRef(angle_[0].Calc());
    speed_[1].SetRef(angle_[1].Calc());
    speed_[2].SetRef(angle_[2].Calc());

    speed_[0].SetMeasure(INS.Gyro[0] * math::rad_2_deg);
    speed_[1].SetMeasure(gimbal.motor_[1].GetSpd_rad() * 9.549296f);
    speed_[2].SetMeasure(INS.Gyro[2] * math::rad_2_deg);

    output_speed_[0] = speed_[0].Calc();
    output_speed_[1] = speed_[1].Calc();
    output_speed_[2] = speed_[2].Calc();

    chassis_wz_filter = chassis_wz_lpf.Calc(board_comm.GetSpinSpeed());
    yaw_spin_compensation.SetRef(-chassis_wz_filter);
    yaw_spin_compensation.SetMeasure(0);
    yaw_spin_compensation.Calc();

    output_torque_[0] = (float)(gimbal.output_speed_[0] / 16384.0f * 10.0f);
    output_torque_[1] = -(float)(gimbal.output_speed_[1] / 16384.0f * 10.0f);
    output_torque_[2] = (float)((gimbal.output_speed_[2] / 16384.0f * 10.0f) + yaw_spin_compensation.GetOutput());// + yaw_spin_compensation.GetOutput()
  }
  // else {
    // gimbal_lqr_.SetData(INS.Pitch * math::deg_2_rad, pitch_gyro_lpf.Calc(INS.Gyro[1]),
    //                     INS.YawTotalAngle * math::deg_2_rad, yaw_gyro_lpf.Calc(INS.Gyro[2]), 0.002f);
    // gimbal_lqr_.Calc();
    // yaw_eso_.Update(INS.Gyro[2], yaw_torque);
    // yaw_torque = gimbal_lqr_.GetYawOutput();
    // // yaw_torque = math::Constrain(yaw_torque - yaw_eso_.GetDisturbance(), -2.2f, 2.2f);
    // yaw_torque = math::Constrain(yaw_torque, -2.2f, 2.2f);
    // output_speed_[0] = yaw_torque * 7370.2204f;

    // pitch_eso_.Update(INS.Gyro[1], pitch_torque);
    // pitch_torque = gimbal_lqr_.GetPitchOutput();
    // // pitch_torque = math::Constrain(pitch_torque - pitch_eso_.GetDisturbance() + PITCH_INERTIA * set_pitch_acc_, -2.2f, 2.2f);
    // pitch_torque = math::Constrain(pitch_torque, -2.2f, 2.2f);
    // // pitch_torque = math::Constrain(pitch_torque + PITCH_INERTIA * set_pitch_acc_, -2.2f, 2.2f);
    // output_speed_[1] = -pitch_torque * 7370.2204f;
    //   
    //   acc_[0].SetMeasure(INS.YawTotalAngle * math::deg_2_rad);
    //   acc_[0].Calc(set_vyaw_, INS.Gyro[2]);

    //   float yaw_gyro = INS.Gyro[2] * math::rad_2_deg;
    //   //力控三个分力：惯性力，阻尼力，库仑摩擦
    //   yaw_torque = YAW_INERTIA * (set_yaw_acc_ + acc_[0].GetOutput()) +
    //     YAW_DAMPING_COEFF * yaw_gyro +
    //     YAW_COULOMB_FORCE * (yaw_gyro > 0 ? 1.0f : -1.0f);


    //   chassis_wz_filter = chassis_wz_lpf.Calc(board_comm.GetSpinSpeed());
    //   yaw_spin_compensation.SetRef(-chassis_wz_filter);
    //   yaw_spin_compensation.SetMeasure(0);
    //   yaw_spin_compensation.Calc();
    //   yaw_torque = math::AbsLimit(yaw_torque + yaw_spin_compensation.GetOutput(), 2.2f);
    //   //输出为力矩,单位Nm,6020映射到电流
    //pitch
    //   pitch_gyro = pitch_gyro_lpf.Calc(INS.Gyro[1] * math::rad_2_deg);

    //   acc_[1].SetMeasure(INS.Pitch * math::deg_2_rad);
    //   acc_[1].Calc(set_vpitch_, pitch_gyro * math::deg_2_rad);

    //   //力控四个分力：惯性力,阻尼力,库仑摩擦,重力补偿
    //   pitch_torque = PITCH_INERTIA * (set_pitch_acc_ + acc_[1].GetOutput()) +
    //     PITCH_DAMPING_COEFF * pitch_gyro +
    //     PITCH_COULOMB_FORCE * (pitch_gyro > 0 ? 1.0f : -1.0f) -
    //     (0.1857f * INS.Pitch * INS.Pitch * INS.Pitch - 3.2109f * INS.Pitch * INS.Pitch + 62.82f * INS.Pitch - 1564.8f) * 1.3568115e-4f;

    //   pitch_torque = math::AbsLimit(pitch_torque, 2.2f);

    //   //输出为力矩,单位Nm,方向为负,直接mit控达妙
    //   //力矩转换为控制值： T/转矩常数0.741*控制精度16384/控制对应转矩电流3A
    //   output_speed_[0] = (int16_t)(yaw_torque * 7370.2204f);
    //   output_speed_[1] = -(int16_t)(pitch_torque * 7370.2204f);
  
}

void Gimbal::SyncReferenceToCurrent() {
  set_yaw_ = INS.YawTotalAngle;
  set_pitch_follow_ = INS.Pitch;
  set_pitch_ = gimbal.motor_[1].GetAngle();
  set_pitch_follow_ = math::Constrain(set_pitch_, k_pitch_follow_min, k_pitch_follow_max);
  set_pitch_ = math::Constrain(set_pitch_, k_pitch_min, k_pitch_max);
  
  set_vyaw_ = 0.0f;
  set_vpitch_follow_ = 0.0f;
  set_vpitch_ = 0.0f;
  set_yaw_acc_ = 0.0f;
  set_pitch_acc_ = 0.0f;
  yaw_torque = 0.0f;
  pitch_torque = 0.0f;
  pitch_follow_torque = 0.0f;

  angle_[0].Reset();
  angle_[1].Reset();
  angle_[2].Reset();
  speed_[0].Reset();
  speed_[1].Reset();
  speed_[2].Reset();
  acc_[0].Reset();
  acc_[1].Reset();
  acc_[2].Reset();
  yaw_spin_compensation.Reset();

  // angle_[0].SetRef(set_yaw_);
  // angle_[0].SetMeasure(INS.YawTotalAngle);
  angle_[0].SetRef(set_pitch_follow_);             
  angle_[0].SetMeasure(INS.Pitch);
  angle_[1].SetRef(set_pitch_);
  angle_[1].SetMeasure(gimbal.motor_[1].GetAngle());
  angle_[2].SetRef(set_yaw_);
  angle_[2].SetMeasure(INS.YawTotalAngle);
  // acc_[0].SetRef(set_yaw_ * math::deg_2_rad);
  // acc_[0].SetMeasure(INS.YawTotalAngle * math::deg_2_rad);
  // acc_[1].SetRef(set_pitch_ * math::deg_2_rad);
  // acc_[1].SetMeasure(INS.Pitch * math::deg_2_rad);

  pitch_filter = set_pitch_;
  pitch_speed_filter = 0.0f;
  pitch_gyro = 0.0f;
  chassis_wz_filter = 0.0f;
  pitch_measure_init = 1;
}

// void Gimbal::ResetAutoAimState() {
//   acc_[0].Reset();
//   acc_[1].Reset();

//   acc_[0].SetRef(set_yaw_ * math::deg_2_rad);
//   acc_[0].SetMeasure(INS.YawTotalAngle * math::deg_2_rad);
//   acc_[1].SetRef(set_pitch_ * math::deg_2_rad);
//   acc_[1].SetMeasure(INS.Pitch * math::deg_2_rad);
// }

/**
 * @brief Set the pitch position of the gimbal.
 *
 * This function sets the desired pitch position for the gimbal.
 *
 * @param position The desired pitch position in degrees.
 */
void Gimbal::SetPitchPosition(float _ang) {
  float add_pace = 0.06f;
  if (fabsf(set_pitch_ - _ang) <= add_pace )
  {
    set_pitch_ = _ang;
  }
  else if (set_pitch_ > _ang)
  {
    set_pitch_ -= add_pace;
  }
  else if(set_pitch_ < _ang)
  {
    set_pitch_ += add_pace;
  }
  set_pitch_ = math::Constrain(set_pitch_, k_pitch_min, k_pitch_max);
  angle_[1].SetRef(set_pitch_);
}//限位进行了更改

/**
 * @brief Sets the yaw position of the gimbal.
 *
 * @param _ang The desired yaw angle in degrees.
 */
void Gimbal::SetYawPosition(float _ang) {
  set_vyaw_ = 0;
  set_yaw_acc_ = 0;

  set_yaw_ -= _ang;
  angle_[2].SetRef(set_yaw_); 
  // gimbal_lqr_.SetYawAngle(set_yaw_ * math::deg_2_rad);
}

void Gimbal::SetPitchFollowPosition(float _ang) {
  set_pitch_follow_ = _ang;
  angle_[0].SetRef(set_pitch_follow_);
}

// void Gimbal::SetVisionPitch(float _ang, float _ang_vel, float _ang_acc) {
//   set_pitch_ = _ang;
//   set_pitch_ = math::Constrain(set_pitch_, k_pitch_min, k_pitch_max);
//   set_vpitch_ = math::Constrain(_ang_vel, -k_vel_limit, k_vel_limit);
//   set_pitch_acc_ = math::Constrain(_ang_acc, -k_acc_limit, k_acc_limit);

//   if (set_pitch_ >= (k_pitch_max - k_pitch_limit_guard)) {
//     if (set_vpitch_ > 0.0f) {
//       set_vpitch_ = 0.0f;
//     }
//     if (set_pitch_acc_ > 0.0f) {
//       set_pitch_acc_ = 0.0f;
//     }
//   }
//   else if (set_pitch_ <= (k_pitch_min + k_pitch_limit_guard)) {
//     if (set_vpitch_ < 0.0f) {
//       set_vpitch_ = 0.0f;
//     }
//     if (set_pitch_acc_ < 0.0f) {
//       set_pitch_acc_ = 0.0f;
//     }
//   }

//   // if (gimbalmode != GimbalMode::AUTO_AIM && false)
//   if (gimbalmode != GimbalMode::AUTO_AIM)
//     angle_[1].SetRef(set_pitch_);
//   else
//     acc_[1].SetRef(set_pitch_ * math::deg_2_rad);
//   // angle_[1].SetRef(set_pitch_);
//   gimbal_lqr_.SetPitchAngle(set_pitch_ * math::deg_2_rad);
//   gimbal_lqr_.SetPitchVel(set_vpitch_);

//   err_pitch_ = set_pitch_ * math::deg_2_rad - INS.Pitch_rad;
// }

// void Gimbal::SetVisionYaw(float _ang, float _ang_vel, float _ang_acc) {
//   set_yaw_ = _ang + INS.YawCount * 360.f;
//   set_vyaw_ = math::Constrain(_ang_vel, -k_vel_limit, k_vel_limit);
//   set_yaw_acc_ = math::Constrain(_ang_acc, -k_acc_limit, k_acc_limit);

//   if (gimbalmode != GimbalMode::AUTO_AIM)
//     // if (gimbalmode != GimbalMode::AUTO_AIM && false)
//     angle_[0].SetRef(set_yaw_);
//   else
//     acc_[0].SetRef(set_yaw_ * math::deg_2_rad);

//   gimbal_lqr_.SetYawAngle(set_yaw_ * math::deg_2_rad);
//   gimbal_lqr_.SetYawVel(set_vyaw_);

//   err_yaw_ = set_yaw_ * math::deg_2_rad - INS.Yaw_rad;
// }

/**
 * @brief Callback function for the pitch motor.
 */
static void PitchMotorCallback() {
  offline_detect.Feed(DeviceId::DEV_PITCH_MOTOR);
  gimbal.motor_[1].Update();
}

/**
 * @brief Callback function for the yaw motor.
 */
static void YawMotorCallback() {
  offline_detect.Feed(DeviceId::DEV_YAW_MOTOR);
  gimbal.motor_[2].Update();
}

/**
 * @brief Callback function for the pitch follow motor.
 */
static void PitchFollowMotorCallback() {
  offline_detect.Feed(DeviceId::DEV_PITCH_FOLLOW_MOTOR);
  gimbal.motor_[0].Update();
}

