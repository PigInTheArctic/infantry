/**
 *******************************************************************************
 * @file      : LQR.h
 * @brief     : 云台特供版
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-09-11     origincy        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2026 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
 /* Includes ------------------------------------------------------------------*/
#include "LQR.h"
#include "user_lib.h"
#include "stdint.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

//0 yaw; 1 pitch
const float k[2][2] = {
    // { 27.0379021f, 1.881892246f },      // yaw
    // { 23.83283876f, 1.214104842f }, // pitch
        { 23.80046189f, 1.22358269f },      // yaw
    { 19.48992918f, 1.144987554f }, // pitch
};

/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

void Lqr::Calc() {
    output_[0] = -k[0][0] * (yaw_ang_ - yaw_ang_ref_) - k[0][1] * (yaw_vel_ - yaw_vel_ref_);
    output_[1] = -k[1][0] * (pitch_ang_ - pitch_ang_ref_) - k[1][1] * (pitch_vel_ - pitch_vel_ref_);

    if (math::Abs(output_[0]) < 1.0f) {
        yaw_integral_ += 10 * (yaw_ang_ref_ - yaw_ang_) * dt_;
        yaw_integral_ = math::Constrain(yaw_integral_, -0.5f, 0.5f);
        output_[1] += yaw_integral_;
    }
    if (math::Abs(output_[1]) < 1.0f) {
        pitch_integral_ += 10 * (pitch_ang_ref_ - pitch_ang_) * dt_;
        pitch_integral_ = math::Constrain(pitch_integral_, -0.5f, 0.5f);
        output_[1] += pitch_integral_;
    }

}