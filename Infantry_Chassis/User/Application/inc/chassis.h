/**
 *******************************************************************************
 * @file      : chassis.h
 * @brief     : 力控舵轮底盘 — 双舵轮（RF+LB）+ 两全向轮
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2026-05-24      origincy        1. 初版，移植自 Sentinel force_crl
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2026 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CHASSIS_H_
#define CHASSIS_H_

#ifdef __cplusplus

/** Includes ------------------------------------------------------------------*/
#include "DJI_Motor.h"
#include "dm_motor.h"
#include "PID.h"
#include "slope.h"
#include "kalman_filter.h"
#include "user_lib.h"
#include <cstdint>

/** Exported macro ------------------------------------------------------------*/
/** Exported constants --------------------------------------------------------*/
/** Exported types ------------------------------------------------------------*/

/**
 * @brief 底盘模式枚举
 */
enum class ChassisMode : uint8_t
{
    ZERO = 0xAA,
    NO_MOVE = 0x0A,
    FOLLOW_GIMBAL_YAW = 0x0B,
    FOLLOW_GIMBAL_YAW_SLANT = 0x0F,
    SPIN = 0x05,
};

/**
 * @brief 底盘电机配置
 */
struct ChassisConfig
{
    dji_motor::MotorCfg Steer_RF;
    dji_motor::MotorCfg Steer_LB;
    dji_motor::MotorCfg Wheel_RF;
    dji_motor::MotorCfg Wheel_LB;
};

/**
 * @brief 力控舵轮底盘类
 */
class Chassis
{
public:
    Chassis(ChassisConfig *config);

    void Init();
    void Control();

    void SetMode(ChassisMode mode) { mode_ = mode; }
    ChassisMode mode() const { return mode_; }
    void SetSpeed(float vx, float vy, float vw)
    {
        vx_set_ = vx;
        vy_set_ = vy;
        vw_set_ = vw;
    }

    dji_motor::DJIMotor Wheel_RF_;
    dji_motor::DJIMotor Wheel_LB_;
    dji_motor::DJIMotor Steer_RF_;
    dji_motor::DJIMotor Steer_LB_;
    DM_MOTOR::DM_Motor Yaw_Motor_;

private:
    /** PID 控制器 */
    PID_t pid_vx_;
    PID_t pid_vy_;
    PID_t pid_vw_;
    PID_t follow_;
    PID_t wheel_vel_pid[2]; // RF, LB
    PID_t steer_ang_[2];
    PID_t steer_vel_[2];
    PID_t power_ctrl_;

    /** 坡道软启动 */
    Slope slope_vx_;
    Slope slope_vy_;

    static constexpr float kWheelRadius = 0.0625f;
    static constexpr float kMountDist = 0.240f;
    static constexpr float kSteerHoldSpeed = 0.08f; // m/s

    /** 状态 */
    ChassisMode mode_{ChassisMode::NO_MOVE};
    ChassisMode last_mode_{ChassisMode::NO_MOVE};

    float vx_set_ = 0.0f, vy_set_ = 0.0f, vw_set_ = 0.0f;
    float vx_ = 0.0f, vy_ = 0.0f, vw_ = 0.0f;

    float slope_ang_;        // 底盘倾斜角度
    float slope_toward_ang_; // 底盘相对坡面角度

    float steer_ang_ref_[2]{};
    float wheel_vel_ref_[2]{};

    float wheel_tor[2]{};

    int16_t steer_out[2]{};
    int16_t wheel_out[2]{};

    float yaw_angle_ = 0.0f;
    float yaw_angle_set_ = 0.0f;

    float power_limit_ = 100.0f;

    /** 速度融合 */
    KalmanFilter_t kf_v_{};
    uint32_t dwt_cnt_{};
    float dt_;

    void ModeSet();
    void FeedbackUpdate();
    void SetControl();
    void InverseKinematics();
    void NearestTransposition();
    void OutputToDynamics();
    void DynamicsInverseResolution();
    void OutputToMotor();
    void PowerLimit();
    void SteerMotorSend();
    void WheelMotorSend();
    void WheelPowerControl();
    static float RadFormat(float ang)
    {
        return math::LoopFloatConstrain(ang, -3.14159265f, 3.14159265f);
    }
};

#endif

#endif /* CHASSIS_H_ */
