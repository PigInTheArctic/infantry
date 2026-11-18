/**
 *******************************************************************************
 * @file      : chassis.cpp
 * @brief     : 力控舵轮底盘 — 双舵轮（RF+LB）+ 两全向轮
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2026-05-24      origincy        1. note
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2026 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
/** Includes ------------------------------------------------------------------*/
#include "chassis.h"

#include "dr16.h"
#include "ins.h"
#include "bsp_dwt.h"
#include "user_lib.h"
#include "arm_math.h"
#include "board_comm.h"
#include "referee.h"
#include "cap.h"
#include "dm_motor.h"
/** Private macro -------------------------------------------------------------*/
/** Private constants ---------------------------------------------------------*/
/** Private types -------------------------------------------------------------*/
/** Private variables ---------------------------------------------------------*/
/** External variables --------------------------------------------------------*/
/** Exported variables --------------------------------------------------------*/
/** Private function prototypes -----------------------------------------------*/
static void YawMotorCallback(void *context, uint8_t *data, uint8_t len);

float output1, output2;
static uint16_t count;

namespace
{
    constexpr float max_speed = 2.0f;       // m/s
    constexpr float max_rotate_omege = 6.f; // rad/s

    constexpr float M3508Torque2Out = 3326.26322391f;
    constexpr float M3508Torque2Current = 4.060379912f;
    constexpr float M3508Current2Out = 819.2f;
    constexpr float GM6020Torque2Out = 7370.22042285f;
    constexpr float GM6020Torque2Current = 1.349527665f;
    constexpr float GM6020Current2Out = 5461.3333f;

    constexpr float wholeWeight = 10.f; // 整车重量(kg)
    constexpr float gravity = 9.8015f;  // 重力加速度(N/kg)
    /* 功率控制 */
    float scaled_m3508_power[2]{}; // 削减后的功率
    float power_scale = 0;

    float sumWheelPowerCmd = 0;
    float wheel_cur[2]{};
    float wheel_cmd_power[2]{};

    constexpr float GM6020_PWR_Coefficient[4] = {0.741f, -0.0005f, 6.0021f, 1.3715f};
    /* 268/17系数 */
    constexpr float M3508_PWR_Coefficient[4] = {0.246282373f, 1.325e-05f, 8.2543903e-02, 4.081f};

    inline void RotateToChassisFrame(float x_gimbal, float y_gimbal, float yaw,
                                     float &x_chassis, float &y_chassis)
    {
        const float cp = arm_cos_f32(yaw);
        const float sp = arm_sin_f32(yaw);
        x_chassis = x_gimbal * cp - y_gimbal * sp;
        y_chassis = x_gimbal * sp + y_gimbal * cp;
    }

    inline void RotateToGimbalFrame(float x_chassis, float y_chassis, float yaw,
                                    float &x_gimbal, float &y_gimbal)
    {
        const float cp = arm_cos_f32(yaw);
        const float sp = arm_sin_f32(yaw);
        x_gimbal = x_chassis * cp + y_chassis * sp;
        y_gimbal = -x_chassis * sp + y_chassis * cp;
    }

    float PowerFit(float K_PWR, float K_Fe, float K_Cu, float A, float vel, float cur);

    void PowerReso(float K_PWR, float K_Fe, float K_Cu, float A, float vel, float limit, float &cur);

}

/**
 * @brief 初始化电机成员
 */
Chassis::Chassis(ChassisConfig *config)
    : Wheel_RF_(config->Wheel_RF.hcan, config->Wheel_RF.id, config->Wheel_RF.ratio, config->Wheel_RF.offset, config->Wheel_RF.dev_id), Wheel_LB_(config->Wheel_LB.hcan, config->Wheel_LB.id, config->Wheel_LB.ratio, config->Wheel_LB.offset, config->Wheel_LB.dev_id), Steer_RF_(config->Steer_RF.hcan, config->Steer_RF.id, config->Steer_RF.ratio, config->Steer_RF.offset, config->Steer_RF.dev_id), Steer_LB_(config->Steer_LB.hcan, config->Steer_LB.id, config->Steer_LB.ratio, config->Steer_LB.offset, config->Steer_LB.dev_id) {}

/**
 * @brief PID、斜坡、卡尔曼滤波器参数配置
 */
void Chassis::Init()
{
    Yaw_Motor_.Init(0x50, 0x05, &hcan2, DM_MOTOR::MIT);
    if (Yaw_Motor_.pdm_motor_instance != nullptr)
        Yaw_Motor_.pdm_motor_instance->pCanCallBack = YawMotorCallback;
    pid_vx_.Init(8.0f, 0.0f, 0.1f, 20.0f);
    pid_vy_.Init(8.0f, 0.0f, 0.1f, 20.0f);
    pid_vw_.Init(1.0f, 0.0f, 0.0f, 5.0f);

    follow_.Init(8.f, 0.0f, 1.f, 9.0f, 0.f);
    follow_.Improve(INTEGRAL_LIMIT,
                    0.2f, 0.0f, 0.0f, 0.0f, 0.05f);
    pid_vw_.Improve(INTEGRAL_LIMIT,
                    1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    pid_vy_.Improve(INTEGRAL_LIMIT,
                    2.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    pid_vx_.Improve(INTEGRAL_LIMIT,
                    2.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    wheel_vel_pid[0].Init(2000, 0, 20, 16384);
    wheel_vel_pid[1].Init(2000, 0, 20, 16384);

    wheel_vel_pid[0].Improve(DERIVATIVE_FILTER | DERIVATIVE_ON_MEASUREMENT,
                             0, 0, 0, 0, 0.005f);
    wheel_vel_pid[1].Improve(DERIVATIVE_FILTER | DERIVATIVE_ON_MEASUREMENT,
                             0, 0, 0, 0, 0.005f);

    steer_ang_[0].Init(20.f, 0.0f, 0.0f, 100);    // 舵轮
    steer_vel_[0].Init(400.f, 0.0f, 0.0f, 16384); // 驱动轮
    steer_ang_[1].Init(20.f, 0.0f, 0.0f, 100);
    steer_vel_[1].Init(400.f, 0.0f, 0.0f, 16384);

    steer_ang_[0].Improve(INTEGRAL_LIMIT | DERIVATIVE_FILTER | DERIVATIVE_ON_MEASUREMENT,
                          25.0f, 0.0f, 0.0f, 0.0f, 0.005f);
    steer_vel_[0].Improve(INTEGRAL_LIMIT | CHANGING_INTEGRATION_RATE,
                          5000.0f, 0.2f, 0.0f, 0.0f, 0.0f);
    steer_ang_[1].Improve(INTEGRAL_LIMIT | DERIVATIVE_FILTER | DERIVATIVE_ON_MEASUREMENT,
                          25.0f, 0.0f, 0.0f, 0.0f, 0.005f);
    steer_vel_[1].Improve(INTEGRAL_LIMIT | CHANGING_INTEGRATION_RATE,
                          5000.0f, 0.2f, 0.0f, 0.0f, 0.0f);

    power_ctrl_.Init(1.4f, 0.0f, 0.1f, 10.0f);

    slope_vx_.Init(5.0f / 500.0f, 10.0f / 500.0f);
    slope_vy_.Init(5.0f / 500.0f, 10.0f / 500.0f);

    power_limit_ = 100.0f; // 100W的功率控制

    Kalman_Filter_Init(&kf_v_, 3, 2, 3);
    kf_v_.UseAutoAdjustment = 0;

    constexpr float dt = 0.002f;
    const float F_data[9] = {
        1,
        0,
        0,
        0,
        1,
        0,
        0,
        0,
        1,
    };
    const float B_data[6] = {dt, 0, 0, dt, 0, 0};
    const float H_data[9] = {1, 0, 0,
                             0, 1, 0,
                             0, 0, 1};
    const float Q_data[9] = {0.5f, 0, 0,
                             0, 0.5f, 0,
                             0, 0, 5.0f};
    const float R_data[9] = {0.5f, 0, 0,
                             0, 0.5f, 0,
                             0, 0, 2.0f};
    const float P0_data[9] = {1, 0, 0,
                              0, 1, 0,
                              0, 0, 1};
    const float min_var[4] = {0.01f, 0.01f, 0.001f, 0.0f};
    memcpy(kf_v_.F_data, F_data, sizeof(F_data));
    memcpy(kf_v_.B_data, B_data, sizeof(B_data));
    memcpy(kf_v_.H_data, H_data, sizeof(H_data));
    memcpy(kf_v_.Q_data, Q_data, sizeof(Q_data));
    memcpy(kf_v_.R_data, R_data, sizeof(R_data));
    memcpy(kf_v_.P_data, P0_data, sizeof(P0_data));
    memcpy(kf_v_.Pminus_data, P0_data, sizeof(P0_data));
    memcpy(kf_v_.StateMinVariance, min_var, sizeof(min_var));
}

void Chassis::Control()
{
    ModeSet();
    FeedbackUpdate();
    SetControl();
    InverseKinematics();
    NearestTransposition();
    OutputToDynamics();
    DynamicsInverseResolution();
    OutputToMotor();
}

void Chassis::ModeSet()
{
    if (board_comm.IsOnline())
    {
        mode_ = board_comm.GetChassisMode();
    }
    else
    {
        /* 断链保护：超时未收到云台数据时强制停车，避免保持最后速度 */
        mode_ = ChassisMode::NO_MOVE;
    }

    if (last_mode_ == mode_)
    {
    }
    else
    {
        if (mode_ == ChassisMode::FOLLOW_GIMBAL_YAW ||
            mode_ == ChassisMode::SPIN)
        {
            yaw_angle_set_ = 0.0f;
        }
        if (mode_ == ChassisMode::FOLLOW_GIMBAL_YAW_SLANT)
        {
            yaw_angle_set_ = PI * 0.25f;
        }

        vx_set_ = 0.0f;
        vy_set_ = 0.0f;
        vw_set_ = 0.0f;
        slope_vx_.Reset();
        slope_vy_.Reset();
        follow_.Reset();
        pid_vw_.Reset();
    }
    last_mode_ = mode_;
}

void Chassis::FeedbackUpdate()
{
    Yaw_Motor_.Update();
    float psi = Yaw_Motor_.GetAngle() * DEG_2_RAD;

    /* 正运动学：两轮速度 → 云台坐标系 Vx, Vy, Vw */
    float wrf = Wheel_RF_.vel(), wlb = Wheel_LB_.vel();
    float srf = Steer_RF_.pos(), slb = Steer_LB_.pos();
    float cos_rf = arm_cos_f32(srf - psi - yaw_angle_set_);
    float sin_rf = arm_sin_f32(srf - psi - yaw_angle_set_);
    float cos_lb = arm_cos_f32(slb - psi - yaw_angle_set_);
    float sin_lb = arm_sin_f32(slb - psi - yaw_angle_set_);

    float vx_w = (wrf * cos_rf + wlb * cos_lb) * kWheelRadius * 0.5f;
    float vy_w = (wrf * sin_rf + wlb * sin_lb) * kWheelRadius * 0.5f;
    float vw_w = (wrf * arm_sin_f32(srf - psi - yaw_angle_set_ - PI * 0.25f) +
                  wlb * arm_sin_f32(slb - psi - yaw_angle_set_ + PI * 0.75f)) *
                 kWheelRadius * 0.5f / kMountDist;

    /* 控制量：MotionAccel_b 旋转到云台系 */
    RotateToGimbalFrame(INS.MotionAccel_b[0] * gravity, INS.MotionAccel_b[1] * gravity,
                        -psi,
                        kf_v_.ControlVector[0], kf_v_.ControlVector[1]);

    /* 量测：正运动学 + 陀螺仪 */
    kf_v_.MeasuredVector[0] = vx_w;
    kf_v_.MeasuredVector[1] = vy_w;
    kf_v_.MeasuredVector[2] = vw_w;

    /* 动态 dt 更新 B 矩阵 */
    dt_ = DWT_GetDeltaT(&dwt_cnt_);
    kf_v_.B_data[0] = dt_;
    kf_v_.B_data[3] = dt_;

    /* KF 更新 */
    float *result = Kalman_Filter_Update(&kf_v_);
    vx_ = result[0];
    vy_ = result[1];
    vw_ = result[2];

    /* 底盘相对云台偏航角反馈 */
    yaw_angle_ = psi;
}

void Chassis::SetControl()
{
    auto normalize = [](int16_t v) -> float
    {
        if (math::Abs(v) > 66)
            return static_cast<float>(max_speed * v) / 660.0f;
        return 0.f;
    };
    constexpr float kFollowYawDeadband = 0.1f;

    vx_set_ = normalize(board_comm.GetXSpeed());
    vy_set_ = normalize(board_comm.GetYSpeed());

    vx_set_ = math::Abs(vx_set_) > 0.05f ? vx_set_ : 0.0f;
    vy_set_ = math::Abs(vy_set_) > 0.05f ? vy_set_ : 0.0f;

    const float qw = INS.q[0];
    const float qx = INS.q[1];
    const float qy = INS.q[2];
    const float qz = INS.q[3];
    const float gravity_x = 2.0f * (qx * qz - qw * qy);
    const float gravity_y = 2.0f * (qy * qz + qw * qx);
    const float gravity_z = 2.0f * (qw * qw - 0.5f + qz * qz);
    float gravity_horizontal = 0.0f;
    arm_sqrt_f32(gravity_x * gravity_x + gravity_y * gravity_y, &gravity_horizontal);

    slope_ang_ = 180.0f / PI * atan2f(gravity_horizontal, gravity_z);
    slope_toward_ang_ = gravity_horizontal > 1.0e-6f
                            ? 180.0f / PI * atan2f(gravity_x, gravity_y)
                            : 0.0f;

    switch (mode_)
    {
    case ChassisMode::ZERO:
    case ChassisMode::NO_MOVE:
        vx_set_ = 0.0f;
        vy_set_ = 0.0f;
        vw_set_ = 0.0f;
        board_comm.SetSpinSpeed(0);
        return;

    case ChassisMode::FOLLOW_GIMBAL_YAW:
    {
        float raw = RadFormat(Yaw_Motor_.GetAngle() * DEG_2_RAD);
        if (raw > PI * 0.5f)
            yaw_angle_set_ = PI;
        else if (raw < -PI * 0.5f)
            yaw_angle_set_ = -PI;
        else
            yaw_angle_set_ = 0;

        slope_vx_.Calc(vx_set_, vx_);
        slope_vy_.Calc(vy_set_, vy_);
        vx_set_ = slope_vx_.output();
        vy_set_ = slope_vy_.output();
        const float angle_error = RadFormat(yaw_angle_set_ - yaw_angle_);
        if (math::Abs(angle_error) < kFollowYawDeadband)
        {
            vw_set_ = 0.0f;
            follow_.Reset();
        }
        else
        {
            follow_.SetRef(0.0f);
            follow_.SetFdb(angle_error);
            follow_.Calc(0.0f, -Yaw_Motor_.GetSpd_rad()); // 设置角度环
            vw_set_ = -follow_.GetOutput();
        }
        board_comm.SetSpinSpeed(0);
        break;
    } // 底盘跟随

    case ChassisMode::FOLLOW_GIMBAL_YAW_SLANT:
    {
        float raw = RadFormat(Yaw_Motor_.GetAngle() * DEG_2_RAD);
        if (raw < 0.75f * PI && raw > -0.25f * PI)
            yaw_angle_set_ = 0.25f * PI;
        else
            yaw_angle_set_ = -0.75f * PI;

        slope_vx_.Calc(vx_set_, vx_);
        slope_vy_.Calc(vy_set_, vy_);
        vx_set_ = slope_vx_.output();
        vy_set_ = slope_vy_.output();
        const float angle_error = RadFormat(yaw_angle_set_ - yaw_angle_);
        if (math::Abs(angle_error) < kFollowYawDeadband)
        {
            vw_set_ = 0.0f;
            follow_.Reset();
        }
        else
        {
            follow_.SetRef(0.0f);
            follow_.SetFdb(angle_error);
            follow_.Calc(0.0f, -Yaw_Motor_.GetSpd_rad());
            vw_set_ = -follow_.GetOutput();
        }
        board_comm.SetSpinSpeed(0);
        break;
    }

    case ChassisMode::SPIN:
        slope_vx_.Calc(vx_set_, vx_);
        slope_vy_.Calc(vy_set_, vy_);
        vx_set_ = slope_vx_.output();
        vy_set_ = slope_vy_.output();
        vw_set_ = -max_rotate_omege;
        board_comm.SetSpinSpeed((int16_t)(INS.Gyro[2] * 10.0f));
        break; // 旋转
    default:
        vx_set_ = 0.0f;
        vy_set_ = 0.0f;
        vw_set_ = 0.0f;
        board_comm.SetSpinSpeed(0);
        return;
    }
}

/**
 * @brief 求逆解
 */
void Chassis::InverseKinematics()
{
    static constexpr float cos45 = 0.70710678f;
    float x_mount = kMountDist * cos45;
    float y_mount = kMountDist * cos45;
    float vx_body, vy_body;
    RotateToChassisFrame(vx_set_, vy_set_, -yaw_angle_, vx_body, vy_body);

    /* RF 轮 */
    float vx_r = vx_body - vw_set_ * y_mount;
    float vy_r = vy_body + vw_set_ * x_mount;
    float mod_r;
    arm_sqrt_f32(vx_r * vx_r + vy_r * vy_r, &mod_r);

    if (mod_r < kSteerHoldSpeed)
    {
        if (slope_ang_ > 5.f)
            steer_ang_ref_[0] = slope_toward_ang_ * PI / 180.0f;
        else
            steer_ang_ref_[0] = Steer_RF_.pos();
        wheel_vel_ref_[0] = 0.0f;
    }
    else
    {
        wheel_vel_ref_[0] = mod_r / kWheelRadius;
        steer_ang_ref_[0] = atan2f(vy_r, vx_r);
    }

    /* LB 轮 */
    float vx_l = vx_body + vw_set_ * y_mount;
    float vy_l = vy_body - vw_set_ * x_mount;
    float mod_l;
    arm_sqrt_f32(vx_l * vx_l + vy_l * vy_l, &mod_l);

    if (mod_l < kSteerHoldSpeed)
    {
        if (slope_ang_ > 5.f)
            steer_ang_ref_[1] = slope_toward_ang_ * PI / 180.0f;
        else
            steer_ang_ref_[1] = Steer_LB_.pos();
        wheel_vel_ref_[1] = 0.0f;
    }
    else
    {
        wheel_vel_ref_[1] = mod_l / kWheelRadius;
        steer_ang_ref_[1] = atan2f(vy_l, vx_l);
    }
}

/**
 * @brief 最近角处理
 */
void Chassis::NearestTransposition()
{
    float delta = RadFormat(steer_ang_ref_[0] - Steer_RF_.pos());

    if (-PI / 2.0f <= delta && delta <= PI / 2.0f)
    {
        steer_ang_ref_[0] = delta + Steer_RF_.pos();
    }
    else
    {
        steer_ang_ref_[0] = RadFormat(delta + PI) + Steer_RF_.pos();
        wheel_vel_ref_[0] *= -1.0f;
    }

    delta = RadFormat(steer_ang_ref_[1] - Steer_LB_.pos());

    if (-PI / 2.0f <= delta && delta <= PI / 2.0f)
    {
        steer_ang_ref_[1] = delta + Steer_LB_.pos();
    }
    else
    {
        steer_ang_ref_[1] = RadFormat(delta + PI) + Steer_LB_.pos();
        wheel_vel_ref_[1] *= -1.0f;
    }
}

/**
 * @brief PID 计算期望合力矩
 */
void Chassis::OutputToDynamics()
{
    if (mode_ == ChassisMode::NO_MOVE || mode_ == ChassisMode::ZERO)
    {
        pid_vx_.Reset();
        pid_vy_.Reset();
        pid_vw_.Reset();
        return;
    }

    pid_vx_.SetRef(vx_set_);
    pid_vx_.SetFdb(vx_);
    pid_vx_.Calc();

    pid_vy_.SetRef(vy_set_);
    pid_vy_.SetFdb(vy_);
    pid_vy_.Calc();

    pid_vw_.SetRef(vw_set_);
    pid_vw_.SetFdb(vw_);
    pid_vw_.Calc();
}
/**
 * @brief 动态求逆解
 */
void Chassis::DynamicsInverseResolution()
{
    wheel_vel_pid[0].SetRef(wheel_vel_ref_[0]);
    wheel_vel_pid[0].SetFdb(Wheel_RF_.vel());

    wheel_vel_pid[1].SetRef(wheel_vel_ref_[1]);
    wheel_vel_pid[1].SetFdb(Wheel_LB_.vel());

    wheel_vel_pid[0].Calc();
    wheel_vel_pid[1].Calc();

    float fx_body = 0, fy_body = 0;
    RotateToChassisFrame(pid_vx_.GetOutput(), pid_vy_.GetOutput(), -yaw_angle_, fx_body, fy_body);
    float tq = -pid_vw_.GetOutput();

    float fx_g_comp = 0, fy_g_comp = 0;
    if (mode_ != ChassisMode::NO_MOVE && mode_ != ChassisMode::ZERO && slope_ang_ > 5.0f)
    {
        const float slope_ang_rad = slope_ang_ * PI / 180.0f;
        const float slope_toward_ang_rad = slope_toward_ang_ * PI / 180.0f;
        const float g_comp = wholeWeight * gravity * arm_sin_f32(slope_ang_rad);
        fx_g_comp = g_comp * arm_cos_f32(slope_toward_ang_rad);
        fy_g_comp = g_comp * arm_sin_f32(slope_toward_ang_rad);
    }

    float sr = Steer_RF_.pos();
    float f_r = (fx_body + fx_g_comp) * arm_cos_f32(sr) + (fy_body + fy_g_comp) * arm_sin_f32(sr) - tq / kMountDist * arm_sin_f32(PI / 4.0f - sr);
    wheel_tor[0] = f_r * kWheelRadius;

    float sl = Steer_LB_.pos();
    float f_l = (fx_body + fx_g_comp) * arm_cos_f32(sl) + (fy_body + fy_g_comp) * arm_sin_f32(sl) - tq / kMountDist * arm_sin_f32(5.0f * PI / 4.0f - sl);
    wheel_tor[1] = f_l * kWheelRadius;

    if (mode_ != ChassisMode::SPIN)
    {
        wheel_out[0] = wheel_tor[0] * M3508Torque2Out + wheel_vel_pid[0].GetOutput();
        wheel_out[1] = wheel_tor[1] * M3508Torque2Out + wheel_vel_pid[1].GetOutput();
        output1 = wheel_tor[0] * M3508Torque2Out;
        output2 = wheel_vel_pid[0].GetOutput();
    }
    else
    {
        wheel_out[0] = wheel_vel_pid[0].GetOutput();
        wheel_out[1] = wheel_vel_pid[1].GetOutput();
    }
}

/**
 * @brief 输出力矩
 */
void Chassis::OutputToMotor()
{
    steer_ang_[0].SetRef(steer_ang_ref_[0]);
    steer_ang_[0].SetFdb(Steer_RF_.pos());
    steer_ang_[0].Calc();

    steer_vel_[0].SetRef(steer_ang_[0].GetOutput());
    steer_vel_[0].SetFdb(Steer_RF_.vel());
    steer_vel_[0].Calc();

    steer_ang_[1].SetRef(steer_ang_ref_[1]);
    steer_ang_[1].SetFdb(Steer_LB_.pos());
    steer_ang_[1].Calc();

    steer_vel_[1].SetRef(steer_ang_[1].GetOutput());
    steer_vel_[1].SetFdb(Steer_LB_.vel());
    steer_vel_[1].Calc();

    if (mode_ == ChassisMode::NO_MOVE || mode_ == ChassisMode::ZERO)
    {
        wheel_out[0] = 0;
        wheel_out[1] = 0;
        steer_out[0] = 0;
        steer_out[1] = 0;
    }
    else
    {
        PowerLimit();
    }

    SteerMotorSend();
    WheelMotorSend();

    // count++;
    // if (count % 10 == 0)
    // {
    //     Yaw_Motor_.MotorEnable();
    //     count = 0;
    // }
}

/**
 * @brief 功率控制
 */
void Chassis::PowerLimit()
{
    wheel_out[0] = math::Constrain<int16_t>(wheel_out[0], -16384, 16384);
    wheel_out[1] = math::Constrain<int16_t>(wheel_out[1], -16384, 16384);

    steer_out[0] = math::Constrain<int16_t>(steer_vel_[0].GetOutput(), -16384, 16384);
    steer_out[1] = math::Constrain<int16_t>(steer_vel_[1].GetOutput(), -16384, 16384);

    // WheelPowerControl();
}
// 底盘功率控制
void Chassis::WheelPowerControl()
{
    float input_power = 0; // 输入功率(缓冲能量环)

    power_limit_ = referee.game_robot_state_.chassis_power_limit; // 最终底盘的最大功率初始化

    power_ctrl_.SetFdb(referee.power_heat_data_.buffer_energy);
    power_ctrl_.SetRef(30);
    power_ctrl_.Calc();

    input_power = referee.game_robot_state_.chassis_power_limit - power_ctrl_.GetOutput(); // 通过裁判系统的最大功率
    // 将最小功率限制到45W防止掉线导致
    input_power = math::Constrain(input_power, 45.f, 200.f);
    cap.SetChargePower(input_power);

    if (cap.GetVolt() > 3.5f && referee.power_heat_data_.buffer_energy > 20)
    {
        // 云台 cap_flag 为 1 bit，当前只会上报 0/1，大超电分支（==2）暂时不会触发
        if ((cap.GetVolt() > 16.0f && board_comm.GetCapFlag() == 2 && mode_ != ChassisMode::SPIN)) // 大超电，用于飞坡，冲刺
            power_limit_ = 400;
        else if (board_comm.GetCapFlag() == 1) // 小超电，用于上坡，平地加速
        {
            if (cap.GetVolt() > 16)
                power_limit_ = input_power + 50;
            else if (cap.GetVolt() > 13 && cap.GetVolt() < 16)
                power_limit_ = input_power + 35;
            else
                power_limit_ = input_power + 15;
        }
        else // 被动超电
        {
            if (referee.game_robot_state_.chassis_power_limit > 55)
            { // 被动超电（限制高等级浪费）
                if (cap.GetVolt() > 18)
                    power_limit_ = 80;
                else
                    power_limit_ = 60;
            }
            else if (referee.game_robot_state_.chassis_power_limit < 40)
            { // 被动超电（补偿虚弱）
                if (cap.GetVolt() > 18)
                    power_limit_ = 50;
                else
                    power_limit_ = 45;
            }
            else
            {
                if (cap.GetVolt() > 22) // 被动超电（初始状态）
                    power_limit_ = input_power + 25;
                else if (cap.GetVolt() > 20 && cap.GetVolt() < 22)
                    power_limit_ = input_power + 20;
                else if (cap.GetVolt() > 18 && cap.GetVolt() < 20)
                    power_limit_ = input_power + 10;
                else
                    power_limit_ = input_power - 5;
            }
        }
    }
    else
    {
        power_limit_ = input_power;
    }

    wheel_cur[0] = wheel_out[0] / M3508Current2Out;
    wheel_cur[1] = wheel_out[1] / M3508Current2Out;
    /* 获得所有3508电机的功率和总功率 */
    wheel_cmd_power[0] = PowerFit(M3508_PWR_Coefficient[0], M3508_PWR_Coefficient[1], M3508_PWR_Coefficient[2],
                                  M3508_PWR_Coefficient[3], Wheel_RF_.vel(), wheel_cur[0]);
    wheel_cmd_power[1] = PowerFit(M3508_PWR_Coefficient[0], M3508_PWR_Coefficient[1], M3508_PWR_Coefficient[2],
                                  M3508_PWR_Coefficient[3], Wheel_LB_.vel(), wheel_cur[1]);

    sumWheelPowerCmd = wheel_cmd_power[0] + wheel_cmd_power[1];

    if (sumWheelPowerCmd > power_limit_) // 超功率模型衰减
    {
        power_scale = power_limit_ / sumWheelPowerCmd;
        scaled_m3508_power[0] = wheel_cmd_power[0] > 0 ? power_scale * wheel_cmd_power[0] : wheel_cmd_power[0];
        scaled_m3508_power[1] = wheel_cmd_power[1] > 0 ? power_scale * wheel_cmd_power[1] : wheel_cmd_power[1];

        PowerReso(M3508_PWR_Coefficient[0], M3508_PWR_Coefficient[1], M3508_PWR_Coefficient[2],
                  M3508_PWR_Coefficient[3], Wheel_RF_.vel(), scaled_m3508_power[0], wheel_cur[0]);
        PowerReso(M3508_PWR_Coefficient[0], M3508_PWR_Coefficient[1], M3508_PWR_Coefficient[2],
                  M3508_PWR_Coefficient[3], Wheel_LB_.vel(), scaled_m3508_power[1], wheel_cur[1]);
        wheel_out[0] = M3508Current2Out * wheel_cur[0];
        wheel_out[1] = M3508Current2Out * wheel_cur[1];
    }
}

/**
 * @brief 舵电机CAN发送函数
 */
void Chassis::SteerMotorSend()
{
    uint32_t tx_mailbox;
    CAN_TxHeaderTypeDef tx_conf;
    uint8_t tx_data_can1[8]{};
    tx_conf.StdId = 0x2FE;
    tx_conf.IDE = CAN_ID_STD;
    tx_conf.RTR = CAN_RTR_DATA;
    tx_conf.DLC = 0x08;
    tx_data_can1[2] = steer_out[0] >> 8;
    tx_data_can1[3] = steer_out[0];
    tx_data_can1[4] = steer_out[1] >> 8;
    tx_data_can1[5] = steer_out[1];
    HAL_CAN_AddTxMessage(&hcan1, &tx_conf, tx_data_can1, &tx_mailbox);
}

/**
 * @brief 轮电机CAN发送函数
 */
void Chassis::WheelMotorSend()
{
    uint32_t tx_mailbox;
    CAN_TxHeaderTypeDef tx_conf;
    uint8_t tx_data_can1[8]{};
    tx_conf.StdId = 0x200;
    tx_conf.IDE = CAN_ID_STD;
    tx_conf.RTR = CAN_RTR_DATA;
    tx_conf.DLC = 0x08;
    tx_data_can1[0] = wheel_out[0] >> 8;
    tx_data_can1[1] = wheel_out[0];
    tx_data_can1[4] = wheel_out[1] >> 8;
    tx_data_can1[5] = wheel_out[1];
    HAL_CAN_AddTxMessage(&hcan1, &tx_conf, tx_data_can1, &tx_mailbox);
}

/**
 * @brief 达妙 yaw 电机接收回调（在 CAN 接收中断中调用）
 * @note  数据解析由 FeedbackUpdate() 周期调用 Yaw_Motor_.Update() 完成，
 *        此处只负责刷新离线检测，避免中断中重复解析导致圈数重复累加
 */
static void YawMotorCallback(void * /*context*/, uint8_t * /*data*/, uint8_t /*len*/)
{
    offline_detect.Feed(DeviceId::DEV_YAW_MOTOR);
}

namespace
{
    void PowerReso(float K_PWR, float K_Fe, float K_Cu, float A, float vel, float limit, float &cur)
    {
        static float a, b, c, delta, h;
        float real1, real2;
        a = K_Cu;
        b = K_PWR * vel;
        c = K_Fe * vel * vel + A - limit;
        delta = b * b - 4 * a * c;
        if (delta < 0)
            cur = 0;
        else
        {
            arm_sqrt_f32(delta, &h);
            real1 = (-b + h) * 0.5 / a;
            real2 = (-b - h) * 0.5 / a;
            if (cur > 0)
                cur = real1;
            else
                cur = real2;
        }
    }

    /**
     * @brief 功率拟合函数，估计下一时刻功率
     * @param K_PWR 输出功率系数(将单位转换成扭矩乘速度)
     * @param K_Fe 铁损系数
     * @param K_Cu 铜损系数
     * @param A 常数项
     * @param vel 转速，单位rad/s
     * @param cur 电流
     */
    float PowerFit(float K_PWR, float K_Fe, float K_Cu, float A, float vel, float cur)
    {
        float pow = K_PWR * cur * vel + K_Fe * vel * vel + K_Cu * cur * cur + A;
        return pow;
    };
}
