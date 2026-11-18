#include "RLS.hpp"

#include "bsp_dwt.h"

namespace {
/* 积分窗口长度,20ms 对应 50Hz 的 RLS 推进率 */
constexpr float kWindowSeconds = 0.02f;
/* 单拍允许的最大间隔,超时说明调用出现空档(急停/自动瞄准分支),丢弃当前窗口 */
constexpr float kMaxStepSeconds = 0.01f;

/* delta 为协方差初值,lambda 为遗忘因子(50Hz 下约 2 秒记忆) */
Core::Control::Math::RLS<2> rls(100.0f, 0.99f);

Matrixf<2, 1> sample = Matrixf<2, 1>::zeros();

uint32_t dwt_cnt = 0;
bool seeded = false;
float last_torque = 0.0f;
float last_velocity = 0.0f;
float sum_delta_velocity = 0.0f;  // 窗口内 integral(d(omega)/dt dt) = delta_omega
float sum_velocity_dt = 0.0f;     // 窗口内 integral(omega dt)
float sum_torque_dt = 0.0f;       // 窗口内 integral(tau dt)
float window_time = 0.0f;

/**
 * @brief 以当前采样点作为新窗口的起点
 */
void ResetWindow(float torque, float velocity) {
    seeded = true;
    last_torque = torque;
    last_velocity = velocity;
    sum_delta_velocity = 0.0f;
    sum_velocity_dt = 0.0f;
    sum_torque_dt = 0.0f;
    window_time = 0.0f;
}
}  // namespace

float J, B;

void UpdateRLS(float torque, float velocity) {
    const float step = DWT_GetDeltaT(&dwt_cnt);

    // 首次调用、调用空档或异常步长:丢弃当前窗口并重新播种
    if (seeded == false || step <= 0.0f || step > kMaxStepSeconds) {
        ResetWindow(torque, velocity);
        return;
    }

    // 梯形法累加一个采样周期
    sum_delta_velocity += velocity - last_velocity;
    sum_velocity_dt += 0.5f * (velocity + last_velocity) * step;
    sum_torque_dt += 0.5f * (torque + last_torque) * step;
    window_time += step;

    last_torque = torque;
    last_velocity = velocity;

    if (window_time < kWindowSeconds) {
        return;
    }

    sample[0][0] = sum_delta_velocity;
    sample[1][0] = sum_velocity_dt;

    const auto& params = rls.update(sample, sum_torque_dt);

    J = params[0][0];
    B = params[1][0];

    ResetWindow(torque, velocity);
}
