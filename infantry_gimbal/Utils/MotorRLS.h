#include "RLS.hpp"

/**
 * @brief 用一个积分窗口推进一次 RLS 辨识
 * @param torque   力矩指令(原始指令值,单位与 output_speed_ 一致)
 * @param velocity 机体系角速度,单位 rad/s
 * @note 拟合 tau = J * d(omega)/dt + B * omega 的积分形式:
 *       integral(tau dt) = J * delta_omega + B * integral(omega dt)
 *       因此不需要对角速度求导
 */
void UpdateRLS(float torque, float velocity);
