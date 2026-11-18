/**
 *******************************************************************************
 * @file      : ADRC.cpp
 * @brief     : Active disturbance rejection controller based on NTD and ESO
 * @history   :
 *  Version     Date          Author          Note
 *  V1.0.0      2026-08-31    origincy        1. Initial version
 *  V1.1.0      2026-09-13    origincy        1. ESO新增二阶/三阶可选
 *******************************************************************************
 */
 /* Includes ------------------------------------------------------------------*/
#include "ADRC.h"

#include "stm32f4xx.h"
#include "arm_math.h"
#include "bsp_dwt.h"

namespace {
    constexpr float kMinDt = 1.0e-5f;
    constexpr float kMinB0 = 1.0e-6f;
    constexpr float kDefaultBandwidth = 1.0f;

    float GetObserverStep(uint32_t* dwt_cnt) {
        const float dt = DWT_GetDeltaT(dwt_cnt);
        return dt > kMinDt ? dt : kMinDt;
    }
}

LESO::LESO()
    : dwt_cnt_(0), bandwidth_(kDefaultBandwidth), b0_(1.0f),
      order_(ESOOrder::THIRD),
      z1_(0.0f), z2_(0.0f), z3_(0.0f) {}

void LESO::Init(float bandwidth, float b0, ESOOrder order) {
    bandwidth_ = fabsf(bandwidth) > kMinDt ? fabsf(bandwidth) : kDefaultBandwidth;
    b0_ = fabsf(b0) > kMinB0 ? b0 : 1.0f;
    order_ = order;
    Reset();
}

void LESO::Update(float feedback, float control) {
    const float step = GetObserverStep(&dwt_cnt_);
    const float error = z1_ - feedback;

    if (order_ == ESOOrder::SECOND) {
        const float beta1 = 2.0f * bandwidth_;
        const float beta2 = bandwidth_ * bandwidth_;

        z1_ += step * (z2_ - beta1 * error);
        z2_ += step * (-beta2 * error + b0_ * control);
        return;
    }

    const float beta1 = 3.0f * bandwidth_;
    const float beta2 = 3.0f * bandwidth_ * bandwidth_;
    const float beta3 = bandwidth_ * bandwidth_ * bandwidth_;

    z1_ += step * (z2_ - beta1 * error);
    z2_ += step * (z3_ - beta2 * error + b0_ * control);
    z3_ += step * (-beta3 * error);
}

void LESO::Reset() {
    z1_ = 0.0f;
    z2_ = 0.0f;
    z3_ = 0.0f;
    dwt_cnt_ = 0;
}

NLESO::NLESO()
    : dwt_cnt_(0), b0_(1.0f),
      beta1_(0.0f), beta2_(0.0f), beta3_(0.0f), delta_(0.001f),
      order_(ESOOrder::THIRD),
      z1_(0.0f), z2_(0.0f), z3_(0.0f) {}

void NLESO::Init(float b0, float beta1, float beta2, float beta3, float delta,
                 ESOOrder order) {
    b0_ = fabsf(b0) > kMinB0 ? b0 : 1.0f;
    beta1_ = beta1;
    beta2_ = beta2;
    beta3_ = beta3;
    delta_ = delta > 0.0f ? delta : 0.001f;
    order_ = order;
    Reset();
}

void NLESO::Update(float feedback, float control) {
    const float step = GetObserverStep(&dwt_cnt_);
    const float error = z1_ - feedback;

    z1_ += step * (z2_ - beta1_ * error);

    if (order_ == ESOOrder::SECOND) {
        z2_ += step * (-beta2_ * Fal(error, 1, delta_) + b0_ * control);
        return;
    }

    z2_ += step * (z3_ - beta2_ * Fal(error, 1, delta_) + b0_ * control);
    z3_ += step * (-beta3_ * Fal(error, 2, delta_));
}

void NLESO::Reset() {
    z1_ = 0.0f;
    z2_ = 0.0f;
    z3_ = 0.0f;
    dwt_cnt_ = 0;
}

float NLESO::Sign(const float value) {
    return value > 0.0f ? 1.0f : (value == 0.0f ? 0.0f : -1.0f);
}

float NLESO::Fal(float value, uint8_t n, float delta) {
    if (delta <= 0.0f) {
        return 0.0f;
    }

    float root = fabsf(value);
    for (uint8_t i = 0; i < n; ++i) {
        float next_root;
        arm_sqrt_f32(root, &next_root);
        root = next_root;
    }

    if (fabsf(value) <= delta) {
        float delta_root = delta;
        for (uint8_t i = 0; i < n; ++i) {
            float next_root;
            arm_sqrt_f32(delta_root, &next_root);
            delta_root = next_root;
        }
        return value * delta_root / delta;
    }

    return Sign(value) * root;
}

ADRC::ADRC()
    : dwt_cnt_(0), dt_(0.0f),
    ntd_rate_(0.0f), b0_(1.0f),
    nlsef_rate_(0.0f), damp_(0.0f), max_out_(0.0f),
    ref_(0.0f), fdb_(0.0f),
    eso_type_(ESOType::NLESO),
    eso_order_(ESOOrder::THIRD),
    x1_(0.0f), x2_(0.0f),
    z1_(0.0f), z2_(0.0f), z3_(0.0f),
    output_(0.0f) {}

void ADRC::Init(float ntd_rate, float b0,
                float beta1, float beta2, float beta3,
                float nlsef_rate, float damp, float max_out, float delta,
                ESOType eso_type, float leso_bandwidth, ESOOrder eso_order) {
    ntd_rate_ = fabsf(ntd_rate);
    b0_ = std::fabs(b0) > kMinB0 ? b0 : 1.0f;
    nlsef_rate_ = fabsf(nlsef_rate);
    damp_ = damp;
    max_out_ = fabsf(max_out);
    eso_type_ = eso_type;
    eso_order_ = eso_order;
    nleso_.Init(b0_, beta1, beta2, beta3, delta, eso_order);
    leso_.Init(leso_bandwidth, b0_, eso_order);
    Reset();
}

void ADRC::Reset() {
    ref_ = 0.0f;
    fdb_ = 0.0f;
    x1_ = 0.0f;
    x2_ = 0.0f;
    nleso_.Reset();
    leso_.Reset();
    z1_ = 0.0f;
    z2_ = 0.0f;
    z3_ = 0.0f;
    output_ = 0.0f;
    dwt_cnt_ = 0;
    dt_ = 0.0f;
}

float ADRC::Calc() {
    dt_ = DWT_GetDeltaT(&dwt_cnt_);
    const float step = dt_ > kMinDt ? dt_ : kMinDt;

    //NTD
    const float tracking_error = x1_ - ref_;
    const float tracking_acceleration = Fhan(tracking_error, x2_, ntd_rate_, step);
    x1_ += step * x2_;
    x2_ += step * tracking_acceleration;

    if (eso_type_ == ESOType::LESO) {
        leso_.Update(fdb_, output_);
        z1_ = leso_.GetZ1();
        z2_ = leso_.GetZ2();
        z3_ = leso_.GetZ3();
    }
    else {
        nleso_.Update(fdb_, output_);
        z1_ = nleso_.GetZ1();
        z2_ = nleso_.GetZ2();
        z3_ = nleso_.GetZ3();
    }

    //NLSEF
    const float state_error = x1_ - z1_;
    const float state_rate_error = x2_ - z2_;
    const float disturbance = (eso_order_ == ESOOrder::THIRD) ? z3_ : z2_;
    const float control = (Fhan(state_error, damp_ * state_rate_error, nlsef_rate_, step) - disturbance) / b0_;
    output_ = Constrain(control, -max_out_, max_out_);
    return output_;
}

float ADRC::Sign(const float value) {
    return value > 0.0f ? 1.0f : (value == 0 ? 0.0f : -1.0f);
}

float ADRC::Fhan(float x1, float x2, float rate, float step) {
    if (rate <= 0.0f) {
        return 0.0f;
    }

    const float d = rate * step * step;
    const float a0 = step * x2;
    const float y = x1 + a0;
    float a1;
    arm_sqrt_f32(d * d + 8.0f * fabsf(y) * d, &a1);
    const float a2 = a0 + Sign(y) * (a1 - d) * 0.5f;
    const float sy = (Sign(y + d) - Sign(y - d)) * 0.5f;
    const float a = (a0 + y - a2) * sy + a2;
    const float sa = (Sign(a + d) - Sign(a - d)) * 0.5f;

    const float fhan = -rate * (a / d - Sign(a)) * sa - rate * Sign(a);
    return fhan;
}

float ADRC::Constrain(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}
