/**
 *******************************************************************************
 * @file      : filter.cpp
 * @brief     : Filter wrapper based on CMSIS-DSP
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-04-18      OpenAI          1. Add FIR and biquad wrappers
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2026 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
 /* Includes ------------------------------------------------------------------*/
#include "filter.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
namespace {
constexpr float k_filter_min_frequency = 1e-3f;
constexpr float k_filter_min_quality_factor = 1e-3f;
}
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

arm_status Class_Filter_Biquad::Init(float _sample_freq,
                                     float _cutoff_freq,
                                     Enum_Filter_Biquad_Type _type,
                                     float _quality_factor) {
    // 这个接口适合最常用的场景：
    // 已知采样频率和截止频率，直接生成一个二阶滤波器。
    return Init(_sample_freq, _cutoff_freq, _quality_factor, _type);
}

arm_status Class_Filter_Biquad::Init(float _sample_freq,
                                     float _center_freq,
                                     float _quality_factor,
                                     Enum_Filter_Biquad_Type _type) {
    if (_sample_freq <= 0.0f || _center_freq <= 0.0f ||
        _center_freq >= (_sample_freq * 0.5f)) {
        return ARM_MATH_ARGUMENT_ERROR;
    }

    _center_freq = math::Constrain(_center_freq,
                                   k_filter_min_frequency,
                                   _sample_freq * 0.5f - k_filter_min_frequency);
    _quality_factor = math::Constrain(_quality_factor,
                                      k_filter_min_quality_factor,
                                      1000.0f);

    float omega = 2.0f * PI * _center_freq / _sample_freq;
    float cos_omega = arm_cos_f32(omega);
    float sin_omega = arm_sin_f32(omega);
    float alpha = sin_omega / (2.0f * _quality_factor);

    // 下面使用的是常见 RBJ Audio EQ Cookbook 形式的二阶系数公式。
    // 这里虽然常见于音频领域，但同样适用于一般离散二阶滤波器实现。
    //
    // 先算出未归一化系数：
    // b0, b1, b2, a0, a1, a2
    //
    // 再统一除以 a0，变成标准形式：
    // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2]
    //      - a1*y[n-1] - a2*y[n-2]
    float b0 = 0.0f;
    float b1 = 0.0f;
    float b2 = 0.0f;
    float a0 = 1.0f;
    float a1 = 0.0f;
    float a2 = 0.0f;

    switch (_type) {
    case FILTER_BIQUAD_LOWPASS:
        // 二阶低通
        b0 = (1.0f - cos_omega) * 0.5f;
        b1 = 1.0f - cos_omega;
        b2 = (1.0f - cos_omega) * 0.5f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cos_omega;
        a2 = 1.0f - alpha;
        break;

    case FILTER_BIQUAD_HIGHPASS:
        // 二阶高通
        b0 = (1.0f + cos_omega) * 0.5f;
        b1 = -(1.0f + cos_omega);
        b2 = (1.0f + cos_omega) * 0.5f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cos_omega;
        a2 = 1.0f - alpha;
        break;

    case FILTER_BIQUAD_NOTCH:
        // 二阶陷波：保留大多数频率，抑制某一个中心频率附近
        b0 = 1.0f;
        b1 = -2.0f * cos_omega;
        b2 = 1.0f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cos_omega;
        a2 = 1.0f - alpha;
        break;

    default:
        return ARM_MATH_ARGUMENT_ERROR;
    }

    b0 /= a0;
    b1 /= a0;
    b2 /= a0;
    a1 /= a0;
    a2 /= a0;

    // CMSIS-DSP 的 biquad 系数顺序为：
    // {b0, b1, b2, a1, a2}
    //
    // 而很多教材写成：
    // y[n] = ... - a1*y[n-1] - a2*y[n-2]
    //
    // 所以这里把符号翻转后再存进去，便于和常见设计公式对齐。
    SetCoefficient(b0, b1, b2, -a1, -a2);

    return ARM_MATH_SUCCESS;
}

arm_status Class_Filter_Biquad::InitByCoefficient(const float _coefficient[5]) {
    if (_coefficient == nullptr) {
        return ARM_MATH_ARGUMENT_ERROR;
    }

    std::memcpy(coefficient_, _coefficient, sizeof(coefficient_));
    Reset();
    arm_biquad_cascade_df1_init_f32(&filter_, 1, coefficient_, state_);
    return ARM_MATH_SUCCESS;
}

void Class_Filter_Biquad::Reset() {
    // 二阶 DF1 结构每级需要 4 个状态量，这里全部清零。
    std::memset(state_, 0, sizeof(state_));
    output_ = 0.0f;
}

float Class_Filter_Biquad::Calc(float _input) {
    arm_biquad_cascade_df1_f32(&filter_, &_input, &output_, 1);
    return output_;
}

void Class_Filter_Biquad::Calc(const float* _input, float* _output, uint32_t _length) {
    arm_biquad_cascade_df1_f32(&filter_, const_cast<float*>(_input), _output, _length);
    if (_length > 0U) {
        output_ = _output[_length - 1U];
    }
}

void Class_Filter_Biquad::SetCoefficient(float _b0,
                                         float _b1,
                                         float _b2,
                                         float _a1,
                                         float _a2) {
    // CMSIS-DSP 单级 biquad 的系数排列固定为 5 个：
    // coefficient_[0] = b0
    // coefficient_[1] = b1
    // coefficient_[2] = b2
    // coefficient_[3] = a1
    // coefficient_[4] = a2
    coefficient_[0] = _b0;
    coefficient_[1] = _b1;
    coefficient_[2] = _b2;
    coefficient_[3] = _a1;
    coefficient_[4] = _a2;

    Reset();
    arm_biquad_cascade_df1_init_f32(&filter_, 1, coefficient_, state_);
}
