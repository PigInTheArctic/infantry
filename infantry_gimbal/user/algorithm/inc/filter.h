/**
 *******************************************************************************
 * @file      : filter.h
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
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FILTER_H_
#define FILTER_H_

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include <cstring>
#include <stm32f4xx.h>
#include "arm_math.h"
#include "user_lib.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

typedef enum {
    FILTER_BIQUAD_LOWPASS = 0U,
    FILTER_BIQUAD_HIGHPASS,
    FILTER_BIQUAD_NOTCH,
} Enum_Filter_Biquad_Type;

/**
 * @brief 单个二阶 IIR 滤波器封装
 *
 * 基于 CMSIS-DSP 的 `arm_biquad_cascade_df1_f32` 实现。
 * 虽然底层接口叫 cascade，但当 stage = 1 时，本质上就是一个二阶滤波器。
 *
 * 系数格式为：
 * `b0, b1, b2, a1, a2`
 *
 * 差分方程可理解为：
 * `y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]`
 *
 * 说明：
 * 1. `Init()` 可直接按截止频率/Q 值生成常用二阶滤波器
 * 2. `InitByCoefficient()` 可直接使用外部设计好的系数
 * 3. 适合控制回路中的低通、高通、陷波
 */
class Class_Filter_Biquad {
public:
    /**
     * @brief 初始化单个二阶滤波器
     *
     * @param _sample_freq 采样频率，单位 Hz
     * @param _cutoff_freq 截止频率/中心频率，单位 Hz
     * @param _type 滤波器类型：低通/高通/陷波
     * @param _quality_factor 品质因数 Q，默认 0.70710678，对应较常见的二阶 Butterworth
     * @retval ARM_MATH_SUCCESS 初始化成功
     * @retval ARM_MATH_ARGUMENT_ERROR 参数非法
     */
    arm_status Init(float _sample_freq,
                    float _cutoff_freq,
                    Enum_Filter_Biquad_Type _type = FILTER_BIQUAD_LOWPASS,
                    float _quality_factor = 0.70710678f);
    /**
     * @brief 初始化单个二阶滤波器
     *
     * 这个重载主要是为了显式传入 Q 值后再指定类型。
     *
     * @param _sample_freq 采样频率，单位 Hz
     * @param _center_freq 截止频率/中心频率，单位 Hz
     * @param _quality_factor 品质因数 Q
     * @param _type 滤波器类型
     */
    arm_status Init(float _sample_freq,
                    float _center_freq,
                    float _quality_factor,
                    Enum_Filter_Biquad_Type _type);
    /**
     * @brief 直接按系数初始化
     *
     * @param _coefficient 系数数组，长度为 5，格式为 `{b0, b1, b2, a1, a2}`
     */
    arm_status InitByCoefficient(const float _coefficient[5]);
    /**
     * @brief 清空内部状态
     *
     * 会清零历史输入/输出状态，适合在模式切换或上电复位时调用。
     */
    void Reset();
    /**
     * @brief 单点计算
     *
     * 每次输入一个采样点，输出一个采样点。
     *
     * @param _input 当前输入
     * @return float 当前输出
     */
    float Calc(float _input);
    /**
     * @brief 批量计算
     *
     * @param _input 输入数组
     * @param _output 输出数组
     * @param _length 本次处理长度
     */
    void Calc(const float* _input, float* _output, uint32_t _length);
    /**
     * @brief 获取当前使用的滤波器系数
     *
     * 返回顺序固定为：`b0, b1, b2, a1, a2`
     */
    const float* GetCoefficient() const {
        return coefficient_;
    }

protected:
    /**
     * @brief 写入系数并重建 CMSIS-DSP 滤波器实例
     */
    void SetCoefficient(float _b0, float _b1, float _b2, float _a1, float _a2);

    arm_biquad_casd_df1_inst_f32 filter_;
    float coefficient_[5]{};
    float state_[4]{};
    float output_{ 0.0f };
};

template<uint8_t Filter_Stage_Num, uint16_t Filter_Block_Size = 1>
class Class_Filter_Biquad_Cascade {
public:
    /**
     * @brief 初始化多级二阶节级联滤波器
     *
     * 每一级都是一个二阶节，每一级占用 5 个系数：
     * `{b0, b1, b2, a1, a2}`
     *
     * 所以总系数长度为：
     * `Filter_Stage_Num * 5`
     *
     * 例如：
     * - 1 级：二阶滤波器
     * - 2 级：四阶滤波器
     * - 3 级：六阶滤波器
     *
     * @param _coefficient 级联系数数组
     */
    arm_status Init(const float _coefficient[Filter_Stage_Num * 5]) {
        std::memcpy(coefficient_, _coefficient, sizeof(coefficient_));
        Reset();
        arm_biquad_cascade_df1_init_f32(&filter_, Filter_Stage_Num, coefficient_, state_);
        return ARM_MATH_SUCCESS;
    }

    /**
     * @brief 清空级联滤波器内部状态
     */
    void Reset() {
        std::memset(state_, 0, sizeof(state_));
        output_ = 0.0f;
    }

    /**
     * @brief 单点计算
     */
    float Calc(float _input) {
        arm_biquad_cascade_df1_f32(&filter_, &_input, &output_, 1);
        return output_;
    }

    /**
     * @brief 批量计算
     *
     * @param _input 输入数组
     * @param _output 输出数组
     * @param _length 本次处理长度，不能超过模板参数 `Filter_Block_Size`
     * @retval true 处理成功
     * @retval false 输入长度超出模板上限
     */
    bool Calc(const float* _input, float* _output, uint16_t _length) {
        if (_length > Filter_Block_Size) {
            return false;
        }

        arm_biquad_cascade_df1_f32(&filter_, const_cast<float*>(_input), _output, _length);
        if (_length > 0U) {
            output_ = _output[_length - 1U];
        }
        return true;
    }

    float GetOutput() const {
        return output_;
    }

protected:
    arm_biquad_casd_df1_inst_f32 filter_{};
    float coefficient_[Filter_Stage_Num * 5]{};
    float state_[Filter_Stage_Num * 4]{};
    float output_{ 0.0f };
};

template<uint16_t Filter_FIR_Num_Taps, uint16_t Filter_Block_Size = 1>
class Class_Filter_FIR {
public:
    /**
     * @brief 初始化 FIR 滤波器
     *
     * `Filter_FIR_Num_Taps` 表示 FIR 抽头数，也就是系数个数。
     * 若一个 FIR 有 N 个系数，则其阶数通常是 `N - 1`。
     *
     * 例如：
     * - `Class_Filter_FIR<3>`：2 阶 FIR
     * - `Class_Filter_FIR<51>`：50 阶 FIR
     *
     * @param _coefficient FIR 系数数组，长度为 `Filter_FIR_Num_Taps`
     */
    arm_status Init(const float _coefficient[Filter_FIR_Num_Taps]) {
        std::memcpy(coefficient_, _coefficient, sizeof(coefficient_));
        Reset();
        arm_fir_init_f32(&filter_,
                         Filter_FIR_Num_Taps,
                         coefficient_,
                         state_,
                         Filter_Block_Size);
        return ARM_MATH_SUCCESS;
    }

    /**
     * @brief 清空 FIR 内部状态
     */
    void Reset() {
        std::memset(state_, 0, sizeof(state_));
        output_ = 0.0f;
    }

    /**
     * @brief 单点计算
     *
     * 本质上执行的是：
     * `y[n] = h[0]x[n] + h[1]x[n-1] + ... + h[N-1]x[n-(N-1)]`
     */
    float Calc(float _input) {
        arm_fir_f32(&filter_, &_input, &output_, 1);
        return output_;
    }

    /**
     * @brief 批量计算
     *
     * @param _input 输入数组
     * @param _output 输出数组
     * @param _length 本次处理长度，不能超过模板参数 `Filter_Block_Size`
     * @retval true 处理成功
     * @retval false 输入长度超出模板上限
     */
    bool Calc(const float* _input, float* _output, uint16_t _length) {
        if (_length > Filter_Block_Size) {
            return false;
        }

        arm_fir_f32(&filter_, const_cast<float*>(_input), _output, _length);
        if (_length > 0U) {
            output_ = _output[_length - 1U];
        }
        return true;
    }

    float GetOutput() const {
        return output_;
    }

protected:
    arm_fir_instance_f32 filter_{};
    float coefficient_[Filter_FIR_Num_Taps]{};
    float state_[Filter_FIR_Num_Taps + Filter_Block_Size - 1U]{};
    float output_{ 0.0f };
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#endif

#ifdef __cplusplus
extern "C" {
#endif

    /* Includes ------------------------------------------------------------------*/
    /* Exported macro ------------------------------------------------------------*/
    /* Exported constants --------------------------------------------------------*/
    /* Exported types ------------------------------------------------------------*/
    /* Exported variables --------------------------------------------------------*/
    /* Exported function prototypes ----------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __FILTER_H_ */
