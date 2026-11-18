/**
 *******************************************************************************
 * @file      : PID.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026/04/26     origincy        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2026 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
 /* Includes ------------------------------------------------------------------*/
#include "PID.h"
#include "cmath"
#include "bsp_dwt.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/

PID_t::PID_t() : DWT_CNT(0), dt(0), kp_(0), ki_(0), kd_(0), max_out_(0), dead_band_(0),
improve_(0), integral_limit_(0), coef_a_(0), coef_b_(0),
output_lpf_rc_(0), derivative_lpf_rc_(0),
ref_(0), fdb_(0), last_fdb_(0),
err_(0), last_err_(0),
p_out_(0), i_out_(0), d_out_(0), last_d_out_(0),
i_term_(0), last_i_term_(0),
output_(0), last_output_(0) {}

void PID_t::Reset() {
    ref_ = 0.0f;
    fdb_ = 0.0f;
    last_fdb_ = 0.0f;
    err_ = 0.0f;
    last_err_ = 0.0f;
    p_out_ = 0.0f;
    i_out_ = 0.0f;
    d_out_ = 0.0f;
    last_d_out_ = 0.0f;
    i_term_ = 0.0f;
    last_i_term_ = 0.0f;
    output_ = 0.0f;
    last_output_ = 0.0f;
    DWT_CNT = 0;
    dt = 0.0f;
}

/**
 * @brief Calculates the output of the PID controller.
 *
 * @retval The calculated output value
 */
float PID_t::Calc() {
    dt = DWT_GetDeltaT(&DWT_CNT);
    const float safe_dt = dt > 1e-5f ? dt : 1e-5f;
    err_ = ref_ - fdb_;

    if (std::abs(err_) > dead_band_) {
        p_out_ = kp_ * err_;
        i_term_ = ki_ * err_ * dt;
        d_out_ = kd_ * (err_ - last_err_) / safe_dt;

        if (improve_ & PID_improve::TRAPEZOID_INTEGRAL) {
            TrapezoidIntergral();
        }
        if (improve_ & PID_improve::CHANGING_INTEGRATION_RATE) {
            ChangingIntegratioRate();
        }
        if (improve_ & PID_improve::DERIVATIVE_ON_MEASUREMENT) {
            DerivativeOnMeasurement();
        }
        if (improve_ & PID_improve::DERIVATIVE_FILTER) {
            DerivativeFilter();
        }
        if (improve_ & PID_improve::INTEGRAL_LIMIT) {
            IntegralLimit();
        }
        i_out_ += i_term_;
        output_ = p_out_ + i_out_ + d_out_;
        if (improve_ & PID_improve::OUTPUT_FILTER) {
            OutputFilter();
        }
        OutputLimit();
    }
    else {
        output_ = 0.0f;
        i_term_ = 0.0f;
    }

    last_fdb_ = fdb_;
    last_err_ = err_;
    last_output_ = output_;
    last_d_out_ = d_out_;
    last_i_term_ = i_term_;

    return output_;
}

float PID_t::Calc(float set_d, float fdb_d) {
    dt = DWT_GetDeltaT(&DWT_CNT);
    err_ = ref_ - fdb_;

    if (std::abs(err_) > dead_band_) {
        p_out_ = kp_ * err_;
        i_term_ = ki_ * err_ * dt;
        d_out_ = kd_ * (set_d - fdb_d);

        if (improve_ & PID_improve::TRAPEZOID_INTEGRAL) {
            TrapezoidIntergral();
        }
        if (improve_ & PID_improve::CHANGING_INTEGRATION_RATE) {
            ChangingIntegratioRate();
        }
        if (improve_ & PID_improve::DERIVATIVE_FILTER) {
            DerivativeFilter();
        }
        if (improve_ & PID_improve::INTEGRAL_LIMIT) {
            IntegralLimit();
        }
        i_out_ += i_term_;
        output_ = p_out_ + i_out_ + d_out_;
        if (improve_ & PID_improve::OUTPUT_FILTER) {
            OutputFilter();
        }
        OutputLimit();
    }
    else {
        output_ = 0.0f;
        i_term_ = 0.0f;
    }

    last_fdb_ = fdb_;
    last_err_ = err_;
    last_output_ = output_;
    last_d_out_ = d_out_;
    last_i_term_ = i_term_;

    return output_;
}

void PID_t::TrapezoidIntergral() {
    i_term_ = ki_ * ((err_ + last_err_) / 2.0f) * dt;
}

void PID_t::ChangingIntegratioRate() {
    if (err_ * i_out_ > 0) {
        if (std::abs(err_) <= coef_b_) {
            return;
        }
        if (std::abs(err_) <= (coef_a_ + coef_b_)) {
            i_term_ *= (coef_a_ - std::abs(err_) + coef_b_) / coef_a_;
        }
        else {
            i_term_ = 0;
        }
    }
}

void PID_t::IntegralLimit() {
    float temp_i_out = i_out_ + i_term_;
    float temp_output = p_out_ + temp_i_out + d_out_;
    if (std::abs(temp_output) > max_out_) {
        if (err_ * i_out_ > 0) {
            i_term_ = 0;
        }
    }

    if (temp_i_out > integral_limit_) {
        i_term_ = 0;
        i_out_ = integral_limit_;
    }
    if (temp_i_out < -integral_limit_) {
        i_term_ = 0;
        i_out_ = -integral_limit_;
    }
}

void PID_t::DerivativeOnMeasurement() {
    const float safe_dt = dt > 1e-5f ? dt : 1e-5f;
    d_out_ = kd_ * (last_fdb_ - fdb_) / safe_dt;
}

void PID_t::DerivativeFilter() {
    const float safe_dt = dt > 1e-5f ? dt : 1e-5f;
    const float denominator = derivative_lpf_rc_ + safe_dt;
    d_out_ = d_out_ * safe_dt / denominator + last_d_out_ * derivative_lpf_rc_ / denominator;
}

void PID_t::OutputFilter() {
    const float safe_dt = dt > 1e-5f ? dt : 1e-5f;
    const float denominator = output_lpf_rc_ + safe_dt;
    output_ = output_ * safe_dt / denominator + last_output_ * output_lpf_rc_ / denominator;
}

void PID_t::OutputLimit() {
    if (output_ > max_out_) {
        output_ = max_out_;
    }
    if (output_ < -max_out_) {
        output_ = -max_out_;
    }
}
