/**
 *******************************************************************************
 * @file      : PID.h
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
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PID_H_
#define PID_H_

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include <cstdint>
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
enum PID_improve : uint8_t {
    IMPROVE_NONE = 0b00000000,
    INTEGRAL_LIMIT = 0b00000001,
    DERIVATIVE_ON_MEASUREMENT = 0b00000010,
    TRAPEZOID_INTEGRAL = 0b00000100,
    PROPORTIONAL_ON_MEASUREMENT = 0b00001000,
    OUTPUT_FILTER = 0b00010000,
    CHANGING_INTEGRATION_RATE = 0b00100000,
    DERIVATIVE_FILTER = 0b01000000
};

constexpr uint8_t operator|(PID_improve a, PID_improve b) {
    return static_cast<uint8_t>(a) | static_cast<uint8_t>(b);
}
constexpr bool operator|(uint8_t a, PID_improve b) {
    return a | static_cast<uint8_t>(b);
}
constexpr bool operator|(PID_improve b, uint8_t a) {
    return a | static_cast<uint8_t>(b);
}
constexpr bool operator&(uint8_t flags, PID_improve bit) {
    return static_cast<bool>(flags & static_cast<uint8_t>(bit));
}


class PID_t final {
public:
    PID_t();
    void Init(float kp, float ki, float kd, float max_out, float dead_band = 0.0f) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        max_out_ = max_out;
        dead_band_ = dead_band;
    }

    void Improve(uint8_t improve, float i_limit, float coef_a, float coef_b, float output_lpf_rc,
                 float derivative_lpf_rc) {
        improve_ = improve;
        integral_limit_ = i_limit;
        coef_a_ = coef_a;
        coef_b_ = coef_b;
        output_lpf_rc_ = output_lpf_rc;
        derivative_lpf_rc_ = derivative_lpf_rc;
    }

    void SetRef(float ref) {
        ref_ = ref;
    }

    void SetFdb(float fdb) {
        fdb_ = fdb;
    }

    void Reset();
    float Calc();
    float Calc(float set_d, float fdb_d);
    void TrapezoidIntergral();
    void ChangingIntegratioRate();
    void IntegralLimit();
    void DerivativeOnMeasurement();
    void DerivativeFilter();
    void OutputFilter();
    void OutputLimit();
    [[nodiscard]] float GetOutput() const { return output_; }

private:
    uint32_t DWT_CNT;
    float dt;
    float kp_, ki_, kd_;
    float max_out_, dead_band_;

    uint8_t improve_;
    float integral_limit_;
    float coef_a_, coef_b_;
    float output_lpf_rc_, derivative_lpf_rc_; // seconds, based on the PID calculation period

    float ref_, fdb_, last_fdb_; // feedback values share the reference unit
    float err_, last_err_;
    float p_out_, i_out_, d_out_, last_d_out_, i_term_, last_i_term_;
    float output_, last_output_;
};

#endif

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
#endif //PID_H_
