/**
 *******************************************************************************
 * @file      : slope.h
 * @brief     : 斜坡/梯形速度发生器，用于速度软启动
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2026-05-24      origincy        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2026 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
/** Define to prevent recursive inclusion -------------------------------------*/
#ifndef SLOPE_H_
#define SLOPE_H_

#ifdef __cplusplus

class Slope {
public:
    Slope() = default;
    Slope(float inc_per_step, float dec_per_step);

    void Init(float inc_per_step, float dec_per_step);

    float Calc(float target, float feedback);

    void Reset(float out = 0.0f);
    float output() const { return out_; }

private:
    float out_ = 0.0f;
    float inc_step_ = 0.0f;
    float dec_step_ = 0.0f;
};

#endif

#endif /* SLOPE_H_ */
