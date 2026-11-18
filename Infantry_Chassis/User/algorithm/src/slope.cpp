/**
 *******************************************************************************
 * @file      : slope.cpp
 * @brief     : 斜坡/梯形速度发生器
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
/** Includes ------------------------------------------------------------------*/
#include "slope.h"

Slope::Slope(float inc_per_step, float dec_per_step)
    : inc_step_(inc_per_step), dec_step_(dec_per_step) {
}

void Slope::Init(float inc_per_step, float dec_per_step) {
    inc_step_ = inc_per_step;
    dec_step_ = dec_per_step;
    out_ = 0.0f;
}

float Slope::Calc(float target, float feedback) {
    (void)feedback;

    float diff = target - out_;
    if (diff > 0.0f) {
        out_ += inc_step_;
        if (out_ > target) {
            out_ = target;
        }
    } else if (diff < 0.0f) {
        out_ -= dec_step_;
        if (out_ < target) {
            out_ = target;
        }
    }
    return out_;
}

void Slope::Reset(float out) {
    out_ = out;
}
