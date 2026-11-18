/**
 *******************************************************************************
 * @file      : PowerRLS.hpp
 * @brief     : wrapper around RLS to estimate k1,k2 for power model
 * @history   :
 *  Version     Date            Author          Note
 *  V1.1.0      2024-9-23      origincy        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 ENTERPRIZE Team, HKUST.
 *  All Rights Reserved.
 *******************************************************************************
 */
 /* Includes ------------------------------------------------------------------*/
#pragma once
#include "RLS.hpp"
#include "Matrix.hpp"
#include "cap.h"

class PowerRLS {
public:
    PowerRLS(float delta = 1000.0f, float lambda = 0.995f) : rls(delta, lambda) {
        sample = Matrixf<2, 1>::zeros();
        last_k1 = 0.0f;
        last_k2 = 0.0f;
    }

    void reset() {
        rls.reset();
    }

    void setInitParams(float k1, float k2) {
        Matrixf<2, 1> init = Matrixf<2, 1>::zeros();
        init[0][0] = k1;
        init[1][0] = k2;
        rls.setParamVector(init);
    }

    // Read measurements from global `cap` and `chassis`, update RLS
    void update() {
        float P = cap.GetOutPower();
        float v_l = chassis.l_wheel_.GetSpeed();
        float v_r = chassis.r_wheel_.GetSpeed();
        float lT = chassis.l_wheel_T_;
        float rT = chassis.r_wheel_T_;
        float a0 = CtrlStaticError;

        float y = P - (lT * v_l + rT * v_r) - a0;

        sample[0][0] = lT * lT + rT * rT;
        sample[1][0] = v_l * v_l + v_r * v_r;

        auto params = rls.update(sample, y);
        last_k1 = params[0][0];
        last_k2 = params[1][0];
    }

    float getK1() const { return last_k1; }
    float getK2() const { return last_k2; }

private:
    Core::Control::Math::RLS<2> rls;
    Matrixf<2, 1> sample;
    float last_k1;
    float last_k2;
};
