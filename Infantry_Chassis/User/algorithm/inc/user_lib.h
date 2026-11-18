/**
 *******************************************************************************
 * @file      : user_lib.h
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026/2/9       origincy        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2026 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef USER_LIB_H_
#define USER_LIB_H_

#ifdef __cplusplus
#include <stdint.h>
/* Includes ------------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
namespace math {
    template <class T>
    constexpr void ValLimit(T& val, const T& min, const T& max) {
        if (val <= min)
            val = min;
        else if (val >= max)
            val = max;
    }

    template <class T>
    constexpr void AngleLimit360(T& val, const T& angle) {
        val = angle - static_cast<int>(angle);
        val += static_cast<int>(angle) % 360;
    }

    template <class T>
    constexpr T ValMax(const T& a, const T& b) {
        return a > b ? a : b;
    }

    template <class T>
    constexpr T ValMin(const T& a, const T& b) {
        return a < b ? a : b;
    }

    template <typename T>
    T Constrain(T x, T Min, T Max) {
        if (x < Min)
            x = Min;
        else if (x > Max)
            x = Max;
        return (x);
    }

    template <typename T>
    T Constrain(T* x, T Min, T Max) {
        if (*x < Min)
            *x = Min;
        else if (*x > Max)
            *x = Max;
        return (*x);
    }

    template <typename T>
    T Abs(T x) {
        return ((x > 0) ? x : -x);
    }

    /* Exported constants --------------------------------------------------------*/
    constexpr float pi = 3.141593f;
    constexpr float rad_2_deg = 57.29578f;
    constexpr float deg_2_rad = 0.017453f;
    /* Exported types ------------------------------------------------------------*/
    /* Exported variables --------------------------------------------------------*/
    /* Exported function prototypes ----------------------------------------------*/
    float Sqrt(float x);

    float AbsLimit(float x, float limit);

    int8_t Sign(float x);

    float LoopFloatConstrain(float value, float min_value, float max_value);

    float AngleFormat(float ang);

    int16_t FloatRounding(float raw);

    float* Normalized3d(float* v);

    float NormOf3d(const float* v);

    void Cross3d(const float* v1, const float* v2, float* res);

    float Dot3d(const float* v1, const float* v2);

    bool FloatEqual(float a, float b);
}

#endif

#endif /* USER_LIB_H_ */
