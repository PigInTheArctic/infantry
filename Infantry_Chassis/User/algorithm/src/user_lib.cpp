/**
 *******************************************************************************
 * @file      : user_lib.cpp
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
 /* Includes ------------------------------------------------------------------*/
#include "user_lib.h"
#include "arm_math.h"

namespace math {
    /* Private macro -------------------------------------------------------------*/
    /* Private constants ---------------------------------------------------------*/
    /* Private types -------------------------------------------------------------*/
    /* Private variables ---------------------------------------------------------*/
    /* External variables --------------------------------------------------------*/
    /* Private function declarations ---------------------------------------------*/
    /* Function prototypes -------------------------------------------------------*/

    /**
     * @brief 对arm_math库中的arm_sqrt_f32()进行封装
     * @param x 被开方数
     * @return 给定数字的平方根
     */
    float Sqrt(const float x) {
        float result = 0;
        arm_sqrt_f32(x, &result);
        return result;
    }

    /**
     * @brief 限制一个数字的绝对值
     * @param x 要限制的数
     * @param limit 限制量的绝对值
     * @return 限制后的数
     */
    float AbsLimit(float x, const float limit) {
        if (x > limit)
            x = limit;
        else if (x < -limit)
            x = -limit;
        return x;
    }

    /**
     * @brief 返回数的符号
     * @param x 需要判断符号的数
     * @return 1如果数为正或零，否则为-1
     */
    int8_t Sign(const float x) {
        return x > 0.0f ? 1 : -1;
    }

    /**
     * @brief 循环限制浮点数值在指定范围内
     * @param value 被限制的数
     * @param min_value 范围最小值
     * @param max_value 范围最大值
     * @return 限制后的值
     */
    float LoopFloatConstrain(float value, const float min_value, const float max_value) {
        if (max_value < min_value)
            return value;
        const float length = max_value - min_value;
        if (value < min_value) {
            while (value < min_value)
                value += length;
        }
        else if (value > max_value) {
            while (value > max_value)
                value -= length;
        }
        return value;
    }

    /**
     * @brief 将角度值格式化为-180至180度范围内
     * @param ang 要被格式化的角度
     * @return 格式化后的角度
     */
    float AngleFormat(const float ang) {
        return LoopFloatConstrain(ang, -180.0f, 180.0f);
    }

    /**
     * @brief 对小数进行近似
     * @param raw 近似前的浮点数值
     * @return 近似后的整数值
     */
    int16_t FloatRounding(const float raw) {
        auto result = static_cast<int16_t>(raw);
        if (raw - static_cast<float>(result) > 0.5f)
            result++;
        return result;
    }

    /**
     * @brief 单位化三维向量
     * @param v 三维向量，以数组形式的指针
     * @return 标准化后的三维向量指针
     */
    float* Normalized3d(float* v) {
        const float length = Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        v[0] /= length;
        v[1] /= length;
        v[2] /= length;
        return v;
    }

    /**
     * @brief 对三维向量进行求模
     * @param v 三维向量，以数组形式的指针
     * @return 返回向量的模
     */
    float NormOf3d(const float* v) {
        return Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }

    /**
     * @brief 对两三维向量进行叉乘
     * @param v1 叉乘左侧向量
     * @param v2 叉乘右侧向量
     * @param res 求得结果向量
     */
    void Cross3d(const float* v1, const float* v2, float* res) {
        res[0] = v1[1] * v2[2] - v1[2] * v2[1];
        res[1] = v1[2] * v2[0] - v1[0] * v2[2];
        res[2] = v1[0] * v2[1] - v1[1] * v2[0];
    }

    /**
     * @brief 对两三维向量进行点乘
     * @param v1 点乘左侧向量
     * @param v2 点乘右侧向量
     * @return 求得结果值
     */
    float Dot3d(const float* v1, const float* v2) {
        return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
    }

    /**
     * @brief 判断浮点数是否相等
     * @param a float
     * @param b float
     * @return bool
     */
    bool FloatEqual(const float a, const float b) {
        return fabs(a - b) < 1e-5f;
    }

    /**
     * @brief 调换2字节指针大小端
     * @param Address 原数据指针以及反转后的数据指针
     */
    void EndianReserve16(uint16_t* Address) {
        const auto temp_address_8 = reinterpret_cast<uint8_t*>(Address);
        *Address = temp_address_8[0] << 8 | temp_address_8[1];
    }
}
