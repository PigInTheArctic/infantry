/**
 *******************************************************************************
 * @file      : INS.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
 /* Includes ------------------------------------------------------------------*/
#include "ins.h"
#include "bsp_dwt.h"
#include "offline_detect.h"
#include "pid.h"
#include "quaternion_ekf.h"
#include "tim.h"
#include "user_lib.h"
#include "cmath"
#include "Fusion_AHRS.h"
#include "filter.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
#define RAD_2_DEG 57.29578f           // 180/pi
#define DEG_2_RAD 0.017453292f           // pi/180
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
INS_t INS;
// 全局 AHRS 和偏置校正实例
FusionAhrs ahrs;
FusionOffset offset;

Class_Filter_Biquad gyro_z_filter;
/* Private function prototypes -----------------------------------------------*/

const float xb[3] = { 1, 0, 0 };
const float yb[3] = { 0, 1, 0 };
const float zb[3] = { 0, 0, 1 };

uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
static uint8_t yaw_initialized = 0;

void INS_Init(void) {
    yaw_initialized = 0;

    // 初始化 AHRS
    FusionAhrsInitialise(&ahrs);
    FusionAhrsSettings ahrs_settings;
    ahrs_settings.convention = FusionConventionNwu;
    ahrs_settings.gain = 0.5f;
    ahrs_settings.gyroscopeRange = 2000.0f;
    ahrs_settings.accelerationRejection = 8.0f;
    ahrs_settings.recoveryTriggerPeriod = 250;
    FusionAhrsSetSettings(&ahrs, &ahrs_settings);
    // 在 500 Hz 采样率下初始化偏置校正
    FusionOffsetInitialise(&offset, 500);
    IMU_QuaternionEKF_Init(10.0f, 0.001f, 10000000.0f, 1.0f, 0.0f);

    gyro_z_filter.Init(500, 100, FILTER_BIQUAD_HIGHPASS);
}

void INS_Task(void) {
    const float gravity[3] = { 0, 0, 9.805f };
    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;

    // ins update
    BMI088_Read(&BMI088);
    if (BMI088.Accel[X] != 0.f || BMI088.Accel[Y] != 0.f || BMI088.Accel[Z] != 0.f)
        offline_detect.Feed(DeviceId::DEV_INS);

    INS.Accel[X] = BMI088.Accel[X];
    INS.Accel[Y] = BMI088.Accel[Y];
    INS.Accel[Z] = BMI088.Accel[Z];

    INS.Gyro[X] = BMI088.Gyro[X];
    INS.Gyro[Y] = BMI088.Gyro[Y];   
    INS.Gyro[Z] = BMI088.Gyro[Z];

    // 读取原始传感器数据（单位：deg/s, g）
    FusionVector raw_gyro = { .axis = { INS.Gyro[X] * math::rad_2_deg, INS.Gyro[Y] * math::rad_2_deg, INS.Gyro[Z] * math::rad_2_deg } };
    FusionVector accel = { .axis = { INS.Accel[X] / gravity[2], INS.Accel[Y] / gravity[2], INS.Accel[Z] / gravity[2] } };

    // 校正陀螺仪偏置
    FusionVector gyro = FusionOffsetUpdate(&offset, raw_gyro);
    INS.Gyro[X] = gyro.axis.x * math::deg_2_rad;
    INS.Gyro[Y] = gyro.axis.y * math::deg_2_rad;
    INS.Gyro[Z] = gyro.axis.z * math::deg_2_rad;

    IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z],
                             INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);

    // 更新 AHRS（dt = 0.01s 对应 100 Hz）
    const float ahrs_dt = math::Constrain(dt, 0.001f, 0.003f);
    FusionAhrsUpdate(&ahrs, gyro, accel, ahrs_dt);

    // 获取最终姿态
    FusionQuaternion quaternion = FusionAhrsGetQuaternion(&ahrs);

    // 获取最终姿态
    memcpy(INS.q, QEKF_INS.q, sizeof(INS.q));
    memcpy(INS.q, quaternion.array, sizeof(INS.q));

    // 如需要可转换为欧拉角
    FusionEuler euler = FusionQuaternionToEuler(quaternion);

    // 此处为fusion法的输出值
    INS.Yaw = euler.angle.yaw;
    INS.Pitch = euler.angle.pitch;
    INS.Roll = euler.angle.roll;

    // INS.Yaw = QEKF_INS.Yaw;
    // INS.Pitch = QEKF_INS.Pitch;
    // INS.Roll = QEKF_INS.Roll;
    // INS.Yaw_rad = INS.Yaw * DEG_2_RAD;
    // INS.Pitch_rad = INS.Pitch * DEG_2_RAD;
    // INS.Roll_rad = INS.Roll * DEG_2_RAD;

    // const float q0 = QEKF_INS.q[0];
    // const float q1 = QEKF_INS.q[1];
    // const float q2 = QEKF_INS.q[2];
    // const float q3 = QEKF_INS.q[3];
    // const float gravity_b[3] = {
    //     2.0f * (q1 * q3 - q0 * q2) * gravity[2],
    //     2.0f * (q0 * q1 + q2 * q3) * gravity[2],
    //     (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * gravity[2],
    // };
    // for (uint8_t i = 0; i < 3; i++) {
    //     INS.MotionAccel_b[i] = INS.Accel[i] - gravity_b[i];
    // }
    // INS.MotionAccel_n[0] = 2.0f * ((0.5f - q2 * q2 - q3 * q3) * INS.MotionAccel_b[0] +
    //                                (q1 * q2 - q0 * q3) * INS.MotionAccel_b[1] +
    //                                (q1 * q3 + q0 * q2) * INS.MotionAccel_b[2]);
    // INS.MotionAccel_n[1] = 2.0f * ((q1 * q2 + q0 * q3) * INS.MotionAccel_b[0] +
    //                                (0.5f - q1 * q1 - q3 * q3) * INS.MotionAccel_b[1] +
    //                                (q2 * q3 - q0 * q1) * INS.MotionAccel_b[2]);
    // INS.MotionAccel_n[2] = 2.0f * ((q1 * q3 - q0 * q2) * INS.MotionAccel_b[0] +
    //                                (q2 * q3 + q0 * q1) * INS.MotionAccel_b[1] +
    //                                (0.5f - q1 * q1 - q2 * q2) * INS.MotionAccel_b[2]);

    if (!yaw_initialized) {
        INS.YawLast = INS.Yaw;
        yaw_initialized = 1;
    }
    else {
        if (INS.Yaw - INS.YawLast > 180.0f) {
            INS.YawCount--;
        }
        else if (INS.Yaw - INS.YawLast < -180.0f) {
            INS.YawCount++;
        }
    }
    INS.YawTotalAngle = 360.0f * INS.YawCount + INS.Yaw;
    INS.YawLast = INS.Yaw;
}
