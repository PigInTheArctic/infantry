/**
 *******************************************************************************
 * @file      : ins.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026/04/25        origincy        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2026 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
#include "ins.h"
#include "bmi088_driver.h"
#include "bsp_dwt.h"
#include "user_lib.h"
#include "Fusion_AHRS.h"
#include <cstring>
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
namespace
{
    FusionAhrs ahrs;
    FusionOffset offset;

    constexpr uint8_t X = 0;
    constexpr uint8_t Y = 1;
    constexpr uint8_t Z = 2;

    const float xb[3] = {1, 0, 0};
    const float yb[3] = {0, 1, 0};
    const float zb[3] = {0, 0, 1};
    const float gravity[3] = {0, 0, 9.80f};

    uint32_t INS_DWT_Count = 0;
    float dt = 0;
}

/* Exported variables --------------------------------------------------------*/
INS_t INS;
/* Exported function prototypes ----------------------------------------------*/

void INS_Init()
{
    FusionAhrsInitialise(&ahrs);
    FusionAhrsSettings ahrs_settings;
    ahrs_settings.convention = FusionConventionNwu;
    ahrs_settings.gain = 0.5f;
    ahrs_settings.gyroscopeRange = 2000.0f;
    ahrs_settings.accelerationRejection = 8.0f;
    ahrs_settings.recoveryTriggerPeriod = 250;
    FusionAhrsSetSettings(&ahrs, &ahrs_settings);

    FusionOffsetInitialise(&offset, 500);
}

void INS_Task()
{
    dt = DWT_GetDeltaT(&INS_DWT_Count);

    BMI088_Read(&BMI088);

    INS.Accel[X] = BMI088.Accel[Y];
    INS.Accel[Y] = -BMI088.Accel[X];
    INS.Accel[Z] = BMI088.Accel[Z];

    INS.Gyro[X] = BMI088.Gyro[Y];
    INS.Gyro[Y] = -BMI088.Gyro[X]; // 换作右手系
    INS.Gyro[Z] = BMI088.Gyro[Z];  // 底盘和云台同时解码

    FusionVector raw_gyro = {
        .axis = {INS.Gyro[X] * math::rad_2_deg, INS.Gyro[Y] * math::rad_2_deg, INS.Gyro[Z] * math::rad_2_deg}};
    FusionVector accel = {.axis = {INS.Accel[X] / gravity[Z], INS.Accel[Y] / gravity[Z], INS.Accel[Z] / gravity[Z]}};

    FusionVector gyro = FusionOffsetUpdate(&offset, raw_gyro);

    const float ahrs_dt = math::Constrain(dt, 0.001f, 0.003f);
    FusionAhrsUpdate(&ahrs, gyro, accel, ahrs_dt);

    FusionQuaternion quaternion = FusionAhrsGetQuaternion(&ahrs);

    FusionEuler euler = FusionQuaternionToEuler(quaternion);

    FusionVector Accel_n = FusionAhrsGetEarthAcceleration(&ahrs);
    FusionVector Accel_b = FusionAhrsGetLinearAcceleration(&ahrs);

    INS.Yaw = euler.angle.yaw;
    INS.Pitch = euler.angle.pitch;
    INS.Roll = euler.angle.roll;

    memcpy(INS.q, quaternion.array, sizeof(INS.q));
    memcpy(INS.MotionAccel_b, Accel_b.array, sizeof(INS.MotionAccel_b));
    memcpy(INS.MotionAccel_n, Accel_n.array, sizeof(INS.MotionAccel_n));

    if (INS.Yaw - INS.YawLast > 180.0f)
    {
        INS.YawCount--;
    }
    else if (INS.Yaw - INS.YawLast < -180.0f)
    {
        INS.YawCount++;
    }
    INS.YawTotalAngle = 360.0f * INS.YawCount + INS.Yaw;
    INS.YawLast = INS.Yaw;
}
