/**
 *******************************************************************************
 * @file      : ins.h
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
#ifndef INS_H_
#define INS_H_

#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct {
    float q[4];  // 四元数

    float Gyro[3];  // 导航系角速度
    float Accel[3];          // 加速度
    float MotionAccel_b[3];  // 机体坐标加速度
    float MotionAccel_n[3];  // 绝对系加速度

    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawLast;
    float YawCount;
    float YawTotalAngle;
} INS_t;

/* Exported variables --------------------------------------------------------*/
extern INS_t INS;

#endif

#ifdef __cplusplus
extern "C" {
#endif
    /* Exported function prototypes ----------------------------------------------*/

    void INS_Init(void);
    void INS_Task(void);

#ifdef __cplusplus
}
#endif

#endif //INS_H_
