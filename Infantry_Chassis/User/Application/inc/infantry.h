/**
 *******************************************************************************
 * @file      : infantry_chassis.h
 * @brief     : C 接口封装，供 FreeRTOS 任务调用
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
#ifndef INFANTRY_CHASSIS_H_
#define INFANTRY_CHASSIS_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 底盘初始化，C 接口
 */
void Chassis_Init(void);
/**
 * @brief 底盘任务，C 接口
 */
void Chassis_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* INFANTRY_CHASSIS_H_ */
