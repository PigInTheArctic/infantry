/**
 *******************************************************************************
 * @file      : detect.h
 * @brief     : 设备检测，将在线状态返回给云台
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-05-26      origincy        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2026 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DETECT_H_
#define DETECT_H_

/** Includes ------------------------------------------------------------------*/
#include "stdint.h"

#ifdef __cplusplus
/** Exported macro ------------------------------------------------------------*/
/** Exported constants --------------------------------------------------------*/
/** Exported types ------------------------------------------------------------*/
enum class offline_state
{
    Ignore,
    Online,
    Offline
};

constexpr uint8_t DevNum = 8;

enum class DeviceId
{
    DEV_Wheel_RF = 0, // 右前轮
    DEV_Wheel_LB,     // 左后轮
    DEV_Steer_RF,     // 右前舵
    DEV_Steer_LB,     // 左后舵
    DEV_Referee,      // 裁判系统
    DEV_CAP,          // 电容
    DEV_CTRL,
    DEV_YAW_MOTOR, // 板间控制
    DEV_NONE
};

struct OfflineItem
{
    offline_state state;
    float last_online_ms;
    float timeout_ms;
};

class Offline_Detect
{
public:
private:
    bool EnableList[DevNum]{};
    OfflineItem offline_item[DevNum]{};

public:
    /**
     * @brief 初始化单个设备的离线检测参数
     * @note 只有调用过Init的设备才会进入在线/离线判定
     * @param id 设备id
     * @param timeout 判离线超时时间，单位ms
     */
    void Init(DeviceId id, uint16_t timeout);

    /**
     * @brief 刷新全部设备状态
     * @note 需要在周期任务中反复调用
     */
    void Update(void);

    /**
     * @brief 设备喂狗函数
     * @note 当设备收到有效数据时调用，用于刷新在线时间戳
     * @param id 设备id
     */
    void Feed(DeviceId id);

    /**
     * @brief 获取设备是否在线
     * @note Ignore状态默认返回true，避免未使能设备影响逻辑
     * @param id 设备id
     * @retval true 在线或未纳入检测
     * @retval false 已纳入检测且当前离线
     */
    bool IsOnline(DeviceId id);

    /**
     * @brief 获取设备原始状态枚举
     * @note 适合在任务内部做更细粒度的状态分支
     * @param id 设备id
     */
    offline_state GetState(DeviceId id) const
    {
        return offline_item[static_cast<uint8_t>(id)].state;
    }
};

/* Exported variables --------------------------------------------------------*/
extern Offline_Detect offline_detect;

#endif

#ifdef __cplusplus
extern "C"
{
#endif
    /* Exported function prototypes ----------------------------------------------*/
    void Detect_Init();
    void Detect_Task();

#ifdef __cplusplus
}
#endif

#endif