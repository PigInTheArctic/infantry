#ifndef OFFLINE_DETECT_H_
#define OFFLINE_DETECT_H_

#ifdef __cplusplus

#include "stdint.h"
#include "beep.h"
/**
 * @brief 设备在线状态枚举
 * @note Ignore表示该设备当前未加入离线检测
 */
enum class offline_state {
  Ignore,
  Online,
  Offline
};

constexpr uint8_t DevNum = 15;

/**
 * @brief 系统内需要检测的设备编号
 * @note 枚举顺序需要与offline_item/EnableList数组索引保持一致
 */
enum class DeviceId {
  DEV_REMOTE = 0,     // 遥控器
  DEV_REFEREE,        // 裁判系统
  DEV_VISION,         // 视觉
  DEV_PITCH_FOLLOW_MOTOR, 
  DEV_PITCH_MOTOR,    // Pitch电机
  DEV_FRIC_L,         // 左摩擦轮
  DEV_FRIC_R,         // 右摩擦轮
  DEV_TRIGGER,        // 拨盘电机
  DEV_BOARD_COMM,     // 板间通讯
  DEV_INS,            // 陀螺仪
  DEV_CAP,            // 底盘超级电容
  DEV_WHEEL,          // 底盘轮组
  DEV_RUDDER,         // 底盘舵轮
  DEV_CHASSIS_REFEREE, // 底盘裁判系统
  DEV_YAW_MOTOR = 14   // Yaw电机
};

/**
 * @brief 单个设备的离线检测信息
 * @param state 当前设备状态
 * @param last_online_ms 最近一次成功喂狗的时间戳，单位ms
 * @param timeout_ms 判定离线的超时时间，单位ms
 */
typedef struct {
  offline_state state;
  float last_online_ms;
  float timeout_ms;
} OfflineItem;

class Offline_Detect {
private:
  /* 标记对应设备是否参与离线检测 */
  bool EnableList[DevNum] = { 0 };
  /* 按DeviceId索引存储每个设备的检测信息 */
  OfflineItem offline_item[DevNum] = {};
  /* 蜂鸣器由离线检测模块统一调度，用于启动提示和离线报警 */
  Beep beep;
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
   * @brief 根据外部在线标志直接刷新设备状态
   * @note 适用于由其他板上报的从属设备状态
   * @param id 设备id
   * @param is_online 当前标志位表示的在线状态
   */
  void UpdateFromFlag(DeviceId id, bool is_online);
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
  offline_state GetState(DeviceId id) const {
    return offline_item[static_cast<uint8_t>(id)].state;
  }
  /**
   * @brief 获取蜂鸣器对象引用
   * @note 主要供检测任务调用播放接口
   */
  Beep& GetBeep() { return this->beep; }
};

extern Offline_Detect offline_detect;

#endif 

/**
 * @brief 离线检测任务入口
 * @note 任务内部会完成蜂鸣器初始化提示、状态刷新和分类报警
 */

#ifdef __cplusplus
extern "C" {
#endif

  extern void Detect_task();
  extern void Detect_Init();

#ifdef __cplusplus
}
#endif

#endif
