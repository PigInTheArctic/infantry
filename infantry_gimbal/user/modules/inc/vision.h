#ifndef __PACKET_H__
#define __PACKET_H__
#include <stdint.h>
#include <stdbool.h>
#include "crc.h"
#include "can.h"
#include <math.h>
#include "usbd_cdc_if.h"
#include "main.h"


#pragma pack(push,1)

#ifdef __cplusplus
extern "C" {
#endif

  typedef enum GimbalMode {
    GIMBAL_IDLE = 0,    // 空闲
    AUTO_AIM,    // 自瞄
    SMALL_BUFF,  // 小符
    BIG_BUFF     // 大符
  }GimbalMode;

  typedef enum VisionMode {
    VISION_IDLE = 0,             //空闲
    AUTO_AIM_HOLD_FIRE,   //自瞄不开火
    AUTO_AIM_FIRE         //自瞄开火
  }VisionMode;




  typedef struct {
    uint8_t head[2];
    uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符  //1
    float q[4];    // wxyz顺序
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed; //22.0f
    uint16_t bullet_count;  // 子弹累计发送次数
    uint16_t crc16;
  }__attribute__((packed))GimbalToVision;

  typedef struct {
    uint8_t head[2];
    uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    uint16_t crc16;
  }__attribute__((packed))VisionToGimbal;

  struct GimbalState {
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    uint16_t bullet_count;
  };

  extern VisionToGimbal recepakge;
  extern GimbalToVision sendpakge;

  extern uint8_t Buf_temp_t[sizeof(VisionToGimbal)];
  extern uint8_t Buf_temp[sizeof(sendpakge)];

  //四元数换系，发送前需要先整定四元数
  void ChangeQuat(const float q_in[4], float q_out[4]);
  void SendQuaternionCAN(uint32_t can_id, float x, float y, float z, float w);
  void SendBulletSpeedCAN(uint32_t can_id, float bullet_speed, uint8_t mode, uint8_t shoot_mode, float ft_angle);
  void Send_Vision(uint8_t mode, float q[4], float yaw, float yaw_vel, float pitch, float pitch_vel, float bullet_speed, uint16_t bullet_count);
  void Receive_Vision();

#ifdef __cplusplus
}
#endif

#pragma pack(pop)


#endif


