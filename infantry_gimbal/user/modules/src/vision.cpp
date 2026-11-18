/**
 *******************************************************************************
 * @file      : gimbal.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
 /* Includes ------------------------------------------------------------------*/
#include "vision.h"
#include "offline_detect.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/

VisionToGimbal recepakge;
GimbalToVision sendpakge;
uint8_t Buf_temp_t[sizeof(VisionToGimbal)] = { 0 };
uint8_t Buf_temp[sizeof(GimbalToVision)] = { 0 };

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief 四元数整定函数
 * @note 将欧拉角四元数转换为视觉使用的泰特-布莱恩角四元数
 * @param q_in : 欧拉角四元数
 * @param o_out : 泰特-布莱恩角四元数
 */
void ChangeQuat(const float q_in[4], float q_out[4]) {
    /* 提取 q */
    float w = q_in[0];
    float x = q_in[1];
    float y = q_in[2];
    float z = q_in[3];

    /* 转为 Euler (roll, pitch, yaw) - 使用 Tait-Bryan ZYX (yaw-pitch-roll) 顺序 */
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    float roll = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (w * y - z * x);
    if (sinp >= 1.0f) sinp = 1.0f;
    if (sinp <= -1.0f) sinp = -1.0f;
    float pitch = asinf(sinp);

    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    float yaw = atan2f(siny_cosp, cosy_cosp);


    //在此处修改欧拉角的正负和顺序
    float new_roll = roll;
    float new_pitch = pitch;
    float new_yaw = yaw;

    /* 从新的 Euler 重新生成四元数 */
    float cr = cosf(new_roll * 0.5f);
    float sr = sinf(new_roll * 0.5f);
    float cp = cosf(new_pitch * 0.5f);
    float sp = sinf(new_pitch * 0.5f);
    float cy = cosf(new_yaw * 0.5f);
    float sy = sinf(new_yaw * 0.5f);

    float nw = cr * cp * cy + sr * sp * sy;
    float nx = sr * cp * cy - cr * sp * sy;
    float ny = cr * sp * cy + sr * cp * sy;
    float nz = cr * cp * sy - sr * sp * cy;

    float norm = sqrtf(nw * nw + nx * nx + ny * ny + nz * nz);
    if (norm > 1e-6f) {
        nw /= norm; nx /= norm; ny /= norm; nz /= norm;
    }

    q_out[0] = nw;
    q_out[1] = nx;
    q_out[2] = ny;
    q_out[3] = nz;
}



/* 将 int16 按大端放入 buf[index], buf[index+1] */
static void pack_int16_be(uint8_t* buf, int index, int16_t v) {
    buf[index] = (uint8_t)((v >> 8) & 0xFF);
    buf[index + 1] = (uint8_t)(v & 0xFF);
}

/* 发送四元数：按接收端解码，data[0..1]=x,2..3=y,4..5=z,6..7=w；
   每分量 int16 = roundf(comp * 1e4)，存为大端 */
void SendQuaternionCAN(uint32_t can_id, float x, float y, float z, float w) {
    /* 先归一化，避免接收端因非单位四元数被判定为无效 */
    float norm = sqrtf(x * x + y * y + z * z + w * w);
    if (norm > 1e-6f) {
        x /= norm; y /= norm; z /= norm; w /= norm;
    }

    uint8_t data[8] = { 0 };
    int16_t xi = (int16_t)fmaxf(fminf(roundf(x * 1e4f), 32767.0f), -32768.0f);
    int16_t yi = (int16_t)fmaxf(fminf(roundf(y * 1e4f), 32767.0f), -32768.0f);
    int16_t zi = (int16_t)fmaxf(fminf(roundf(z * 1e4f), 32767.0f), -32768.0f);
    int16_t wi = (int16_t)fmaxf(fminf(roundf(w * 1e4f), 32767.0f), -32768.0f);

    pack_int16_be(data, 0, xi);
    pack_int16_be(data, 2, yi);
    pack_int16_be(data, 4, zi);
    pack_int16_be(data, 6, wi);

    CAN_TxHeaderTypeDef txHeader = { 0 };
    txHeader.StdId = can_id;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 8;
    uint32_t txMailbox;
    HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &txMailbox);
}

/* 发送弹速：data[0..1] int16(bullet_speed*1e2), data[2]=mode, data[3]=shoot_mode,
   data[4..5] int16(ft_angle*1e4)（大端）其余字节置0 */
void SendBulletSpeedCAN(uint32_t can_id, float bullet_speed, uint8_t mode, uint8_t shoot_mode, float ft_angle) {
    uint8_t data[8] = { 0 };
    int16_t bs = (int16_t)fmaxf(fminf(roundf(bullet_speed * 1e2f), 32767.0f), -32768.0f);
    int16_t ft = (int16_t)fmaxf(fminf(roundf(ft_angle * 1e4f), 32767.0f), -32768.0f);

    pack_int16_be(data, 0, bs);
    data[2] = mode;
    data[3] = shoot_mode;
    pack_int16_be(data, 4, ft);

    CAN_TxHeaderTypeDef txHeader = { 0 };
    txHeader.StdId = can_id;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 8;
    uint32_t txMailbox;
    HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &txMailbox);
}



void Send_Vision(uint8_t mode, float q[4], float yaw, float yaw_vel, float pitch, float pitch_vel, float bullet_speed, uint16_t bullet_count) {

    sendpakge.head[0] = 'S';
    sendpakge.head[1] = 'P';

    sendpakge.mode = mode;

    sendpakge.q[0] = q[0];
    sendpakge.q[1] = q[1];
    sendpakge.q[2] = q[2];
    sendpakge.q[3] = q[3];

    sendpakge.yaw = yaw;
    sendpakge.yaw_vel = yaw_vel;
    sendpakge.pitch = pitch;
    sendpakge.pitch_vel = pitch_vel;
    sendpakge.bullet_speed = bullet_speed;
    sendpakge.bullet_count = bullet_count;

    sendpakge.crc16 = Get_CRC16_Check_Sum((uint8_t*)&sendpakge, (uint32_t)sizeof(sendpakge) - 2, 0xffff);
    memcpy(Buf_temp, &sendpakge, sizeof(sendpakge));
    CDC_Transmit_FS(reinterpret_cast<uint8_t*>(Buf_temp), sizeof(Buf_temp));
}

void Receive_Vision() {
    if (Buf_temp_t[0] != 'S' || Buf_temp_t[1] != 'P') {
        return;
    }
    if (!Verify_CRC16_Check_Sum(Buf_temp_t, sizeof(VisionToGimbal))) {
        return;
    }

    memcpy(&recepakge, Buf_temp_t, sizeof(VisionToGimbal));
    offline_detect.Feed(DeviceId::DEV_VISION);
}
