/**
 *******************************************************************************
 * @file      : dm_motor.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2026-01-28      origincy        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2026 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
 /* Includes ------------------------------------------------------------------*/
#include "dm_motor.h"
#include "string.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
namespace {
    int16_t float_to_uint(float x_float, float x_min, float x_max, uint8_t bits);
    float uint_to_float(int x_int, float x_min, float x_max, int bits);
    HAL_StatusTypeDef CanTransmit(CAN_HandleTypeDef* _phcan,
                                  CAN_TxHeaderTypeDef* _ptx_conf,
                                  uint8_t* _ptx_buff);

    uint16_t temp[3] = { 0 };
    int16_t pos_temp, vel_temp, kp_temp, kd_temp, torq_temp;
    uint8_t tx_data[8];
}

namespace DM_MOTOR {
    void DM_Motor::Update() {
        /* 未注册成功或尚未收到过反馈帧时不解析，避免用全 0 缓冲解析出错误角度 */
        if (pdm_motor_instance == nullptr || pdm_motor_instance->rx_len == 0)
            return;

        uint16_t temp[3] = { 0 };
        switch (mode_) {
        case MIT:
            err_ = pdm_motor_instance->rx_buff[0] >> 4;
            temp[0] = (pdm_motor_instance->rx_buff[1] << 8 | pdm_motor_instance->rx_buff[2]);
            temp[1] = (pdm_motor_instance->rx_buff[3] << 4 | pdm_motor_instance->rx_buff[4] >> 4);
            temp[2] = ((pdm_motor_instance->rx_buff[4] & 0x0F) << 8 | pdm_motor_instance->rx_buff[5]);

            ang_ = uint_to_float(temp[0], P_MAX, P_MIN, 16);
            spd_rad_ = uint_to_float(temp[1], V_MAX, V_MIN, 12);
            spd_rpm_ = spd_rad_ * RAD_PER_SEC_2_RPM;
            tor_ = uint_to_float(temp[2], T_MAX, T_MIN, 12);

            temp_ = static_cast<float>(pdm_motor_instance->rx_buff[6]);

            break;
        case DM_1_to_4:
            ang_ = static_cast<int16_t>(pdm_motor_instance->rx_buff[0] << 8 | pdm_motor_instance->rx_buff[1]) / 8091.f * 2 * PI - PI;
            spd_rpm_ = static_cast<int16_t>(pdm_motor_instance->rx_buff[2] << 8 | pdm_motor_instance->rx_buff[3]) / 100.f;
            cur_ = pdm_motor_instance->rx_buff[4] << 8 | pdm_motor_instance->rx_buff[5];
            temp_ = (float)pdm_motor_instance->rx_buff[6];

            spd_rad_ = spd_rpm_ * RPM_2_RAD_PER_SEC;
            break;
        default:
            break;
        }

        if (!angle_initialized_) {
            last_ang_ = ang_;
            round_cnt_ = 0;
            angle_initialized_ = true;
        }
        else {
            if (ang_ - last_ang_ > PI / 2) {
                round_cnt_--;
            }
            else if (ang_ - last_ang_ < -PI / 2) {
                round_cnt_++;
            }
        }

        //round_cnt不是圈
        if (mode_ == MIT)
            ang_real_ = ang_ + round_cnt_ * 2 * P_MAX - ang_offest_;
        else if (mode_ == DM_1_to_4)
            ang_real_ = ang_ + round_cnt_ * 2 * PI - ang_offest_;
        else {}
        ang_real_ = ang_real_ * RAD_2_DEG;
        last_ang_ = ang_;
    }

    void DM_Motor::Init(uint32_t _rx_id, uint32_t _tx_id, CAN_HandleTypeDef* _phcan, Ctrl_Mode _mode) {
        CanInitConf conf{};
        mode_ = _mode;
        conf.hcan = _phcan;
        if (_mode == MIT) {
            conf.rx_id = _rx_id;
            conf.tx_id = _tx_id;
        }
        else if (_mode == DM_1_to_4) {
            if (_rx_id < 5)
                conf.tx_id = 0x3FE;
            else
                conf.tx_id = 0x4FE;
            conf.rx_id = _rx_id + 0x300;
        }
        pdm_motor_instance = pCanRegister(&conf);
        // 初始化变量
        ang_ = 0.0f;
        last_ang_ = 0.0f;
        round_cnt_ = 0;
        angle_initialized_ = false;
        ang_offest_ = 0.0f;//达妙电机的初始化方法
        spd_rpm_ = 0.0f;
        spd_rad_ = 0.0f;
        tor_ = 0.0f;
        temp_ = 0.0f;
        err_ = 0;
        ang_real_ = 0.0f;
    }

    void DM_Motor::SendMIT(float _pos, float _vel, float _Kp, float _Kd, float _torq) {
        if (pdm_motor_instance == nullptr)
            return;

        pos_temp = float_to_uint(_pos, P_MIN, P_MAX, 16);
        vel_temp = float_to_uint(_vel, V_MIN, V_MAX, 12);
        kp_temp = float_to_uint(_Kp, KP_MIN, KP_MAX, 12);
        kd_temp = float_to_uint(_Kd, KD_MIN, KD_MAX, 12);
        torq_temp = float_to_uint(_torq, T_MIN, T_MAX, 12);

        tx_data[0] = pos_temp >> 8;
        tx_data[1] = pos_temp & 0xFF;
        tx_data[2] = vel_temp >> 4;
        tx_data[3] = ((vel_temp & 0x0F) << 4) | (kp_temp >> 8);
        tx_data[4] = kp_temp & 0xFF;
        tx_data[5] = kd_temp >> 4;
        tx_data[6] = ((kd_temp & 0x0F) << 4) | (torq_temp >> 8);
        tx_data[7] = torq_temp & 0xFF;

        CanTransmit(pdm_motor_instance->hcan, &pdm_motor_instance->tx_conf, tx_data);
    }

    void DM_Motor::Send1to4(int16_t _current) {
        memset(tx_data, 0, 8 * sizeof(uint8_t));
        switch ((pdm_motor_instance->rx_id - 1) % 4) {
        case 0:
            tx_data[0] = _current >> 8;
            tx_data[1] = _current;
            break;
        case 1:
            tx_data[2] = _current >> 8;
            tx_data[3] = _current;
            break;
        case 2:
            tx_data[4] = _current >> 8;
            tx_data[5] = _current;
            break;
        case 3:
            tx_data[6] = _current >> 8;
            tx_data[7] = _current;
            break;
        default:
            break;
        }
        CanTransmit(pdm_motor_instance->hcan, &pdm_motor_instance->tx_conf, tx_data);
    }

    void DM_Motor::MotorEnable() {
        memset(tx_data, 0, 8 * sizeof(uint8_t));
        tx_data[0] = 0xFF;
        tx_data[1] = 0xFF;
        tx_data[2] = 0xFF;
        tx_data[3] = 0xFF;
        tx_data[4] = 0xFF;
        tx_data[5] = 0xFF;
        tx_data[6] = 0xFF;
        tx_data[7] = 0xFC;
        CanTransmit(pdm_motor_instance->hcan, &pdm_motor_instance->tx_conf, tx_data);
    }

    void DM_Motor::MotorDisable() {
        memset(tx_data, 0, 8 * sizeof(uint8_t));
        tx_data[0] = 0xFF;
        tx_data[1] = 0xFF;
        tx_data[2] = 0xFF;
        tx_data[3] = 0xFF;
        tx_data[4] = 0xFF;
        tx_data[5] = 0xFF;
        tx_data[6] = 0xFF;
        tx_data[7] = 0xFD;
        CanTransmit(pdm_motor_instance->hcan, &pdm_motor_instance->tx_conf, tx_data);
    }

    void DM_Motor::MotorClearError() {
        memset(tx_data, 0, 8 * sizeof(uint8_t));
        tx_data[0] = 0xFF;
        tx_data[1] = 0xFF;
        tx_data[2] = 0xFF;
        tx_data[3] = 0xFF;
        tx_data[4] = 0xFF;
        tx_data[5] = 0xFF;
        tx_data[6] = 0xFF;
        tx_data[7] = 0xFB;
        CanTransmit(pdm_motor_instance->hcan, &pdm_motor_instance->tx_conf, tx_data);
    }
}

namespace {
    /**
    * @brief:      	float_to_uint: 浮点数转换为无符号整数函数
    * @param[in]:   x_float:	待转换的浮点数
    * @param[in]:   x_min:		范围最小值
    * @param[in]:   x_max:		范围最大值
    * @param[in]:   bits: 		目标无符号整数的位数
    * @retval:     	无符号整数结果
    * @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
    **/
    int16_t float_to_uint(float x_float, float x_min, float x_max, uint8_t bits) {
        float span = x_max - x_min;
        float offset = x_min;
        return (int16_t)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
    }

    /**
    * @brief:      	uint_to_float: 无符号整数转换为浮点数函数
    * @param[in]:   x_int: 待转换的无符号整数
    * @param[in]:   x_min: 范围最小值
    * @param[in]:   x_max: 范围最大值
    * @param[in]:   bits:  无符号整数的位数
    * @retval:     	浮点数结果
    * @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
    **/
    float uint_to_float(int x_int, float x_min, float x_max, int bits) {
        /* converts unsigned int to float, given range and number of bits */
        float span = x_max - x_min;
        float offset = x_min;
        return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
    }

    HAL_StatusTypeDef CanTransmit(CAN_HandleTypeDef* _phcan,
                                  CAN_TxHeaderTypeDef* _ptx_conf,
                                  uint8_t* _ptx_buff) {
        uint32_t tx_mailbox;

        if (HAL_CAN_GetTxMailboxesFreeLevel(_phcan) == 0U) {
            return HAL_BUSY;
        }

        return HAL_CAN_AddTxMessage(_phcan, _ptx_conf, _ptx_buff, &tx_mailbox);
    }

}
