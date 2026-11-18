/**
 *******************************************************************************
 * @file      : referee.cpp
 * @brief     : RoboMaster 2026 裁判系统串口协议解析（V2.0.0，2026-06-26）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-06-26      origincy        1. 按 2026 V2.0.0 协议重写
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2026 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
 /* Includes ------------------------------------------------------------------*/
#include "referee.h"
#include "detect.h"
#include "board_comm.h"
#include "crc.h"
#include <string.h>
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
Referee referee;
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief 解析一帧或多帧裁判系统数据
 * @note  帧格式：5 字节帧头 + 2 字节 CmdID + n 字节数据 + 2 字节 CRC16
 *        帧头 DataLength 为 uint16 小端，表示 data 段长度
 * @param _p_data 串口 DMA 缓冲区
 * @param len     本次收到的字节数
 */
void Referee::Update(uint8_t* _p_data, uint16_t len) {
    if (_p_data == nullptr || len < (LEN_HEADER + LEN_CMDID + LEN_TAIL)) {
        return;
    }

    uint32_t offset = 0;
    while ((offset + LEN_HEADER + LEN_CMDID + LEN_TAIL) <= len) {
        uint8_t* frame = _p_data + offset;

        /* 起始字节不对时逐字节找 0xA5，避免噪声导致整包丢弃 */
        if (frame[SOF] != JUDGE_FRAME_HEADER) {
            offset++;
            continue;
        }

        /* DataLength 是 uint16 小端，位于偏移 1、2 */
        const uint16_t data_length =
            static_cast<uint16_t>(frame[DATA_LENGTH] | (frame[DATA_LENGTH + 1] << 8));
        const uint32_t frame_length =
            static_cast<uint32_t>(data_length) + LEN_HEADER + LEN_CMDID + LEN_TAIL;

        /* 本包不完整：等下一次串口数据 */
        if (frame_length > static_cast<uint32_t>(len) - offset) {
            break;
        }

        /* 帧头 CRC8 */
        if (Verify_CRC8_Check_Sum(frame, LEN_HEADER) != TRUE) {
            offset++;
            continue;
        }

        /* 整包 CRC16；校验失败则跳过本帧继续找下一帧 */
        if (Verify_CRC16_Check_Sum(frame, frame_length) != TRUE) {
            offset += frame_length;
            continue;
        }

        memcpy(&FrameHeader, frame, LEN_HEADER);
        CmdID = static_cast<uint16_t>((frame[6] << 8) | frame[5]);

        /* 按 2026 V2.0.0 表 1-5 的数据段长度解析 */
        switch (CmdID) {
        case ID_game_state:
            if (data_length == LEN_game_state) {
                memcpy(&game_state_, frame + DATA, LEN_game_state);
            }
            break;

        case ID_game_result:
            if (data_length == LEN_game_result) {
                memcpy(&game_result_, frame + DATA, LEN_game_result);
            }
            break;

        case ID_game_robot_survivors:
            if (data_length == LEN_game_robot_HP) {
                memcpy(&game_robot_HP_, frame + DATA, LEN_game_robot_HP);
            }
            break;

        case ID_event_data:
            if (data_length == LEN_event_data) {
                memcpy(&event_data_, frame + DATA, LEN_event_data);
            }
            break;

        case ID_referee_warning:
            if (data_length == LEN_referee_warning) {
                memcpy(&referee_warning_, frame + DATA, LEN_referee_warning);
            }
            break;

        case ID_dart_info:
            if (data_length == LEN_dart_info) {
                memcpy(&dart_info_, frame + DATA, LEN_dart_info);
            }
            break;

        case ID_game_robot_state:
            if (data_length == LEN_game_robot_state) {
                memcpy(&game_robot_state_, frame + DATA, LEN_game_robot_state);
            }
            break;

        case ID_power_heat_data:
            if (data_length == LEN_power_heat_data) {
                memcpy(&power_heat_data_, frame + DATA, LEN_power_heat_data);
            }
            break;

        case ID_game_robot_pos:
            if (data_length == LEN_game_robot_pos) {
                memcpy(&game_robot_pos_, frame + DATA, LEN_game_robot_pos);
            }
            break;

        case ID_buff_musk:
            if (data_length == LEN_buff_musk) {
                memcpy(&buff_musk_, frame + DATA, LEN_buff_musk);
            }
            break;

        case ID_robot_hurt:
            if (data_length == LEN_robot_hurt) {
                memcpy(&robot_hurt_, frame + DATA, LEN_robot_hurt);
            }
            break;

        case ID_shoot_data:
            if (data_length == LEN_shoot_data) {
                memcpy(&shoot_data_, frame + DATA, LEN_shoot_data);
            }
            break;

        case ID_projectile_allowance:
            if (data_length == LEN_projectile_allowance) {
                memcpy(&projectile_allowance, frame + DATA, LEN_projectile_allowance);
            }
            break;

        case ID_rfid_status:
            if (data_length == LEN_rfid_status) {
                memcpy(&rfid_status_, frame + DATA, LEN_rfid_status);
            }
            break;

        case ID_dart_client_cmd:
            if (data_length == LEN_dart_client_cmd) {
                memcpy(&dart_client_cmd_, frame + DATA, LEN_dart_client_cmd);
            }
            break;

        case ID_ground_robot_position:
            if (data_length == LEN_ground_robot_position) {
                memcpy(&ground_robot_position_, frame + DATA, LEN_ground_robot_position);
            }
            break;

        case ID_radar_mark_data:
            if (data_length == LEN_radar_mark_data) {
                memcpy(&radar_mark_data_, frame + DATA, LEN_radar_mark_data);
            }
            break;

        case ID_sentry_info:
            if (data_length == LEN_sentry_info) {
                memcpy(&sentry_info_, frame + DATA, LEN_sentry_info);
            }
            break;

        case ID_radar_info:
            if (data_length == LEN_radar_info) {
                memcpy(&radar_info_, frame + DATA, LEN_radar_info);
            }
            break;

        case ID_student_interactive:
            if (data_length == LEN_student_interactive) {
                memcpy(&student_interactive_, frame + DATA, LEN_student_interactive);
            }
            break;

        default:
            break;
        }

        /* 只有 CRC 正确的帧才刷新主控通信和在线状态 */
        board_comm.SetCoolingHeat(game_robot_state_.shooter_barrel_cooling_value);
        board_comm.SetHeatLimit(game_robot_state_.shooter_barrel_heat_limit);
        board_comm.SetCurrentHeat(power_heat_data_.shooter_17mm_barrel_heat);
        board_comm.SetShooterOutput(game_robot_state_.power_management_shooter_output);
        board_comm.SetRobotID(game_robot_state_.robot_id);
        offline_detect.Feed(DeviceId::DEV_Referee);

        offset += frame_length;
    }
}

void Referee::Init(UART_HandleTypeDef* _phuart) {
    UartInitConfig conf{};
    conf.huart = _phuart;
    conf.rx_buffer_size = 256;
    conf.context = this;
    conf.callback_function = UartCallbackEntry;
    p_instance_ = pUartRegister(&conf);
}

void Referee::UartCallbackEntry(void* context, uint8_t* data, uint16_t len) {
    if (context == nullptr) {
        return;
    }
    static_cast<Referee*>(context)->Update(data, len);
}
