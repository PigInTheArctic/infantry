/**
 *******************************************************************************
 * @file      : referee.cpp
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
#include "referee.h"

#include "board_comm.h"
#include "crc.h"
#include "offline_detect.h"
#include "string.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
Referee referee;
/* Private function prototypes -----------------------------------------------*/

void Referee::Update(uint8_t* _p_data) {
    uint16_t judge_length;  // 统计一帧数据长度
    //	CmdID = 0; //数据命令码解析
    // 空数据包，则不作任何处理
    if (_p_data == NULL)
        return;

    if (_p_data[0] == 0xA9 && _p_data[1] == 0x53) {
        memcpy(&comma_data, _p_data + 10, 9);
        KeyProcess();
    }
}

void Referee::KeyProcess() {
    for (short i = 0; i < 16; i++) {
        if (comma_data.keyboard_value & (0x01 << i)) {
            keyboard_[i].is_pressed = 1;
        }
        else {
            keyboard_[i].is_pressed = 0;
        }
    }
    offline_detect.Feed(DeviceId::DEV_REFEREE);
    if (comma_data.left_button_down == 0x01) {
        keyboard_[MOUSE_L].is_pressed = 1;
    }
    else {
        keyboard_[MOUSE_L].is_pressed = 0;
    }
    if (comma_data.right_button_down == 0x01) {
        keyboard_[MOUSE_R].is_pressed = 1;
    }
    else {
        keyboard_[MOUSE_R].is_pressed = 0;
    }

    if (comma_data.mid_button_down == 0x01) {
        keyboard_[MOUSE_M].is_pressed = 1;
    }
    else {
        keyboard_[MOUSE_M].is_pressed = 0;
    }

    for (short i = 0; i < 19; i++) {
        if (keyboard_[i].is_pressed == 1 && keyboard_[i].last_state == 0) {
            keyboard_[i].is_ticked++;
        }

        if (keyboard_[i].is_ticked % 2 == 0) {
            keyboard_[i].is_ticked = 0;
        }

        keyboard_[i].last_state = keyboard_[i].is_pressed;
    }
}

static void RefereeCallback() {
    referee.Update(referee.p_instance_->rx_buffer);
}

void Referee::Init(UART_HandleTypeDef* _phuart) {
    UartInitConfig conf;
    conf.huart = _phuart;
    conf.rx_buffer_size = 256;
    conf.callback_function = RefereeCallback;
    referee.p_instance_ = pUartRegister(&conf);
    return;
}
