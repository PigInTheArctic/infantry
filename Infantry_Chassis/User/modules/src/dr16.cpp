/**
 *******************************************************************************
 * @file      : DR16.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026/04/28        origincy        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2026 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/
 /* Includes ------------------------------------------------------------------*/
#include "dr16.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
namespace {
    constexpr int16_t rc_ch_offset = 1024;
}

/* External variables --------------------------------------------------------*/
DR16 dr16;
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

void DR16::Init(UART_HandleTypeDef* huart) {
    UartInitConfig conf = {
        .huart = huart,
        .rx_buffer_size = RC_RX_BUF_SIZE,
        .callback_function = UartCallbackEntry,
        .context = this,
    };
    phuart_instance = pUartRegister(&conf);
}

void DR16::Reset() {
    ch = {};
    sw = {};
    rc_raw_ = {};
}

void DR16::UartCallbackEntry(void* context, uint8_t* data, uint16_t len) {
    HAL_UARTEx_ReceiveToIdle_DMA(static_cast<DR16*>(context)->phuart_instance->huart, static_cast<DR16*>(context)->phuart_instance->rx_buffer, 18);
    __HAL_DMA_DISABLE_IT(static_cast<DR16*>(context)->phuart_instance->huart->hdmarx, DMA_IT_HT);
    static_cast<DR16*>(context)->Callback(data);
}

void DR16::Callback(const uint8_t* rx_data_) {
    rc_raw_.ch[0] = static_cast<int16_t>((rx_data_[0] | rx_data_[1] << 8) & 0x07FF);
    rc_raw_.ch[1] = static_cast<int16_t>((rx_data_[1] >> 3 | rx_data_[2] << 5) & 0x07FF);
    rc_raw_.ch[2] = static_cast<int16_t>((rx_data_[2] >> 6 | rx_data_[3] << 2 | rx_data_[4] << 10) & 0x07FF);
    rc_raw_.ch[3] = static_cast<int16_t>((rx_data_[4] >> 1 | rx_data_[5] << 7) & 0x07FF);
    rc_raw_.ch[4] = static_cast<int16_t>(rx_data_[16] | rx_data_[17] << 8);
    rc_raw_.s[0] = ((rx_data_[5] >> 4) & 0x000C) >> 2;
    rc_raw_.s[1] = ((rx_data_[5] >> 4) & 0x0003);

    ch.r_row = static_cast<int16_t>(rc_raw_.ch[0] - rc_ch_offset);
    ch.r_col = static_cast<int16_t>(rc_raw_.ch[1] - rc_ch_offset);
    ch.l_row = static_cast<int16_t>(rc_raw_.ch[2] - rc_ch_offset);
    ch.l_col = static_cast<int16_t>(rc_raw_.ch[3] - rc_ch_offset);
    ch.dial_wheel = static_cast<int16_t>(rc_raw_.ch[4] - rc_ch_offset);
    if (rc_raw_.s[0] == 1)
        sw.l = UP;
    else if (rc_raw_.s[0] == 2)
        sw.l = DOWN;
    else if (rc_raw_.s[0] == 3)
        sw.l = MID;
    if (rc_raw_.s[1] == 1)
        sw.r = UP;
    else if (rc_raw_.s[1] == 2)
        sw.r = DOWN;
    else if (rc_raw_.s[1] == 3)
        sw.r = MID;
}
