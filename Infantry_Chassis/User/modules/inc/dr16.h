/**
 *******************************************************************************
 * @file      : DR16.h
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
#ifndef DR16_H_
#define DR16_H_
/* Includes ------------------------------------------------------------------*/
#include "bsp_uart.h"

#ifdef __cplusplus
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
constexpr uint8_t RC_RX_BUF_SIZE = 36;
constexpr uint8_t RC_FRAME_LEN = 18;

/* Exported types ------------------------------------------------------------*/
class DR16 final {
public:
    DR16() = default;
    void Init(UART_HandleTypeDef* huart);
    static void UartCallbackEntry(void* context, uint8_t* data, uint16_t len);
    void Reset();

    enum switchState : uint8_t {
        UP = 1,
        MID = 2,
        DOWN = 3,
    };

    struct channel {
        int16_t r_row;
        int16_t r_col;
        int16_t l_row;
        int16_t l_col;
        int16_t dial_wheel;
    };

    struct RCSwitch {
        switchState l;
        switchState r;
    };

    channel ch{};
    RCSwitch sw{};

private:
    void Callback(const uint8_t* rx_data_);

    UartInstance* phuart_instance = nullptr;

    struct RCRaw {
        int16_t ch[5]{};
        uint8_t s[2]{};
    } rc_raw_{};
};

/* Exported variables --------------------------------------------------------*/
extern DR16 dr16;
#endif

#ifdef __cplusplus
extern "C" {
#endif

    /* Exported function prototypes ----------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif //DR16_H_
