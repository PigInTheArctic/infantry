/**
 *******************************************************************************
 * @file      : remote_keyboard.h
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
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __REMOTE_KEYBOARD_H_
#define __REMOTE_KEYBOARD_H_

#include "stdint.h"


#ifdef __cplusplus
extern "C" {
#endif

    /* Includes ------------------------------------------------------------------*/
    /* Exported macro ------------------------------------------------------------*/
    /* Exported constants --------------------------------------------------------*/
    /* Exported types ------------------------------------------------------------*/
    enum CtrlMode {
        RemoteCtrl,
        KeyboardCtrl
    };

    enum RemoteCtrlMode {
        AutoAimMode,
        ChassisMode
    };

    enum LongPressDetectState {
        ButtonRelease,
        ButtonPressed,
        ButtonClick,
        ButtonLongPress,
    };
    /* Exported variables --------------------------------------------------------*/
    extern float SW_signal, Step_signal, Sin_signal, Cos_signal;
    extern float last_Step_signal;
    extern uint8_t aimingFlag;
    extern uint8_t fric_flag;
    /* Exported function prototypes ----------------------------------------------*/
    void ModeTask();

#ifdef __cplusplus
}
#endif

#endif /* __REMOTE_KEYBOARD_H_ */
