/**
 *******************************************************************************
 * @file      : cap.h
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-01-12      origincy        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2026 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAP_H_
#define __CAP_H_

/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"

#ifdef __cplusplus
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#pragma pack(push, 1)
typedef enum {
    CapEnable = 0,
    CapDisable = 1,
}CapState;

typedef struct {
    uint16_t cap_energy;
    uint16_t input_power;
    uint16_t output_power;
    float cap_volt;
    uint8_t cap_state;
} CAP;

#pragma pack(pop)

class CAP_t {
public:
    CAP_t() = default;
    CAP_t(const CAP_t&) = delete;
    void Init(CAN_HandleTypeDef* _p_hcan, uint16_t _send_id, uint16_t _recv_id);
    void Callback(uint8_t* data, uint8_t len);
    void SetCapState();
    float GetVolt() {
        return data_.cap_volt;
    }
    void SetChargePower(uint16_t _p_in) {
        power_set_ = _p_in;
    }
    uint8_t GetCapState() {
        return data_.cap_state;
    }
    float GetOutPower() {
        return data_.output_power;
    }
    void EnableCap(uint8_t _CapEnableFlag) {
        CapEnableFlag_ = _CapEnableFlag;
    }
    void Send() {
        SendData();
    }


private:
    CanInstance* p_instance_;
    void SendData();
    CAP data_;
    uint16_t power_set_ = 45 - 3;
    uint8_t CapEnableFlag_;

    static void CanCallbackEntry(void* context, uint8_t* data, uint8_t len);
};
/* Exported variables --------------------------------------------------------*/
extern CAP_t cap;
/* Exported function prototypes ----------------------------------------------*/
#endif

#endif /* __FILE_H_ */