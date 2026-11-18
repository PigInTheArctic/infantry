/**
 *******************************************************************************
 * @file      : bsp_uart.h
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V1.1.0      2026-5-23      origincy        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2026 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSP_UART_H_
#define BSP_UART_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

typedef struct {
    UART_HandleTypeDef* huart;
    uint8_t* rx_buffer;
    uint16_t rx_buffer_size;
    void (*callback_function)(void* context, uint8_t* data, uint16_t len);
    void* context;
} UartInstance;

typedef struct {
    UART_HandleTypeDef* huart;
    uint16_t rx_buffer_size;
    void (*callback_function)(void* context, uint8_t* data, uint16_t len);
    void* context;
} UartInitConfig;

typedef enum {
    UART_TRAMSMIT_NONE = 0,
    UART_TRAMSMIT_BLOCKING,
    UART_TRANSMIT_IT,
    UART_TRAMSMIT_DMA,
} UART_TRANSMIT_MODE;

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

UartInstance* pUartRegister(const UartInitConfig* _pconfig);
void UartSendData(UART_HandleTypeDef* _phuart, const uint8_t* _send_buf, uint16_t _send_size, UART_TRANSMIT_MODE _mode);
void UartInit(const UartInstance* _pinstance);

#ifdef __cplusplus
}
#endif

#endif
