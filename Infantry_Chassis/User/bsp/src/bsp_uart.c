/**
 *******************************************************************************
 * @file      : bsp_uart.c
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
 /* Includes ------------------------------------------------------------------*/
#include "bsp_uart.h"
#include "stdlib.h"
#include "memory.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t idx;
static UartInstance* uart_instance[5] = { NULL };
/* External variables --------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 使用指定的初始化配置注册 UART 实例
 *
 *  此函数用于使用提供的初始化配置注册 UART 实例
 *
 * @param _pconfig 指向包含初始化配置的 UartInitConfig 结构的指针
 * @return 指向表示已注册 UART 实例的 UartInstance 结构的指针
 */
UartInstance* pUartRegister(const UartInitConfig* _pconfig) {
    if (idx >= 5) {
        return NULL;
    }
    UartInstance* instance = (UartInstance*)malloc(sizeof(UartInstance));
    memset(instance, 0, sizeof(UartInstance));

    instance->huart = _pconfig->huart;
    instance->rx_buffer_size = _pconfig->rx_buffer_size;
    instance->callback_function = _pconfig->callback_function;

    uart_instance[idx++] = instance;
    UartInit(instance);
    return instance;
}

/**
 * @brief Initializes the UART instance.
 *
 * This function initializes the UART instance specified by `_instance`.
 *
 * @param _instance The UART instance to be initialized.
 */
void UartInit(const UartInstance* _pinstance) {
    HAL_UARTEx_ReceiveToIdle_DMA(_pinstance->huart, _pinstance->rx_buffer, _pinstance->rx_buffer_size);
    __HAL_DMA_DISABLE_IT(_pinstance->huart->hdmarx, DMA_IT_HT);
}

/**
 * @brief Sends data over UART.
 *
 * This function sends data over UART using the specified UART handle, send buffer, send size, and transmit mode.
 *
 * @param _huart The UART handle.
 * @param _send_buf The send buffer containing the data to be sent.
 * @param _send_size The size of the data to be sent.
 * @param _mode The transmit mode.
 */
void UartSendData(UART_HandleTypeDef* _phuart, const uint8_t* _send_buf, uint16_t _send_size, UART_TRANSMIT_MODE _mode) {
    switch (_mode) {
    case UART_TRAMSMIT_BLOCKING:
        HAL_UART_Transmit(_phuart, _send_buf, _send_size, 10);
        break;
    case UART_TRANSMIT_IT:
        HAL_UART_Transmit_IT(_phuart, _send_buf, _send_size);
        break;
    case UART_TRAMSMIT_DMA:
        HAL_UART_Transmit_DMA(_phuart, _send_buf, _send_size);
        break;
    default:
        break;
    }
}

/**
 * @brief Callback function for UART receive event.
 *
 * This function is called when a UART receive event occurs.
 *
 * @param huart Pointer to the UART handle.
 * @param Size Number of bytes received.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    for (uint8_t i = 0; i < idx; ++i) {
        if (huart == uart_instance[i]->huart) {
            if (uart_instance[i]->callback_function != NULL) {
                uart_instance[i]->callback_function(uart_instance[i]->context,
                                                    uart_instance[i]->rx_buffer,
                                                    Size);
                memset(uart_instance[i]->rx_buffer, 0, Size);  // 接收结束后清空buffer,对于变长数据是必要的
            }
            HAL_UARTEx_ReceiveToIdle_DMA(uart_instance[i]->huart, uart_instance[i]->rx_buffer, uart_instance[i]->rx_buffer_size);
            __HAL_DMA_DISABLE_IT(uart_instance[i]->huart->hdmarx, DMA_IT_HT);
            return;
        }
    }
}
