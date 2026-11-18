#ifndef DSHOT_H
#define DSHOT_H

#include <stdbool.h> 
#include <stdint.h>   

#define MOTOR_BIT_0 7
#define MOTOR_BIT_1 14
#define MOTOR_BITLENGTH 20  //100%
#define MOTOR_CMD_BUF_LEN 18


// 定义模式
typedef enum {
    DSHOT_INIT,          // 初始化
    DSHOT_SEND,          // 发送命令
    DSHOT_DISABLE
} DshotMode;

#ifdef __cplusplus
extern "C" {
#endif


    extern void motorset(int16_t value);
    extern void motor_enable_3d(void);
    extern DshotMode dshot_mode;
    extern uint16_t state_counter;

    extern void pwmWriteDigital(uint16_t* esc_cmd_ch1, uint16_t* esc_cmd_ch2, uint16_t value_ch1, uint16_t value_ch2, bool request_telemetry);

#ifdef __cplusplus
}
#endif
#endif
