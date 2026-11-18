#include <cmsis_os.h>
#include <stdint.h>

#include "tim.h"
#include "Dshot.h"

DshotMode dshot_mode = DSHOT_INIT;

// 电机命令缓存数组
uint16_t MOTOR_CMD_CH1[MOTOR_CMD_BUF_LEN] = { 0 };
uint16_t MOTOR_CMD_CH2[MOTOR_CMD_BUF_LEN] = { 0 };

// 记录初始化的执行次数
uint16_t state_counter = 0;


// 设置电机油门值
void motorset(int16_t value) {
    if (value <= 0) {
        pwmWriteDigital(MOTOR_CMD_CH1, MOTOR_CMD_CH2, 0, 0, false);
        return;
    }

    uint16_t throttle = (value > 1999) ? 1999 : value;
    uint16_t half_throttle = (uint32_t)throttle * 999 / 1999;
    pwmWriteDigital(MOTOR_CMD_CH1, MOTOR_CMD_CH2,
                    1048 + half_throttle, 48 + half_throttle, false);
}

void motor_enable_3d(void) {
    pwmWriteDigital(MOTOR_CMD_CH1, MOTOR_CMD_CH2, 10, 10, true);
}

//CRC校验以及要不要回传信息
static uint16_t prepareDshotPacket(const uint16_t value, bool requestTelemetry) {
    uint16_t packet = (value << 1) | (requestTelemetry ? 1 : 0);

    // 2. 计算4位校验和（采用3次4位异或的算法，符合Dshot协议规范）
    int csum = 0;                  // 校验和变量，初始化为0
    int csum_data = packet;        // 临时变量，用于分步处理12位基础数据
    for (int i = 0; i < 3; i++) {  // 循环3次，每次处理4位数据（12位共3组）
        csum ^= csum_data;           // 当前校验和与4位数据异或（异或结果更新校验和）
        csum_data >>= 4;             // 右移4位，处理下一组4位数据
    }
    csum &= 0xf;  // 取校验和的低4位（确保校验和为4位，高位截断）

    packet = (packet << 4) | csum;
    return packet;
}

//发送函数
void pwmWriteDigital(uint16_t* esc_cmd_ch1, uint16_t* esc_cmd_ch2, uint16_t value_ch1, uint16_t value_ch2, bool request_telemetry) {
    value_ch1 = prepareDshotPacket(value_ch1, request_telemetry);
    value_ch2 = prepareDshotPacket(value_ch2, request_telemetry);

    // 填充通道1缓冲区
    esc_cmd_ch1[0] = (value_ch1 & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch1[1] = (value_ch1 & 0x4000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch1[2] = (value_ch1 & 0x2000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch1[3] = (value_ch1 & 0x1000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch1[4] = (value_ch1 & 0x0800) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch1[5] = (value_ch1 & 0x0400) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch1[6] = (value_ch1 & 0x0200) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch1[7] = (value_ch1 & 0x0100) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch1[8] = (value_ch1 & 0x0080) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch1[9] = (value_ch1 & 0x0040) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch1[10] = (value_ch1 & 0x0020) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch1[11] = (value_ch1 & 0x0010) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch1[12] = (value_ch1 & 0x8) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch1[13] = (value_ch1 & 0x4) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch1[14] = (value_ch1 & 0x2) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch1[15] = (value_ch1 & 0x1) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch1[16] = 0;
    esc_cmd_ch1[17] = 0;

    // 填充通道2缓冲区
    esc_cmd_ch2[0] = (value_ch2 & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch2[1] = (value_ch2 & 0x4000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch2[2] = (value_ch2 & 0x2000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch2[3] = (value_ch2 & 0x1000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch2[4] = (value_ch2 & 0x0800) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch2[5] = (value_ch2 & 0x0400) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch2[6] = (value_ch2 & 0x0200) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch2[7] = (value_ch2 & 0x0100) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch2[8] = (value_ch2 & 0x0080) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch2[9] = (value_ch2 & 0x0040) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch2[10] = (value_ch2 & 0x0020) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch2[11] = (value_ch2 & 0x0010) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch2[12] = (value_ch2 & 0x8) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch2[13] = (value_ch2 & 0x4) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch2[14] = (value_ch2 & 0x2) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch2[15] = (value_ch2 & 0x1) ? MOTOR_BIT_1 : MOTOR_BIT_0;
    esc_cmd_ch2[16] = 0;
    esc_cmd_ch2[17] = 0;

    // 启动两个通道的DMA传输，使用各自的缓冲区·
    // if (TIM_CHANNEL_STATE_GET(&htim8, TIM_CHANNEL_1) == HAL_TIM_CHANNEL_STATE_BUSY)
    //     HAL_TIM_PWM_Stop_DMA(&htim8, TIM_CHANNEL_1);
    // if (TIM_CHANNEL_STATE_GET(&htim8, TIM_CHANNEL_2) == HAL_TIM_CHANNEL_STATE_BUSY)
    //     HAL_TIM_PWM_Stop_DMA(&htim8, TIM_CHANNEL_2);
    if (TIM_CHANNEL_STATE_GET(&htim8, TIM_CHANNEL_1) != HAL_TIM_CHANNEL_STATE_BUSY)
        HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_1, (uint32_t*)esc_cmd_ch1, MOTOR_CMD_BUF_LEN);
    if (TIM_CHANNEL_STATE_GET(&htim8, TIM_CHANNEL_2) != HAL_TIM_CHANNEL_STATE_BUSY)
        HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_2, (uint32_t*)esc_cmd_ch2, MOTOR_CMD_BUF_LEN);
}
