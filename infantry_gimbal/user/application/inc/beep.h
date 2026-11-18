/**
 *******************************************************************************
 * @file      : beep.h
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      2026-03-23      origincy          1. add non-blocking beep driver
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2026 Reborn Team, USTB.
 *  All Rights Reserved.
 *******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BEEP_H_
#define BEEP_H_

#ifdef __cplusplus
extern "C" {
#endif

    /* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdint.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
    typedef enum {
        BEEP_SILENT = 0,
        BEEP_STARTUP,
        BEEP_SUCCESS,
        BEEP_WARNING,
        BEEP_ERROR,
        BEEP_OFFLINE,
        BEEP_CUSTOM
    } BeepEffect_e;

    typedef struct {
        uint16_t freq_hz;
        uint16_t time_ms;
    } BeepNote_t;

#ifdef __cplusplus
}

class Beep {
public:
    /**
     * @brief 蜂鸣器初始化
     * @note 通过外部传入定时器和通道，避免模块内部绑定具体硬件资源
     * @param _htim PWM定时器句柄
     * @param _channel PWM通道
     */
    void Init(TIM_HandleTypeDef* _htim, uint32_t _channel);
    /**
     * @brief 蜂鸣器任务
     * @note 需要在周期任务中调用，用于推进音符序列
     */
    void Task();

    /**
     * @brief 启动PWM输出
     * @note 一般Init内部已自动启动，保留该接口用于手动控制
     */
    void Start();
    /**
     * @brief 停止当前音效并清空播放状态
     */
    void Stop();
    /**
     * @brief 静音
     * @note 仅将比较值拉为0，不会清空当前音效状态
     */
    void Mute();

    /**
     * @brief 直接设置蜂鸣器频率
     * @note 设置后会退出当前音效序列，进入自定义频率输出模式
     * @param _freq_hz 目标频率，单位Hz；传0等价于静音
     */
    void SetFreq(uint16_t _freq_hz);
    /**
     * @brief 播放预设音效
     * @param _effect 音效类型
     */
    void Play(BeepEffect_e _effect);
    /**
     * @brief 播放自定义音符序列
     * @param _notes 音符数组首地址
     * @param _note_num 音符数量
     */
    void Play(const BeepNote_t* _notes, uint16_t _note_num);

    uint8_t IsBusy() const { return is_busy_; }
    BeepEffect_e GetEffect() const { return effect_; }

private:
    /* 将目标频率转换成ARR/CCR，更新到底层PWM定时器 */
    void ApplyTone(uint16_t _freq_hz);
    /* 根据预设音效装载对应音符表 */
    void LoadEffect(BeepEffect_e _effect);
    /* 获取当前定时器实际输入时钟 */
    uint32_t GetTimerClock() const;

    TIM_HandleTypeDef* htim_ = nullptr;
    uint32_t channel_ = 0;

    /* 当前正在播放的音符序列 */
    const BeepNote_t* note_list_ = nullptr;
    uint16_t note_num_ = 0;
    uint16_t note_idx_ = 0;
    uint32_t note_tick_ = 0;

    /* 模块状态量 */
    uint8_t is_init_ = 0;
    uint8_t is_busy_ = 0;
    uint8_t is_started_ = 0;
    BeepEffect_e effect_ = BEEP_SILENT;
};

#endif

#ifdef __cplusplus
extern "C" {
#endif

    /* Exported variables --------------------------------------------------------*/
    /* Exported function prototypes ----------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __BEEP_H_ */
