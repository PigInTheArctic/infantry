/**
 *******************************************************************************
 * @file      : beep.cpp
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
 /* Includes ------------------------------------------------------------------*/
#include "beep.h"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
namespace {
    /* 占空比固定为50%，兼顾响度与驱动简单性 */
    constexpr uint16_t k_beep_duty_div = 2;

    /* 常用音阶频率，单位Hz */
    constexpr uint16_t NOTE_C5 = 523;
    constexpr uint16_t NOTE_E5 = 659;
    constexpr uint16_t NOTE_G5 = 784;
    constexpr uint16_t NOTE_A5 = 880;
    constexpr uint16_t NOTE_C6 = 1047;
    constexpr uint16_t NOTE_E6 = 1319;

    const BeepNote_t k_startup_notes[] = {
        { NOTE_C5, 80 },
        { 0, 20 },
        { NOTE_E5, 80 },
        { 0, 20 },
        { NOTE_G5, 100 }
    };

    const BeepNote_t k_success_notes[] = {
        { NOTE_G5, 70 },
        { 0, 20 },
        { NOTE_C6, 120 }
    };

    const BeepNote_t k_warning_notes[] = {
        { NOTE_A5, 120 },
        { 0, 80 },
        { NOTE_A5, 120 }
    };

    const BeepNote_t k_error_notes[] = {
        { NOTE_E6, 100 },
        { 0, 30 },
        { NOTE_C5, 180 },
        { 0, 30 },
        { NOTE_C5, 180 }
    };

    const BeepNote_t k_offline_notes[] = {
        { NOTE_C6, 60 },
        { 0, 40 },
        { NOTE_C6, 60 },
        { 0, 40 },
        { NOTE_C5, 200 }
    };
}  // namespace
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

void Beep::Init(TIM_HandleTypeDef* _htim, uint32_t _channel) {
    htim_ = _htim;
    channel_ = _channel;
    is_init_ = (htim_ != nullptr);
    effect_ = BEEP_SILENT;
    note_list_ = nullptr;
    note_num_ = 0;
    note_idx_ = 0;
    note_tick_ = HAL_GetTick();
    is_busy_ = 0;
    is_started_ = 0;

    if (is_init_) {
        Start();
        Mute();
    }
}

void Beep::Task() {
    /* 未初始化、未在播放或音符表为空时直接返回 */
    if (!is_init_ || !is_busy_ || note_list_ == nullptr || note_num_ == 0) {
        return;
    }

    uint32_t now = HAL_GetTick();
    /* 当前音符持续时间未到，不切换下一音符 */
    if (now - note_tick_ < note_list_[note_idx_].time_ms) {
        return;
    }

    note_idx_++;
    /* 音符表播放完成后自动停止 */
    if (note_idx_ >= note_num_) {
        Stop();
        return;
    }

    note_tick_ = now;
    ApplyTone(note_list_[note_idx_].freq_hz);
}

void Beep::Start() {
    if (!is_init_ || is_started_) {
        return;
    }

    HAL_TIM_PWM_Start(htim_, channel_);
    is_started_ = 1;
}

void Beep::Stop() {
    if (!is_init_) {
        return;
    }

    Mute();
    if (is_started_) {
        HAL_TIM_PWM_Stop(htim_, channel_);
        is_started_ = 0;
    }
    note_list_ = nullptr;
    note_num_ = 0;
    note_idx_ = 0;
    is_busy_ = 0;
    effect_ = BEEP_SILENT;
}

void Beep::Mute() {
    if (!is_init_) {
        return;
    }

    __HAL_TIM_SET_COMPARE(htim_, channel_, 0);
}

void Beep::SetFreq(uint16_t _freq_hz) {
    if (!is_init_) {
        return;
    }

    Start();
    effect_ = BEEP_CUSTOM;
    is_busy_ = 0;
    ApplyTone(_freq_hz);
}

void Beep::Play(BeepEffect_e _effect) {
    if (!is_init_) {
        return;
    }
    //加载音效
    LoadEffect(_effect);
    if (note_list_ == nullptr || note_num_ == 0) {
        Stop();
        return;
    }

    Start();
    note_idx_ = 0;
    note_tick_ = HAL_GetTick();
    is_busy_ = 1;
    effect_ = _effect;
    ApplyTone(note_list_[0].freq_hz);
}

void Beep::Play(const BeepNote_t* _notes, uint16_t _note_num) {
    if (!is_init_ || _notes == nullptr || _note_num == 0) {
        return;
    }

    Start();
    note_list_ = _notes;
    note_num_ = _note_num;
    note_idx_ = 0;
    note_tick_ = HAL_GetTick();
    is_busy_ = 1;
    effect_ = BEEP_CUSTOM;
    ApplyTone(note_list_[0].freq_hz);
}

void Beep::ApplyTone(uint16_t _freq_hz) {
    if (!is_init_) {
        return;
    }

    /* 约定频率为0表示休止符 */
    if (_freq_hz == 0) {
        Mute();
        return;
    }

    uint32_t timer_clk = GetTimerClock();
    uint32_t psc = htim_->Instance->PSC + 1U;
    uint32_t counter_clk = timer_clk / psc;
    uint32_t period = counter_clk / _freq_hz;
    /* 避免频率过高导致ARR无效 */
    if (period < 2U) {
        period = 2U;
    }

    __HAL_TIM_SET_AUTORELOAD(htim_, period - 1U);
    __HAL_TIM_SET_COMPARE(htim_, channel_, (period - 1U) / k_beep_duty_div);
    __HAL_TIM_SET_COUNTER(htim_, 0U);
    HAL_TIM_GenerateEvent(htim_, TIM_EVENTSOURCE_UPDATE);
}

void Beep::LoadEffect(BeepEffect_e _effect) {
    switch (_effect) {
    case BEEP_STARTUP:
        note_list_ = k_startup_notes;
        note_num_ = sizeof(k_startup_notes) / sizeof(k_startup_notes[0]);
        break;
    case BEEP_SUCCESS:
        note_list_ = k_success_notes;
        note_num_ = sizeof(k_success_notes) / sizeof(k_success_notes[0]);
        break;
    case BEEP_WARNING:
        note_list_ = k_warning_notes;
        note_num_ = sizeof(k_warning_notes) / sizeof(k_warning_notes[0]);
        break;
    case BEEP_ERROR:
        note_list_ = k_error_notes;
        note_num_ = sizeof(k_error_notes) / sizeof(k_error_notes[0]);
        break;
    case BEEP_OFFLINE:
        note_list_ = k_offline_notes;
        note_num_ = sizeof(k_offline_notes) / sizeof(k_offline_notes[0]);
        break;
    case BEEP_SILENT:
    case BEEP_CUSTOM:
    default:
        note_list_ = nullptr;
        note_num_ = 0;
        break;
    }
}

uint32_t Beep::GetTimerClock() const {
    if (!is_init_ || htim_ == nullptr) {
        return 0U;
    }

    if (htim_->Instance == TIM1 || htim_->Instance == TIM8 ||
        htim_->Instance == TIM9 || htim_->Instance == TIM10 ||
        htim_->Instance == TIM11) {
        uint32_t timer_clk = HAL_RCC_GetPCLK2Freq();
        if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1) {
            timer_clk *= 2U;
        }
        return timer_clk;
    }

    uint32_t timer_clk = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
        timer_clk *= 2U;
    }
    return timer_clk;
}
