/**
 *******************************************************************************
 * @file      : ADRC.h
 * @brief     : Active disturbance rejection controller based on NTD and LESO
 * @history   :
 *  Version     Date          Author          Note
 *  V1.0.0      2026-08-31    origincy        1. Initial version
 *  V1.1.0      2026-09-13    origincy        1. ESO新增二阶/三阶可选
 *******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ADRC_H_
#define ADRC_H_

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

/* Exported macro -------------------------------------------------------------*/
/* Exported constants ---------------------------------------------------------*/
/* Exported types -------------------------------------------------------------*/
enum class ESOType : uint8_t {
    LESO,
    NLESO,
};

/**
 * @brief Extended state observer order
 * @note SECOND estimates z1/z2 for first-order plants, THIRD estimates
 *       z1/z2/z3 for second-order plants.
 */
enum class ESOOrder : uint8_t {
    SECOND = 0U,
    THIRD,
};

class LESO final {
public:
    LESO();

    void Init(float bandwidth, float b0, ESOOrder order = ESOOrder::THIRD);
    void Update(float feedback, float control);
    void Reset();

    [[nodiscard]] float GetZ1() const { return z1_; }
    [[nodiscard]] float GetZ2() const { return z2_; }
    [[nodiscard]] float GetZ3() const { return z3_; }

private:
    uint32_t dwt_cnt_;
    float bandwidth_;
    float b0_;
    ESOOrder order_;
    float z1_;
    float z2_;
    float z3_;
};

class NLESO final {
public:
    NLESO();

    void Init(float b0, float beta1, float beta2, float beta3, float delta,
              ESOOrder order = ESOOrder::THIRD);
    void Update(float feedback, float control);
    void Reset();

    [[nodiscard]] float GetZ1() const { return z1_; }
    [[nodiscard]] float GetZ2() const { return z2_; }
    [[nodiscard]] float GetZ3() const { return z3_; }
    [[nodiscard]] float GetDisturbance() const {
        if (order_ == ESOOrder::SECOND)
            return z2_ / b0_;
        else if (order_ == ESOOrder::THIRD)
            return z3_ / b0_;
        else            return 0;
    }

private:
    static float Sign(float value);
    static float Fal(float value, uint8_t n, float delta);

    uint32_t dwt_cnt_;
    float b0_;
    float beta1_;
    float beta2_;
    float beta3_;
    float delta_;
    ESOOrder order_;
    float z1_;
    float z2_;
    float z3_;
};

class ADRC final {
public:
    ADRC();

    void Init(float ntd_rate, float b0,
              float beta1, float beta2, float beta3,
              float nlsef_rate, float damp, float max_out, float delta = 0.001f,
              ESOType eso_type = ESOType::NLESO, float leso_bandwidth = 1.0f,
              ESOOrder eso_order = ESOOrder::THIRD);

    void SetRef(float ref) { ref_ = ref; }
    void SetFdb(float fdb) { fdb_ = fdb; }

    void Reset();
    float Calc();

    [[nodiscard]] float GetOutput() const { return output_; }

private:
    static float Sign(float value);
    static float Fhan(float x1, float x2, float rate, float step);
    static float Constrain(float value, float min_value, float max_value);

    uint32_t dwt_cnt_;
    float dt_;

    float ntd_rate_;
    float b0_;
    float nlsef_rate_;
    float damp_;
    float max_out_;

    float ref_;
    float fdb_;

    ESOType eso_type_;
    ESOOrder eso_order_;
    LESO leso_;
    NLESO nleso_;

    float x1_;
    float x2_;
    float z1_;
    float z2_;
    float z3_;
    float output_;
};

#endif

/* Exported variables ---------------------------------------------------------*/
/* Exported function prototypes -----------------------------------------------*/
#endif // ADRC_H_
