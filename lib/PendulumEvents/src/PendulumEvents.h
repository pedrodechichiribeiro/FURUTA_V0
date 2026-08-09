#pragma once

#include <Arduino.h>

// ============================================================
// EVENTOS CINEMATICOS DO PENDULO
//
// Convencao esperada:
//   alpha = 0     -> DOWN
//   alpha > 0     -> um lado
//   alpha < 0     -> lado oposto
//
// O detector usa histerese/armamento para evitar eventos
// duplicados e guarda o extremo geometrico real dos picos.
// ============================================================

enum class PendulumEventType : uint8_t
{
    NONE,
    DOWN_POSITIVE,
    DOWN_NEGATIVE,
    PEAK_POSITIVE,
    PEAK_NEGATIVE
};

struct PendulumEvent
{
    PendulumEventType type = PendulumEventType::NONE;
    uint32_t timeUs = 0;
    float alphaRad = 0.0F;
    float alphaDotRadS = 0.0F;
};

class PendulumEvents
{
public:
    PendulumEvents();

    void reset();

    PendulumEvent update(
        float alphaRad,
        float alphaDotRadS,
        uint32_t nowUs
    );

private:
    static constexpr float DOWN_REARM_RAD =
        0.0523598776F;   // 3 graus

    static constexpr float PEAK_MIN_ANGLE_RAD =
        0.0872664626F;   // 5 graus

    static constexpr float PEAK_ARM_SPEED_RAD_S =
        0.20F;

    bool downPositiveArmed_;
    bool downNegativeArmed_;

    bool peakPositiveArmed_;
    bool peakNegativeArmed_;

    float positivePeakAlphaRad_;
    float negativePeakAlphaRad_;

    uint32_t positivePeakTimeUs_;
    uint32_t negativePeakTimeUs_;
};
