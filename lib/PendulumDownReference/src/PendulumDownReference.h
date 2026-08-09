#pragma once

#include <Arduino.h>

// ============================================================
// REFERENCIA DOWN POR MEDIA ANGULAR CIRCULAR
//
// A classe aceita uma pequena oscilacao natural durante a
// calibracao. Em vez de exigir imobilidade, estima o centro da
// oscilacao por media circular durante uma janela temporal.
// ============================================================

class PendulumDownReference
{
public:
    PendulumDownReference(
        uint32_t calibrationTimeMs = 3000UL,
        float maximumSpanDeg = 20.0F
    );

    void start(uint32_t nowUs);

    // Retorna true quando a janela de calibracao terminou,
    // seja com sucesso ou rejeicao.
    bool update(
        float rawRelativeAngleRad,
        uint32_t nowUs
    );

    bool isRunning() const;
    bool isReady() const;
    bool wasRejected() const;

    float offsetRad() const;
    float offsetDeg() const;
    float observedSpanDeg() const;

    float correctedAngleRad(
        float rawRelativeAngleRad
    ) const;

private:
    static float wrapToPi(float angleRad);

    uint32_t calibrationTimeUs_;
    float maximumSpanDeg_;

    bool running_;
    bool ready_;
    bool rejected_;

    uint32_t startUs_;
    uint32_t samples_;

    float sumSin_;
    float sumCos_;

    float minimumAngleDeg_;
    float maximumAngleDeg_;

    float spanDeg_;
    float offsetRad_;
};
