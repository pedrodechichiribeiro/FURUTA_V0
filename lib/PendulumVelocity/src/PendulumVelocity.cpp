#include "PendulumVelocity.h"

#include <math.h>

PendulumVelocity::PendulumVelocity(
    float filterTauSeconds
)
    : filterTauSeconds_(filterTauSeconds)
{
}

void PendulumVelocity::begin(
    float initialPositionRad,
    uint32_t initialTimeUs
)
{
    previousPositionRad_ = initialPositionRad;
    previousTimeUs_ = initialTimeUs;

    rawVelocityRadS_ = 0.0F;
    filteredVelocityRadS_ = 0.0F;
    sampleTimeSeconds_ = 0.0F;

    initialized_ = true;
    filterInitialized_ = false;

    warmupSamplesRemaining_ = WARMUP_SAMPLES;
}

bool PendulumVelocity::update(
    float positionRad,
    uint32_t currentTimeUs
)
{
    if (!initialized_) {
        begin(positionRad, currentTimeUs);
        return false;
    }

    /*
     * A subtração com uint32_t continua funcionando
     * corretamente mesmo quando micros() transborda.
     */
    const uint32_t elapsedUs =
        static_cast<uint32_t>(
            currentTimeUs - previousTimeUs_
        );

    /*
     * Evita divisão por zero ou intervalos
     * absurdamente pequenos.
     */
    if (elapsedUs == 0) {
        return false;
    }

    sampleTimeSeconds_ =
        static_cast<float>(elapsedUs) *
        1.0e-6F;

    /*
     * Caso o programa fique parado por muito tempo,
     * não calculamos uma velocidade usando um intervalo
     * inadequado.
     */
    if (sampleTimeSeconds_ > 0.100F) {
        reset(positionRad, currentTimeUs);
        return false;
    }

    const float positionDifference =
        positionRad - previousPositionRad_;

    rawVelocityRadS_ =
        positionDifference /
        sampleTimeSeconds_;

    previousPositionRad_ = positionRad;
    previousTimeUs_ = currentTimeUs;

    if (
        isnan(rawVelocityRadS_) ||
        isinf(rawVelocityRadS_)
    ) {
        rawVelocityRadS_ = 0.0F;
        return false;
    }

    if (warmupSamplesRemaining_ > 0) {
        warmupSamplesRemaining_--;

        filteredVelocityRadS_ = 0.0F;
        filterInitialized_ = false;

        return false;
    }

    if (!filterInitialized_) {
        filteredVelocityRadS_ =
            rawVelocityRadS_;

        filterInitialized_ = true;

        return true;
    }

    /*
     * Filtro passa-baixas de primeira ordem:
     *
     * beta = dt / (tau + dt)
     *
     * velocidadeFiltrada +=
     *     beta * (velocidadeBruta - velocidadeFiltrada)
     */
    const float beta =
        sampleTimeSeconds_ /
        (
            filterTauSeconds_ +
            sampleTimeSeconds_
        );

    filteredVelocityRadS_ +=
        beta *
        (
            rawVelocityRadS_ -
            filteredVelocityRadS_
        );

    return true;
}

void PendulumVelocity::reset(
    float currentPositionRad,
    uint32_t currentTimeUs
)
{
    begin(
        currentPositionRad,
        currentTimeUs
    );
}

void PendulumVelocity::setFilterTau(
    float filterTauSeconds
)
{
    if (filterTauSeconds > 0.0F) {
        filterTauSeconds_ =
            filterTauSeconds;
    }
}

float PendulumVelocity::rawVelocityRadS() const
{
    return rawVelocityRadS_;
}

float PendulumVelocity::filteredVelocityRadS() const
{
    return filteredVelocityRadS_;
}

float PendulumVelocity::sampleTimeSeconds() const
{
    return sampleTimeSeconds_;
}

bool PendulumVelocity::isInitialized() const
{
    return initialized_;
}