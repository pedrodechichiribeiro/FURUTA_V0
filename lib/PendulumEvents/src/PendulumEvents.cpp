#include "PendulumEvents.h"

PendulumEvents::PendulumEvents()
{
    reset();
}

void PendulumEvents::reset()
{
    downPositiveArmed_ = false;
    downNegativeArmed_ = false;

    peakPositiveArmed_ = false;
    peakNegativeArmed_ = false;

    positivePeakAlphaRad_ = 0.0F;
    negativePeakAlphaRad_ = 0.0F;

    positivePeakTimeUs_ = 0;
    negativePeakTimeUs_ = 0;
}

PendulumEvent PendulumEvents::update(
    float alphaRad,
    float alphaDotRadS,
    uint32_t nowUs
)
{
    PendulumEvent event;

    // ========================================================
    // ARMA DOWN
    // ========================================================

    if (alphaRad <= -DOWN_REARM_RAD)
    {
        downPositiveArmed_ = true;
    }

    if (alphaRad >= DOWN_REARM_RAD)
    {
        downNegativeArmed_ = true;
    }

    // ========================================================
    // DOWN+
    // ========================================================

    if (
        downPositiveArmed_
        && alphaRad >= 0.0F
        && alphaDotRadS > 0.0F
    )
    {
        downPositiveArmed_ = false;

        event.type = PendulumEventType::DOWN_POSITIVE;
        event.timeUs = nowUs;
        event.alphaRad = alphaRad;
        event.alphaDotRadS = alphaDotRadS;

        return event;
    }

    // ========================================================
    // DOWN-
    // ========================================================

    if (
        downNegativeArmed_
        && alphaRad <= 0.0F
        && alphaDotRadS < 0.0F
    )
    {
        downNegativeArmed_ = false;

        event.type = PendulumEventType::DOWN_NEGATIVE;
        event.timeUs = nowUs;
        event.alphaRad = alphaRad;
        event.alphaDotRadS = alphaDotRadS;

        return event;
    }

    // ========================================================
    // PEAK+
    //
    // A velocidade significativa arma o detector. Enquanto
    // armado, guardamos o maior alpha realmente observado.
    // ========================================================

    if (
        alphaRad >= PEAK_MIN_ANGLE_RAD
        && alphaDotRadS >= PEAK_ARM_SPEED_RAD_S
    )
    {
        if (!peakPositiveArmed_)
        {
            peakPositiveArmed_ = true;
            positivePeakAlphaRad_ = alphaRad;
            positivePeakTimeUs_ = nowUs;
        }

        if (alphaRad > positivePeakAlphaRad_)
        {
            positivePeakAlphaRad_ = alphaRad;
            positivePeakTimeUs_ = nowUs;
        }
    }

    if (
        peakPositiveArmed_
        && alphaDotRadS <= 0.0F
    )
    {
        peakPositiveArmed_ = false;

        event.type = PendulumEventType::PEAK_POSITIVE;
        event.timeUs = positivePeakTimeUs_;
        event.alphaRad = positivePeakAlphaRad_;
        event.alphaDotRadS = 0.0F;

        return event;
    }

    // ========================================================
    // PEAK-
    // ========================================================

    if (
        alphaRad <= -PEAK_MIN_ANGLE_RAD
        && alphaDotRadS <= -PEAK_ARM_SPEED_RAD_S
    )
    {
        if (!peakNegativeArmed_)
        {
            peakNegativeArmed_ = true;
            negativePeakAlphaRad_ = alphaRad;
            negativePeakTimeUs_ = nowUs;
        }

        if (alphaRad < negativePeakAlphaRad_)
        {
            negativePeakAlphaRad_ = alphaRad;
            negativePeakTimeUs_ = nowUs;
        }
    }

    if (
        peakNegativeArmed_
        && alphaDotRadS >= 0.0F
    )
    {
        peakNegativeArmed_ = false;

        event.type = PendulumEventType::PEAK_NEGATIVE;
        event.timeUs = negativePeakTimeUs_;
        event.alphaRad = negativePeakAlphaRad_;
        event.alphaDotRadS = 0.0F;

        return event;
    }

    return event;
}
