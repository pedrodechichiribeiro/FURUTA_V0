#include "FourStateController.h"

#include <math.h>

FourStateController::FourStateController(
    float kPhi,
    float kPhiDot,
    float kBeta,
    float kBetaDot,
    float maximumAccelerationRadS2
)
    : kPhi_(kPhi),
      kPhiDot_(kPhiDot),
      kBeta_(kBeta),
      kBetaDot_(kBetaDot),
      maximumAccelerationRadS2_(fabsf(maximumAccelerationRadS2))
{
}

ControlSignal FourStateController::compute(
    const StateVector &state
) const
{
    ControlSignal signal;

    signal.phiTermRadS2 =
        kPhi_ * state.phi;

    signal.phiDotTermRadS2 =
        kPhiDot_ * state.phiDot;

    signal.betaTermRadS2 =
        kBeta_ * state.beta;

    signal.betaDotTermRadS2 =
        kBetaDot_ * state.betaDot;

    signal.rawRadS2 =
        signal.phiTermRadS2
        + signal.phiDotTermRadS2
        + signal.betaTermRadS2
        + signal.betaDotTermRadS2;

    signal.appliedRadS2 =
        clamp(
            signal.rawRadS2,
            -maximumAccelerationRadS2_,
            maximumAccelerationRadS2_
        );

    signal.saturated =
        fabsf(
            signal.rawRadS2
            -
            signal.appliedRadS2
        )
        >
        0.0001F;

    return signal;
}

float FourStateController::clamp(
    float value,
    float minimum,
    float maximum
)
{
    if (value > maximum)
    {
        return maximum;
    }

    if (value < minimum)
    {
        return minimum;
    }

    return value;
}
