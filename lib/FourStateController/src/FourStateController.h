#pragma once

#include <Arduino.h>
#include <StateVector.h>

struct ControlSignal
{
    float phiTermRadS2;
    float phiDotTermRadS2;
    float betaTermRadS2;
    float betaDotTermRadS2;

    float rawRadS2;
    float appliedRadS2;

    bool saturated;
};

class FourStateController
{
public:
    FourStateController(
        float kPhi,
        float kPhiDot,
        float kBeta,
        float kBetaDot,
        float maximumAccelerationRadS2
    );

    ControlSignal compute(const StateVector &state) const;

private:
    float kPhi_;
    float kPhiDot_;
    float kBeta_;
    float kBetaDot_;
    float maximumAccelerationRadS2_;

    static float clamp(
        float value,
        float minimum,
        float maximum
    );
};
