#include "PendulumDownReference.h"

#include <math.h>

PendulumDownReference::PendulumDownReference(
    uint32_t calibrationTimeMs,
    float maximumSpanDeg
)
    :
    calibrationTimeUs_(calibrationTimeMs * 1000UL),
    maximumSpanDeg_(maximumSpanDeg),
    running_(false),
    ready_(false),
    rejected_(false),
    startUs_(0),
    samples_(0),
    sumSin_(0.0F),
    sumCos_(0.0F),
    minimumAngleDeg_(0.0F),
    maximumAngleDeg_(0.0F),
    spanDeg_(0.0F),
    offsetRad_(0.0F)
{
}

void PendulumDownReference::start(uint32_t nowUs)
{
    running_ = true;
    ready_ = false;
    rejected_ = false;

    startUs_ = nowUs;
    samples_ = 0;

    sumSin_ = 0.0F;
    sumCos_ = 0.0F;

    minimumAngleDeg_ = 1000.0F;
    maximumAngleDeg_ = -1000.0F;

    spanDeg_ = 0.0F;
    offsetRad_ = 0.0F;
}

bool PendulumDownReference::update(
    float rawRelativeAngleRad,
    uint32_t nowUs
)
{
    if (!running_)
    {
        return false;
    }

    sumSin_ += sinf(rawRelativeAngleRad);
    sumCos_ += cosf(rawRelativeAngleRad);
    ++samples_;

    const float angleDeg =
        rawRelativeAngleRad * RAD_TO_DEG;

    if (angleDeg < minimumAngleDeg_)
    {
        minimumAngleDeg_ = angleDeg;
    }

    if (angleDeg > maximumAngleDeg_)
    {
        maximumAngleDeg_ = angleDeg;
    }

    if ((nowUs - startUs_) < calibrationTimeUs_)
    {
        return false;
    }

    running_ = false;

    spanDeg_ =
        maximumAngleDeg_ - minimumAngleDeg_;

    if (
        samples_ == 0
        || spanDeg_ > maximumSpanDeg_
    )
    {
        ready_ = false;
        rejected_ = true;
        return true;
    }

    offsetRad_ = atan2f(sumSin_, sumCos_);

    ready_ = true;
    rejected_ = false;

    return true;
}

bool PendulumDownReference::isRunning() const
{
    return running_;
}

bool PendulumDownReference::isReady() const
{
    return ready_;
}

bool PendulumDownReference::wasRejected() const
{
    return rejected_;
}

float PendulumDownReference::offsetRad() const
{
    return offsetRad_;
}

float PendulumDownReference::offsetDeg() const
{
    return offsetRad_ * RAD_TO_DEG;
}

float PendulumDownReference::observedSpanDeg() const
{
    return spanDeg_;
}

float PendulumDownReference::correctedAngleRad(
    float rawRelativeAngleRad
) const
{
    return wrapToPi(
        rawRelativeAngleRad - offsetRad_
    );
}

float PendulumDownReference::wrapToPi(float angleRad)
{
    while (angleRad >= PI)
    {
        angleRad -= TWO_PI;
    }

    while (angleRad < -PI)
    {
        angleRad += TWO_PI;
    }

    return angleRad;
}
