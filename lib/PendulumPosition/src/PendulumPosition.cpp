#include "PendulumPosition.h"


PendulumPosition::PendulumPosition(
    AS5600 &sensor,
    float directionSign
)
    : sensor_(sensor),
      directionSign_(directionSign),
      raw_(0),
      topRaw_(0),
      absoluteAngleRadians_(0.0F),
      betaRadians_(0.0F),
      topDefined_(false),
      lastError_(AS5600_OK)
{
}


bool PendulumPosition::begin()
{
    sensor_.begin();

    return sensor_.isConnected();
}


bool PendulumPosition::update()
{
    // readAngle() retorna 0 ... 4095.
    raw_ = sensor_.readAngle();

    lastError_ = sensor_.lastError();

    if (lastError_ != AS5600_OK)
    {
        return false;
    }

    absoluteAngleRadians_ =
        static_cast<float>(raw_) * AS5600_RAW_TO_RADIANS;

    if (topDefined_)
    {
        const int16_t delta =
            circularDifference(raw_, topRaw_);

        betaRadians_ =
            directionSign_ *
            static_cast<float>(delta) *
            AS5600_RAW_TO_RADIANS;
    }

    return true;
}


bool PendulumPosition::calibrateTop(
    uint8_t numberOfSamples,
    uint16_t intervalMilliseconds
)
{
    if (numberOfSamples == 0)
    {
        return false;
    }

    // Primeira leitura servirá como referência.
    uint16_t reference = sensor_.readAngle();

    lastError_ = sensor_.lastError();

    if (lastError_ != AS5600_OK)
    {
        return false;
    }

    int32_t accumulatedDifference = 0;

    for (uint8_t i = 0; i < numberOfSamples; ++i)
    {
        uint16_t value = sensor_.readAngle();

        lastError_ = sensor_.lastError();

        if (lastError_ != AS5600_OK)
        {
            return false;
        }

        accumulatedDifference +=
            circularDifference(value, reference);

        delay(intervalMilliseconds);
    }

    const int32_t meanDifference =
        accumulatedDifference /
        static_cast<int32_t>(numberOfSamples);

    int32_t calculatedTop =
        static_cast<int32_t>(reference) +
        meanDifference;

    // Coloca novamente no intervalo 0 ... 4095.
    while (calculatedTop < 0)
    {
        calculatedTop += 4096;
    }

    while (calculatedTop >= 4096)
    {
        calculatedTop -= 4096;
    }

    topRaw_ = static_cast<uint16_t>(calculatedTop);

    topDefined_ = true;

    // Atualiza beta usando a nova referência.
    return update();
}


bool PendulumPosition::topIsDefined() const
{
    return topDefined_;
}


uint16_t PendulumPosition::raw() const
{
    return raw_;
}


float PendulumPosition::absoluteAngleRadians() const
{
    return absoluteAngleRadians_;
}


float PendulumPosition::betaRadians() const
{
    return betaRadians_;
}


uint16_t PendulumPosition::topRaw() const
{
    return topRaw_;
}


int PendulumPosition::lastError() const
{
    return lastError_;
}


bool PendulumPosition::magnetDetected()
{
    return sensor_.magnetDetected();
}


bool PendulumPosition::magnetTooWeak()
{
    return sensor_.magnetTooWeak();
}


bool PendulumPosition::magnetTooStrong()
{
    return sensor_.magnetTooStrong();
}


int16_t PendulumPosition::circularDifference(
    uint16_t current,
    uint16_t reference
)
{
    int16_t difference =
        static_cast<int16_t>(current) -
        static_cast<int16_t>(reference);

    if (difference > 2048)
    {
        difference -= 4096;
    }
    else if (difference < -2048)
    {
        difference += 4096;
    }

    return difference;
}