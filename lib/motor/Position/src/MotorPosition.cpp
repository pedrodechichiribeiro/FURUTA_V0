#include "MotorPosition.h"

#include <math.h>

MotorPosition::MotorPosition(
    uint8_t stepPin,
    uint8_t dirPin,
    uint8_t enablePin,
    uint16_t fullStepsPerRevolution,
    uint8_t microstepFactor,
    float minimumAngle,
    float maximumAngle,
    bool invertDirection
)
    : stepper_(
          AccelStepper::DRIVER,
          stepPin,
          dirPin
      ),
      enablePin_(enablePin),
      fullStepsPerRevolution_(
          fullStepsPerRevolution
      ),
      microstepFactor_(
          microstepFactor
      ),
      stepsPerDegree_(
          (
              static_cast<float>(
                  fullStepsPerRevolution
              ) *
              static_cast<float>(
                  microstepFactor
              )
          ) /
          360.0F
      ),
      minimumAngle_(
          minimumAngle
      ),
      maximumAngle_(
          maximumAngle
      ),
      invertDirection_(
          invertDirection
      ),
      enabled_(false)
{
}

void MotorPosition::begin(
    float maximumSpeedDegreesPerSecond,
    float accelerationDegreesPerSecondSquared,
    uint16_t minimumPulseWidth
)
{
    pinMode(enablePin_, OUTPUT);

    // ENABLE do A4988 é ativo em nível baixo.
    digitalWrite(enablePin_, HIGH);
    enabled_ = false;

    stepper_.setPinsInverted(
        invertDirection_,
        false,
        false
    );

    stepper_.setMinPulseWidth(
        minimumPulseWidth
    );

    /*
     * Os valores recebidos são expressos em graus.
     * Os métodos fazem a conversão para micropassos.
     */
    setMaxSpeed(
        maximumSpeedDegreesPerSecond
    );

    setAcceleration(
        accelerationDegreesPerSecondSquared
    );

    stepper_.setCurrentPosition(0);
}

void MotorPosition::update()
{
    if (enabled_)
    {
        stepper_.run();
    }
}

void MotorPosition::enable()
{
    digitalWrite(enablePin_, LOW);

    enabled_ = true;
}

void MotorPosition::disable()
{
    /*
     * Cancela o destino atual antes de liberar o motor.
     */
    const long currentMicrosteps =
        stepper_.currentPosition();

    stepper_.setCurrentPosition(
        currentMicrosteps
    );

    digitalWrite(enablePin_, HIGH);

    enabled_ = false;
}

bool MotorPosition::isEnabled() const
{
    return enabled_;
}

bool MotorPosition::moveTo(
    float angleDegrees
)
{
    if (!enabled_)
    {
        return false;
    }

    if (!isWithinLimits(angleDegrees))
    {
        return false;
    }

    stepper_.moveTo(
        degreesToMicrosteps(angleDegrees)
    );

    return true;
}

void MotorPosition::stop()
{
    if (enabled_)
    {
        stepper_.stop();
    }
}

void MotorPosition::emergencyStop()
{
    const long currentMicrosteps =
        stepper_.currentPosition();

    stepper_.setCurrentPosition(
        currentMicrosteps
    );

    digitalWrite(enablePin_, HIGH);

    enabled_ = false;
}

void MotorPosition::setCurrentPosition(
    float angleDegrees
)
{
    stepper_.setCurrentPosition(
        degreesToMicrosteps(angleDegrees)
    );
}

void MotorPosition::setMaxSpeed(
    float speedDegreesPerSecond
)
{
    if (speedDegreesPerSecond <= 0.0F)
    {
        return;
    }

    stepper_.setMaxSpeed(
        speedDegreesPerSecond *
        stepsPerDegree_
    );
}

void MotorPosition::setAcceleration(
    float accelerationDegreesPerSecondSquared
)
{
    if (
        accelerationDegreesPerSecondSquared <= 0.0F
    )
    {
        return;
    }

    stepper_.setAcceleration(
        accelerationDegreesPerSecondSquared *
        stepsPerDegree_
    );
}

bool MotorPosition::isMoving()
{
    return
        enabled_ &&
        stepper_.distanceToGo() != 0;
}

bool MotorPosition::isWithinLimits(
    float angleDegrees
) const
{
    return
        angleDegrees >= minimumAngle_ &&
        angleDegrees <= maximumAngle_;
}

float MotorPosition::currentPosition()
{
    return microstepsToDegrees(
        stepper_.currentPosition()
    );
}

float MotorPosition::targetPosition()
{
    return microstepsToDegrees(
        stepper_.targetPosition()
    );
}

float MotorPosition::remainingDistance()
{
    return microstepsToDegrees(
        stepper_.distanceToGo()
    );
}

long MotorPosition::degreesToMicrosteps(
    float angleDegrees
) const
{
    return lroundf(
        angleDegrees *
        stepsPerDegree_
    );
}

float MotorPosition::microstepsToDegrees(
    long microsteps
) const
{
    return
        static_cast<float>(microsteps) /
        stepsPerDegree_;
}