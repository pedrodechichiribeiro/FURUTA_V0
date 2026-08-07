#include "MotorVelocity.h"

#include <math.h>

MotorVelocity::MotorVelocity(
    uint8_t stepPin,
    uint8_t dirPin,
    uint8_t enablePin,
    uint16_t fullStepsPerRevolution,
    uint8_t microstepFactor,
    bool invertDirection
)
    : stepper_(AccelStepper::DRIVER, stepPin, dirPin),
      enablePin_(enablePin),
      fullStepsPerRevolution_(fullStepsPerRevolution),
      microstepFactor_(microstepFactor),
      invertDirection_(invertDirection),
      enabled_(false),
      commandedSpeed_(0.0F)
{
}

void MotorVelocity::begin(
    float maximumSpeedMicrostepsPerSecond,
    uint16_t minimumPulseWidth
)
{
    pinMode(enablePin_, OUTPUT);
    disable();

    stepper_.setPinsInverted(invertDirection_, false, false);
    stepper_.setMaxSpeed(maximumSpeedMicrostepsPerSecond);
    stepper_.setMinPulseWidth(minimumPulseWidth);
    stepper_.setCurrentPosition(0);
    stepper_.setSpeed(0.0F);
}

void MotorVelocity::update()
{
    if (enabled_)
    {
        stepper_.runSpeed();
    }
}

void MotorVelocity::enable()
{
    digitalWrite(enablePin_, LOW);
    enabled_ = true;
}

void MotorVelocity::disable()
{
    digitalWrite(enablePin_, HIGH);
    enabled_ = false;
}

bool MotorVelocity::isEnabled() const
{
    return enabled_;
}

void MotorVelocity::setSpeedMicrosteps(float microstepsPerSecond)
{
    commandedSpeed_ = microstepsPerSecond;
    stepper_.setSpeed(commandedSpeed_);
}

void MotorVelocity::setSpeedDegrees(float degreesPerSecond)
{
    const float microstepsPerRevolution =
        static_cast<float>(fullStepsPerRevolution_) * microstepFactor_;

    setSpeedMicrosteps(
        degreesPerSecond * microstepsPerRevolution / 360.0F
    );
}

void MotorVelocity::stop()
{
    setSpeedMicrosteps(0.0F);
}

float MotorVelocity::speedMicrosteps() const
{
    return commandedSpeed_;
}

float MotorVelocity::speedDegrees() const
{
    const float microstepsPerRevolution =
        static_cast<float>(fullStepsPerRevolution_) * microstepFactor_;

    return commandedSpeed_ * 360.0F / microstepsPerRevolution;
}

float MotorVelocity::currentPosition() const
{
    return microstepsToDegrees(stepper_.currentPosition());
}

void MotorVelocity::setCurrentPosition(float angleDegrees)
{
    stepper_.setCurrentPosition(degreesToMicrosteps(angleDegrees));
}

long MotorVelocity::degreesToMicrosteps(float angleDegrees) const
{
    const float microstepsPerRevolution =
        static_cast<float>(fullStepsPerRevolution_) * microstepFactor_;

    return lroundf(angleDegrees * microstepsPerRevolution / 360.0F);
}

float MotorVelocity::microstepsToDegrees(long microsteps) const
{
    const float microstepsPerRevolution =
        static_cast<float>(fullStepsPerRevolution_) * microstepFactor_;

    return static_cast<float>(microsteps) * 360.0F /
           microstepsPerRevolution;
}
