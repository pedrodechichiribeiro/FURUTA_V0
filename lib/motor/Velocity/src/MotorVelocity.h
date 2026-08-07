#pragma once

#include <Arduino.h>
#include <AccelStepper.h>

class MotorVelocity
{
public:
    MotorVelocity(
        uint8_t stepPin,
        uint8_t dirPin,
        uint8_t enablePin,
        uint16_t fullStepsPerRevolution,
        uint8_t microstepFactor,
        bool invertDirection = false
    );

    void begin(
        float maximumSpeedMicrostepsPerSecond,
        uint16_t minimumPulseWidth = 2
    );

    void update();

    void enable();
    void disable();
    bool isEnabled() const;

    void setSpeedMicrosteps(float microstepsPerSecond);
    void setSpeedDegrees(float degreesPerSecond);
    void stop();

    float speedMicrosteps() const;
    float speedDegrees() const;
    float currentPosition() const;
    void setCurrentPosition(float angleDegrees);

private:
    long degreesToMicrosteps(float angleDegrees) const;
    float microstepsToDegrees(long microsteps) const;

    AccelStepper stepper_;

    uint8_t enablePin_;
    uint16_t fullStepsPerRevolution_;
    uint8_t microstepFactor_;
    bool invertDirection_;
    bool enabled_;
    float commandedSpeed_;
};
