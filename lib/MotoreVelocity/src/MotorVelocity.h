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
        float minimumAngleDegrees,
        float maximumAngleDegrees,
        bool invertDirection = false
    );


    void begin(
        float maximumSpeedDegreesPerSecond,
        uint16_t minimumPulseWidth = 2
    );


    // Deve ser chamada continuamente no loop.
    void update();


    void enable();
    void disable();

    bool isEnabled() const;


    void setCurrentPosition(
        float angleDegrees
    );


    // Entrada fundamental desta biblioteca:
    //
    // aceleração [graus/s²]
    //
    // A velocidade é integrada internamente.
    bool commandAcceleration(
        float accelerationDegreesPerSecondSquared,
        float dtSeconds
    );


    // Zera imediatamente a referência de velocidade.
    void stop();


    void emergencyStop();


    float currentPositionDegrees();

    float speedReferenceDegreesPerSecond() const;

    float maximumSpeedDegreesPerSecond() const;


    bool isWithinLimits(
        float angleDegrees
    ) const;


private:

    long degreesToMicrosteps(
        float degrees
    ) const;


    float microstepsToDegrees(
        long microsteps
    ) const;


    float degreesPerSecondToMicrostepsPerSecond(
        float degreesPerSecond
    ) const;


    AccelStepper stepper_;

    uint8_t enablePin_;

    uint16_t fullStepsPerRevolution_;

    uint8_t microstepFactor_;

    float minimumAngleDegrees_;
    float maximumAngleDegrees_;

    bool invertDirection_;
    bool enabled_;

    float speedReferenceDegreesPerSecond_;
    float maximumSpeedDegreesPerSecond_;
};