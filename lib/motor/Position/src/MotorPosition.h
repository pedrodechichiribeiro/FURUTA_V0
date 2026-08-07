#pragma once

#include <Arduino.h>
#include <AccelStepper.h>

class MotorPosition
{
public:
    MotorPosition(
        uint8_t stepPin,
        uint8_t dirPin,
        uint8_t enablePin,
        uint16_t fullStepsPerRevolution,
        uint8_t microstepFactor,
        float minimumAngle,
        float maximumAngle,
        bool invertDirection = false
    );

    void begin(
        float maximumSpeedDegreesPerSecond,
        float accelerationDegreesPerSecondSquared,
        uint16_t minimumPulseWidth = 2
    );

    void update();

    void enable();
    void disable();

    bool isEnabled() const;

    bool moveTo(float angleDegrees);

    void stop();
    void emergencyStop();

    void setCurrentPosition(float angleDegrees);

    void setMaxSpeed(
        float speedDegreesPerSecond
    );

    void setAcceleration(
        float accelerationDegreesPerSecondSquared
    );

    /*
     * Estes métodos não são const porque a versão 1.64
     * do AccelStepper não declara seus métodos de consulta
     * como const.
     */
    bool isMoving();

    float currentPosition();
    float targetPosition();
    float remainingDistance();

    bool isWithinLimits(
        float angleDegrees
    ) const;

private:
    long degreesToMicrosteps(
        float angleDegrees
    ) const;

    float microstepsToDegrees(
        long microsteps
    ) const;

    AccelStepper stepper_;

    uint8_t enablePin_;

    uint16_t fullStepsPerRevolution_;
    uint8_t microstepFactor_;

    float stepsPerDegree_;

    float minimumAngle_;
    float maximumAngle_;

    bool invertDirection_;
    bool enabled_;
};