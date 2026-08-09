#include "MotorVelocity.h"

#include <math.h>


MotorVelocity::MotorVelocity(
    uint8_t stepPin,
    uint8_t dirPin,
    uint8_t enablePin,
    uint16_t fullStepsPerRevolution,
    uint8_t microstepFactor,
    float minimumAngleDegrees,
    float maximumAngleDegrees,
    bool invertDirection
)
    :
    stepper_(
        AccelStepper::DRIVER,
        stepPin,
        dirPin
    ),
    enablePin_(enablePin),
    fullStepsPerRevolution_(fullStepsPerRevolution),
    microstepFactor_(microstepFactor),
    minimumAngleDegrees_(minimumAngleDegrees),
    maximumAngleDegrees_(maximumAngleDegrees),
    invertDirection_(invertDirection),
    enabled_(false),
    speedReferenceDegreesPerSecond_(0.0F),
    maximumSpeedDegreesPerSecond_(0.0F)
{
}


void MotorVelocity::begin(
    float maximumSpeedDegreesPerSecond,
    uint16_t minimumPulseWidth
)
{
    pinMode(
        enablePin_,
        OUTPUT
    );

    disable();

    stepper_.setPinsInverted(
        invertDirection_,
        false,
        false
    );

    stepper_.setMinPulseWidth(
        minimumPulseWidth
    );

    maximumSpeedDegreesPerSecond_ =
        maximumSpeedDegreesPerSecond;

    stepper_.setMaxSpeed(
        degreesPerSecondToMicrostepsPerSecond(
            maximumSpeedDegreesPerSecond_
        )
    );

    stepper_.setCurrentPosition(0);

    speedReferenceDegreesPerSecond_ =
        0.0F;

    stepper_.setSpeed(0.0F);
}


void MotorVelocity::update()
{
    if (!enabled_)
    {
        return;
    }

    stepper_.runSpeed();
}


void MotorVelocity::enable()
{
    digitalWrite(
        enablePin_,
        LOW
    );

    enabled_ = true;
}


void MotorVelocity::disable()
{
    // Zera qualquer comando anterior.
    speedReferenceDegreesPerSecond_ = 0.0F;

    stepper_.setSpeed(0.0F);

    // A4988 desabilitado.
    digitalWrite(
        enablePin_,
        HIGH
    );

    enabled_ = false;
}


bool MotorVelocity::isEnabled() const
{
    return enabled_;
}


void MotorVelocity::setCurrentPosition(
    float angleDegrees
)
{
    stepper_.setCurrentPosition(
        degreesToMicrosteps(
            angleDegrees
        )
    );
}


bool MotorVelocity::commandAcceleration(
    float accelerationDegreesPerSecondSquared,
    float dtSeconds
)
{
    if (!enabled_)
    {
        return false;
    }


    speedReferenceDegreesPerSecond_ +=
        accelerationDegreesPerSecondSquared
        *
        dtSeconds;


    // Saturação de velocidade.
    if (
        speedReferenceDegreesPerSecond_
        >
        maximumSpeedDegreesPerSecond_
    )
    {
        speedReferenceDegreesPerSecond_ =
            maximumSpeedDegreesPerSecond_;
    }


    if (
        speedReferenceDegreesPerSecond_
        <
        -maximumSpeedDegreesPerSecond_
    )
    {
        speedReferenceDegreesPerSecond_ =
            -maximumSpeedDegreesPerSecond_;
    }


    const float position =
        currentPositionDegrees();


    // Limite positivo.
    if (
        position >= maximumAngleDegrees_
        &&
        speedReferenceDegreesPerSecond_ > 0.0F
    )
    {
        stop();
        return false;
    }


    // Limite negativo.
    if (
        position <= minimumAngleDegrees_
        &&
        speedReferenceDegreesPerSecond_ < 0.0F
    )
    {
        stop();
        return false;
    }


    stepper_.setSpeed(
        degreesPerSecondToMicrostepsPerSecond(
            speedReferenceDegreesPerSecond_
        )
    );


    return true;
}


void MotorVelocity::stop()
{
    speedReferenceDegreesPerSecond_ =
        0.0F;

    stepper_.setSpeed(
        0.0F
    );
}


void MotorVelocity::emergencyStop()
{
    stop();
    disable();
}


float MotorVelocity::currentPositionDegrees()
{
    return microstepsToDegrees(
        stepper_.currentPosition()
    );
}


float MotorVelocity::speedReferenceDegreesPerSecond() const
{
    return speedReferenceDegreesPerSecond_;
}


float MotorVelocity::maximumSpeedDegreesPerSecond() const
{
    return maximumSpeedDegreesPerSecond_;
}


bool MotorVelocity::isWithinLimits(
    float angleDegrees
) const
{
    return
        angleDegrees >= minimumAngleDegrees_
        &&
        angleDegrees <= maximumAngleDegrees_;
}


long MotorVelocity::degreesToMicrosteps(
    float degrees
) const
{
    const float microstepsPerRevolution =
        static_cast<float>(
            fullStepsPerRevolution_
        )
        *
        static_cast<float>(
            microstepFactor_
        );


    return lroundf(
        degrees
        *
        microstepsPerRevolution
        /
        360.0F
    );
}


float MotorVelocity::microstepsToDegrees(
    long microsteps
) const
{
    const float microstepsPerRevolution =
        static_cast<float>(
            fullStepsPerRevolution_
        )
        *
        static_cast<float>(
            microstepFactor_
        );


    return
        static_cast<float>(
            microsteps
        )
        *
        360.0F
        /
        microstepsPerRevolution;
}


float MotorVelocity::
degreesPerSecondToMicrostepsPerSecond(
    float degreesPerSecond
) const
{
    const float microstepsPerRevolution =
        static_cast<float>(
            fullStepsPerRevolution_
        )
        *
        static_cast<float>(
            microstepFactor_
        );


    return
        degreesPerSecond
        *
        microstepsPerRevolution
        /
        360.0F;
}