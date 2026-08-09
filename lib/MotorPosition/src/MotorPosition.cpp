#include "MotorPosition.h"

#include <math.h>


// ============================================================
// CONSTRUTOR
// ============================================================

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
    :
    stepper_(
        AccelStepper::DRIVER,
        stepPin,
        dirPin
    ),

    enablePin_(
        enablePin
    ),

    fullStepsPerRevolution_(
        fullStepsPerRevolution
    ),

    microstepFactor_(
        microstepFactor
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

    enabled_(
        false
    ),

    maxSpeedDegrees_(
        0.0F
    ),

    accelerationDegrees_(
        0.0F
    )
{
}


// ============================================================
// BEGIN — UNIDADES INTERNAS
// ============================================================

void MotorPosition::begin(
    float maximumSpeedMicrostepsPerSecond,
    float accelerationMicrostepsPerSecondSquared,
    uint16_t minimumPulseWidth
)
{
    // --------------------------------------------------------
    // ENABLE do A4988
    // --------------------------------------------------------

    pinMode(
        enablePin_,
        OUTPUT
    );


    // Começa sempre desabilitado por segurança.
    disable();


    // --------------------------------------------------------
    // DIREÇÃO
    // --------------------------------------------------------

    stepper_.setPinsInverted(
        invertDirection_,
        false,
        false
    );


    // --------------------------------------------------------
    // CONFIGURAÇÃO DO ACCELSTEPPER
    // --------------------------------------------------------

    stepper_.setMaxSpeed(
        maximumSpeedMicrostepsPerSecond
    );


    stepper_.setAcceleration(
        accelerationMicrostepsPerSecondSquared
    );


    stepper_.setMinPulseWidth(
        minimumPulseWidth
    );


    // --------------------------------------------------------
    // REFERÊNCIA LÓGICA INICIAL
    // --------------------------------------------------------

    stepper_.setCurrentPosition(
        0
    );


    // --------------------------------------------------------
    // Guarda também os valores em unidades físicas
    // --------------------------------------------------------

    const float degreesPerMicrostep =
        360.0F
        /
        (
            static_cast<float>(
                fullStepsPerRevolution_
            )
            *
            static_cast<float>(
                microstepFactor_
            )
        );


    maxSpeedDegrees_ =
        maximumSpeedMicrostepsPerSecond
        *
        degreesPerMicrostep;


    accelerationDegrees_ =
        accelerationMicrostepsPerSecondSquared
        *
        degreesPerMicrostep;
}


// ============================================================
// BEGIN — UNIDADES FÍSICAS
// ============================================================

void MotorPosition::beginDegrees(
    float maximumSpeedDegreesPerSecond,
    float accelerationDegreesPerSecondSquared,
    uint16_t minimumPulseWidth
)
{
    // --------------------------------------------------------
    // ENABLE
    // --------------------------------------------------------

    pinMode(
        enablePin_,
        OUTPUT
    );


    // Motor inicia desabilitado por segurança.
    disable();


    // --------------------------------------------------------
    // DIREÇÃO
    // --------------------------------------------------------

    stepper_.setPinsInverted(
        invertDirection_,
        false,
        false
    );


    // --------------------------------------------------------
    // LARGURA MÍNIMA DO PULSO STEP
    // --------------------------------------------------------

    stepper_.setMinPulseWidth(
        minimumPulseWidth
    );


    // --------------------------------------------------------
    // REFERÊNCIA LÓGICA
    // --------------------------------------------------------

    stepper_.setCurrentPosition(
        0
    );


    // --------------------------------------------------------
    // VELOCIDADE
    // --------------------------------------------------------

    setMaxSpeedDegrees(
        maximumSpeedDegreesPerSecond
    );


    // --------------------------------------------------------
    // ACELERAÇÃO
    // --------------------------------------------------------

    setAccelerationDegrees(
        accelerationDegreesPerSecondSquared
    );
}


// ============================================================
// UPDATE
// ============================================================

void MotorPosition::update()
{
    // AccelStepper::run() precisa ser chamada continuamente.
    //
    // Quando o driver está desabilitado não há razão para
    // continuar gerando movimentos lógicos.
    if (enabled_)
    {
        stepper_.run();
    }
}


// ============================================================
// ENABLE
// ============================================================

void MotorPosition::enable()
{
    // A4988:
    //
    // ENABLE = LOW -> saídas habilitadas.
    digitalWrite(
        enablePin_,
        LOW
    );


    enabled_ =
        true;
}


// ============================================================
// DISABLE
// ============================================================

void MotorPosition::disable()
{
    // A4988:
    //
    // ENABLE = HIGH -> saídas desabilitadas.
    digitalWrite(
        enablePin_,
        HIGH
    );


    enabled_ =
        false;
}


// ============================================================
// ESTADO ENABLE
// ============================================================

bool MotorPosition::isEnabled() const
{
    return enabled_;
}


// ============================================================
// MOVE TO
// ============================================================

bool MotorPosition::moveTo(
    float angleDegrees
)
{
    // Não aceitamos comando de movimento com o driver
    // desabilitado.
    if (!enabled_)
    {
        return false;
    }


    // Proteção contra saída da região mecânica.
    if (
        !isWithinLimits(
            angleDegrees
        )
    )
    {
        return false;
    }


    // Converte graus em micropassos e envia o destino
    // ao AccelStepper.
    stepper_.moveTo(
        degreesToMicrosteps(
            angleDegrees
        )
    );


    return true;
}


// ============================================================
// STOP CONTROLADO
// ============================================================

void MotorPosition::stop()
{
    if (!enabled_)
    {
        return;
    }


    // AccelStepper calcula uma posição de parada compatível
    // com a aceleração/desaceleração configurada.
    stepper_.stop();
}


// ============================================================
// EMERGENCY STOP
// ============================================================

void MotorPosition::emergencyStop()
{
    // Obtém apenas a posição lógica que o software acredita
    // possuir naquele instante.
    const long currentMicrosteps =
        stepper_.currentPosition();


    // Cancela o movimento pendente.
    stepper_.setCurrentPosition(
        currentMicrosteps
    );


    // Remove torque imediatamente.
    disable();
}


// ============================================================
// DEFINIR POSIÇÃO ATUAL
// ============================================================

void MotorPosition::setCurrentPosition(
    float angleDegrees
)
{
    stepper_.setCurrentPosition(
        degreesToMicrosteps(
            angleDegrees
        )
    );
}


// ============================================================
// VELOCIDADE MÁXIMA EM GRAUS/S
// ============================================================

void MotorPosition::setMaxSpeedDegrees(
    float degreesPerSecond
)
{
    // AccelStepper requer velocidade positiva.
    if (degreesPerSecond <= 0.0F)
    {
        return;
    }


    maxSpeedDegrees_ =
        degreesPerSecond;


    const float microstepsPerSecond =
        degreesToMicrostepsFactor(
            degreesPerSecond
        );


    stepper_.setMaxSpeed(
        microstepsPerSecond
    );
}


// ============================================================
// ACELERAÇÃO EM GRAUS/S²
// ============================================================

void MotorPosition::setAccelerationDegrees(
    float degreesPerSecondSquared
)
{
    // AccelStepper requer aceleração positiva.
    if (
        degreesPerSecondSquared
        <= 0.0F
    )
    {
        return;
    }


    accelerationDegrees_ =
        degreesPerSecondSquared;


    const float microstepsPerSecondSquared =
        degreesToMicrostepsFactor(
            degreesPerSecondSquared
        );


    stepper_.setAcceleration(
        microstepsPerSecondSquared
    );
}


// ============================================================
// GET VELOCIDADE
// ============================================================

float MotorPosition::maxSpeedDegrees() const
{
    return maxSpeedDegrees_;
}


// ============================================================
// GET ACELERAÇÃO
// ============================================================

float MotorPosition::accelerationDegrees() const
{
    return accelerationDegrees_;
}


// ============================================================
// MOTOR EM MOVIMENTO?
// ============================================================

bool MotorPosition::isMoving()
{
    // OBSERVAÇÃO:
    //
    // Não declaramos esta função como const porque
    // AccelStepper::distanceToGo() não é const na versão
    // 1.64.0 da biblioteca.

    return
        enabled_
        &&
        (
            stepper_.distanceToGo()
            != 0
        );
}


// ============================================================
// LIMITES
// ============================================================

bool MotorPosition::isWithinLimits(
    float angleDegrees
) const
{
    return
        (
            angleDegrees
            >=
            minimumAngle_
        )
        &&
        (
            angleDegrees
            <=
            maximumAngle_
        );
}


// ============================================================
// POSIÇÃO ATUAL
// ============================================================

float MotorPosition::currentPosition()
{
    // Também não é const devido à interface da AccelStepper.

    return
        microstepsToDegrees(
            stepper_.currentPosition()
        );
}


// ============================================================
// POSIÇÃO DE DESTINO
// ============================================================

float MotorPosition::targetPosition()
{
    return
        microstepsToDegrees(
            stepper_.targetPosition()
        );
}


// ============================================================
// DISTÂNCIA RESTANTE
// ============================================================

float MotorPosition::remainingDistance()
{
    return
        microstepsToDegrees(
            stepper_.distanceToGo()
        );
}


// ============================================================
// GRAUS -> MICROSTEPS
// ============================================================

long MotorPosition::degreesToMicrosteps(
    float angleDegrees
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


    const float microsteps =
        angleDegrees
        *
        microstepsPerRevolution
        /
        360.0F;


    return lroundf(
        microsteps
    );
}


// ============================================================
// MICROSTEPS -> GRAUS
// ============================================================

float MotorPosition::microstepsToDegrees(
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


// ============================================================
// CONVERSÃO DE TAXAS
// ============================================================

float MotorPosition::degreesToMicrostepsFactor(
    float valueInDegrees
) const
{
    // Para velocidade:
    //
    // deg/s -> microsteps/s
    //
    // Para aceleração:
    //
    // deg/s² -> microsteps/s²
    //
    // O fator espacial é exatamente o mesmo.

    const float microstepsPerRevolution =
        static_cast<float>(
            fullStepsPerRevolution_
        )
        *
        static_cast<float>(
            microstepFactor_
        );


    return
        valueInDegrees
        *
        microstepsPerRevolution
        /
        360.0F;
}