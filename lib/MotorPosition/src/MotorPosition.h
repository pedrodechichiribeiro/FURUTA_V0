#pragma once

#include <Arduino.h>
#include <AccelStepper.h>


class MotorPosition
{
public:

    // ========================================================
    // CONSTRUTOR
    // ========================================================

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


    // ========================================================
    // INICIALIZAÇÃO
    // ========================================================

    // Inicialização em unidades internas do stepper:
    //
    // velocidade  -> micropassos/s
    // aceleração  -> micropassos/s²
    //
    // Mantida para compatibilidade com fases anteriores.
    void begin(
        float maximumSpeedMicrostepsPerSecond,
        float accelerationMicrostepsPerSecondSquared,
        uint16_t minimumPulseWidth = 2
    );


    // Inicialização preferida a partir da Fase 06:
    //
    // velocidade  -> graus/s
    // aceleração  -> graus/s²
    void beginDegrees(
        float maximumSpeedDegreesPerSecond,
        float accelerationDegreesPerSecondSquared,
        uint16_t minimumPulseWidth = 2
    );


    // Deve ser chamada continuamente no loop().
    void update();


    // ========================================================
    // ENABLE / DISABLE
    // ========================================================

    void enable();

    void disable();

    bool isEnabled() const;


    // ========================================================
    // MOVIMENTO
    // ========================================================

    // Define uma nova posição de destino em graus.
    //
    // Retorna false se:
    // - motor estiver desabilitado;
    // - destino estiver fora dos limites.
    bool moveTo(
        float angleDegrees
    );


    // Parada desacelerada usando AccelStepper.
    void stop();


    // Parada imediata + desabilitação do A4988.
    //
    // A referência física deve ser considerada perdida
    // depois desta operação.
    void emergencyStop();


    // Define a posição lógica atual.
    //
    // Não movimenta fisicamente o motor.
    void setCurrentPosition(
        float angleDegrees
    );


    // ========================================================
    // CONFIGURAÇÃO DINÂMICA
    // ========================================================

    void setMaxSpeedDegrees(
        float degreesPerSecond
    );


    void setAccelerationDegrees(
        float degreesPerSecondSquared
    );


    float maxSpeedDegrees() const;

    float accelerationDegrees() const;


    // ========================================================
    // ESTADO
    // ========================================================

    // Estes métodos NÃO são const porque a biblioteca
    // AccelStepper 1.64.0 não declara os métodos correspondentes
    // como const.

    bool isMoving();

    float currentPosition();

    float targetPosition();

    float remainingDistance();


    // Este pode continuar const porque não acessa um método
    // não-const da AccelStepper.
    bool isWithinLimits(
        float angleDegrees
    ) const;


private:

    // ========================================================
    // CONVERSÕES
    // ========================================================

    long degreesToMicrosteps(
        float angleDegrees
    ) const;


    float microstepsToDegrees(
        long microsteps
    ) const;


    // A conversão numérica é a mesma tanto para velocidade
    // quanto para aceleração:
    //
    // graus -> micropassos
    //
    // mudando apenas a unidade temporal.
    float degreesToMicrostepsFactor(
        float valueInDegrees
    ) const;


    // ========================================================
    // OBJETO ACCELSTEPPER
    // ========================================================

    AccelStepper stepper_;


    // ========================================================
    // HARDWARE
    // ========================================================

    uint8_t enablePin_;


    // ========================================================
    // CONFIGURAÇÃO MECÂNICA
    // ========================================================

    uint16_t fullStepsPerRevolution_;

    uint8_t microstepFactor_;


    // ========================================================
    // LIMITES
    // ========================================================

    float minimumAngle_;

    float maximumAngle_;


    // ========================================================
    // CONFIGURAÇÃO
    // ========================================================

    bool invertDirection_;

    bool enabled_;


    // Valores armazenados em unidades físicas para permitir
    // consulta posterior pela aplicação.
    float maxSpeedDegrees_;

    float accelerationDegrees_;
};