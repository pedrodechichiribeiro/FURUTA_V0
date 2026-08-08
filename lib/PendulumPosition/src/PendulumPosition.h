#pragma once

#include <Arduino.h>
#include <AS5600.h>

/**
 * Responsável exclusivamente por:
 *
 * - inicializar o AS5600;
 * - ler uma amostra angular;
 * - acompanhar a posição contínua;
 * - definir a posição inferior como referência;
 * - calcular o ângulo em relação à posição inferior;
 * - calcular o erro em torno da vertical.
 *
 * Esta classe não calcula velocidade.
 */



class PendulumPosition
{
public:

    PendulumPosition(AS5600 &sensor, float directionSign = 1.0F);

    // Inicializa o AS5600.
    bool begin();

    // Faz uma nova leitura do sensor.
    bool update();

    // Define a posição vertical superior como beta = 0.
    // Faz várias leituras para reduzir o efeito de ruído.
    bool calibrateTop(
        uint8_t numberOfSamples = 32,
        uint16_t intervalMilliseconds = 2
    );

    bool topIsDefined() const;

    // Leitura bruta 0 ... 4095.
    uint16_t raw() const;

    // Ângulo absoluto do AS5600 em radianos.
    float absoluteAngleRadians() const;

    // Erro angular em relação ao topo:
    //
    // -PI <= beta < PI
    //
    float betaRadians() const;

    uint16_t topRaw() const;

    int lastError() const;

    bool magnetDetected();
    bool magnetTooWeak();
    bool magnetTooStrong();


private:

    // Diferença circular entre duas leituras de 12 bits.
    //
    // resultado:
    //
    // -2048 ... +2047
    //
    static int16_t circularDifference(
        uint16_t current,
        uint16_t reference
    );

    AS5600 &sensor_;

    float directionSign_;

    uint16_t raw_;
    uint16_t topRaw_;

    float absoluteAngleRadians_;
    float betaRadians_;

    bool topDefined_;

    int lastError_;
};