#ifndef PENDULUM_POSITION_H
#define PENDULUM_POSITION_H

#include <Arduino.h>
#include <Wire.h>
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
    /**
     * Inicializa o barramento I2C e o AS5600.
     *
     * Retorna true quando o sensor responde no endereço 0x36.
     */
    bool begin(
        uint32_t i2cClockHz = 100000UL,
        uint8_t direction = AS5600_CLOCK_WISE
    );

    /**
     * Realiza uma única leitura do AS5600 e atualiza
     * todas as variáveis de posição.
     */
    bool update();

    /**
     * Define a posição atual como a posição inferior:
     *
     * alpha = 0 rad.
     *
     * O pêndulo deve estar parado e voltado para baixo.
     */
    void defineLowerReference();

    bool isConnected() const;
    bool isReferenceDefined() const;

    bool magnetDetected();
    bool magnetTooWeak();
    bool magnetTooStrong();

    uint16_t magneticMagnitude();

    /**
     * Leitura direta do AS5600:
     *
     * 0 até 4095.
     */
    uint16_t rawAngle() const;

    /**
     * Posição acumulada em contagens do AS5600.
     *
     * Pode ultrapassar 4095 ou assumir valores negativos.
     */
    int32_t cumulativeCounts() const;

    /**
     * Posição angular contínua em radianos.
     *
     * A origem desta variável é interna à biblioteca AS5600.
     * Ela é usada pela biblioteca de velocidade porque não
     * sofre saltos entre 0 e 360 graus.
     */
    float continuousAngleRad() const;

    /**
     * Ângulo contínuo medido a partir da posição inferior.
     *
     * Pode ultrapassar uma volta.
     */
    float angleFromLowerUnwrappedRad() const;

    /**
     * Ângulo medido a partir da posição inferior,
     * limitado entre 0 e 2*PI.
     *
     * 0 rad  = pêndulo para baixo.
     * PI rad = pêndulo para cima.
     */
    float angleFromLowerWrappedRad() const;

    /**
     * Erro em torno da posição vertical:
     *
     * 0 rad = pêndulo vertical para cima.
     *
     * Resultado limitado entre -PI e +PI.
     */
    float equilibriumErrorRad() const;

private:
    static constexpr float RAD_PER_COUNT =
        6.28318530717958647692F / 4096.0F;

    AS5600 sensor_;

    bool connected_ = false;
    bool referenceDefined_ = false;

    uint16_t rawAngle_ = 0;
    int32_t cumulativeCounts_ = 0;

    float continuousAngleRad_ = 0.0F;

    float lowerReferenceContinuousRad_ = 0.0F;

    float angleFromLowerUnwrappedRad_ = 0.0F;
    float angleFromLowerWrappedRad_ = 0.0F;
    float equilibriumErrorRad_ = 0.0F;

    void updateDerivedAngles();

    static float wrapToPi(float angle);
    static float wrapToTwoPi(float angle);
};

#endif