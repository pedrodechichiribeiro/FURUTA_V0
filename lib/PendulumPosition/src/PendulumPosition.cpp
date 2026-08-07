#include "PendulumPosition.h"

bool PendulumPosition::begin(
    uint32_t i2cClockHz,
    uint8_t direction
)
{
    Wire.begin();
    Wire.setClock(i2cClockHz);

    /*
     * begin() sem parâmetro:
     *
     * - não utiliza um pino Arduino para controlar DIR;
     * - a direção pode ser definida por software;
     * - evita conflito com o D4 usado pelo ENABLE do A4988.
     */
    sensor_.begin();
    sensor_.setDirection(direction);

    delay(20);

    connected_ = sensor_.isConnected();

    if (!connected_) {
        return false;
    }

    /*
     * readAngle() realiza a leitura I2C.
     *
     * getCumulativePosition(false) reutiliza a mesma
     * amostra, evitando uma segunda leitura do sensor.
     */
    rawAngle_ = sensor_.readAngle();

    cumulativeCounts_ =
        sensor_.getCumulativePosition(false);

    continuousAngleRad_ =
        static_cast<float>(cumulativeCounts_) *
        RAD_PER_COUNT;

    updateDerivedAngles();

    return true;
}

bool PendulumPosition::update()
{
    if (!connected_) {
        return false;
    }

    /*
     * Uma única leitura física do AS5600.
     */
    rawAngle_ = sensor_.readAngle();

    /*
     * false informa à biblioteca que deve reutilizar
     * o valor obtido por readAngle().
     */
    cumulativeCounts_ =
        sensor_.getCumulativePosition(false);

    continuousAngleRad_ =
        static_cast<float>(cumulativeCounts_) *
        RAD_PER_COUNT;

    updateDerivedAngles();

    return true;
}

void PendulumPosition::defineLowerReference()
{
    /*
     * A posição contínua atual passa a representar
     * o ponto inferior do pêndulo.
     */
    lowerReferenceContinuousRad_ =
        continuousAngleRad_;

    referenceDefined_ = true;

    updateDerivedAngles();
}

void PendulumPosition::updateDerivedAngles()
{
    if (!referenceDefined_) {
        angleFromLowerUnwrappedRad_ = 0.0F;
        angleFromLowerWrappedRad_ = 0.0F;
        equilibriumErrorRad_ = 0.0F;

        return;
    }

    angleFromLowerUnwrappedRad_ =
        continuousAngleRad_ -
        lowerReferenceContinuousRad_;

    angleFromLowerWrappedRad_ =
        wrapToTwoPi(
            angleFromLowerUnwrappedRad_
        );

    /*
     * A posição vertical fica PI radianos distante
     * da posição inferior.
     *
     * Portanto:
     *
     * erro = alpha - PI
     */
    equilibriumErrorRad_ =
        wrapToPi(
            angleFromLowerUnwrappedRad_ - PI
        );
}

float PendulumPosition::wrapToPi(float angle)
{
    while (angle >= PI) {
        angle -= TWO_PI;
    }

    while (angle < -PI) {
        angle += TWO_PI;
    }

    return angle;
}

float PendulumPosition::wrapToTwoPi(float angle)
{
    while (angle >= TWO_PI) {
        angle -= TWO_PI;
    }

    while (angle < 0.0F) {
        angle += TWO_PI;
    }

    return angle;
}

bool PendulumPosition::isConnected() const
{
    return connected_;
}

bool PendulumPosition::isReferenceDefined() const
{
    return referenceDefined_;
}

bool PendulumPosition::magnetDetected()
{
    return sensor_.magnetDetected();
}

bool PendulumPosition::magnetTooWeak()
{
    return sensor_.magnetTooWeak();
}

bool PendulumPosition::magnetTooStrong()
{
    return sensor_.magnetTooStrong();
}

uint16_t PendulumPosition::magneticMagnitude()
{
    return sensor_.readMagnitude();
}

uint16_t PendulumPosition::rawAngle() const
{
    return rawAngle_;
}

int32_t PendulumPosition::cumulativeCounts() const
{
    return cumulativeCounts_;
}

float PendulumPosition::continuousAngleRad() const
{
    return continuousAngleRad_;
}

float PendulumPosition::angleFromLowerUnwrappedRad() const
{
    return angleFromLowerUnwrappedRad_;
}

float PendulumPosition::angleFromLowerWrappedRad() const
{
    return angleFromLowerWrappedRad_;
}

float PendulumPosition::equilibriumErrorRad() const
{
    return equilibriumErrorRad_;
}