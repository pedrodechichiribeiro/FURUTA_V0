#include "PendulumVelocity.h"

#include <math.h>


// ============================================================
// CONSTRUTOR
// ============================================================

PendulumVelocity::PendulumVelocity()
    : sampleCount_(0),
      previousWrappedBeta_(0.0F),
      continuousBeta_(0.0F),
      unwrapInitialized_(false),
      velocityRadiansPerSecond_(0.0F),
      samplePeriodSeconds_(0.0F),
      previousTimeMicroseconds_(0),
      initialized_(false)
{
}


// ============================================================
// RESET
// ============================================================

void PendulumVelocity::reset(
    float betaRadians,
    uint32_t timeMicroseconds
)
{
    sampleCount_ = 0;

    velocityRadiansPerSecond_ = 0.0F;
    samplePeriodSeconds_ = 0.0F;

    previousTimeMicroseconds_ =
        timeMicroseconds;


    // Inicializa o desenrolamento angular.
    previousWrappedBeta_ =
        betaRadians;

    continuousBeta_ =
        betaRadians;

    unwrapInitialized_ =
        true;


    // Primeira amostra da janela.
    betaBuffer_[0] =
        continuousBeta_;

    timeBuffer_[0] =
        timeMicroseconds;

    sampleCount_ = 1;

    initialized_ = true;
}


// ============================================================
// UPDATE
// ============================================================

bool PendulumVelocity::update(
    float betaRadians,
    uint32_t timeMicroseconds
)
{
    if (!initialized_)
    {
        reset(
            betaRadians,
            timeMicroseconds
        );

        return false;
    }


    // --------------------------------------------------------
    // Período real da última aquisição
    // --------------------------------------------------------

    const uint32_t deltaTimeUs =
        timeMicroseconds -
        previousTimeMicroseconds_;

    previousTimeMicroseconds_ =
        timeMicroseconds;


    if (deltaTimeUs > 0)
    {
        samplePeriodSeconds_ =
            static_cast<float>(deltaTimeUs)
            * 1.0e-6F;
    }


    // --------------------------------------------------------
    // Remove a descontinuidade +/- PI
    // --------------------------------------------------------

    const float continuousBeta =
        unwrap(betaRadians);


    // --------------------------------------------------------
    // Coloca na janela
    // --------------------------------------------------------

    insertSample(
        continuousBeta,
        timeMicroseconds
    );


    // --------------------------------------------------------
    // Calcula velocidade
    // --------------------------------------------------------

    calculateRegression();


    // Para uso no controle consideramos a velocidade válida
    // somente depois que a janela completa estiver preenchida.
    return isReady();
}


// ============================================================
// VELOCIDADE
// ============================================================

float PendulumVelocity::radiansPerSecond() const
{
    return velocityRadiansPerSecond_;
}


// ============================================================
// PERÍODO DE AMOSTRAGEM
// ============================================================

float PendulumVelocity::samplePeriodSeconds() const
{
    return samplePeriodSeconds_;
}


// ============================================================
// ESTADO DO ESTIMADOR
// ============================================================

bool PendulumVelocity::isReady() const
{
    return sampleCount_ >= WINDOW_SIZE;
}


// ============================================================
// WRAP
// ============================================================

float PendulumVelocity::wrapToPi(float angle)
{
    while (angle >= PI)
    {
        angle -= TWO_PI;
    }

    while (angle < -PI)
    {
        angle += TWO_PI;
    }

    return angle;
}


// ============================================================
// UNWRAP
// ============================================================

float PendulumVelocity::unwrap(
    float betaRadians
)
{
    if (!unwrapInitialized_)
    {
        previousWrappedBeta_ =
            betaRadians;

        continuousBeta_ =
            betaRadians;

        unwrapInitialized_ =
            true;

        return continuousBeta_;
    }


    // Calcula apenas o pequeno deslocamento ocorrido
    // desde a última leitura.
    const float delta =
        wrapToPi(
            betaRadians -
            previousWrappedBeta_
        );


    continuousBeta_ += delta;

    previousWrappedBeta_ =
        betaRadians;


    return continuousBeta_;
}


// ============================================================
// INSERÇÃO NA JANELA
// ============================================================

void PendulumVelocity::insertSample(
    float continuousBeta,
    uint32_t timeMicroseconds
)
{
    // Enquanto a janela ainda não estiver cheia.
    if (sampleCount_ < WINDOW_SIZE)
    {
        betaBuffer_[sampleCount_] =
            continuousBeta;

        timeBuffer_[sampleCount_] =
            timeMicroseconds;

        ++sampleCount_;

        return;
    }


    // Janela cheia:
    // desloca tudo uma posição para a esquerda.
    //
    // Como são apenas 7 amostras, esta solução é simples
    // e suficientemente rápida para 250 Hz no Arduino Nano.
    for (
        uint8_t i = 0;
        i < WINDOW_SIZE - 1;
        ++i
    )
    {
        betaBuffer_[i] =
            betaBuffer_[i + 1];

        timeBuffer_[i] =
            timeBuffer_[i + 1];
    }


    betaBuffer_[WINDOW_SIZE - 1] =
        continuousBeta;

    timeBuffer_[WINDOW_SIZE - 1] =
        timeMicroseconds;
}


// ============================================================
// REGRESSÃO LINEAR
// ============================================================

void PendulumVelocity::calculateRegression()
{
    if (sampleCount_ < 3)
    {
        velocityRadiansPerSecond_ =
            0.0F;

        return;
    }


    // Utilizamos o primeiro instante como t = 0.
    //
    // Isso é importante porque micros() possui números grandes
    // e queremos boa precisão numérica no float do AVR.
    const uint32_t referenceTimeUs =
        timeBuffer_[0];


    float meanTime = 0.0F;
    float meanBeta = 0.0F;


    // --------------------------------------------------------
    // Média de t e beta
    // --------------------------------------------------------

    for (
        uint8_t i = 0;
        i < sampleCount_;
        ++i
    )
    {
        const float timeSeconds =
            static_cast<float>(
                timeBuffer_[i] -
                referenceTimeUs
            )
            * 1.0e-6F;


        meanTime +=
            timeSeconds;

        meanBeta +=
            betaBuffer_[i];
    }


    const float numberOfSamples =
        static_cast<float>(
            sampleCount_
        );


    meanTime /=
        numberOfSamples;

    meanBeta /=
        numberOfSamples;


    // --------------------------------------------------------
    // Inclinação da reta de mínimos quadrados
    // --------------------------------------------------------

    float numerator = 0.0F;
    float denominator = 0.0F;


    for (
        uint8_t i = 0;
        i < sampleCount_;
        ++i
    )
    {
        const float timeSeconds =
            static_cast<float>(
                timeBuffer_[i] -
                referenceTimeUs
            )
            * 1.0e-6F;


        const float dt =
            timeSeconds -
            meanTime;


        const float dbeta =
            betaBuffer_[i] -
            meanBeta;


        numerator +=
            dt * dbeta;

        denominator +=
            dt * dt;
    }


    if (denominator <= 0.0F)
    {
        velocityRadiansPerSecond_ =
            0.0F;

        return;
    }


    // --------------------------------------------------------
    // Inclinação = d(beta)/dt
    // --------------------------------------------------------

    velocityRadiansPerSecond_ =
        numerator /
        denominator;
}