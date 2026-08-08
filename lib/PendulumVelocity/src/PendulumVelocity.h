#pragma once

#include <Arduino.h>


class PendulumVelocity
{
public:

    // Janela adotada experimentalmente.
    static constexpr uint8_t WINDOW_SIZE = 7;

    PendulumVelocity();

    // Reinicia completamente o estimador.
    //
    // Deve ser chamado quando beta = 0 for redefinido
    // ou quando iniciarmos uma nova aquisição.
    void reset(
        float betaRadians,
        uint32_t timeMicroseconds
    );

    // Insere uma nova posição angular e atualiza beta_dot.
    //
    // Retorna true quando a janela completa de 7 amostras
    // já está disponível.
    bool update(
        float betaRadians,
        uint32_t timeMicroseconds
    );

    // Velocidade estimada por regressão linear [rad/s].
    float radiansPerSecond() const;

    // Período real entre as duas últimas aquisições [s].
    float samplePeriodSeconds() const;

    // Indica se já existem 7 amostras válidas.
    bool isReady() const;


private:

    // Coloca um ângulo no intervalo [-PI, +PI).
    static float wrapToPi(float angle);

    // Converte beta circular em uma posição angular contínua.
    float unwrap(float betaRadians);

    // Insere nova amostra na janela.
    void insertSample(
        float continuousBeta,
        uint32_t timeMicroseconds
    );

    // Calcula a inclinação da reta beta(t).
    void calculateRegression();


    // --------------------------------------------------------
    // Janela de regressão
    // --------------------------------------------------------

    float betaBuffer_[WINDOW_SIZE];
    uint32_t timeBuffer_[WINDOW_SIZE];

    uint8_t sampleCount_;


    // --------------------------------------------------------
    // Unwrap
    // --------------------------------------------------------

    float previousWrappedBeta_;
    float continuousBeta_;

    bool unwrapInitialized_;


    // --------------------------------------------------------
    // Resultado
    // --------------------------------------------------------

    float velocityRadiansPerSecond_;
    float samplePeriodSeconds_;

    uint32_t previousTimeMicroseconds_;

    bool initialized_;
};