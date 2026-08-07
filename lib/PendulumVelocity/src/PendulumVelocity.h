#ifndef PENDULUM_VELOCITY_H
#define PENDULUM_VELOCITY_H

#include <Arduino.h>

/**
 * Calcula a velocidade angular a partir da posição
 * angular contínua.
 *
 * Esta classe não conhece o AS5600 e não realiza
 * comunicação I2C.
 *
 * Entrada:
 *   posição contínua em radianos;
 *   instante da amostra em microssegundos.
 *
 * Saídas:
 *   velocidade bruta em rad/s;
 *   velocidade filtrada em rad/s.
 */
class PendulumVelocity
{
public:
    /**
     * filterTauSeconds:
     *
     * Constante de tempo do filtro passa-baixas.
     *
     * Valor menor:
     *   - resposta mais rápida;
     *   - mais ruído.
     *
     * Valor maior:
     *   - resposta mais suave;
     *   - maior atraso.
     */
    explicit PendulumVelocity(
        float filterTauSeconds = 0.030F
    );

    /**
     * Inicializa o estimador com a posição atual.
     */
    void begin(
        float initialPositionRad,
        uint32_t initialTimeUs
    );

    /**
     * Atualiza a velocidade usando:
     *
     * velocidade = variação da posição / variação do tempo
     *
     * Retorna true quando a atualização foi válida.
     */
    bool update(
        float positionRad,
        uint32_t currentTimeUs
    );

    /**
     * Reinicia o estimador.
     *
     * Deve ser usado após alterações de estado,
     * calibrações ou pausas longas.
     */
    void reset(
        float currentPositionRad,
        uint32_t currentTimeUs
    );

    void setFilterTau(float filterTauSeconds);

    float rawVelocityRadS() const;
    float filteredVelocityRadS() const;

    float sampleTimeSeconds() const;

    bool isInitialized() const;

private:
    float filterTauSeconds_;

    bool initialized_ = false;
    bool filterInitialized_ = false;

    float previousPositionRad_ = 0.0F;
    uint32_t previousTimeUs_ = 0;

    float rawVelocityRadS_ = 0.0F;
    float filteredVelocityRadS_ = 0.0F;

    float sampleTimeSeconds_ = 0.0F;

    /*
     * Evita considerar imediatamente as primeiras
     * amostras após a inicialização.
     */
    uint8_t warmupSamplesRemaining_ = 0;

    static constexpr uint8_t WARMUP_SAMPLES = 3;
};

#endif