#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>

#include <PendulumPosition.h>
#include <PendulumVelocity.h>
#include <PendulumDownReference.h>
#include <MotorVelocity.h>
#include <StateVector.h>
#include <FourStateController.h>
#include <BalanceTelemetry.h>
#include <FurutaConfig.h>

#include <math.h>
#include <ctype.h>
#include <string.h>

// ============================================================
// FASE 10 REVISAO
//
// Objetivo:
// - preservar TODAS as bibliotecas existentes;
// - preservar os ganhos e limites validados na Fase 09/10;
// - tornar a referencia DOWN mais robusta;
// - permitir diagnostico do TOP com o motor desabilitado OU habilitado em HOLD;
// - abortar/registrar falhas de leitura do AS5600;
// - imprimir os parametros realmente compilados;
// - manter ZERO telemetria Serial durante BALANCE;
// - testar uma captura/soltura mais estrita sem alterar os ganhos;
// - corrigir o primeiro intervalo de 8 ms na entrada do BALANCE;
// - registrar em RAM o inicio da resposta para diagnostico posterior.
//
// Nenhum arquivo em lib/ precisa ser alterado.
// ============================================================

namespace RevisionConfig
{
    // Parametros internos de instrumentacao/calibracao.
    // Em principio nao precisam ser alterados durante a sintonia.
    constexpr uint32_t DOWN_STABLE_TIME_US = 1000000UL;
    constexpr float DOWN_STABLE_SPEED_RAD_S = 0.08F;
    constexpr float DOWN_COLLECT_SPEED_RAD_S = 0.12F;
    constexpr uint32_t DOWN_CALIBRATION_TIME_MS = 3000UL;
    constexpr float DOWN_MAXIMUM_SPAN_DEG = 1.0F;

    constexpr uint32_t TOP_DIAGNOSTIC_PERIOD_US = 100000UL;
    constexpr uint32_t TOP_STABLE_TIME_US = 400000UL;
    constexpr uint32_t TOP_MEASURE_TIME_US = 2000000UL;
    constexpr float TOP_POSITION_MAX_DEG = 5.0F;
    constexpr float TOP_STABLE_SPEED_RAD_S = 0.20F;
    constexpr float TOP_COLLECT_SPEED_RAD_S = 0.25F;
    constexpr float TOP_MAX_SPAN_DEG = 0.80F;

    constexpr uint8_t MAX_CONSECUTIVE_SENSOR_ERRORS = 3;

    // Trace em RAM. Mantido fixo para controlar o uso de SRAM do Nano.
    constexpr uint8_t DEBUG_DECIMATION = 4;
    constexpr uint8_t DEBUG_MAX_SAMPLES = 48;
}

// ============================================================
// PARAMETROS DE ENSAIO / SINTONIA
//
// Estes valores sao fornecidos pelo main.cpp. Assim podemos testar
// ganhos, offset e limites sem editar qualquer arquivo em lib/.
// ============================================================
struct RevisionSettings
{
    float kPhi;
    float kPhiDot;
    float kBeta;
    float kBetaDot;

    uint32_t controlPeriodUs;
    float motorMaxSpeedDegS;
    float maxAccelDegS2;
    uint32_t maxControlDtUs;

    float topReferenceOffsetDeg;

    float captureArmMaxDeg;
    float captureBetaMaxDeg;
    float captureBetaDotMaxRadS;
    uint32_t captureStableTimeUs;

    float releaseBetaMaxDeg;
    float releaseVelocityRadS;
    uint8_t releaseConfirmSamples;

    float betaAbortDeg;
    float armAbortDeg;
    uint32_t balanceTestTimeUs;
};

class FurutaRevisionSystem
{
public:
    explicit FurutaRevisionSystem(const RevisionSettings &settings);
    void begin();
    void update();

private:
    enum class ControlState : uint8_t
    {
        IDLE,
        WAIT_CAPTURE,
        WAIT_RELEASE,
        BALANCE
    };

    struct Statistics
    {
        float maxAbsBetaRad;
        float maxAbsBetaDotRadS;
        float maxAbsPhiDeg;
        float maxAbsPhiDotDegS;
        float maxAbsControlDegS2;
        uint32_t saturationCount;
        uint32_t sampleCount;
        uint32_t sensorErrorCount;
        uint32_t maxControlDtUs;
        uint32_t minControlDtUs;
        uint32_t sumControlDtUs;
        uint32_t maxTickExecUs;
        uint32_t missedScheduleSlots;
    };

    // Estado compactado para diagnostico do inicio da instabilidade.
    // Os quatro termos de controle sao reconstruidos no dump a partir
    // desses estados e dos mesmos ganhos congelados em FurutaConfig.
    struct DebugSample
    {
        int16_t phi1e4;
        int16_t phiDot1e4;
        int16_t beta1e5;
        int16_t betaDot5e3;
    };

    // Parametros definidos no main.cpp.
    RevisionSettings settings_;

    // Hardware / libs existentes.
    AS5600 as5600_;
    PendulumPosition pendulumPosition_;
    PendulumVelocity pendulumVelocity_;
    PendulumVelocity downVelocity_;
    PendulumDownReference downReference_;
    MotorVelocity motor_;
    FourStateController controller_;
    BalanceTelemetry telemetry_;

    // Estado da aplicacao.
    ControlState controlState_;
    StateVector state_;
    ControlSignal control_;
    Statistics statistics_;

    bool armZeroDefined_;
    bool downReferenceDefined_;
    bool downCalibrationActive_;
    bool downCollecting_;
    bool topDiagnosticActive_;

    // Medicao/correcao experimental do TOP, exclusiva desta fase.
    bool topMeasureActive_;
    bool topCollecting_;
    uint32_t topStableStartUs_;
    uint32_t topCollectStartUs_;
    uint32_t topSampleCount_;
    float topSumBetaRad_;
    float topMinBetaRad_;
    float topMaxBetaRad_;

    // A PendulumDownReference deve receber um angulo RELATIVO.
    // Guardamos a primeira leitura da calibracao como ancora.
    float downAnchorAbsoluteRad_;
    uint32_t downStableStartUs_;
    uint32_t downCalibrationSamples_;
    uint32_t downCalibrationReadErrors_;

    uint8_t consecutiveSensorErrors_;
    uint32_t totalSensorErrors_;

    // Timing.
    uint32_t nextControlTimeUs_;
    uint32_t lastControlTimeUs_;
    uint32_t captureStableStartUs_;
    uint8_t releaseConfirmCount_;
    uint32_t balanceStartUs_;
    uint32_t nextTopDiagnosticUs_;

    // Snapshot do inicio do BALANCE, guardado sem imprimir na soltura.
    StateVector balanceInitialState_;

    // Trace em RAM: 48 x 8 = 384 bytes. Mantido pequeno para o ATmega328P.
    DebugSample debugSamples_[RevisionConfig::DEBUG_MAX_SAMPLES];
    uint8_t debugSampleCount_;
    uint8_t debugDecimationCounter_;

    // Serial.
    char serialBuffer_[FurutaConfig::SERIAL_BUFFER_SIZE];
    uint8_t serialBufferIndex_;

    // Utilitarios angulares.
    static float wrapToPi(float angleRad);

    // Medicao e referencia.
    bool acquireState(uint32_t nowUs);
    float rawRelativeToDownAnchor() const;
    float betaFromRobustDownReference() const;
    void startDownCalibration();
    void serviceDownCalibration(uint32_t nowUs, bool sensorReadingValid);
    void finishDownCalibration();

    // Diagnostico/medicao do TOP.
    void toggleTopDiagnostic();
    void serviceTopDiagnostic(uint32_t nowUs);
    void toggleTopMeasurement();
    void serviceTopMeasurement(uint32_t nowUs);
    void resetTopWindow();

    // Controle.
    void controlTick(uint32_t nowUs);
    void serviceWaitCapture(uint32_t nowUs);
    void serviceWaitRelease(uint32_t nowUs);
    void startBalance(uint32_t nowUs);
    void serviceBalance(uint32_t nowUs, float dtSeconds, uint32_t dtUs);
    void endBalance(const char *reason);
    void stopControl(bool printMessage = true);

    // Referencia do braco / motor.
    void defineArmZero();
    void enableMotor();
    void disableMotor();
    void armBalance();
    void emergencyStop();

    // Estatisticas.
    void resetControlSignal();
    void resetStatistics();
    void updateStatistics(uint32_t dtUs);

    // Trace de diagnostico.
    static int16_t toInt16Scaled(float value, float scale);
    void resetDebugTrace();
    void recordDebugTrace(uint32_t nowUs);
    void dumpDebugTrace();

    // Serial.
    void serviceSerial();
    void handleCommand(char *command);
    void printParameters();
    void printStatus();
    void printHelp();
    void printBalanceSummary(const char *reason);
};

