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
// - usar referencia DOWN fixa em RAW=2888, sem calibracao T no fluxo;
// - permitir diagnostico do TOP com o motor desabilitado OU habilitado em HOLD;
// - abortar/registrar falhas de leitura do AS5600;
// - imprimir os parametros realmente compilados;
// - manter ZERO telemetria Serial durante BALANCE;
// - permitir ajuste dos ganhos em tempo de execucao pela Serial;
// - corrigir o primeiro intervalo de 8 ms na entrada do BALANCE;
// - registrar continuamente em RAM a cauda da resposta para diagnostico posterior;
// - medir medias de phi, beta e u para separar drift lento de chatter rapido.
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

    // Trace circular em RAM.
    // Ts=4 ms e decimacao 5 -> 20 ms = 50 Hz.
    // 16 amostras -> ~0,32 s imediatamente ANTES do fim do BALANCE.
    // Cada amostra = 8 bytes; buffer total = 128 bytes.
    constexpr uint8_t DEBUG_DECIMATION = 5;
    constexpr uint8_t DEBUG_MAX_SAMPLES = 16;

    // Permite enviar os quatro ganhos em uma unica linha pela Serial.
    constexpr uint8_t SERIAL_BUFFER_SIZE = 64;
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

    // RAW DOWN fixo usado diretamente no calculo de beta.
    uint16_t rawReferenceCounts;
    bool rawHistoryEnabled;
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

        // Medias durante todo o BALANCE: uteis para diagnosticar bias/offset.
        float sumPhiDeg;
        float sumBetaDeg;
        float sumControlDegS2;
    };

    // Cauda compactada do BALANCE. Guardamos apenas o necessario para
    // separar: (1) movimento do braco, (2) movimento do pendulo,
    // (3) velocidade estimada e (4) comando aplicado.
    struct DebugSample
    {
        int16_t phiCdeg;       // phi [deg] * 100
        int16_t betaMdeg;      // beta [deg] * 1000
        int16_t betaDot5e3;    // betaDot [rad/s] * 5000
        int16_t uDegS2;        // u aplicado [deg/s2]
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

    // Trace circular em RAM: 16 x 8 = 128 bytes.
    DebugSample debugSamples_[RevisionConfig::DEBUG_MAX_SAMPLES];
    uint8_t debugSampleCount_;
    uint8_t debugWriteIndex_;
    uint8_t debugDecimationCounter_;

    // Serial.
    char serialBuffer_[RevisionConfig::SERIAL_BUFFER_SIZE];
    uint8_t serialBufferIndex_;

    // Utilitarios angulares.
    static float wrapToPi(float angleRad);

    // Medicao e referencia.
    bool acquireState(uint32_t nowUs);
    float rawRelativeToDownAnchor() const;
    float betaFromRobustDownReference() const;
    void applyFixedDownReference();
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
    ControlSignal computeRuntimeControl() const;
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

    // Historico persistente do RAW da calibracao DOWN (EEPROM).
    void rawHistoryInit();
    void rawHistoryAppend(uint16_t raw);
    void printRawHistory();
    uint16_t rawHistoryReadValue(uint8_t logicalIndex) const;

    // Serial.
    void serviceSerial();
    void handleCommand(char *command);
    bool handleTuningCommand(char *command);
    bool setSingleRuntimeParameter(const char *name, float value);
    static bool parseOneFloat(const char *text, float &value);
    static bool parseFourFloats(const char *text, float &a, float &b, float &c, float &d);
    void printParameters();
    void printStatus();
    void printHelp();
    void printBalanceSummary(const char *reason);
};
