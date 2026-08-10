#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>

#include <FurutaConfig.h>
#include <MotorVelocity.h>
#include <PendulumPosition.h>
#include <PendulumVelocity.h>
#include <PendulumEvents.h>
#include <PendulumDownReference.h>
#include <StateVector.h>
#include <FourStateController.h>

#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <avr/pgmspace.h>

// ============================================================
// FASE 16 — CAPTURA AUTOMATICA
//
// A -> KICK -> SYNC -> SWING_UP -> PRECAPTURE
//   -> CAPTURE -> BALANCE
//
// KICK:
//   +A por T
//   -A por T
//
// Perfil triangular de velocidade. A velocidade do braco volta
// aproximadamente a zero, mas o braco TERMINA deslocado.
// Isso evita o cancelamento excessivo observado no perfil anterior
// +A / -2A / +A.
//
// O driver permanece HABILITADO. No fim do KICK usamos apenas
// motor.stop(), nunca motor.disable().
// ============================================================

namespace cfg16
{
    constexpr float PI_F = 3.14159265358979323846F;
    constexpr float TWO_PI_F = 6.28318530717958647692F;

    // Swing-up validado.
    constexpr float OMEGA0_RAD_S = 6.310F;
    constexpr int8_t ENERGY_SIGN = +1;
    constexpr float K_ENERGY_DEG_S2 = 1000.0F;
    constexpr float U_ENERGY_MAX_DEG_S2 = 300.0F;
    constexpr float PHASE_HYST = 0.030F;
    float energyReference = 1.10F;

    // PreCapture.
    float preCaptureEnergyReference = 1.03F;
    float preCaptureTaperStartEnergy = 0.980F;
    float preCaptureEntryDeg = 22.0F;
    float preCaptureEnergyMaxDegS2 = 95.0F;
    constexpr float PREENTRY_MIN_DEG = 10.0F;
    constexpr float PREENTRY_MAX_DEG = 40.0F;
    constexpr float UPRECAP_MIN_DEG_S2 = 40.0F;
    constexpr float UPRECAP_MAX_DEG_S2 = U_ENERGY_MAX_DEG_S2;

    // Contencao do braco durante swing/pre-capture.
    constexpr float K_PHI_SWING = 0.750F;
    constexpr float K_DPHI_SWING = 2.000F;
    constexpr float U_ARM_MAX_DEG_S2 = 150.0F;
    constexpr float ARM_CENTER_BAND_DEG = 3.0F;
    constexpr float ARM_SWING_ABORT_DEG = 60.0F;

    // PREARM:
    // 0 = comportamento antigo no PRECAPTURE
    // 1 = no PRECAPTURE, termo D do braco so atua quando
    //     |phi| > PREBAND E o braco esta se afastando.
    uint8_t preArmMode = 1;
    float preArmBandDeg = 3.0F;
    constexpr float PREARM_BAND_MIN_DEG = 3.0F;
    constexpr float PREARM_BAND_MAX_DEG = 15.0F;

    // --------------------------------------------------------
    // KICK AUTOMATICO
    // --------------------------------------------------------
    // Com os defaults:
    // A = 180 deg/s2, T = 180 ms
    // pico teorico de velocidade do braco ~= 32.4 deg/s
    // deslocamento final teorico ~= A*T^2 ~= 5.83 deg.
    // Ao final: phiDot ~= 0, mas phi != 0.
    float kickAccelDegS2 = 180.0F;
    uint16_t kickStageMs = 180;
    int8_t kickSign = +1;

    constexpr float KICKACC_MIN_DEG_S2 = 100.0F;
    constexpr float KICKACC_MAX_DEG_S2 = 200.0F;
    constexpr uint16_t KICKMS_MIN = 120;
    constexpr uint16_t KICKMS_MAX = 200;
    constexpr float KICK_ARM_ABORT_DEG = 15.0F;

    // SYNC: apos o KICK, motor parado e habilitado; esperamos
    // o primeiro PEAK real antes de ligar a lei de energia.
    constexpr uint32_t SYNC_TIMEOUT_US = 4000000UL;

    // Detector de pico ESPECIFICO do SYNC.
    //
    // IMPORTANTE: o armamento NAO exige mais angulo e velocidade
    // simultaneamente. Os ensaios mostraram picos claros de ~4.7 deg,
    // mas a velocidade ja estava proxima de zero quando o angulo
    // atingia o extremo.
    //
    // Nova logica:
    // 1) arma apenas por movimento claro: |alphaDot| >= 0.10 rad/s;
    // 2) acompanha o extremo no sentido do movimento;
    // 3) confirma quando a velocidade inverte >= 0.03 rad/s;
    // 4) aceita somente se o extremo medido tiver |alphaPeak| >= 1.5 deg.
    //
    // PendulumEvents permanece inalterado e volta a ser usado depois
    // do SYNC_LOCK, durante SWING_UP/PRECAPTURE.
    constexpr float SYNC_PEAK_ACCEPT_ANGLE_DEG = 1.50F;
    constexpr float SYNC_PEAK_ACCEPT_ANGLE_RAD =
        SYNC_PEAK_ACCEPT_ANGLE_DEG * DEG_TO_RAD;
    constexpr float SYNC_ARM_SPEED_RAD_S = 0.10F;
    constexpr float SYNC_REVERSE_SPEED_RAD_S = 0.03F;

    // Durante SYNC ainda nao ha controle energetico. Podemos imprimir
    // alpha/alphaDot lentamente para diagnosticar o primeiro pico sem
    // carregar a malha rapida.
    constexpr uint32_t SYNC_TELEMETRY_US = 100000UL;  // 10 Hz

    // Supervisao swing/pre-capture.
    constexpr uint8_t MAX_CYCLES = 60;
    constexpr uint8_t STALL_CYCLES = 3;
    constexpr uint32_t ENERGY_TIMEOUT_US = 80000000UL;
    constexpr uint32_t PRECAPTURE_TIMEOUT_US = 12000000UL;

    // --------------------------------------------------------
    // GATE DE CAPTURA
    // --------------------------------------------------------
    // CAPENTRY foi reduzido para a regiao realmente local.
    float captureEntryDeg = 1.50F;
    constexpr float CAPENTRY_MIN_DEG = 1.0F;
    constexpr float CAPENTRY_MAX_DEG = 2.0F;

    constexpr float CAPTURE_GATE_BETA_DOT_RAD_S = 0.25F;
    constexpr float CAPTURE_GATE_PHI_DEG = 5.0F;
    constexpr float CAPTURE_GATE_PHI_DOT_DEG_S = 45.0F;

    // Cruzamento direto do TOP: ainda mais estrito em beta.
    constexpr float DIRECT_CAPTURE_BETA_DEG = 1.0F;
    constexpr float TOP_CROSS_GUARD_DEG = 10.0F;

    // CAPTURE_LOCK — criterio do controlador vertical antigo.
    constexpr float CAPTURE_READY_BETA_DEG = 1.5F;
    constexpr float CAPTURE_READY_BETA_RAD =
        CAPTURE_READY_BETA_DEG * DEG_TO_RAD;
    constexpr float CAPTURE_READY_BETA_DOT_RAD_S = 0.25F;
    constexpr float CAPTURE_READY_PHI_DEG = 5.0F;
    constexpr uint32_t CAPTURE_READY_TIME_US = 80000UL;
    constexpr uint32_t CAPTURE_TIMEOUT_US = 2000000UL;

    // Balance validado.
    constexpr float K_PHI = 0.77419355F;
    constexpr float K_PHI_DOT = 2.31978594F;
    constexpr float K_BETA = 95.71078856F;
    constexpr float K_BETA_DOT = 14.31971458F;
    constexpr float BALANCE_MAX_ACCEL_RAD_S2 =
        FurutaConfig::MAX_ACCEL_DEG_S2 * DEG_TO_RAD;

    constexpr float BETA_ABORT_DEG = 8.0F;
    constexpr float BETA_ABORT_RAD = BETA_ABORT_DEG * DEG_TO_RAD;
    constexpr float ARM_LOCAL_ABORT_DEG = 75.0F;
    constexpr uint32_t BALANCE_TEST_TIME_US = 5000000UL;

    constexpr uint32_t MAX_CONTROL_DT_US = 20000UL;

    // 10 Hz para nao carregar o loop de 250 Hz.
    constexpr uint32_t LOCAL_TELEMETRY_US = 100000UL;

    constexpr uint8_t SERIAL_BUFFER_SIZE = 24;
}

// ============================================================
// HARDWARE / BIBLIOTECAS
// ============================================================

AS5600 as5600;

PendulumPosition pendulumPosition(as5600, 1.0F);
PendulumVelocity pendulumVelocity;
PendulumEvents pendulumEvents;
PendulumDownReference downReference(3000UL, 3.0F);

MotorVelocity motor(
    FurutaConfig::STEP_PIN,
    FurutaConfig::DIR_PIN,
    FurutaConfig::ENABLE_PIN,
    FurutaConfig::FULL_STEPS_PER_REVOLUTION,
    FurutaConfig::MICROSTEP_FACTOR,
    FurutaConfig::ARM_MIN_DEG,
    FurutaConfig::ARM_MAX_DEG,
    false
);

FourStateController balanceController(
    cfg16::K_PHI,
    cfg16::K_PHI_DOT,
    cfg16::K_BETA,
    cfg16::K_BETA_DOT,
    cfg16::BALANCE_MAX_ACCEL_RAD_S2
);

// ============================================================
// ESTADOS
// ============================================================

enum class Phase : uint8_t
{
    IDLE,
    CALIBRATING,
    READY,
    ARMED,
    KICK,
    SYNC,
    SWING_UP,
    PRECAPTURE,
    CAPTURE,
    BALANCE
};

Phase phase = Phase::IDLE;

enum class FinishReason : uint8_t
{
    NONE,
    CAPTURE_SUCCESS,
    SYNC_TIMEOUT,
    SWING_STALLED,
    PRECAPTURE_STALLED,
    ENERGY_TIMEOUT,
    TOP_CROSS_REJECT,
    CAPTURE_LOST,
    CAPTURE_TIMEOUT,
    BETA_LIMIT,
    ARM_LIMIT,
    CONTROL_OVERRUN,
    MOTOR_REJECTED,
    USER_STOP
};

bool armZeroDefined = false;

float alphaRad = 0.0F;
float alphaDotRadS = 0.0F;

StateVector state{0.0F, 0.0F, 0.0F, 0.0F};
ControlSignal localControl{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, false};

// Energia.
float energyHeld = 0.0F;
float phaseSignal = 0.0F;
int8_t phaseSignState = 0;
float uEnergyDegS2 = 0.0F;
float uArmDegS2 = 0.0F;
float uEnergyPhaseDegS2 = 0.0F;

// Picos/ciclos.
bool previousPositivePeakValid = false;
bool previousNegativePeakValid = false;
float previousPositivePeakEnergy = 0.0F;
float previousNegativePeakEnergy = 0.0F;
uint8_t fullCycleCount = 0;
uint8_t consecutiveNonPositiveCycles = 0;

// Cruzamento do TOP.
bool previousBetaValid = false;
float previousBetaRad = 0.0F;

// Timing.
uint32_t nextControlTimeUs = 0;
uint32_t lastControlTimeUs = 0;
uint32_t kickStartUs = 0;
uint32_t syncStartUs = 0;
uint32_t nextSyncTelemetryUs = 0;

// Detector pequeno exclusivo do SYNC:
//  0 = aguardando armamento
// +1 = pico positivo armado
// -1 = pico negativo armado
int8_t syncPeakArm = 0;
float syncPeakCandidateAlphaRad = 0.0F;

uint32_t energyStartUs = 0;
uint32_t preCaptureStartUs = 0;
uint32_t captureStartUs = 0;
uint32_t captureStableStartUs = 0;
uint32_t balanceStartUs = 0;
uint32_t nextLocalTelemetryUs = 0;

// Estatisticas essenciais.
struct Statistics
{
    float maxAbsAlphaDeg;
    float maxAbsPhiEnergyDeg;
    float maxAbsPhiDotEnergyDegS;
    float maxAbsUEnergy;
    float maxAbsUArm;

    float maxAbsLocalBetaDeg;
    float maxAbsLocalBetaDot;
    float maxAbsLocalPhiDeg;
    float maxAbsLocalPhiDotDegS;
    float maxAbsLocalU;
    uint32_t localSaturations;
    uint32_t localSamples;
};

Statistics stats{};

struct CaptureSnapshot
{
    bool valid;
    float betaDeg;
    float betaDotRadS;
    float phiDeg;
    float phiDotDegS;
    float predictedRawDegS2;
};

CaptureSnapshot captureSnapshot{};

char serialBuffer[cfg16::SERIAL_BUFFER_SIZE];
uint8_t serialIndex = 0;

// ============================================================
// AUXILIARES
// ============================================================

float clampFloat(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

float wrapToPi(float angle)
{
    while (angle > cfg16::PI_F) angle -= cfg16::TWO_PI_F;
    while (angle <= -cfg16::PI_F) angle += cfg16::TWO_PI_F;
    return angle;
}

float peakEnergyFromAlpha(float alpha)
{
    return 0.5F * (1.0F - cosf(alpha));
}

bool isEnergyPhase()
{
    return phase == Phase::SWING_UP || phase == Phase::PRECAPTURE;
}

bool isActivePhase()
{
    return phase == Phase::ARMED
        || phase == Phase::KICK
        || phase == Phase::SYNC
        || phase == Phase::SWING_UP
        || phase == Phase::PRECAPTURE
        || phase == Phase::CAPTURE
        || phase == Phase::BALANCE;
}

void printReason(FinishReason reason)
{
    switch (reason)
    {
        case FinishReason::CAPTURE_SUCCESS:    Serial.print(F("capture_success")); break;
        case FinishReason::SYNC_TIMEOUT:       Serial.print(F("sync_timeout")); break;
        case FinishReason::SWING_STALLED:      Serial.print(F("swing_stalled")); break;
        case FinishReason::PRECAPTURE_STALLED: Serial.print(F("precap_stalled")); break;
        case FinishReason::ENERGY_TIMEOUT:     Serial.print(F("energy_timeout")); break;
        case FinishReason::TOP_CROSS_REJECT:   Serial.print(F("top_cross_reject")); break;
        case FinishReason::CAPTURE_LOST:       Serial.print(F("capture_lost")); break;
        case FinishReason::CAPTURE_TIMEOUT:    Serial.print(F("capture_timeout")); break;
        case FinishReason::BETA_LIMIT:         Serial.print(F("beta_limit")); break;
        case FinishReason::ARM_LIMIT:          Serial.print(F("arm_limit")); break;
        case FinishReason::CONTROL_OVERRUN:    Serial.print(F("control_overrun")); break;
        case FinishReason::MOTOR_REJECTED:     Serial.print(F("motor_rejected")); break;
        case FinishReason::USER_STOP:          Serial.print(F("user_stop")); break;
        default:                               Serial.print(F("none")); break;
    }
}

// ============================================================
// AQUISICAO
// ============================================================

bool acquireState(uint32_t nowUs)
{
    if (!pendulumPosition.update()) return false;

    alphaRad = downReference.correctedAngleRad(
        pendulumPosition.betaRadians()
    );

    pendulumVelocity.update(alphaRad, nowUs);

    if (!pendulumVelocity.isReady()) return false;

    alphaDotRadS = pendulumVelocity.radiansPerSecond();

    state.beta = wrapToPi(alphaRad - cfg16::PI_F);
    state.betaDot = alphaDotRadS;
    state.phi = motor.currentPositionDegrees() * DEG_TO_RAD;
    state.phiDot = motor.speedReferenceDegreesPerSecond() * DEG_TO_RAD;

    return true;
}

// ============================================================
// RESET / CONFIG
// ============================================================

void resetRun()
{
    stats = {};
    captureSnapshot = {};

    energyHeld = 0.0F;
    phaseSignal = 0.0F;
    phaseSignState = 0;
    uEnergyDegS2 = 0.0F;
    uArmDegS2 = 0.0F;
    uEnergyPhaseDegS2 = 0.0F;

    previousPositivePeakValid = false;
    previousNegativePeakValid = false;
    previousPositivePeakEnergy = 0.0F;
    previousNegativePeakEnergy = 0.0F;
    fullCycleCount = 0;
    consecutiveNonPositiveCycles = 0;

    previousBetaValid = false;
    preCaptureStartUs = 0;
    captureStableStartUs = 0;

    syncPeakArm = 0;
    syncPeakCandidateAlphaRad = 0.0F;

    localControl = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, false};
    pendulumEvents.reset();
}

void printConfig()
{
    Serial.println();
    Serial.println(F("#CFG"));

    Serial.print(F("E="));
    Serial.print(cfg16::energyReference, 2);
    Serial.print(',');
    Serial.print(cfg16::preCaptureEnergyReference, 2);
    Serial.print(',');
    Serial.println(cfg16::preCaptureTaperStartEnergy, 3);

    Serial.print(F("P="));
    Serial.print(cfg16::preCaptureEntryDeg, 1);
    Serial.print(',');
    Serial.println(cfg16::preCaptureEnergyMaxDegS2, 0);

    Serial.print(F("A="));
    Serial.print(cfg16::preArmMode);
    Serial.print(',');
    Serial.println(cfg16::preArmBandDeg, 1);
}

// ============================================================
// ESTATISTICAS
// ============================================================

void updateEnergyStatistics()
{
    const float a = fabsf(alphaRad * RAD_TO_DEG);
    const float p = fabsf(motor.currentPositionDegrees());
    const float pd = fabsf(motor.speedReferenceDegreesPerSecond());

    if (a > stats.maxAbsAlphaDeg) stats.maxAbsAlphaDeg = a;
    if (p > stats.maxAbsPhiEnergyDeg) stats.maxAbsPhiEnergyDeg = p;
    if (pd > stats.maxAbsPhiDotEnergyDegS) stats.maxAbsPhiDotEnergyDegS = pd;
    if (fabsf(uEnergyDegS2) > stats.maxAbsUEnergy) stats.maxAbsUEnergy = fabsf(uEnergyDegS2);
    if (fabsf(uArmDegS2) > stats.maxAbsUArm) stats.maxAbsUArm = fabsf(uArmDegS2);
}

void updateLocalStatistics()
{
    const float b = fabsf(state.beta * RAD_TO_DEG);
    const float p = fabsf(state.phi * RAD_TO_DEG);
    const float pd = fabsf(state.phiDot * RAD_TO_DEG);
    const float u = fabsf(localControl.appliedRadS2 * RAD_TO_DEG);

    if (b > stats.maxAbsLocalBetaDeg) stats.maxAbsLocalBetaDeg = b;
    if (fabsf(state.betaDot) > stats.maxAbsLocalBetaDot) stats.maxAbsLocalBetaDot = fabsf(state.betaDot);
    if (p > stats.maxAbsLocalPhiDeg) stats.maxAbsLocalPhiDeg = p;
    if (pd > stats.maxAbsLocalPhiDotDegS) stats.maxAbsLocalPhiDotDegS = pd;
    if (u > stats.maxAbsLocalU) stats.maxAbsLocalU = u;
    if (localControl.saturated) ++stats.localSaturations;
    ++stats.localSamples;
}

// ============================================================
// RESULTADO
// ============================================================

void printResult(FinishReason reason)
{
    // Resumo compacto: preserva os dados de diagnostico essenciais,
    // mas reduz bastante as strings armazenadas em FLASH.
    Serial.println();
    Serial.print(F("#R="));
    printReason(reason);
    Serial.println();

    Serial.print(F("#cy="));
    Serial.println(fullCycleCount);

    Serial.print(F("#e="));
    Serial.print(stats.maxAbsAlphaDeg, 2);
    Serial.print(',');
    Serial.print(stats.maxAbsPhiEnergyDeg, 2);
    Serial.print(',');
    Serial.print(stats.maxAbsPhiDotEnergyDegS, 2);
    Serial.print(',');
    Serial.print(stats.maxAbsUEnergy, 1);
    Serial.print(',');
    Serial.println(stats.maxAbsUArm, 1);

    if (captureSnapshot.valid)
    {
        Serial.print(F("#c="));
        Serial.print(captureSnapshot.betaDeg, 3);
        Serial.print(',');
        Serial.print(captureSnapshot.betaDotRadS, 4);
        Serial.print(',');
        Serial.print(captureSnapshot.phiDeg, 2);
        Serial.print(',');
        Serial.print(captureSnapshot.phiDotDegS, 2);
        Serial.print(',');
        Serial.println(captureSnapshot.predictedRawDegS2, 1);
    }

    Serial.print(F("#l="));
    Serial.print(stats.maxAbsLocalBetaDeg, 3);
    Serial.print(',');
    Serial.print(stats.maxAbsLocalBetaDot, 4);
    Serial.print(',');
    Serial.print(stats.maxAbsLocalPhiDeg, 2);
    Serial.print(',');
    Serial.print(stats.maxAbsLocalPhiDotDegS, 2);
    Serial.print(',');
    Serial.print(stats.maxAbsLocalU, 1);
    Serial.print(',');
    Serial.print(stats.localSaturations);
    Serial.print(',');
    Serial.println(stats.localSamples);
}

void finishRun(FinishReason reason)
{
    if (!isActivePhase()) return;

    motor.stop();
    phase = Phase::READY;
    printResult(reason);
    pendulumEvents.reset();
}

// ============================================================
// KICK
// ============================================================

void startKick(uint32_t nowUs)
{
    motor.stop();
    phase = Phase::KICK;
    kickStartUs = nowUs;

    Serial.print(F("#KS,A="));
    Serial.print(cfg16::kickAccelDegS2, 1);
    Serial.print(F(",T="));
    Serial.print(cfg16::kickStageMs);
    Serial.print(F(",sign="));
    Serial.println(cfg16::kickSign);
}

void enterSync(uint32_t nowUs)
{
    // Importante: para a excitacao, mas NAO desabilita o A4988.
    motor.stop();
    phase = Phase::SYNC;
    syncStartUs = nowUs;
    nextSyncTelemetryUs = nowUs;

    syncPeakArm = 0;
    syncPeakCandidateAlphaRad = 0.0F;

    // O detector geral nao participa do SYNC. Ele sera reiniciado
    // quando o primeiro pico pequeno for aceito e o SWING_UP comecar.
    pendulumEvents.reset();

    Serial.print(F("#KE,p="));
    Serial.print(motor.currentPositionDegrees(), 2);
    Serial.print(F(",pd="));
    Serial.println(motor.speedReferenceDegreesPerSecond(), 2);
    Serial.println(F("#SW"));
}

void serviceKick(uint32_t nowUs, float dtSeconds, uint32_t dtUs)
{
    if (dtUs > cfg16::MAX_CONTROL_DT_US)
    {
        finishRun(FinishReason::CONTROL_OVERRUN);
        return;
    }

    if (fabsf(motor.currentPositionDegrees()) >= cfg16::KICK_ARM_ABORT_DEG)
    {
        finishRun(FinishReason::ARM_LIMIT);
        return;
    }

    const uint32_t stageUs = static_cast<uint32_t>(cfg16::kickStageMs) * 1000UL;
    const uint32_t elapsedUs = nowUs - kickStartUs;

    float acceleration = 0.0F;

    if (elapsedUs < stageUs)
    {
        // Acelera o braco.
        acceleration =
            static_cast<float>(cfg16::kickSign)
            * cfg16::kickAccelDegS2;
    }
    else if (elapsedUs < 2UL * stageUs)
    {
        // Desacelera ate aproximadamente phiDot = 0, mas NAO
        // devolve o braco a phi = 0.
        acceleration =
            -static_cast<float>(cfg16::kickSign)
            * cfg16::kickAccelDegS2;
    }
    else
    {
        enterSync(nowUs);
        return;
    }

    if (!motor.commandAcceleration(acceleration, dtSeconds))
    {
        finishRun(FinishReason::MOTOR_REJECTED);
        return;
    }

}

// ============================================================
// SYNC: PRIMEIRO PEAK PEQUENO
// ============================================================
//
// O detector geral PendulumEvents foi projetado para os picos maiores
// do SWING_UP. No SYNC a amplitude produzida pelo KICK e de apenas
// cerca de 5 graus. Por isso usamos aqui um detector local, simples:
//
// 1) arma SOMENTE pelo sentido/velocidade: |alphaDot| >= 0.10 rad/s;
// 2) acompanha o extremo angular enquanto o pico se forma;
// 3) confirma a reversao com pelo menos 0.03 rad/s no sentido oposto;
// 4) aceita o pico se |alphaPeak| >= 1.5 deg.
//
// Nao ha mais exigencia simultanea de angulo + velocidade para armar.
//
// Depois do SYNC_LOCK, PendulumEvents e reiniciado e volta a ser o
// unico detector de picos durante SWING_UP/PRECAPTURE.
// ============================================================

void startSwingFromSyncPeak(
    int8_t peakSign,
    float peakAlphaRad,
    uint32_t nowUs
)
{
    const bool positive = peakSign > 0;
    const float ePeak = peakEnergyFromAlpha(peakAlphaRad);

    energyHeld = ePeak;

    if (positive)
    {
        previousPositivePeakValid = true;
        previousPositivePeakEnergy = ePeak;
    }
    else
    {
        previousNegativePeakValid = true;
        previousNegativePeakEnergy = ePeak;
    }

    phaseSignState = 0;
    phaseSignal = 0.0F;
    energyStartUs = nowUs;

    previousBetaRad = state.beta;
    previousBetaValid = true;

    // O SWING_UP volta a usar o detector geral ja validado.
    pendulumEvents.reset();

    syncPeakArm = 0;
    syncPeakCandidateAlphaRad = 0.0F;

    phase = Phase::SWING_UP;

    Serial.print(F("#SL,"));
    Serial.print(positive ? '+' : '-');
    Serial.print(F(",a="));
    Serial.print(peakAlphaRad * RAD_TO_DEG, 2);
    Serial.print(F(",eH="));
    Serial.print(ePeak, 6);
    Serial.print(F(",adNow="));
    Serial.print(alphaDotRadS, 4);
    Serial.print(F(",phi="));
    Serial.println(motor.currentPositionDegrees(), 2);
    Serial.println(F("#SS"));
}

void serviceSync(uint32_t nowUs)
{
    if ((nowUs - syncStartUs) >= cfg16::SYNC_TIMEOUT_US)
    {
        finishRun(FinishReason::SYNC_TIMEOUT);
        return;
    }

    // O braco deve permanecer parado/segurado durante SYNC.
    if (fabsf(motor.currentPositionDegrees()) >= cfg16::KICK_ARM_ABORT_DEG)
    {
        finishRun(FinishReason::ARM_LIMIT);
        return;
    }

    // Mede explicitamente quanto o KICK transferiu ao pendulo.
    // Telemetria lenta somente no SYNC.
    if (static_cast<int32_t>(nowUs - nextSyncTelemetryUs) >= 0)
    {
        nextSyncTelemetryUs += cfg16::SYNC_TELEMETRY_US;

        Serial.print(F("# SYNC,a="));
        Serial.print(alphaRad * RAD_TO_DEG, 2);
        Serial.print(F(",ad="));
        Serial.print(alphaDotRadS, 4);
        Serial.print(F(",phi="));
        Serial.println(motor.currentPositionDegrees(), 2);
    }

    // --------------------------------------------------------
    // 1) ARMAMENTO: somente por movimento claro.
    //
    // Nao exigimos mais um angulo minimo neste instante. Isso evita
    // perder o pico quando alphaDot e alto antes do extremo, mas ja
    // caiu para ~0 quando |alpha| atinge 4--5 graus.
    // --------------------------------------------------------
    if (syncPeakArm == 0)
    {
        if (alphaDotRadS >= cfg16::SYNC_ARM_SPEED_RAD_S)
        {
            syncPeakArm = +1;
            syncPeakCandidateAlphaRad = alphaRad;

            Serial.print(F("#SA+,a="));
            Serial.print(alphaRad * RAD_TO_DEG, 2);
            Serial.print(F(",ad="));
            Serial.println(alphaDotRadS, 4);
        }
        else if (alphaDotRadS <= -cfg16::SYNC_ARM_SPEED_RAD_S)
        {
            syncPeakArm = -1;
            syncPeakCandidateAlphaRad = alphaRad;

            Serial.print(F("#SA-,a="));
            Serial.print(alphaRad * RAD_TO_DEG, 2);
            Serial.print(F(",ad="));
            Serial.println(alphaDotRadS, 4);
        }

        return;
    }

    // --------------------------------------------------------
    // 2) Pico positivo armado: acompanha o maior alpha.
    // --------------------------------------------------------
    if (syncPeakArm > 0)
    {
        if (alphaRad > syncPeakCandidateAlphaRad)
        {
            syncPeakCandidateAlphaRad = alphaRad;
        }

        if (alphaDotRadS <= -cfg16::SYNC_REVERSE_SPEED_RAD_S)
        {
            if (fabsf(syncPeakCandidateAlphaRad) >=
                cfg16::SYNC_PEAK_ACCEPT_ANGLE_RAD)
            {
                startSwingFromSyncPeak(
                    +1,
                    syncPeakCandidateAlphaRad,
                    nowUs
                );
                return;
            }

            // Reversao verdadeira, mas pico pequeno demais.
            // Descarta e permite armar o semiciclo seguinte.
            Serial.print(F("#SR+,a="));
            Serial.println(syncPeakCandidateAlphaRad * RAD_TO_DEG, 2);
            syncPeakArm = 0;
            syncPeakCandidateAlphaRad = 0.0F;
        }

        return;
    }

    // --------------------------------------------------------
    // 3) Pico negativo armado: acompanha o menor alpha.
    // --------------------------------------------------------
    if (alphaRad < syncPeakCandidateAlphaRad)
    {
        syncPeakCandidateAlphaRad = alphaRad;
    }

    if (alphaDotRadS >= cfg16::SYNC_REVERSE_SPEED_RAD_S)
    {
        if (fabsf(syncPeakCandidateAlphaRad) >=
            cfg16::SYNC_PEAK_ACCEPT_ANGLE_RAD)
        {
            startSwingFromSyncPeak(
                -1,
                syncPeakCandidateAlphaRad,
                nowUs
            );
            return;
        }

        Serial.print(F("#SR-,a="));
        Serial.println(syncPeakCandidateAlphaRad * RAD_TO_DEG, 2);
        syncPeakArm = 0;
        syncPeakCandidateAlphaRad = 0.0F;
    }

}

// ============================================================
// SWING / PRECAPTURE
// ============================================================

void updatePhaseSign()
{
    if (phaseSignal > cfg16::PHASE_HYST) phaseSignState = +1;
    else if (phaseSignal < -cfg16::PHASE_HYST) phaseSignState = -1;
}

float computeSwingArmControl(float phiDeg, float phiDotDegS)
{
    const bool movingAway = (phiDeg * phiDotDegS) > 0.0F;
    const bool nearCenter = fabsf(phiDeg) <= cfg16::ARM_CENTER_BAND_DEG;

    float command = -cfg16::K_PHI_SWING * phiDeg;

    if (phase == Phase::PRECAPTURE && cfg16::preArmMode == 1)
    {
        const bool nearPreCenter = fabsf(phiDeg) <= cfg16::preArmBandDeg;

        // PRECAPTURE experimental:
        // dentro de PREBAND nao usa D; fora dela, usa D somente
        // quando o braco esta se afastando do zero.
        if (movingAway && !nearPreCenter)
        {
            command -= cfg16::K_DPHI_SWING * phiDotDegS;
        }
    }
    else
    {
        // SWING_UP continua usando a banda original de 3 graus.
        // PREARM=0 reproduz o comportamento antigo.
        if (movingAway || nearCenter)
        {
            command -= cfg16::K_DPHI_SWING * phiDotDegS;
        }
    }

    return clampFloat(
        command,
        -cfg16::U_ARM_MAX_DEG_S2,
        +cfg16::U_ARM_MAX_DEG_S2
    );
}

void enterPreCapture(float peakBetaDeg, float ePeak, uint32_t nowUs)
{
    phase = Phase::PRECAPTURE;
    preCaptureStartUs = nowUs;
    consecutiveNonPositiveCycles = 0;

    Serial.print(F("#PC,b="));
    Serial.print(peakBetaDeg, 2);
    Serial.print(F(",eH="));
    Serial.print(ePeak, 6);
    Serial.print(F(",ad="));
    Serial.print(alphaDotRadS, 4);
    Serial.print(F(",phi="));
    Serial.println(motor.currentPositionDegrees(), 2);
}

float predictedRawControlDegS2(const StateVector &candidate)
{
    return balanceController.compute(candidate).rawRadS2 * RAD_TO_DEG;
}

bool captureGateOK(
    float betaDeg,
    float betaDotRadS,
    float phiDeg,
    float phiDotDegS
)
{
    return
        fabsf(betaDeg) <= cfg16::captureEntryDeg
        && fabsf(betaDotRadS) <= cfg16::CAPTURE_GATE_BETA_DOT_RAD_S
        && fabsf(phiDeg) <= cfg16::CAPTURE_GATE_PHI_DEG
        && fabsf(phiDotDegS) <= cfg16::CAPTURE_GATE_PHI_DOT_DEG_S;
}

void enterCapture(
    char source,
    uint32_t nowUs,
    float predictedRawDegS2
)
{
    phase = Phase::CAPTURE;
    captureStartUs = nowUs;
    captureStableStartUs = 0;
    nextLocalTelemetryUs = nowUs;

    captureSnapshot.valid = true;
    captureSnapshot.betaDeg = state.beta * RAD_TO_DEG;
    captureSnapshot.betaDotRadS = state.betaDot;
    captureSnapshot.phiDeg = state.phi * RAD_TO_DEG;
    captureSnapshot.phiDotDegS = state.phiDot * RAD_TO_DEG;
    captureSnapshot.predictedRawDegS2 = predictedRawDegS2;

    Serial.print(F("#CE,"));
    Serial.print(source);
    Serial.print(F(",b="));
    Serial.print(captureSnapshot.betaDeg, 3);
    Serial.print(F(",bd="));
    Serial.print(captureSnapshot.betaDotRadS, 4);
    Serial.print(F(",p="));
    Serial.print(captureSnapshot.phiDeg, 2);
    Serial.print(F(",pd="));
    Serial.print(captureSnapshot.phiDotDegS, 2);
    Serial.print(F(",uPred="));
    Serial.println(predictedRawDegS2, 1);
}

void printCaptureGateReject(
    char source,
    float betaDeg,
    float betaDotRadS,
    float phiDeg,
    float phiDotDegS,
    float uPredDegS2
)
{
    Serial.print(F("#CR,"));
    Serial.print(source);
    Serial.print(F(",b="));
    Serial.print(betaDeg, 3);
    Serial.print(F(",bd="));
    Serial.print(betaDotRadS, 4);
    Serial.print(F(",p="));
    Serial.print(phiDeg, 2);
    Serial.print(F(",pd="));
    Serial.print(phiDotDegS, 2);
    Serial.print(F(",uPred="));
    Serial.println(uPredDegS2, 1);
}

void handleEnergyPeak(const PendulumEvent &event, uint32_t nowUs)
{
    const bool positive = event.type == PendulumEventType::PEAK_POSITIVE;
    const float peakDeg = event.alphaRad * RAD_TO_DEG;
    const float ePeak = peakEnergyFromAlpha(event.alphaRad);
    const float peakBetaRad = wrapToPi(event.alphaRad - cfg16::PI_F);
    const float peakBetaDeg = peakBetaRad * RAD_TO_DEG;
    const float topDistanceDeg = fabsf(peakBetaDeg);

    float deltaCycle = 0.0F;
    bool deltaCycleValid = false;

    if (positive)
    {
        if (previousPositivePeakValid)
        {
            deltaCycle = ePeak - previousPositivePeakEnergy;
            deltaCycleValid = true;
        }
        previousPositivePeakValid = true;
        previousPositivePeakEnergy = ePeak;
    }
    else
    {
        if (previousNegativePeakValid)
        {
            deltaCycle = ePeak - previousNegativePeakEnergy;
            deltaCycleValid = true;
            ++fullCycleCount;

            if (deltaCycle > 0.0F) consecutiveNonPositiveCycles = 0;
            else ++consecutiveNonPositiveCycles;
        }
        previousNegativePeakValid = true;
        previousNegativePeakEnergy = ePeak;
    }

    Serial.print(F("# P,"));
    Serial.print(positive ? '+' : '-');
    Serial.print(F(",m="));
    Serial.print(phase == Phase::PRECAPTURE ? 'P' : 'S');
    Serial.print(F(",a="));
    Serial.print(peakDeg, 2);
    Serial.print(F(",beta="));
    Serial.print(peakBetaDeg, 2);
    Serial.print(F(",e="));
    Serial.print(ePeak, 6);

    if (deltaCycleValid)
    {
        Serial.print(F(",dC="));
        Serial.print(deltaCycle, 6);
    }

    Serial.print(F(",UE="));
    Serial.print(uEnergyDegS2, 1);
    Serial.print(F(",UA="));
    Serial.print(uArmDegS2, 1);
    Serial.print(F(",phi="));
    Serial.print(motor.currentPositionDegrees(), 2);
    Serial.print(F(",pd="));
    Serial.println(motor.speedReferenceDegreesPerSecond(), 2);

    energyHeld = ePeak;

    // --------------------------------------------------------
    // Gate por PEAK. O evento marca betaDot geometrico = 0,
    // mas exigimos tambem que a estimativa atual ja esteja
    // lenta, para evitar entrega atrasada do detector.
    // --------------------------------------------------------
    if (topDistanceDeg <= cfg16::captureEntryDeg)
    {
        StateVector candidate = state;
        candidate.beta = peakBetaRad;
        candidate.betaDot = 0.0F;

        const float phiDeg = state.phi * RAD_TO_DEG;
        const float phiDotDegS = state.phiDot * RAD_TO_DEG;
        const float uPred = predictedRawControlDegS2(candidate);

        if (captureGateOK(
                peakBetaDeg,
                state.betaDot,
                phiDeg,
                phiDotDegS
            ))
        {
            enterCapture('P', nowUs, uPred);
            return;
        }

        printCaptureGateReject(
            'P',
            peakBetaDeg,
            state.betaDot,
            phiDeg,
            phiDotDegS,
            uPred
        );
    }

    if (
        phase == Phase::SWING_UP
        && topDistanceDeg <= cfg16::preCaptureEntryDeg
    )
    {
        enterPreCapture(peakBetaDeg, ePeak, nowUs);
        return;
    }

    // "Energia nao crescente por 3 ciclos" continua sendo criterio
    // de stall somente no SWING_UP. Em PRECAPTURE a energia pode cair
    // deliberadamente; portanto esse criterio seria incorreto.
    if (
        phase == Phase::SWING_UP
        && consecutiveNonPositiveCycles >= cfg16::STALL_CYCLES
    )
    {
        finishRun(FinishReason::SWING_STALLED);
        return;
    }

    if (fullCycleCount >= cfg16::MAX_CYCLES)
    {
        finishRun(
            phase == Phase::PRECAPTURE
                ? FinishReason::PRECAPTURE_STALLED
                : FinishReason::SWING_STALLED
        );
    }
}

void checkTopCross(uint32_t nowUs)
{
    const float guardRad = cfg16::TOP_CROSS_GUARD_DEG * DEG_TO_RAD;

    if (previousBetaValid)
    {
        const bool crossed =
            (previousBetaRad < 0.0F && state.beta >= 0.0F)
            || (previousBetaRad > 0.0F && state.beta <= 0.0F);

        const bool nearTop =
            fabsf(previousBetaRad) <= guardRad
            && fabsf(state.beta) <= guardRad;

        if (crossed && nearTop)
        {
            const float betaDeg = state.beta * RAD_TO_DEG;
            const float phiDeg = state.phi * RAD_TO_DEG;
            const float phiDotDegS = state.phiDot * RAD_TO_DEG;
            const float uPred = predictedRawControlDegS2(state);

            const bool betaDirectOK =
                fabsf(betaDeg) <= cfg16::DIRECT_CAPTURE_BETA_DEG;

            if (
                betaDirectOK
                && captureGateOK(
                    betaDeg,
                    state.betaDot,
                    phiDeg,
                    phiDotDegS
                )
            )
            {
                enterCapture('C', nowUs, uPred);
                return;
            }

            printCaptureGateReject(
                'C',
                betaDeg,
                state.betaDot,
                phiDeg,
                phiDotDegS,
                uPred
            );

            finishRun(FinishReason::TOP_CROSS_REJECT);
            return;
        }
    }

    previousBetaRad = state.beta;
    previousBetaValid = true;
}

void serviceEnergyPhase(uint32_t nowUs, float dtSeconds, uint32_t dtUs)
{
    if (dtUs > cfg16::MAX_CONTROL_DT_US)
    {
        finishRun(FinishReason::CONTROL_OVERRUN);
        return;
    }

    if ((nowUs - energyStartUs) >= cfg16::ENERGY_TIMEOUT_US)
    {
        finishRun(FinishReason::ENERGY_TIMEOUT);
        return;
    }

    if (
        phase == Phase::PRECAPTURE
        && (nowUs - preCaptureStartUs) >= cfg16::PRECAPTURE_TIMEOUT_US
    )
    {
        finishRun(FinishReason::PRECAPTURE_STALLED);
        return;
    }

    const float phiDeg = motor.currentPositionDegrees();
    const float phiDotDegS = motor.speedReferenceDegreesPerSecond();

    if (fabsf(phiDeg) >= cfg16::ARM_SWING_ABORT_DEG)
    {
        finishRun(FinishReason::ARM_LIMIT);
        return;
    }

    const PendulumEvent event = pendulumEvents.update(
        alphaRad,
        alphaDotRadS,
        nowUs
    );

    if (
        event.type == PendulumEventType::PEAK_POSITIVE
        || event.type == PendulumEventType::PEAK_NEGATIVE
    )
    {
        handleEnergyPeak(event, nowUs);
        if (!isEnergyPhase()) return;
    }

    checkTopCross(nowUs);
    if (!isEnergyPhase()) return;

    float targetEnergy = cfg16::energyReference;

    if (phase == Phase::PRECAPTURE)
    {
        const float e0 = cfg16::preCaptureTaperStartEnergy;

        if (energyHeld <= e0)
        {
            targetEnergy = cfg16::preCaptureEnergyReference;
        }
        else if (energyHeld >= 1.0F)
        {
            targetEnergy = 1.0F;
        }
        else
        {
            const float r = (1.0F - energyHeld) / (1.0F - e0);
            targetEnergy =
                1.0F
                + (cfg16::preCaptureEnergyReference - 1.0F) * r;
        }
    }

    float energyError = targetEnergy - energyHeld;
    if (energyError < 0.0F) energyError = 0.0F;

    phaseSignal =
        (alphaDotRadS / cfg16::OMEGA0_RAD_S)
        * cosf(alphaRad);

    updatePhaseSign();

    float uEnergyRaw = 0.0F;

    if (phaseSignState != 0)
    {
        uEnergyRaw =
            static_cast<float>(cfg16::ENERGY_SIGN)
            * cfg16::K_ENERGY_DEG_S2
            * energyError
            * static_cast<float>(phaseSignState);
    }

    const float energyLimit =
        phase == Phase::PRECAPTURE
            ? cfg16::preCaptureEnergyMaxDegS2
            : cfg16::U_ENERGY_MAX_DEG_S2;

    uEnergyDegS2 = clampFloat(
        uEnergyRaw,
        -energyLimit,
        +energyLimit
    );

    uArmDegS2 = computeSwingArmControl(phiDeg, phiDotDegS);

    uEnergyPhaseDegS2 = clampFloat(
        uEnergyDegS2 + uArmDegS2,
        -FurutaConfig::MAX_ACCEL_DEG_S2,
        +FurutaConfig::MAX_ACCEL_DEG_S2
    );

    if (!motor.commandAcceleration(uEnergyPhaseDegS2, dtSeconds))
    {
        finishRun(FinishReason::MOTOR_REJECTED);
        return;
    }

    updateEnergyStatistics();
}

// ============================================================
// CAPTURE / BALANCE
// ============================================================

void printLocalTelemetry(uint32_t nowUs)
{
    if (static_cast<int32_t>(nowUs - nextLocalTelemetryUs) < 0) return;

    nextLocalTelemetryUs += cfg16::LOCAL_TELEMETRY_US;

    const uint32_t referenceUs =
        phase == Phase::BALANCE ? balanceStartUs : captureStartUs;

    Serial.print(F("# L,"));
    Serial.print(phase == Phase::BALANCE ? 'B' : 'C');
    Serial.print(F(",t="));
    Serial.print((nowUs - referenceUs) / 1000UL);
    Serial.print(F(",b="));
    Serial.print(state.beta * RAD_TO_DEG, 3);
    Serial.print(F(",bd="));
    Serial.print(state.betaDot, 4);
    Serial.print(F(",p="));
    Serial.print(state.phi * RAD_TO_DEG, 2);
    Serial.print(F(",pd="));
    Serial.print(state.phiDot * RAD_TO_DEG, 2);
    Serial.print(F(",u="));
    Serial.print(localControl.appliedRadS2 * RAD_TO_DEG, 1);
    Serial.print(F(",sat="));
    Serial.println(localControl.saturated ? 1 : 0);
}

void serviceLocalControl(uint32_t nowUs, float dtSeconds, uint32_t dtUs)
{
    if (dtUs > cfg16::MAX_CONTROL_DT_US)
    {
        finishRun(FinishReason::CONTROL_OVERRUN);
        return;
    }

    if (fabsf(state.beta) >= cfg16::BETA_ABORT_RAD)
    {
        finishRun(
            phase == Phase::CAPTURE
                ? FinishReason::CAPTURE_LOST
                : FinishReason::BETA_LIMIT
        );
        return;
    }

    if (fabsf(state.phi * RAD_TO_DEG) >= cfg16::ARM_LOCAL_ABORT_DEG)
    {
        finishRun(FinishReason::ARM_LIMIT);
        return;
    }

    localControl = balanceController.compute(state);

    const float commandDegS2 = localControl.appliedRadS2 * RAD_TO_DEG;

    if (!motor.commandAcceleration(commandDegS2, dtSeconds))
    {
        finishRun(FinishReason::MOTOR_REJECTED);
        return;
    }

    updateLocalStatistics();
    printLocalTelemetry(nowUs);

    if (phase == Phase::CAPTURE)
    {
        if ((nowUs - captureStartUs) >= cfg16::CAPTURE_TIMEOUT_US)
        {
            finishRun(FinishReason::CAPTURE_TIMEOUT);
            return;
        }

        const bool captureReady =
            fabsf(state.beta) <= cfg16::CAPTURE_READY_BETA_RAD
            && fabsf(state.betaDot) <= cfg16::CAPTURE_READY_BETA_DOT_RAD_S
            && fabsf(state.phi * RAD_TO_DEG) <= cfg16::CAPTURE_READY_PHI_DEG;

        if (captureReady)
        {
            if (captureStableStartUs == 0) captureStableStartUs = nowUs;

            if ((nowUs - captureStableStartUs) >= cfg16::CAPTURE_READY_TIME_US)
            {
                phase = Phase::BALANCE;
                balanceStartUs = nowUs;
                nextLocalTelemetryUs = nowUs;

                Serial.print(F("#CL,b="));
                Serial.print(state.beta * RAD_TO_DEG, 3);
                Serial.print(F(",bd="));
                Serial.print(state.betaDot, 4);
                Serial.print(F(",p="));
                Serial.println(state.phi * RAD_TO_DEG, 2);
                return;
            }
        }
        else
        {
            captureStableStartUs = 0;
        }

        return;
    }

    if (
        phase == Phase::BALANCE
        && (nowUs - balanceStartUs) >= cfg16::BALANCE_TEST_TIME_US
    )
    {
        finishRun(FinishReason::CAPTURE_SUCCESS);
    }
}

// ============================================================
// CALIBRACAO / ARMA
// ============================================================

void startCalibration()
{
    if (isActivePhase())
    {
        Serial.println(F("BUSY"));
        return;
    }

    motor.stop();

    if (!pendulumPosition.calibrateTop(32, 2))
    {
        Serial.println(F("AS5600_ERR"));
        return;
    }

    const uint32_t nowUs = micros();

    pendulumVelocity.reset(
        pendulumPosition.betaRadians(),
        nowUs
    );

    downReference.start(nowUs);
    phase = Phase::CALIBRATING;

    lastControlTimeUs = nowUs;
    nextControlTimeUs = nowUs + FurutaConfig::CONTROL_PERIOD_US;

    Serial.println(F("CAL_DOWN"));
}

void armAutomaticCapture()
{
    if (!downReference.isReady())
    {
        Serial.println(F("NEED_T"));
        return;
    }

    if (!armZeroDefined)
    {
        Serial.println(F("NEED_Z"));
        return;
    }

    if (!motor.isEnabled())
    {
        Serial.println(F("NEED_E"));
        return;
    }

    if (phase != Phase::READY && phase != Phase::IDLE)
    {
        Serial.println(F("BUSY"));
        return;
    }

    if (fabsf(motor.currentPositionDegrees()) > 1.0F)
    {
        Serial.println(F("PHI_NOT_ZERO"));
        return;
    }

    resetRun();
    motor.stop();
    phase = Phase::ARMED;

    Serial.println(F("#ARM"));
    printConfig();
}

void emergencyStop()
{
    motor.emergencyStop();
    phase = Phase::IDLE;
    armZeroDefined = false;
    pendulumEvents.reset();
    Serial.println(F("ESTOP"));
}

// ============================================================
// SET / SERIAL
// ============================================================

bool setParameter(const char *name, const char *valueText)
{
    const float value = atof(valueText);

    if (strcmp_P(name, PSTR("EREF")) == 0)
    {
        if (value < 0.90F || value > 1.20F) { Serial.println(F("REJECT")); return false; }
        cfg16::energyReference = value;
        Serial.println(F("OK"));
        return true;
    }

    if (strcmp_P(name, PSTR("EPRE")) == 0)
    {
        if (value < 1.00F || value > 1.10F) { Serial.println(F("REJECT")); return false; }
        cfg16::preCaptureEnergyReference = value;
        Serial.println(F("OK"));
        return true;
    }

    if (strcmp_P(name, PSTR("ETAPER")) == 0)
    {
        if (value < 0.950F || value > 0.995F)
        {
            Serial.println(F("REJECT"));
            return false;
        }
        cfg16::preCaptureTaperStartEnergy = value;
        Serial.println(F("OK"));
        return true;
    }

    if (strcmp_P(name, PSTR("PREARM")) == 0)
    {
        if (value >= -0.1F && value < 0.5F) cfg16::preArmMode = 0;
        else if (value >= 0.5F && value <= 1.1F) cfg16::preArmMode = 1;
        else { Serial.println(F("REJECT")); return false; }
        Serial.println(F("OK"));
        return true;
    }

    if (strcmp_P(name, PSTR("PREBAND")) == 0)
    {
        if (value < cfg16::PREARM_BAND_MIN_DEG || value > cfg16::PREARM_BAND_MAX_DEG)
        {
            Serial.println(F("REJECT"));
            return false;
        }
        cfg16::preArmBandDeg = value;
        Serial.println(F("OK"));
        return true;
    }

    if (strcmp_P(name, PSTR("PREENTRY")) == 0)
    {
        if (
            value < cfg16::PREENTRY_MIN_DEG
            || value > cfg16::PREENTRY_MAX_DEG
            || value <= cfg16::captureEntryDeg
        )
        {
            Serial.println(F("REJECT"));
            return false;
        }
        cfg16::preCaptureEntryDeg = value;
        Serial.println(F("OK"));
        return true;
    }

    if (strcmp_P(name, PSTR("UPRECAP")) == 0)
    {
        if (value < cfg16::UPRECAP_MIN_DEG_S2 || value > cfg16::UPRECAP_MAX_DEG_S2)
        {
            Serial.println(F("REJECT"));
            return false;
        }
        cfg16::preCaptureEnergyMaxDegS2 = value;
        Serial.println(F("OK"));
        return true;
    }

    if (strcmp_P(name, PSTR("CAPENTRY")) == 0)
    {
        if (
            value < cfg16::CAPENTRY_MIN_DEG
            || value > cfg16::CAPENTRY_MAX_DEG
            || value >= cfg16::preCaptureEntryDeg
        )
        {
            Serial.println(F("REJECT"));
            return false;
        }
        cfg16::captureEntryDeg = value;
        Serial.println(F("OK"));
        return true;
    }

    if (strcmp_P(name, PSTR("KICKACC")) == 0)
    {
        if (value < cfg16::KICKACC_MIN_DEG_S2 || value > cfg16::KICKACC_MAX_DEG_S2)
        {
            Serial.println(F("REJECT"));
            return false;
        }
        cfg16::kickAccelDegS2 = value;
        Serial.println(F("OK"));
        return true;
    }

    if (strcmp_P(name, PSTR("KICKMS")) == 0)
    {
        if (value < cfg16::KICKMS_MIN || value > cfg16::KICKMS_MAX)
        {
            Serial.println(F("REJECT"));
            return false;
        }
        cfg16::kickStageMs = static_cast<uint16_t>(lroundf(value));
        Serial.println(F("OK"));
        return true;
    }

    if (strcmp_P(name, PSTR("KICKSIGN")) == 0)
    {
        if (value > 0.5F) cfg16::kickSign = +1;
        else if (value < -0.5F) cfg16::kickSign = -1;
        else { Serial.println(F("REJECT")); return false; }
        Serial.println(F("OK"));
        return true;
    }

    Serial.println(F("BAD_PARAM"));
    return false;
}

void executeSerialLine(char *line)
{
    while (*line == ' ' || *line == '\t') ++line;
    if (*line == '\0') return;

    for (char *p = line; *p; ++p)
    {
        *p = static_cast<char>(toupper(static_cast<unsigned char>(*p)));
    }

    if (isActivePhase())
    {
        if (strcmp_P(line, PSTR("STOP")) == 0)
        {
            finishRun(FinishReason::USER_STOP);
            return;
        }
        if (strcmp_P(line, PSTR("X")) == 0)
        {
            emergencyStop();
            return;
        }
        return;
    }

    if (strcmp_P(line, PSTR("X")) == 0)
    {
        emergencyStop();
        return;
    }

    if (strcmp_P(line, PSTR("D")) == 0)
    {
        motor.stop();
        motor.disable();
        phase = Phase::IDLE;
        armZeroDefined = false;
        pendulumEvents.reset();
        Serial.println(F("DISABLED"));
        return;
    }

    if (strcmp_P(line, PSTR("Z")) == 0)
    {
        if (motor.isEnabled())
        {
            Serial.println(F("DISABLE_FIRST"));
            return;
        }

        motor.setCurrentPosition(0.0F);
        armZeroDefined = true;
        Serial.println(F("PHI0"));
        return;
    }

    if (strcmp_P(line, PSTR("E")) == 0)
    {
        motor.enable();
        Serial.println(F("ENABLED"));
        return;
    }

    if (strcmp_P(line, PSTR("T")) == 0)
    {
        startCalibration();
        return;
    }

    if (strcmp_P(line, PSTR("A")) == 0)
    {
        armAutomaticCapture();
        return;
    }

    if (strcmp_P(line, PSTR("STOP")) == 0)
    {
        motor.stop();
        phase = downReference.isReady() ? Phase::READY : Phase::IDLE;
        Serial.println(F("STOPPED"));
        return;
    }

    if (strcmp_P(line, PSTR("CFG")) == 0)
    {
        printConfig();
        return;
    }

    char *savePtr = nullptr;
    char *command = strtok_r(line, " ", &savePtr);

    if (command != nullptr && strcmp_P(command, PSTR("SET")) == 0)
    {
        char *parameter = strtok_r(nullptr, " ", &savePtr);
        char *value = strtok_r(nullptr, " ", &savePtr);

        if (parameter != nullptr && value != nullptr)
        {
            setParameter(parameter, value);
        }
        return;
    }

    Serial.println(F("BAD_CMD"));
}

void processSerial()
{
    while (Serial.available())
    {
        const char c = static_cast<char>(Serial.read());
        if (c == '\r') continue;

        if (c == '\n')
        {
            serialBuffer[serialIndex] = '\0';
            executeSerialLine(serialBuffer);
            serialIndex = 0;
            continue;
        }

        if (serialIndex < cfg16::SERIAL_BUFFER_SIZE - 1)
        {
            serialBuffer[serialIndex++] = c;
        }
        else
        {
            serialIndex = 0;
        }
    }
}

// ============================================================
// CONTROL TICK
// ============================================================

void controlTick(uint32_t nowUs)
{
    const uint32_t dtUs = nowUs - lastControlTimeUs;
    const float dtSeconds = static_cast<float>(dtUs) * 1.0e-6F;
    lastControlTimeUs = nowUs;

    if (phase == Phase::CALIBRATING)
    {
        if (!pendulumPosition.update()) return;

        if (!downReference.update(pendulumPosition.betaRadians(), nowUs)) return;

        if (!downReference.isReady())
        {
            Serial.print(F("CAL_REJECT,span="));
            Serial.println(downReference.observedSpanDeg(), 2);
            phase = Phase::IDLE;
            return;
        }

        alphaRad = downReference.correctedAngleRad(
            pendulumPosition.betaRadians()
        );

        pendulumVelocity.reset(alphaRad, nowUs);
        pendulumEvents.reset();
        phase = Phase::READY;

        Serial.print(F("DOWN_OK,span="));
        Serial.println(downReference.observedSpanDeg(), 2);
        return;
    }

    if (!downReference.isReady()) return;
    if (!acquireState(nowUs)) return;

    if (phase == Phase::ARMED)
    {
        startKick(nowUs);
        return;
    }

    if (phase == Phase::KICK)
    {
        serviceKick(nowUs, dtSeconds, dtUs);
        return;
    }

    if (phase == Phase::SYNC)
    {
        serviceSync(nowUs);
        return;
    }

    if (isEnergyPhase())
    {
        serviceEnergyPhase(nowUs, dtSeconds, dtUs);

        if (phase == Phase::CAPTURE)
        {
            serviceLocalControl(nowUs, dtSeconds, dtUs);
        }
        return;
    }

    if (phase == Phase::CAPTURE || phase == Phase::BALANCE)
    {
        serviceLocalControl(nowUs, dtSeconds, dtUs);
    }
}

// ============================================================
// SETUP / LOOP
// ============================================================

void setup()
{
    Serial.begin(FurutaConfig::SERIAL_BAUD);
    Wire.begin();
    Wire.setClock(400000UL);

    if (!pendulumPosition.begin())
    {
        Serial.println(F("AS5600_ERR"));
        while (true) delay(1000);
    }

    pendulumPosition.update();

    const uint32_t nowUs = micros();
    pendulumVelocity.reset(0.0F, nowUs);

    motor.begin(FurutaConfig::MOTOR_MAX_SPEED_DEG_S, 2);

    lastControlTimeUs = micros();
    nextControlTimeUs = lastControlTimeUs + FurutaConfig::CONTROL_PERIOD_US;

    Serial.println();
    Serial.println(F("F16.7T LF"));
    printConfig();
}

void loop()
{
    motor.update();
    processSerial();
    motor.update();

    const uint32_t nowUs = micros();

    if (static_cast<int32_t>(nowUs - nextControlTimeUs) >= 0)
    {
        controlTick(nowUs);
        nextControlTimeUs += FurutaConfig::CONTROL_PERIOD_US;

        if (static_cast<int32_t>(micros() - nextControlTimeUs) >= 0)
        {
            nextControlTimeUs = micros() + FurutaConfig::CONTROL_PERIOD_US;
        }
    }

    motor.update();
}