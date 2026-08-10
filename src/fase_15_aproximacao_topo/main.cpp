#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>

#include <FurutaConfig.h>
#include <MotorVelocity.h>
#include <PendulumPosition.h>
#include <PendulumVelocity.h>
#include <PendulumEvents.h>
#include <PendulumDownReference.h>

#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>


// ============================================================
// FASE 15E.6
//
// SWING-UP:
//   energia retida do ultimo pico
//   + referencia energetica EREF configuravel
//   + sinal de fase
//
// BRACO:
//   contencao direcional da 15E.5
//
// alpha = 0      -> DOWN
// alpha = +/-180 -> TOP
//
// uE = KENERGY * (EREF - eHeld) * sign_hyst(q)
//
// q = (alphaDot / omega0) * cos(alpha)
//
// |uE|   <= UEMAX = 300 deg/s2
// |uArm| <= UARM  = 150 deg/s2
//
// Os limites fisicos do atuador permanecem inalterados.
// ============================================================


// ============================================================
// LIMITES FIXOS
// ============================================================

constexpr float ARM_ABORT_DEG = 60.0F;

constexpr float ARM_CENTER_BAND_DEG = 3.0F;

constexpr uint32_t MAX_CONTROL_DT_US = 20000UL;

constexpr float MIN_TARGET_DEG = 60.0F;
constexpr float MAX_TARGET_DEG = 150.0F;


// ============================================================
// CONFIGURACAO
// ============================================================

struct TestConfig
{
    float omega0RadS;

    int8_t controlSign;

    float kEnergyDegS2;

    // NOVO:
    // referencia energetica configuravel pela Serial.
    float energyReference;

    float uEnergyMaxDegS2;
    float phaseHysteresis;

    float kPhi;
    float kDPhi;
    float uArmMaxDegS2;

    float targetDeg;
    float startVelocityRadS;

    uint8_t maxCycles;
    uint8_t stallCycles;

    uint16_t timeoutSeconds;
};


TestConfig cfg;


// ============================================================
// DEFAULTS
// ============================================================

void loadDefaultConfig()
{
    cfg.omega0RadS = 6.31F;

    cfg.controlSign = +1;

    cfg.kEnergyDegS2 = 1000.0F;

    // Mantemos 1.00 como default.
    // O proximo ensaio pode usar:
    //
    // SET EREF 1.10
    //
    cfg.energyReference = 1.00F;

    cfg.uEnergyMaxDegS2 = 300.0F;
    cfg.phaseHysteresis = 0.030F;

    cfg.kPhi = 0.750F;
    cfg.kDPhi = 2.000F;
    cfg.uArmMaxDegS2 = 150.0F;

    cfg.targetDeg = 150.0F;

    cfg.startVelocityRadS = 0.30F;

    cfg.maxCycles = 60;
    cfg.stallCycles = 3;

    cfg.timeoutSeconds = 80;
}


// ============================================================
// HARDWARE
// ============================================================

constexpr float SENSOR_DIRECTION_SIGN = 1.0F;


AS5600 as5600;


PendulumPosition pendulumPosition(
    as5600,
    SENSOR_DIRECTION_SIGN
);


PendulumVelocity pendulumVelocity;

PendulumEvents pendulumEvents;


PendulumDownReference downReference(
    3000UL,
    3.0F
);


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


// ============================================================
// ESTADOS
// ============================================================

enum class Mode : uint8_t
{
    WAITING,
    CALIBRATING,
    READY
};


enum class RunState : uint8_t
{
    IDLE,
    ARMED,
    ACTIVE
};


enum class FinishReason : uint8_t
{
    NONE,

    TARGET_REACHED,
    ENERGY_STALLED,
    MAX_CYCLES,

    ARM_LIMIT,
    TIMEOUT,
    CONTROL_OVERRUN,
    MOTOR_REJECTED,

    USER_STOP
};


Mode mode = Mode::WAITING;

RunState runState = RunState::IDLE;

FinishReason finishReason = FinishReason::NONE;


bool armReferenceDefined = false;


// ============================================================
// PENDULO
// ============================================================

float alphaRad = 0.0F;

float alphaDotRadS = 0.0F;


// Energia instantanea:
// apenas diagnostico.
float energyInstant = 0.0F;


// Energia retida do ultimo pico:
// usada efetivamente pelo controlador.
float energyHeld = 0.0F;


float energyError = 0.0F;


// ============================================================
// FASE
// ============================================================

float phaseSignal = 0.0F;

int8_t phaseSignState = 0;


// ============================================================
// CONTROLE
// ============================================================

float uEnergyRawDegS2 = 0.0F;

float uEnergyDegS2 = 0.0F;


float uArmRawDegS2 = 0.0F;

float uArmDegS2 = 0.0F;


float uRawDegS2 = 0.0F;

float uAppliedDegS2 = 0.0F;


bool armDerivativeActive = true;


// ============================================================
// TEMPO
// ============================================================

uint32_t nextControlTimeUs = 0;

uint32_t lastControlTimeUs = 0;

uint32_t activeStartUs = 0;


// ============================================================
// CONTADORES
// ============================================================

uint8_t fullCycleCount = 0;

uint8_t halfCycleCount = 0;

uint8_t consecutiveNonPositiveCycles = 0;


// ============================================================
// ESTATISTICAS
// ============================================================

struct Statistics
{
    float maxAbsAlphaDeg;

    float maxEnergyInstant;
    float maxEnergyHeld;

    float maxAbsAlphaDot;

    float maxAbsPhiDeg;
    float maxAbsPhiDotDegS;

    float maxAbsUEnergy;
    float maxAbsUArm;
    float maxAbsU;

    uint32_t energySaturations;
    uint32_t armSaturations;
    uint32_t totalSaturations;

    uint32_t phaseSwitches;

    uint32_t armReturnSamples;

    uint32_t samples;
};


Statistics stats;


// ============================================================
// START
// ============================================================

struct StartRecord
{
    bool valid;

    float alphaDeg;
    float alphaDot;

    float energyInstant;
    float energyHeld;

    float phiDeg;
};


StartRecord startRecord;


// ============================================================
// ULTIMO DOWN
// ============================================================

bool lastDownValid = false;

bool lastDownPositive = false;


float lastDownAlphaDot = 0.0F;

float lastDownPhi = 0.0F;

float lastDownPhiDot = 0.0F;


float lastDownEnergyInstant = 0.0F;

float lastDownEnergyHeld = 0.0F;


float lastDownUEnergy = 0.0F;

float lastDownUArm = 0.0F;


bool lastDownDerivativeActive = true;


// ============================================================
// PICOS
// ============================================================

bool previousAnyPeakValid = false;

float previousAnyPeakEnergy = 0.0F;


bool previousPositivePeakValid = false;

float previousPositivePeakEnergy = 0.0F;


bool previousNegativePeakValid = false;

float previousNegativePeakEnergy = 0.0F;


// ============================================================
// SEMICICLO
// ============================================================

float halfMaxUEnergy = 0.0F;

float halfMaxUArm = 0.0F;

float halfMaxU = 0.0F;


uint16_t halfEnergySat = 0;

uint16_t halfArmSat = 0;

uint16_t halfPhaseSwitches = 0;


// ============================================================
// HISTORICO
// ============================================================

constexpr uint8_t RECORD_CAPACITY = 4;


struct PeakRecord
{
    bool used;

    bool positive;

    uint16_t number;

    float angleDeg;

    float energyPeak;

    float deltaHalf;

    bool cycleDeltaValid;

    float deltaCycle;

    bool downValid;

    bool downPositive;

    float downAlphaDot;

    float downPhi;

    float downPhiDot;

    float downEnergyInstant;

    float downEnergyHeld;

    float downUEnergy;

    float downUArm;

    bool downDerivativeActive;

    float maxUEnergy;

    float maxUArm;

    float maxU;

    uint16_t energySat;

    uint16_t armSat;

    uint16_t phaseSwitches;
};


PeakRecord records[RECORD_CAPACITY];


uint16_t peakCount = 0;


// ============================================================
// SERIAL
// ============================================================

constexpr uint8_t SERIAL_BUFFER_SIZE = 48;


char serialBuffer[SERIAL_BUFFER_SIZE];


uint8_t serialIndex = 0;


// ============================================================
// UTILITARIOS
// ============================================================

float clampFloat(
    float value,
    float minimum,
    float maximum
)
{
    if (value < minimum)
    {
        return minimum;
    }


    if (value > maximum)
    {
        return maximum;
    }


    return value;
}


// ============================================================
// ENERGIAS
// ============================================================

float peakEnergyFromAlpha(
    float alpha
)
{
    return
        0.5F *
        (
            1.0F -
            cosf(alpha)
        );
}


float normalizedInstantEnergy(
    float alpha,
    float alphaDot
)
{
    const float w =
        alphaDot /
        cfg.omega0RadS;


    return
        0.25F * w * w
        +
        0.5F *
        (
            1.0F -
            cosf(alpha)
        );
}


float targetPeakEnergy()
{
    return
        peakEnergyFromAlpha(
            cfg.targetDeg *
            DEG_TO_RAD
        );
}


// ============================================================
// FASE
// ============================================================

void updatePhaseSign()
{
    const int8_t oldSign =
        phaseSignState;


    if (
        phaseSignal >
        cfg.phaseHysteresis
    )
    {
        phaseSignState = +1;
    }

    else if (
        phaseSignal <
        -cfg.phaseHysteresis
    )
    {
        phaseSignState = -1;
    }


    if (
        oldSign != 0
        &&
        phaseSignState != oldSign
    )
    {
        ++stats.phaseSwitches;

        ++halfPhaseSwitches;
    }
}


// ============================================================
// CONTROLE ENERGETICO
//
// NOVO:
//
// uE = KENERGY * (EREF - eHeld) * phaseSign
//
// A referencia nao e mais fixa em 1.0.
// ============================================================

float computeEnergyControl()
{
    if (
        phaseSignState == 0
    )
    {
        return 0.0F;
    }


    return
        static_cast<float>(
            cfg.controlSign
        )
        *
        cfg.kEnergyDegS2
        *
        energyError
        *
        static_cast<float>(
            phaseSignState
        );
}


// ============================================================
// CONTENCAO DIRECIONAL DO BRACO
// ============================================================

float computeArmControl(
    float phiDeg,
    float phiDotDegS
)
{
    const bool movingAway =
        (
            phiDeg *
            phiDotDegS
        )
        >
        0.0F;


    const bool nearCenter =
        fabsf(phiDeg)
        <=
        ARM_CENTER_BAND_DEG;


    armDerivativeActive =
        movingAway
        ||
        nearCenter;


    float u =
        -cfg.kPhi *
        phiDeg;


    if (
        armDerivativeActive
    )
    {
        u -=
            cfg.kDPhi *
            phiDotDegS;
    }
    else
    {
        ++stats.armReturnSamples;
    }


    return u;
}


// ============================================================
// VALIDACAO
// ============================================================

bool validateConfig()
{
    if (
        cfg.omega0RadS < 4.0F
        ||
        cfg.omega0RadS > 9.0F
    )
    {
        return false;
    }


    if (
        cfg.controlSign != 1
        &&
        cfg.controlSign != -1
    )
    {
        return false;
    }


    if (
        cfg.kEnergyDegS2 < 100.0F
        ||
        cfg.kEnergyDegS2 > 2000.0F
    )
    {
        return false;
    }


    // EREF fica deliberadamente limitado a uma
    // pequena faixa de compensacao.
    if (
        cfg.energyReference < 0.90F
        ||
        cfg.energyReference > 1.20F
    )
    {
        return false;
    }


    // Autoridade energetica permanece inalterada.
    if (
        cfg.uEnergyMaxDegS2 < 50.0F
        ||
        cfg.uEnergyMaxDegS2 > 300.0F
    )
    {
        return false;
    }


    if (
        cfg.phaseHysteresis < 0.005F
        ||
        cfg.phaseHysteresis > 0.20F
    )
    {
        return false;
    }


    if (
        cfg.kPhi < 0.0F
        ||
        cfg.kPhi > 3.0F
    )
    {
        return false;
    }


    if (
        cfg.kDPhi < 0.0F
        ||
        cfg.kDPhi > 6.0F
    )
    {
        return false;
    }


    if (
        cfg.uArmMaxDegS2 < 25.0F
        ||
        cfg.uArmMaxDegS2 > 150.0F
    )
    {
        return false;
    }


    if (
        cfg.targetDeg < MIN_TARGET_DEG
        ||
        cfg.targetDeg > MAX_TARGET_DEG
    )
    {
        return false;
    }


    if (
        cfg.startVelocityRadS < 0.05F
        ||
        cfg.startVelocityRadS > 2.0F
    )
    {
        return false;
    }


    if (
        cfg.maxCycles < 1
        ||
        cfg.maxCycles > 80
    )
    {
        return false;
    }


    if (
        cfg.stallCycles < 1
        ||
        cfg.stallCycles > 10
    )
    {
        return false;
    }


    if (
        cfg.timeoutSeconds < 5
        ||
        cfg.timeoutSeconds > 120
    )
    {
        return false;
    }


    return true;
}


// ============================================================
// RESET
// ============================================================

void resetExperiment()
{
    startRecord = {};

    stats = {};


    fullCycleCount = 0;

    halfCycleCount = 0;

    consecutiveNonPositiveCycles = 0;


    phaseSignal = 0.0F;

    phaseSignState = 0;


    lastDownValid = false;


    previousAnyPeakValid = false;

    previousPositivePeakValid = false;

    previousNegativePeakValid = false;


    halfMaxUEnergy = 0.0F;

    halfMaxUArm = 0.0F;

    halfMaxU = 0.0F;


    halfEnergySat = 0;

    halfArmSat = 0;

    halfPhaseSwitches = 0;


    peakCount = 0;


    for (
        uint8_t i = 0;
        i < RECORD_CAPACITY;
        ++i
    )
    {
        records[i] = {};
    }


    energyInstant = 0.0F;

    energyHeld = 0.0F;

    energyError = 0.0F;


    uEnergyRawDegS2 = 0.0F;

    uEnergyDegS2 = 0.0F;


    uArmRawDegS2 = 0.0F;

    uArmDegS2 = 0.0F;


    uRawDegS2 = 0.0F;

    uAppliedDegS2 = 0.0F;


    armDerivativeActive = true;


    finishReason =
        FinishReason::NONE;


    pendulumEvents.reset();
}


// ============================================================
// CONFIG
// ============================================================

void printConfig()
{
    Serial.println();

    Serial.println(F("# CFG"));


    Serial.print(F("OMEGA0="));

    Serial.println(
        cfg.omega0RadS,
        3
    );


    Serial.print(F("SIGN="));

    Serial.println(
        cfg.controlSign
    );


    Serial.print(F("KENERGY="));

    Serial.println(
        cfg.kEnergyDegS2,
        1
    );


    // NOVO
    Serial.print(F("EREF="));

    Serial.println(
        cfg.energyReference,
        3
    );


    Serial.print(F("UEMAX="));

    Serial.println(
        cfg.uEnergyMaxDegS2,
        1
    );


    Serial.print(F("PHASEHYS="));

    Serial.println(
        cfg.phaseHysteresis,
        4
    );


    Serial.print(F("KPHI="));

    Serial.println(
        cfg.kPhi,
        3
    );


    Serial.print(F("KDPHI="));

    Serial.println(
        cfg.kDPhi,
        3
    );


    Serial.print(F("UARM="));

    Serial.println(
        cfg.uArmMaxDegS2,
        1
    );


    Serial.print(F("ARMBAND="));

    Serial.println(
        ARM_CENTER_BAND_DEG,
        1
    );


    Serial.print(F("TARGET="));

    Serial.print(
        cfg.targetDeg,
        2
    );


    Serial.print(F(",E="));

    Serial.println(
        targetPeakEnergy(),
        6
    );


    Serial.print(F("STARTVEL="));

    Serial.println(
        cfg.startVelocityRadS,
        3
    );


    Serial.print(F("CYCLES="));

    Serial.println(
        cfg.maxCycles
    );


    Serial.print(F("STALL="));

    Serial.println(
        cfg.stallCycles
    );


    Serial.print(F("TIMEOUT="));

    Serial.println(
        cfg.timeoutSeconds
    );
}


// ============================================================
// RESULTADO
// ============================================================

void printFinishReason()
{
    switch (finishReason)
    {
        case FinishReason::TARGET_REACHED:

            Serial.print(F("target_reached"));

            break;


        case FinishReason::ENERGY_STALLED:

            Serial.print(F("energy_stalled"));

            break;


        case FinishReason::MAX_CYCLES:

            Serial.print(F("max_cycles"));

            break;


        case FinishReason::ARM_LIMIT:

            Serial.print(F("arm_limit"));

            break;


        case FinishReason::TIMEOUT:

            Serial.print(F("timeout"));

            break;


        case FinishReason::CONTROL_OVERRUN:

            Serial.print(F("overrun"));

            break;


        case FinishReason::MOTOR_REJECTED:

            Serial.print(F("motor_rejected"));

            break;


        case FinishReason::USER_STOP:

            Serial.print(F("user_stop"));

            break;


        default:

            Serial.print(F("none"));

            break;
    }
}


void printResult()
{
    Serial.println();

    Serial.println(F("# RESULT_DATA"));


    if (
        startRecord.valid
    )
    {
        Serial.print(F("# START,a="));

        Serial.print(
            startRecord.alphaDeg,
            2
        );


        Serial.print(F(",ad="));

        Serial.print(
            startRecord.alphaDot,
            3
        );


        Serial.print(F(",eI="));

        Serial.print(
            startRecord.energyInstant,
            5
        );


        Serial.print(F(",eH="));

        Serial.print(
            startRecord.energyHeld,
            5
        );


        Serial.print(F(",phi="));

        Serial.println(
            startRecord.phiDeg,
            2
        );
    }


    const uint16_t count =
        (
            peakCount <
            RECORD_CAPACITY
        )
        ?
        peakCount
        :
        RECORD_CAPACITY;


    const uint16_t first =
        (
            peakCount >
            RECORD_CAPACITY
        )
        ?
        peakCount -
        RECORD_CAPACITY
        :
        0;


    for (
        uint16_t n = 0;
        n < count;
        ++n
    )
    {
        PeakRecord &r =
            records[
                (first + n)
                %
                RECORD_CAPACITY
            ];


        if (!r.used)
        {
            continue;
        }


        Serial.print(F("# P,"));

        Serial.print(
            r.number
        );


        Serial.print(F(","));

        Serial.print(
            r.positive
            ?
            '+'
            :
            '-'
        );


        Serial.print(F(",a="));

        Serial.print(
            r.angleDeg,
            2
        );


        Serial.print(F(",e="));

        Serial.print(
            r.energyPeak,
            6
        );


        Serial.print(F(",dH="));

        Serial.print(
            r.deltaHalf,
            6
        );


        if (
            r.cycleDeltaValid
        )
        {
            Serial.print(F(",dC="));

            Serial.print(
                r.deltaCycle,
                6
            );
        }


        Serial.print(F(",UE="));

        Serial.print(
            r.maxUEnergy,
            1
        );


        Serial.print(F(",UA="));

        Serial.print(
            r.maxUArm,
            1
        );


        Serial.print(F(",U="));

        Serial.print(
            r.maxU,
            1
        );


        Serial.print(F(",eSat="));

        Serial.print(
            r.energySat
        );


        Serial.print(F(",aSat="));

        Serial.print(
            r.armSat
        );


        Serial.print(F(",sw="));

        Serial.println(
            r.phaseSwitches
        );


        if (
            r.downValid
        )
        {
            Serial.print(F("# D,"));

            Serial.print(
                r.downPositive
                ?
                '+'
                :
                '-'
            );


            Serial.print(F(",ad="));

            Serial.print(
                r.downAlphaDot,
                3
            );


            Serial.print(F(",phi="));

            Serial.print(
                r.downPhi,
                2
            );


            Serial.print(F(",pd="));

            Serial.print(
                r.downPhiDot,
                2
            );


            Serial.print(F(",eI="));

            Serial.print(
                r.downEnergyInstant,
                5
            );


            Serial.print(F(",eH="));

            Serial.print(
                r.downEnergyHeld,
                5
            );


            Serial.print(F(",uE="));

            Serial.print(
                r.downUEnergy,
                1
            );


            Serial.print(F(",uA="));

            Serial.print(
                r.downUArm,
                1
            );


            Serial.print(F(",D="));

            Serial.println(
                r.downDerivativeActive
                ?
                1
                :
                0
            );
        }
    }


    Serial.print(F("# RESULT="));

    printFinishReason();

    Serial.println();


    Serial.print(F("# cycles="));

    Serial.println(
        fullCycleCount
    );


    Serial.print(F("# maxAlpha="));

    Serial.println(
        stats.maxAbsAlphaDeg,
        2
    );


    Serial.print(F("# maxEI="));

    Serial.println(
        stats.maxEnergyInstant,
        6
    );


    Serial.print(F("# maxEH="));

    Serial.println(
        stats.maxEnergyHeld,
        6
    );


    Serial.print(F("# maxPhi="));

    Serial.println(
        stats.maxAbsPhiDeg,
        2
    );


    Serial.print(F("# maxPhiDot="));

    Serial.println(
        stats.maxAbsPhiDotDegS,
        2
    );


    Serial.print(F("# maxUE="));

    Serial.println(
        stats.maxAbsUEnergy,
        1
    );


    Serial.print(F("# maxUA="));

    Serial.println(
        stats.maxAbsUArm,
        1
    );


    Serial.print(F("# maxU="));

    Serial.println(
        stats.maxAbsU,
        1
    );


    Serial.print(F("# energySat="));

    Serial.println(
        stats.energySaturations
    );


    Serial.print(F("# armSat="));

    Serial.println(
        stats.armSaturations
    );


    Serial.print(F("# totalSat="));

    Serial.println(
        stats.totalSaturations
    );


    Serial.print(F("# armReturnSamples="));

    Serial.println(
        stats.armReturnSamples
    );


    Serial.print(F("# phaseSw="));

    Serial.println(
        stats.phaseSwitches
    );


    Serial.print(F("# samples="));

    Serial.println(
        stats.samples
    );
}


// ============================================================
// FINALIZACAO
// ============================================================

void finishExperiment(
    FinishReason reason
)
{
    if (
        runState ==
        RunState::IDLE
    )
    {
        return;
    }


    finishReason =
        reason;


    motor.stop();


    runState =
        RunState::IDLE;


    printResult();


    pendulumEvents.reset();
}


// ============================================================
// DOWN
// ============================================================

void handleDownEvent(
    bool positive,
    const PendulumEvent &event
)
{
    lastDownValid = true;

    lastDownPositive = positive;


    lastDownAlphaDot =
        event.alphaDotRadS;


    lastDownPhi =
        motor.currentPositionDegrees();


    lastDownPhiDot =
        motor.speedReferenceDegreesPerSecond();


    lastDownEnergyInstant =
        normalizedInstantEnergy(
            event.alphaRad,
            event.alphaDotRadS
        );


    lastDownEnergyHeld =
        energyHeld;


    lastDownUEnergy =
        uEnergyDegS2;


    lastDownUArm =
        uArmDegS2;


    lastDownDerivativeActive =
        armDerivativeActive;
}


// ============================================================
// PEAK
// ============================================================

void handlePeakEvent(
    bool positive,
    const PendulumEvent &event
)
{
    const float angleDeg =
        event.alphaRad *
        RAD_TO_DEG;


    const float absAngleDeg =
        fabsf(
            angleDeg
        );


    const float ePeak =
        peakEnergyFromAlpha(
            event.alphaRad
        );


    PeakRecord r = {};


    r.used = true;

    r.positive = positive;

    r.number =
        peakCount + 1;


    r.angleDeg =
        angleDeg;


    r.energyPeak =
        ePeak;


    // ========================================================
    // DELTA SEMICICLO
    // ========================================================

    if (
        previousAnyPeakValid
    )
    {
        r.deltaHalf =
            ePeak -
            previousAnyPeakEnergy;
    }


    previousAnyPeakValid = true;

    previousAnyPeakEnergy =
        ePeak;


    // ========================================================
    // DELTA CICLO
    // ========================================================

    if (positive)
    {
        if (
            previousPositivePeakValid
        )
        {
            r.cycleDeltaValid = true;

            r.deltaCycle =
                ePeak -
                previousPositivePeakEnergy;
        }


        previousPositivePeakValid = true;

        previousPositivePeakEnergy =
            ePeak;
    }
    else
    {
        if (
            previousNegativePeakValid
        )
        {
            r.cycleDeltaValid = true;

            r.deltaCycle =
                ePeak -
                previousNegativePeakEnergy;


            ++fullCycleCount;


            if (
                r.deltaCycle >
                0.0F
            )
            {
                consecutiveNonPositiveCycles = 0;
            }
            else
            {
                ++consecutiveNonPositiveCycles;
            }
        }


        previousNegativePeakValid = true;

        previousNegativePeakEnergy =
            ePeak;
    }


    ++halfCycleCount;


    // ========================================================
    // DOWN ANTERIOR
    // ========================================================

    r.downValid =
        lastDownValid;


    r.downPositive =
        lastDownPositive;


    r.downAlphaDot =
        lastDownAlphaDot;


    r.downPhi =
        lastDownPhi;


    r.downPhiDot =
        lastDownPhiDot;


    r.downEnergyInstant =
        lastDownEnergyInstant;


    r.downEnergyHeld =
        lastDownEnergyHeld;


    r.downUEnergy =
        lastDownUEnergy;


    r.downUArm =
        lastDownUArm;


    r.downDerivativeActive =
        lastDownDerivativeActive;


    // ========================================================
    // SEMICICLO
    // ========================================================

    r.maxUEnergy =
        halfMaxUEnergy;


    r.maxUArm =
        halfMaxUArm;


    r.maxU =
        halfMaxU;


    r.energySat =
        halfEnergySat;


    r.armSat =
        halfArmSat;


    r.phaseSwitches =
        halfPhaseSwitches;


    halfMaxUEnergy = 0.0F;

    halfMaxUArm = 0.0F;

    halfMaxU = 0.0F;


    halfEnergySat = 0;

    halfArmSat = 0;

    halfPhaseSwitches = 0;


    // ========================================================
    // BUFFER
    // ========================================================

    records[
        peakCount %
        RECORD_CAPACITY
    ] =
        r;


    ++peakCount;


    // ========================================================
    // NOVA ENERGIA RETIDA
    // ========================================================

    energyHeld =
        ePeak;


    // ========================================================
    // NOVO:
    // ERRO COM EREF CONFIGURAVEL
    // ========================================================

    energyError =
        cfg.energyReference
        -
        energyHeld;


    // ========================================================
    // TARGET
    // ========================================================

    if (
        absAngleDeg >=
        cfg.targetDeg
    )
    {
        finishExperiment(
            FinishReason::TARGET_REACHED
        );

        return;
    }


    // ========================================================
    // STALL
    // ========================================================

    if (
        consecutiveNonPositiveCycles >=
        cfg.stallCycles
    )
    {
        finishExperiment(
            FinishReason::ENERGY_STALLED
        );

        return;
    }


    if (
        fullCycleCount >=
        cfg.maxCycles
    )
    {
        finishExperiment(
            FinishReason::MAX_CYCLES
        );
    }
}


// ============================================================
// EVENTOS
// ============================================================

void handleEvent(
    const PendulumEvent &event
)
{
    switch (
        event.type
    )
    {
        case PendulumEventType::DOWN_POSITIVE:

            handleDownEvent(
                true,
                event
            );

            break;


        case PendulumEventType::DOWN_NEGATIVE:

            handleDownEvent(
                false,
                event
            );

            break;


        case PendulumEventType::PEAK_POSITIVE:

            handlePeakEvent(
                true,
                event
            );

            break;


        case PendulumEventType::PEAK_NEGATIVE:

            handlePeakEvent(
                false,
                event
            );

            break;


        default:

            break;
    }
}


// ============================================================
// ESTATISTICAS
// ============================================================

void updateStatistics()
{
    const float absAlpha =
        fabsf(
            alphaRad *
            RAD_TO_DEG
        );


    const float phi =
        motor.currentPositionDegrees();


    const float phiDot =
        motor.speedReferenceDegreesPerSecond();


    if (
        absAlpha >
        stats.maxAbsAlphaDeg
    )
    {
        stats.maxAbsAlphaDeg =
            absAlpha;
    }


    if (
        energyInstant >
        stats.maxEnergyInstant
    )
    {
        stats.maxEnergyInstant =
            energyInstant;
    }


    if (
        energyHeld >
        stats.maxEnergyHeld
    )
    {
        stats.maxEnergyHeld =
            energyHeld;
    }


    if (
        fabsf(alphaDotRadS) >
        stats.maxAbsAlphaDot
    )
    {
        stats.maxAbsAlphaDot =
            fabsf(alphaDotRadS);
    }


    if (
        fabsf(phi) >
        stats.maxAbsPhiDeg
    )
    {
        stats.maxAbsPhiDeg =
            fabsf(phi);
    }


    if (
        fabsf(phiDot) >
        stats.maxAbsPhiDotDegS
    )
    {
        stats.maxAbsPhiDotDegS =
            fabsf(phiDot);
    }


    if (
        fabsf(uEnergyDegS2) >
        stats.maxAbsUEnergy
    )
    {
        stats.maxAbsUEnergy =
            fabsf(uEnergyDegS2);
    }


    if (
        fabsf(uArmDegS2) >
        stats.maxAbsUArm
    )
    {
        stats.maxAbsUArm =
            fabsf(uArmDegS2);
    }


    if (
        fabsf(uAppliedDegS2) >
        stats.maxAbsU
    )
    {
        stats.maxAbsU =
            fabsf(uAppliedDegS2);
    }


    if (
        fabsf(uEnergyDegS2) >
        halfMaxUEnergy
    )
    {
        halfMaxUEnergy =
            fabsf(uEnergyDegS2);
    }


    if (
        fabsf(uArmDegS2) >
        halfMaxUArm
    )
    {
        halfMaxUArm =
            fabsf(uArmDegS2);
    }


    if (
        fabsf(uAppliedDegS2) >
        halfMaxU
    )
    {
        halfMaxU =
            fabsf(uAppliedDegS2);
    }
}


// ============================================================
// ATIVACAO
// ============================================================

void activateEnergyControl(
    uint32_t nowUs
)
{
    runState =
        RunState::ACTIVE;


    activeStartUs =
        nowUs;


    energyInstant =
        normalizedInstantEnergy(
            alphaRad,
            alphaDotRadS
        );


    // Antes do primeiro pico verdadeiro:
    // usa a energia potencial da posicao inicial.
    energyHeld =
        peakEnergyFromAlpha(
            alphaRad
        );


    // NOVO:
    energyError =
        cfg.energyReference
        -
        energyHeld;


    startRecord.valid = true;


    startRecord.alphaDeg =
        alphaRad *
        RAD_TO_DEG;


    startRecord.alphaDot =
        alphaDotRadS;


    startRecord.energyInstant =
        energyInstant;


    startRecord.energyHeld =
        energyHeld;


    startRecord.phiDeg =
        motor.currentPositionDegrees();


    phaseSignal =
        (
            alphaDotRadS /
            cfg.omega0RadS
        )
        *
        cosf(
            alphaRad
        );


    if (
        phaseSignal >
        cfg.phaseHysteresis
    )
    {
        phaseSignState = +1;
    }

    else if (
        phaseSignal <
        -cfg.phaseHysteresis
    )
    {
        phaseSignState = -1;
    }
    else
    {
        phaseSignState = 0;
    }


    pendulumEvents.reset();
}


// ============================================================
// CONTROLE ATIVO
// ============================================================

void serviceActiveControl(
    uint32_t nowUs,
    float dtSeconds,
    uint32_t dtUs
)
{
    // ========================================================
    // TIMING
    // ========================================================

    if (
        dtUs >
        MAX_CONTROL_DT_US
    )
    {
        finishExperiment(
            FinishReason::CONTROL_OVERRUN
        );

        return;
    }


    // ========================================================
    // TIMEOUT
    // ========================================================

    const uint32_t timeoutUs =
        static_cast<uint32_t>(
            cfg.timeoutSeconds
        )
        *
        1000000UL;


    if (
        nowUs -
        activeStartUs
        >=
        timeoutUs
    )
    {
        finishExperiment(
            FinishReason::TIMEOUT
        );

        return;
    }


    // ========================================================
    // BRACO
    // ========================================================

    const float phi =
        motor.currentPositionDegrees();


    const float phiDot =
        motor.speedReferenceDegreesPerSecond();


    if (
        fabsf(phi) >=
        ARM_ABORT_DEG
    )
    {
        finishExperiment(
            FinishReason::ARM_LIMIT
        );

        return;
    }


    // ========================================================
    // ENERGIA INSTANTANEA
    // Apenas diagnostico.
    // ========================================================

    energyInstant =
        normalizedInstantEnergy(
            alphaRad,
            alphaDotRadS
        );


    // ========================================================
    // NOVO:
    //
    // usa EREF configuravel.
    // ========================================================

    energyError =
        cfg.energyReference
        -
        energyHeld;


    // Se algum dia eHeld ultrapassar EREF,
    // nao continuamos bombeando energia no sentido errado.
    //
    // O controlador simplesmente zera a contribuicao
    // energetica nessa condicao.
    if (
        energyError <
        0.0F
    )
    {
        energyError = 0.0F;
    }


    // ========================================================
    // FASE
    // ========================================================

    phaseSignal =
        (
            alphaDotRadS /
            cfg.omega0RadS
        )
        *
        cosf(
            alphaRad
        );


    updatePhaseSign();


    // ========================================================
    // ENERGIA
    // ========================================================

    uEnergyRawDegS2 =
        computeEnergyControl();


    if (
        fabsf(
            uEnergyRawDegS2
        )
        >
        cfg.uEnergyMaxDegS2
    )
    {
        ++stats.energySaturations;

        ++halfEnergySat;
    }


    uEnergyDegS2 =
        clampFloat(
            uEnergyRawDegS2,
            -cfg.uEnergyMaxDegS2,
            +cfg.uEnergyMaxDegS2
        );


    // ========================================================
    // BRACO
    // ========================================================

    uArmRawDegS2 =
        computeArmControl(
            phi,
            phiDot
        );


    if (
        fabsf(
            uArmRawDegS2
        )
        >
        cfg.uArmMaxDegS2
    )
    {
        ++stats.armSaturations;

        ++halfArmSat;
    }


    uArmDegS2 =
        clampFloat(
            uArmRawDegS2,
            -cfg.uArmMaxDegS2,
            +cfg.uArmMaxDegS2
        );


    // ========================================================
    // TOTAL
    // ========================================================

    uRawDegS2 =
        uEnergyDegS2
        +
        uArmDegS2;


    if (
        fabsf(
            uRawDegS2
        )
        >
        FurutaConfig::MAX_ACCEL_DEG_S2
    )
    {
        ++stats.totalSaturations;
    }


    uAppliedDegS2 =
        clampFloat(
            uRawDegS2,
            -FurutaConfig::MAX_ACCEL_DEG_S2,
            +FurutaConfig::MAX_ACCEL_DEG_S2
        );


    // ========================================================
    // MOTOR
    // ========================================================

    if (
        !motor.commandAcceleration(
            uAppliedDegS2,
            dtSeconds
        )
    )
    {
        finishExperiment(
            FinishReason::MOTOR_REJECTED
        );

        return;
    }


    ++stats.samples;


    updateStatistics();


    // ========================================================
    // EVENTOS
    // ========================================================

    const PendulumEvent event =
        pendulumEvents.update(
            alphaRad,
            alphaDotRadS,
            nowUs
        );


    if (
        event.type !=
        PendulumEventType::NONE
    )
    {
        handleEvent(
            event
        );
    }
}


// ============================================================
// TICK 4 ms
// ============================================================

void controlTick(
    uint32_t nowUs
)
{
    const uint32_t dtUs =
        nowUs -
        lastControlTimeUs;


    const float dtSeconds =
        static_cast<float>(
            dtUs
        )
        *
        1.0e-6F;


    lastControlTimeUs =
        nowUs;


    if (
        !pendulumPosition.update()
    )
    {
        return;
    }


    // ========================================================
    // CALIBRACAO
    // ========================================================

    if (
        mode ==
        Mode::CALIBRATING
    )
    {
        if (
            !downReference.update(
                pendulumPosition.betaRadians(),
                nowUs
            )
        )
        {
            return;
        }


        if (
            !downReference.isReady()
        )
        {
            Serial.print(F("CAL_REJECT,span="));

            Serial.println(
                downReference.observedSpanDeg(),
                2
            );


            mode =
                Mode::WAITING;


            return;
        }


        alphaRad =
            downReference.correctedAngleRad(
                pendulumPosition.betaRadians()
            );


        pendulumVelocity.reset(
            alphaRad,
            nowUs
        );


        pendulumEvents.reset();


        mode =
            Mode::READY;


        Serial.print(F("DOWN_OK,span="));

        Serial.println(
            downReference.observedSpanDeg(),
            2
        );


        return;
    }


    if (
        mode !=
        Mode::READY
    )
    {
        return;
    }


    // ========================================================
    // MEDICAO
    // ========================================================

    alphaRad =
        downReference.correctedAngleRad(
            pendulumPosition.betaRadians()
        );


    pendulumVelocity.update(
        alphaRad,
        nowUs
    );


    if (
        !pendulumVelocity.isReady()
    )
    {
        return;
    }


    alphaDotRadS =
        pendulumVelocity.radiansPerSecond();


    // ========================================================
    // ARMED
    // ========================================================

    if (
        runState ==
        RunState::ARMED
    )
    {
        if (
            fabsf(
                alphaDotRadS
            )
            >=
            cfg.startVelocityRadS
        )
        {
            activateEnergyControl(
                nowUs
            );
        }


        return;
    }


    // ========================================================
    // ACTIVE
    // ========================================================

    if (
        runState ==
        RunState::ACTIVE
    )
    {
        serviceActiveControl(
            nowUs,
            dtSeconds,
            dtUs
        );
    }
}


// ============================================================
// CALIBRACAO
// ============================================================

void startCalibration()
{
    if (
        runState !=
        RunState::IDLE
    )
    {
        return;
    }


    motor.stop();


    if (
        !pendulumPosition.calibrateTop(
            32,
            2
        )
    )
    {
        Serial.println(
            F("AS5600_ERR")
        );

        return;
    }


    const uint32_t nowUs =
        micros();


    pendulumVelocity.reset(
        pendulumPosition.betaRadians(),
        nowUs
    );


    downReference.start(
        nowUs
    );


    mode =
        Mode::CALIBRATING;


    lastControlTimeUs =
        nowUs;


    nextControlTimeUs =
        nowUs
        +
        FurutaConfig::CONTROL_PERIOD_US;


    pendulumEvents.reset();


    Serial.println(
        F("CAL_DOWN")
    );
}


// ============================================================
// ARMAR
// ============================================================

void armTest()
{
    if (
        !validateConfig()
    )
    {
        Serial.println(
            F("CFG_ERR")
        );

        return;
    }


    if (
        mode !=
        Mode::READY
    )
    {
        Serial.println(
            F("NEED_T")
        );

        return;
    }


    if (
        !armReferenceDefined
    )
    {
        Serial.println(
            F("NEED_Z")
        );

        return;
    }


    if (
        !motor.isEnabled()
    )
    {
        Serial.println(
            F("NEED_E")
        );

        return;
    }


    if (
        runState !=
        RunState::IDLE
    )
    {
        return;
    }


    if (
        fabsf(
            motor.currentPositionDegrees()
        )
        >
        1.0F
    )
    {
        Serial.println(
            F("PHI_NOT_ZERO")
        );

        return;
    }


    resetExperiment();


    motor.stop();


    runState =
        RunState::ARMED;


    Serial.println(
        F("# ARMED")
    );


    printConfig();
}


// ============================================================
// PARAMETROS
// ============================================================

bool setParameter(
    const char *name,
    const char *valueText
)
{
    TestConfig old =
        cfg;


    const float value =
        atof(
            valueText
        );


    if (
        strcmp(
            name,
            "OMEGA0"
        )
        ==
        0
    )
    {
        cfg.omega0RadS =
            value;
    }


    else if (
        strcmp(
            name,
            "SIGN"
        )
        ==
        0
    )
    {
        cfg.controlSign =
            static_cast<int8_t>(
                atoi(
                    valueText
                )
            );
    }


    else if (
        strcmp(
            name,
            "KENERGY"
        )
        ==
        0
    )
    {
        cfg.kEnergyDegS2 =
            value;
    }


    // ========================================================
    // NOVO COMANDO:
    //
    // SET EREF 1.10
    // ========================================================

    else if (
        strcmp(
            name,
            "EREF"
        )
        ==
        0
    )
    {
        cfg.energyReference =
            value;
    }


    else if (
        strcmp(
            name,
            "UEMAX"
        )
        ==
        0
    )
    {
        cfg.uEnergyMaxDegS2 =
            value;
    }


    else if (
        strcmp(
            name,
            "PHASEHYS"
        )
        ==
        0
    )
    {
        cfg.phaseHysteresis =
            value;
    }


    else if (
        strcmp(
            name,
            "KPHI"
        )
        ==
        0
    )
    {
        cfg.kPhi =
            value;
    }


    else if (
        strcmp(
            name,
            "KDPHI"
        )
        ==
        0
    )
    {
        cfg.kDPhi =
            value;
    }


    else if (
        strcmp(
            name,
            "UARM"
        )
        ==
        0
    )
    {
        cfg.uArmMaxDegS2 =
            value;
    }


    else if (
        strcmp(
            name,
            "TARGET"
        )
        ==
        0
    )
    {
        cfg.targetDeg =
            value;
    }


    else if (
        strcmp(
            name,
            "STARTVEL"
        )
        ==
        0
    )
    {
        cfg.startVelocityRadS =
            value;
    }


    else if (
        strcmp(
            name,
            "CYCLES"
        )
        ==
        0
    )
    {
        cfg.maxCycles =
            static_cast<uint8_t>(
                atoi(
                    valueText
                )
            );
    }


    else if (
        strcmp(
            name,
            "STALL"
        )
        ==
        0
    )
    {
        cfg.stallCycles =
            static_cast<uint8_t>(
                atoi(
                    valueText
                )
            );
    }


    else if (
        strcmp(
            name,
            "TIMEOUT"
        )
        ==
        0
    )
    {
        cfg.timeoutSeconds =
            static_cast<uint16_t>(
                atoi(
                    valueText
                )
            );
    }


    else
    {
        Serial.println(
            F("BAD_PARAM")
        );

        return false;
    }


    if (
        !validateConfig()
    )
    {
        cfg =
            old;


        Serial.println(
            F("REJECT")
        );


        return false;
    }


    Serial.println(
        F("OK")
    );


    return true;
}


// ============================================================
// EMERGENCIA
// ============================================================

void emergencyStop()
{
    motor.emergencyStop();


    runState =
        RunState::IDLE;


    armReferenceDefined =
        false;


    pendulumEvents.reset();


    Serial.println(
        F("ESTOP")
    );
}


// ============================================================
// SERIAL
// ============================================================

void executeSerialLine(
    char *line
)
{
    while (
        *line == ' '
        ||
        *line == '\t'
    )
    {
        ++line;
    }


    if (
        *line == '\0'
    )
    {
        return;
    }


    for (
        char *p = line;
        *p;
        ++p
    )
    {
        *p =
            static_cast<char>(
                toupper(
                    static_cast<unsigned char>(
                        *p
                    )
                )
            );
    }


    // ========================================================
    // ACTIVE:
    // somente STOP / X
    // ========================================================

    if (
        runState ==
        RunState::ACTIVE
    )
    {
        if (
            strcmp(
                line,
                "STOP"
            )
            ==
            0
        )
        {
            finishExperiment(
                FinishReason::USER_STOP
            );

            return;
        }


        if (
            strcmp(
                line,
                "X"
            )
            ==
            0
        )
        {
            emergencyStop();

            return;
        }


        return;
    }


    // ========================================================
    // COMANDOS
    // ========================================================

    if (
        strcmp(
            line,
            "X"
        )
        ==
        0
    )
    {
        emergencyStop();

        return;
    }


    if (
        strcmp(
            line,
            "D"
        )
        ==
        0
    )
    {
        motor.stop();

        motor.disable();


        runState =
            RunState::IDLE;


        armReferenceDefined =
            false;


        pendulumEvents.reset();


        Serial.println(
            F("DISABLED")
        );


        return;
    }


    if (
        strcmp(
            line,
            "STOP"
        )
        ==
        0
    )
    {
        motor.stop();


        runState =
            RunState::IDLE;


        Serial.println(
            F("STOPPED")
        );


        return;
    }


    if (
        strcmp(
            line,
            "CFG"
        )
        ==
        0
    )
    {
        printConfig();

        return;
    }


    if (
        strcmp(
            line,
            "DEFAULT"
        )
        ==
        0
    )
    {
        loadDefaultConfig();


        printConfig();


        return;
    }


    if (
        strcmp(
            line,
            "Z"
        )
        ==
        0
    )
    {
        if (
            motor.isEnabled()
        )
        {
            Serial.println(
                F("DISABLE_FIRST")
            );

            return;
        }


        motor.setCurrentPosition(
            0.0F
        );


        armReferenceDefined =
            true;


        Serial.println(
            F("PHI0")
        );


        return;
    }


    if (
        strcmp(
            line,
            "E"
        )
        ==
        0
    )
    {
        motor.enable();


        Serial.println(
            F("ENABLED")
        );


        return;
    }


    if (
        strcmp(
            line,
            "T"
        )
        ==
        0
    )
    {
        startCalibration();

        return;
    }


    if (
        strcmp(
            line,
            "A"
        )
        ==
        0
    )
    {
        armTest();

        return;
    }


    // ========================================================
    // SET PARAMETRO VALOR
    // ========================================================

    char *savePtr =
        nullptr;


    char *command =
        strtok_r(
            line,
            " ",
            &savePtr
        );


    if (
        command != nullptr
        &&
        strcmp(
            command,
            "SET"
        )
        ==
        0
    )
    {
        char *parameter =
            strtok_r(
                nullptr,
                " ",
                &savePtr
            );


        char *value =
            strtok_r(
                nullptr,
                " ",
                &savePtr
            );


        if (
            parameter != nullptr
            &&
            value != nullptr
        )
        {
            setParameter(
                parameter,
                value
            );
        }


        return;
    }


    Serial.println(
        F("BAD_CMD")
    );
}


void processSerial()
{
    while (
        Serial.available()
    )
    {
        const char c =
            static_cast<char>(
                Serial.read()
            );


        if (
            c == '\r'
        )
        {
            continue;
        }


        if (
            c == '\n'
        )
        {
            serialBuffer[
                serialIndex
            ] =
                '\0';


            executeSerialLine(
                serialBuffer
            );


            serialIndex = 0;


            continue;
        }


        if (
            serialIndex <
            SERIAL_BUFFER_SIZE - 1
        )
        {
            serialBuffer[
                serialIndex++
            ] =
                c;
        }
        else
        {
            serialIndex = 0;
        }
    }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(
        FurutaConfig::SERIAL_BAUD
    );


    Wire.begin();


    Wire.setClock(
        400000UL
    );


    if (
        !pendulumPosition.begin()
    )
    {
        Serial.println(
            F("AS5600_ERR")
        );


        while (true)
        {
            delay(1000);
        }
    }


    pendulumPosition.update();


    const uint32_t nowUs =
        micros();


    pendulumVelocity.reset(
        0.0F,
        nowUs
    );


    motor.begin(
        FurutaConfig::MOTOR_MAX_SPEED_DEG_S,
        2
    );


    loadDefaultConfig();


    lastControlTimeUs =
        micros();


    nextControlTimeUs =
        lastControlTimeUs
        +
        FurutaConfig::CONTROL_PERIOD_US;


    Serial.println();


    Serial.println(
        F("FASE 15E.6")
    );


    Serial.println(
        F("Peak energy + EREF + directional arm PD")
    );


    printConfig();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    motor.update();


    processSerial();


    motor.update();


    const uint32_t nowUs =
        micros();


    if (
        static_cast<int32_t>(
            nowUs -
            nextControlTimeUs
        )
        >=
        0
    )
    {
        controlTick(
            nowUs
        );


        nextControlTimeUs +=
            FurutaConfig::CONTROL_PERIOD_US;


        if (
            static_cast<int32_t>(
                micros() -
                nextControlTimeUs
            )
            >=
            0
        )
        {
            nextControlTimeUs =
                micros()
                +
                FurutaConfig::CONTROL_PERIOD_US;
        }
    }


    motor.update();
}