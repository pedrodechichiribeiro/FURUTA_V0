#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>

#include <FurutaConfig.h>
#include <MotorPosition.h>
#include <PendulumPosition.h>
#include <PendulumVelocity.h>
#include <PendulumEvents.h>
#include <PendulumDownReference.h>

#include <math.h>


// ============================================================
// FASE 13D.1 - REFERENCIA CONTINUA DO BRACO
//
// Lei experimental:
//
//              alpha
// phiRef = A -----------
//            |alphaPeak|
//
// com:
//      A = 8 graus
//      phiRef limitado a +/-8 graus
//
// PEAK:
//      mede amplitude/energia e atualiza |alphaPeak|
//
// DOWN:
//      mede phi, phiRef e phiDot
//
// Nenhum movimento e disparado por PEAK ou DOWN.
// O alvo do motor e atualizado a cada 4 ms.
// ============================================================


// ============================================================
// CONFIGURACAO
// ============================================================

constexpr float SENSOR_DIRECTION_SIGN = 1.0F;

constexpr float PHI_AMPLITUDE_DEG = 8.0F;

constexpr uint8_t MAX_HALF_CYCLES = 4;

constexpr float MIN_SYNC_PEAK_DEG = 8.0F;
constexpr float MAX_PEAK_DEG = 60.0F;

constexpr uint32_t ACTIVE_TIMEOUT_US = 4000000UL;


// ============================================================
// OBJETOS
// ============================================================

AS5600 as5600;

PendulumPosition pendulumPosition(
    as5600,
    SENSOR_DIRECTION_SIGN
);

PendulumVelocity pendulumVelocity;

PendulumEvents pendulumEvents;

PendulumDownReference downReference(
    3000UL,
    20.0F
);

MotorPosition motor(
    FurutaConfig::STEP_PIN,
    FurutaConfig::DIR_PIN,
    FurutaConfig::ENABLE_PIN,

    FurutaConfig::FULL_STEPS_PER_REVOLUTION,
    FurutaConfig::MICROSTEP_FACTOR,

    FurutaConfig::ARM_MIN_DEG,
    FurutaConfig::ARM_MAX_DEG
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

    PREPOSITIONING,

    WAIT_SYNC_PEAK,

    ACTIVE,

    RETURNING_ZERO
};


enum class StartSide : uint8_t
{
    POSITIVE,
    NEGATIVE
};


enum class AbortReason : uint8_t
{
    NONE,

    SYNC_PEAK_TOO_SMALL,

    SYNC_PEAK_TOO_LARGE,

    UNEXPECTED_DOWN,

    UNEXPECTED_PEAK,

    PEAK_WITHOUT_DOWN,

    MOTOR_COMMAND_REJECTED,

    TIMEOUT
};


Mode mode = Mode::WAITING;

RunState runState = RunState::IDLE;

StartSide startSide = StartSide::POSITIVE;

AbortReason abortReason = AbortReason::NONE;


// ============================================================
// REFERENCIA DO BRACO
// ============================================================

bool armReferenceDefined = false;


// ============================================================
// TEMPORIZACAO
// ============================================================

uint32_t nextSampleUs = 0;

uint32_t experimentStartUs = 0;

uint32_t activeStartUs = 0;


// ============================================================
// BASELINE
// ============================================================

struct BaselineRecord
{
    bool valid;

    bool positive;

    uint32_t peakUs;
    uint32_t detectedUs;

    float angleDeg;
    float energy;
};


BaselineRecord baseline;


// ============================================================
// REGISTRO DE CADA MEIO CICLO
// ============================================================

struct HalfCycleRecord
{
    bool used;


    // --------------------------------------------------------
    // DOWN
    // --------------------------------------------------------

    bool downSeen;

    bool downPositive;

    uint32_t downUs;
    uint32_t downDetectedUs;

    float alphaDotDown;

    float phiDown;
    float phiRefDown;
    float phiDotDown;


    // --------------------------------------------------------
    // PEAK FINAL
    // --------------------------------------------------------

    bool peakSeen;

    bool peakPositive;

    uint32_t peakUs;
    uint32_t peakDetectedUs;

    float peakAngleDeg;

    float energy;

    float deltaEnergy;

    float ratio;
};


HalfCycleRecord records[MAX_HALF_CYCLES];


// ============================================================
// ESTADO DA LEI CONTINUA
// ============================================================

float peakAmplitudeRad = 0.0F;

float previousPeakEnergy = 0.0F;

float currentPhiRefDeg = 0.0F;


uint8_t halfCycleCount = 0;


bool expectedDownPositive = false;

bool expectedPeakPositive = false;


// ============================================================
// ESTIMATIVA DA VELOCIDADE DO BRACO
// ============================================================

bool phiVelocityReady = false;

uint32_t previousPhiTimeUs = 0;

float previousPhiDeg = 0.0F;

float phiDotDegS = 0.0F;


// ============================================================
// FINALIZACAO
// ============================================================

bool runFinishedNormally = false;


// ============================================================
// FUNCOES BASICAS
// ============================================================

float peakEnergy(
    float alphaRad
)
{
    return 0.5F *
           (
               1.0F -
               cosf(alphaRad)
           );
}


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
// CALCULO DA REFERENCIA CONTINUA
// ============================================================

float computePhiReference(
    float alphaRad
)
{
    if (
        peakAmplitudeRad <=
        0.001F
    )
    {
        return 0.0F;
    }


    float reference =
        PHI_AMPLITUDE_DEG *
        alphaRad /
        peakAmplitudeRad;


    return clampFloat(
        reference,
        -PHI_AMPLITUDE_DEG,
        +PHI_AMPLITUDE_DEG
    );
}


// ============================================================
// ESTIMATIVA DE phiDot
// ============================================================

void updatePhiVelocity(
    uint32_t nowUs
)
{
    const float phi =
        motor.currentPosition();


    if (!phiVelocityReady)
    {
        previousPhiDeg =
            phi;

        previousPhiTimeUs =
            nowUs;

        phiDotDegS =
            0.0F;

        phiVelocityReady =
            true;

        return;
    }


    const uint32_t dtUs =
        nowUs -
        previousPhiTimeUs;


    if (dtUs == 0)
    {
        return;
    }


    const float dt =
        dtUs *
        1.0e-6F;


    phiDotDegS =
        (
            phi -
            previousPhiDeg
        )
        /
        dt;


    previousPhiDeg =
        phi;


    previousPhiTimeUs =
        nowUs;
}


// ============================================================
// RESET DO ENSAIO
// ============================================================

void resetExperiment()
{
    baseline = {};

    for (
        uint8_t i = 0;
        i < MAX_HALF_CYCLES;
        ++i
    )
    {
        records[i] = {};
    }


    halfCycleCount = 0;


    peakAmplitudeRad =
        0.0F;


    previousPeakEnergy =
        0.0F;


    currentPhiRefDeg =
        0.0F;


    expectedDownPositive =
        false;


    expectedPeakPositive =
        false;


    abortReason =
        AbortReason::NONE;


    runFinishedNormally =
        false;


    phiVelocityReady =
        false;


    pendulumEvents.reset();
}


// ============================================================
// TEXTO DO ABORT
// ============================================================

void printAbortReason()
{
    switch (abortReason)
    {
        case AbortReason::SYNC_PEAK_TOO_SMALL:

            Serial.print(
                F("sync_peak_too_small")
            );

            break;


        case AbortReason::SYNC_PEAK_TOO_LARGE:

            Serial.print(
                F("sync_peak_too_large")
            );

            break;


        case AbortReason::UNEXPECTED_DOWN:

            Serial.print(
                F("unexpected_down")
            );

            break;


        case AbortReason::UNEXPECTED_PEAK:

            Serial.print(
                F("unexpected_peak")
            );

            break;


        case AbortReason::PEAK_WITHOUT_DOWN:

            Serial.print(
                F("peak_without_down")
            );

            break;


        case AbortReason::MOTOR_COMMAND_REJECTED:

            Serial.print(
                F("motor_command_rejected")
            );

            break;


        case AbortReason::TIMEOUT:

            Serial.print(
                F("timeout")
            );

            break;


        case AbortReason::NONE:
        default:

            Serial.print(
                F("none")
            );

            break;
    }
}


// ============================================================
// RETORNO AO ZERO
// ============================================================

void startReturnToZero()
{
    motor.moveTo(
        0.0F
    );


    runState =
        RunState::RETURNING_ZERO;
}


// ============================================================
// ABORT
// ============================================================

void abortExperiment(
    AbortReason reason
)
{
    if (
        runState !=
        RunState::ACTIVE
        &&
        runState !=
        RunState::WAIT_SYNC_PEAK
    )
    {
        return;
    }


    abortReason =
        reason;


    runFinishedNormally =
        false;


    startReturnToZero();
}


// ============================================================
// FINAL NORMAL
// ============================================================

void finishExperiment()
{
    runFinishedNormally =
        true;


    startReturnToZero();
}


// ============================================================
// LOG FINAL
// ============================================================

void printResult()
{
    Serial.println();

    Serial.println(
        F("# ===============================")
    );


    Serial.println(
        F("# FASE13D1_RESULT")
    );


    Serial.println(
        F("# LAW=phiRef=8deg*alpha/abs(lastPeak)")
    );


    // ========================================================
    // BASELINE
    // ========================================================

    if (baseline.valid)
    {
        Serial.print(
            F("# BASELINE,")
        );


        Serial.print(
            baseline.positive
                ?
                F("PEAK+")
                :
                F("PEAK-")
        );


        Serial.print(
            F(",t=")
        );


        Serial.print(
            (
                baseline.peakUs -
                experimentStartUs
            )
            /
            1000UL
        );


        Serial.print(
            F(",angle=")
        );


        Serial.print(
            baseline.angleDeg,
            2
        );


        Serial.print(
            F(",energy=")
        );


        Serial.print(
            baseline.energy,
            6
        );


        Serial.print(
            F(",detection_delay_us=")
        );


        Serial.println(
            baseline.detectedUs -
            baseline.peakUs
        );
    }


    // ========================================================
    // MEIOS CICLOS
    // ========================================================

    for (
        uint8_t i = 0;
        i < halfCycleCount;
        ++i
    )
    {
        HalfCycleRecord &r =
            records[i];


        if (!r.used)
        {
            continue;
        }


        // ----------------------------------------------------
        // DOWN
        // ----------------------------------------------------

        if (r.downSeen)
        {
            Serial.print(
                F("# HALF,")
            );


            Serial.print(
                i + 1
            );


            Serial.print(
                F(",")
            );


            Serial.print(
                r.downPositive
                    ?
                    F("DOWN+")
                    :
                    F("DOWN-")
            );


            Serial.print(
                F(",t=")
            );


            Serial.print(
                (
                    r.downUs -
                    experimentStartUs
                )
                /
                1000UL
            );


            Serial.print(
                F(",alphaDot=")
            );


            Serial.print(
                r.alphaDotDown,
                4
            );


            Serial.print(
                F(",phi=")
            );


            Serial.print(
                r.phiDown,
                2
            );


            Serial.print(
                F(",phiRef=")
            );


            Serial.print(
                r.phiRefDown,
                2
            );


            Serial.print(
                F(",phiDot=")
            );


            Serial.print(
                r.phiDotDown,
                1
            );


            Serial.print(
                F(",event_delay_us=")
            );


            Serial.println(
                r.downDetectedUs -
                r.downUs
            );
        }


        // ----------------------------------------------------
        // PEAK
        // ----------------------------------------------------

        if (r.peakSeen)
        {
            Serial.print(
                F("# PEAK,")
            );


            Serial.print(
                i + 1
            );


            Serial.print(
                F(",")
            );


            Serial.print(
                r.peakPositive
                    ?
                    F("PEAK+")
                    :
                    F("PEAK-")
            );


            Serial.print(
                F(",t=")
            );


            Serial.print(
                (
                    r.peakUs -
                    experimentStartUs
                )
                /
                1000UL
            );


            Serial.print(
                F(",angle=")
            );


            Serial.print(
                r.peakAngleDeg,
                2
            );


            Serial.print(
                F(",energy=")
            );


            Serial.print(
                r.energy,
                6
            );


            Serial.print(
                F(",dE=")
            );


            Serial.print(
                r.deltaEnergy,
                6
            );


            Serial.print(
                F(",ratio=")
            );


            Serial.print(
                r.ratio,
                4
            );


            Serial.print(
                F(",detection_delay_us=")
            );


            Serial.println(
                r.peakDetectedUs -
                r.peakUs
            );
        }
    }


    // ========================================================
    // FINAL
    // ========================================================

    if (runFinishedNormally)
    {
        Serial.println(
            F("# RESULT=OK")
        );
    }
    else
    {
        Serial.print(
            F("# RESULT=ABORT,reason=")
        );


        printAbortReason();


        Serial.println();
    }


    Serial.print(
        F("# half_cycles=")
    );


    Serial.println(
        halfCycleCount
    );


    Serial.println(
        F("# ===============================")
    );


    Serial.println(
        F("A = iniciar por PEAK+ | B = iniciar por PEAK-")
    );
}


// ============================================================
// PEAK
// ============================================================

void handlePeak(
    bool positive,
    const PendulumEvent &event
)
{
    const uint32_t detectedUs =
        micros();


    const float angleDeg =
        event.alphaRad *
        RAD_TO_DEG;


    const float absAngleDeg =
        fabsf(angleDeg);


    // ========================================================
    // PEAK DE SINCRONIZACAO
    // ========================================================

    if (
        runState ==
        RunState::WAIT_SYNC_PEAK
    )
    {
        const bool wantedPositive =
            startSide ==
            StartSide::POSITIVE;


        if (
            positive !=
            wantedPositive
        )
        {
            return;
        }


        if (
            absAngleDeg <
            MIN_SYNC_PEAK_DEG
        )
        {
            abortExperiment(
                AbortReason::SYNC_PEAK_TOO_SMALL
            );

            return;
        }


        if (
            absAngleDeg >
            MAX_PEAK_DEG
        )
        {
            abortExperiment(
                AbortReason::SYNC_PEAK_TOO_LARGE
            );

            return;
        }


        baseline.valid =
            true;


        baseline.positive =
            positive;


        baseline.peakUs =
            event.timeUs;


        baseline.detectedUs =
            detectedUs;


        baseline.angleDeg =
            angleDeg;


        baseline.energy =
            peakEnergy(
                event.alphaRad
            );


        peakAmplitudeRad =
            fabsf(
                event.alphaRad
            );


        previousPeakEnergy =
            baseline.energy;


        // PEAK+ -> DOWN- -> PEAK-
        // PEAK- -> DOWN+ -> PEAK+
        expectedDownPositive =
            !positive;


        expectedPeakPositive =
            !positive;


        activeStartUs =
            micros();


        phiVelocityReady =
            false;


        runState =
            RunState::ACTIVE;


        return;
    }


    // ========================================================
    // PEAK DURANTE CONTROLE
    // ========================================================

    if (
        runState !=
        RunState::ACTIVE
    )
    {
        return;
    }


    if (
        positive !=
        expectedPeakPositive
    )
    {
        abortExperiment(
            AbortReason::UNEXPECTED_PEAK
        );

        return;
    }


    if (
        halfCycleCount >=
        MAX_HALF_CYCLES
    )
    {
        finishExperiment();

        return;
    }


    HalfCycleRecord &r =
        records[
            halfCycleCount
        ];


    r.used =
        true;


    if (!r.downSeen)
    {
        abortExperiment(
            AbortReason::PEAK_WITHOUT_DOWN
        );

        return;
    }


    const float energy =
        peakEnergy(
            event.alphaRad
        );


    r.peakSeen =
        true;


    r.peakPositive =
        positive;


    r.peakUs =
        event.timeUs;


    r.peakDetectedUs =
        detectedUs;


    r.peakAngleDeg =
        angleDeg;


    r.energy =
        energy;


    r.deltaEnergy =
        energy -
        previousPeakEnergy;


    r.ratio =
        previousPeakEnergy >
        0.0F
            ?
            energy /
            previousPeakEnergy
            :
            0.0F;


    previousPeakEnergy =
        energy;


    // ========================================================
    // NOVA AMPLITUDE DE NORMALIZACAO
    // ========================================================

    peakAmplitudeRad =
        fabsf(
            event.alphaRad
        );


    ++halfCycleCount;


    // ========================================================
    // LIMITE EXPERIMENTAL
    // ========================================================

    if (
        halfCycleCount >=
        MAX_HALF_CYCLES
        ||
        absAngleDeg >=
        MAX_PEAK_DEG
    )
    {
        finishExperiment();

        return;
    }


    // Proximo meio ciclo.
    expectedDownPositive =
        !positive;


    expectedPeakPositive =
        !positive;
}


// ============================================================
// DOWN
// ============================================================

void handleDown(
    bool positive,
    const PendulumEvent &event
)
{
    if (
        runState !=
        RunState::ACTIVE
    )
    {
        return;
    }


    if (
        positive !=
        expectedDownPositive
    )
    {
        abortExperiment(
            AbortReason::UNEXPECTED_DOWN
        );

        return;
    }


    if (
        halfCycleCount >=
        MAX_HALF_CYCLES
    )
    {
        return;
    }


    HalfCycleRecord &r =
        records[
            halfCycleCount
        ];


    if (r.downSeen)
    {
        return;
    }


    r.used =
        true;


    r.downSeen =
        true;


    r.downPositive =
        positive;


    r.downUs =
        event.timeUs;


    r.downDetectedUs =
        micros();


    r.alphaDotDown =
        event.alphaDotRadS;


    // Posicao inferida pela contagem de passos.
    r.phiDown =
        motor.currentPosition();


    r.phiRefDown =
        currentPhiRefDeg;


    r.phiDotDown =
        phiDotDegS;
}


// ============================================================
// EVENTOS
// ============================================================

void handleEvent(
    const PendulumEvent &event
)
{
    switch (event.type)
    {
        case PendulumEventType::PEAK_POSITIVE:

            handlePeak(
                true,
                event
            );

            break;


        case PendulumEventType::PEAK_NEGATIVE:

            handlePeak(
                false,
                event
            );

            break;


        case PendulumEventType::DOWN_POSITIVE:

            handleDown(
                true,
                event
            );

            break;


        case PendulumEventType::DOWN_NEGATIVE:

            handleDown(
                false,
                event
            );

            break;


        case PendulumEventType::NONE:
        default:

            break;
    }
}


// ============================================================
// ATUALIZACAO DO MOTOR / MAQUINA DE ESTADOS
// ============================================================

void updateMotorState()
{
    // ========================================================
    // PREPOSICIONAMENTO
    // ========================================================

    if (
        runState ==
        RunState::PREPOSITIONING
    )
    {
        if (motor.isMoving())
        {
            return;
        }


        pendulumEvents.reset();


        experimentStartUs =
            micros();


        runState =
            RunState::WAIT_SYNC_PEAK;


        Serial.println();


        Serial.print(
            F("# PREP_DONE,phi=")
        );


        Serial.println(
            motor.currentPosition(),
            2
        );


        Serial.println(
            startSide ==
            StartSide::POSITIVE
                ?
                F("Aguardando PEAK+ para iniciar controle continuo.")
                :
                F("Aguardando PEAK- para iniciar controle continuo.")
        );


        return;
    }


    // ========================================================
    // RETORNO FINAL
    // ========================================================

    if (
        runState ==
        RunState::RETURNING_ZERO
    )
    {
        if (motor.isMoving())
        {
            return;
        }


        printResult();


        runState =
            RunState::IDLE;


        pendulumEvents.reset();
    }
}


// ============================================================
// CALIBRACAO
// ============================================================

void startCalibration()
{
    Serial.println();


    if (
        !pendulumPosition.calibrateTop(
            32,
            2
        )
    )
    {
        Serial.println(
            F("ERRO na leitura inicial.")
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


    nextSampleUs =
        nowUs +
        FurutaConfig::CONTROL_PERIOD_US;


    mode =
        Mode::CALIBRATING;


    runState =
        RunState::IDLE;


    pendulumEvents.reset();


    Serial.println(
        F("Calibrando centro de DOWN por 3 s...")
    );
}


// ============================================================
// ATUALIZA CALIBRACAO
// ============================================================

void updateCalibration(
    uint32_t nowUs
)
{
    if (
        !pendulumPosition.update()
    )
    {
        return;
    }


    const bool finished =
        downReference.update(
            pendulumPosition.betaRadians(),
            nowUs
        );


    if (!finished)
    {
        return;
    }


    if (
        !downReference.isReady()
    )
    {
        Serial.print(
            F("CALIBRACAO REJEITADA. Span=")
        );


        Serial.print(
            downReference.observedSpanDeg(),
            2
        );


        Serial.println(
            F(" deg")
        );


        mode =
            Mode::WAITING;


        return;
    }


    Serial.print(
        F("DOWN estimado = ")
    );


    Serial.print(
        downReference.offsetDeg(),
        3
    );


    Serial.println(
        F(" deg")
    );


    Serial.print(
        F("Span observado = ")
    );


    Serial.print(
        downReference.observedSpanDeg(),
        2
    );


    Serial.println(
        F(" deg")
    );


    const float alpha =
        downReference.correctedAngleRad(
            pendulumPosition.betaRadians()
        );


    pendulumVelocity.reset(
        alpha,
        nowUs
    );


    pendulumEvents.reset();


    nextSampleUs =
        nowUs +
        FurutaConfig::CONTROL_PERIOD_US;


    mode =
        Mode::READY;


    Serial.println(
        F("DOWN definido.")
    );


    Serial.println(
        F("A = iniciar por PEAK+")
    );


    Serial.println(
        F("B = iniciar por PEAK-")
    );
}


// ============================================================
// AQUISICAO E CONTROLE A 250 Hz
// ============================================================

void acquireAndControl(
    uint32_t nowUs
)
{
    if (
        !pendulumPosition.update()
    )
    {
        return;
    }


    const float alpha =
        downReference.correctedAngleRad(
            pendulumPosition.betaRadians()
        );


    pendulumVelocity.update(
        alpha,
        nowUs
    );


    updatePhiVelocity(
        nowUs
    );


    if (
        !pendulumVelocity.isReady()
    )
    {
        return;
    }


    // ========================================================
    // CONTROLE CONTINUO
    //
    // Esta e a mudanca principal da Fase 13D.
    //
    // Nao esperamos PEAK.
    // Nao esperamos DOWN.
    //
    // A cada amostra:
    //
    // alpha -> phiRef -> novo alvo do stepper
    // ========================================================

    if (
        runState ==
        RunState::ACTIVE
    )
    {
        if (
            (
                nowUs -
                activeStartUs
            )
            >
            ACTIVE_TIMEOUT_US
        )
        {
            abortExperiment(
                AbortReason::TIMEOUT
            );

            return;
        }


        currentPhiRefDeg =
            computePhiReference(
                alpha
            );


        if (
            !motor.moveTo(
                currentPhiRefDeg
            )
        )
        {
            abortExperiment(
                AbortReason::MOTOR_COMMAND_REJECTED
            );

            return;
        }
    }


    // ========================================================
    // EVENTOS
    //
    // Servem para MEDIR a fase/energia.
    // Nao disparam o movimento.
    // ========================================================

    if (
        runState !=
            RunState::WAIT_SYNC_PEAK
        &&
        runState !=
            RunState::ACTIVE
    )
    {
        return;
    }


    const PendulumEvent event =
        pendulumEvents.update(
            alpha,
            pendulumVelocity.radiansPerSecond(),
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


        // Se o PEAK de sincronizacao acabou de ativar
        // o controlador, ja aplicamos a referencia usando
        // o alpha ATUAL, e nao o alpha antigo do evento.
        if (
            runState ==
            RunState::ACTIVE
        )
        {
            currentPhiRefDeg =
                computePhiReference(
                    alpha
                );


            if (
                !motor.moveTo(
                    currentPhiRefDeg
                )
            )
            {
                abortExperiment(
                    AbortReason::MOTOR_COMMAND_REJECTED
                );
            }
        }
    }
}


// ============================================================
// INICIO DO ENSAIO
// ============================================================

void startTest(
    StartSide side
)
{
    if (
        mode !=
        Mode::READY
    )
    {
        Serial.println(
            F("Calibre DOWN com T.")
        );

        return;
    }


    if (
        !armReferenceDefined
    )
    {
        Serial.println(
            F("Defina phi=0 com Z.")
        );

        return;
    }


    if (
        !motor.isEnabled()
    )
    {
        Serial.println(
            F("Habilite motor com E.")
        );

        return;
    }


    if (
        runState !=
            RunState::IDLE
        ||
        motor.isMoving()
    )
    {
        Serial.println(
            F("Sistema ocupado.")
        );

        return;
    }


    if (
        fabsf(
            motor.currentPosition()
        )
        >
        0.5F
    )
    {
        Serial.println(
            F("Braco precisa estar em phi=0.")
        );

        return;
    }


    resetExperiment();


    startSide =
        side;


    const float initialPhi =
        side ==
        StartSide::POSITIVE
            ?
            +PHI_AMPLITUDE_DEG
            :
            -PHI_AMPLITUDE_DEG;


    if (
        !motor.moveTo(
            initialPhi
        )
    )
    {
        Serial.println(
            F("Erro no preposicionamento.")
        );

        return;
    }


    runState =
        RunState::PREPOSITIONING;


    Serial.println();


    Serial.println(
        F("# PHASE13D1_ARMED")
    );


    Serial.print(
        F("# PREPOSITION,target=")
    );


    Serial.println(
        initialPhi,
        2
    );
}


// ============================================================
// STATUS
// ============================================================

void printStatus()
{
    Serial.println();


    Serial.print(
        F("phi=")
    );


    Serial.println(
        motor.currentPosition(),
        2
    );


    Serial.print(
        F("phiRef=")
    );


    Serial.println(
        currentPhiRefDeg,
        2
    );


    Serial.print(
        F("phiDot=")
    );


    Serial.println(
        phiDotDegS,
        1
    );


    Serial.print(
        F("motor=")
    );


    Serial.println(
        motor.isEnabled()
            ?
            F("ENABLED")
            :
            F("DISABLED")
    );
}


// ============================================================
// SERIAL
// ============================================================

void processSerial()
{
    if (!Serial.available())
    {
        return;
    }


    const char c =
        Serial.read();


    // ========================================================
    // D = desabilita sempre
    // ========================================================

    if (
        c == 'D' ||
        c == 'd'
    )
    {
        motor.stop();

        motor.disable();


        armReferenceDefined =
            false;


        runState =
            RunState::IDLE;


        pendulumEvents.reset();


        Serial.println(
            F("Motor desabilitado. Referencia perdida.")
        );


        return;
    }


    // Demais comandos somente fora do ensaio.
    if (
        runState !=
        RunState::IDLE
    )
    {
        return;
    }


    // ========================================================
    // Z
    // ========================================================

    if (
        c == 'Z' ||
        c == 'z'
    )
    {
        motor.setCurrentPosition(
            0.0F
        );


        armReferenceDefined =
            true;


        Serial.println(
            F("Braco: phi=0 definido.")
        );


        return;
    }


    // ========================================================
    // E
    // ========================================================

    if (
        c == 'E' ||
        c == 'e'
    )
    {
        motor.enable();


        Serial.println(
            F("Motor habilitado.")
        );


        return;
    }


    // ========================================================
    // T
    // ========================================================

    if (
        c == 'T' ||
        c == 't'
    )
    {
        startCalibration();

        return;
    }


    // ========================================================
    // A = inicia sincronizado pelo lado positivo
    // ========================================================

    if (
        c == 'A' ||
        c == 'a'
    )
    {
        startTest(
            StartSide::POSITIVE
        );

        return;
    }


    // ========================================================
    // B = inicia sincronizado pelo lado negativo
    // ========================================================

    if (
        c == 'B' ||
        c == 'b'
    )
    {
        startTest(
            StartSide::NEGATIVE
        );

        return;
    }


    // ========================================================
    // S
    // ========================================================

    if (
        c == 'S' ||
        c == 's'
    )
    {
        printStatus();

        return;
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


    motor.beginDegrees(
        FurutaConfig::MOTOR_MAX_SPEED_DEG_S,
        FurutaConfig::MAX_ACCEL_DEG_S2
    );


    if (
        !pendulumPosition.begin()
    )
    {
        Serial.println(
            F("ERRO: AS5600 nao encontrado.")
        );


        while (true)
        {
            delay(1000);
        }
    }


    Serial.println();


    Serial.println(
        F("FASE 13D.1 - REFERENCIA CONTINUA")
    );


    Serial.println(
        F("Z = definir phi=0")
    );


    Serial.println(
        F("E = habilitar motor")
    );


    Serial.println(
        F("D = desabilitar motor")
    );


    Serial.println(
        F("T = calibrar DOWN")
    );


    Serial.println(
        F("A = iniciar por PEAK+")
    );


    Serial.println(
        F("B = iniciar por PEAK-")
    );


    Serial.println(
        F("S = status")
    );
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    // A geracao de passos precisa rodar continuamente.
    motor.update();


    updateMotorState();


    processSerial();


    if (
        mode ==
        Mode::WAITING
    )
    {
        return;
    }


    const uint32_t nowUs =
        micros();


    if (
        static_cast<int32_t>(
            nowUs -
            nextSampleUs
        )
        <
        0
    )
    {
        return;
    }


    // Nao executa catch-up de amostras atrasadas.
    nextSampleUs =
        nowUs +
        FurutaConfig::CONTROL_PERIOD_US;


    if (
        mode ==
        Mode::CALIBRATING
    )
    {
        updateCalibration(
            nowUs
        );
    }
    else if (
        mode ==
        Mode::READY
    )
    {
        acquireAndControl(
            nowUs
        );
    }
}