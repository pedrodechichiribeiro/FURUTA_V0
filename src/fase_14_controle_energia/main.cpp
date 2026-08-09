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
// FASE 14C.1
// BOMBEAMENTO BIDIRECIONAL ASSIMETRICO - EXTENSAO A 12 MEIOS CICLOS
//
// LADO ADAPTATIVO:
//
//   PEAK- -> DOWN+ -> PEAK+
//
//   A_PLUS(E) = 8 - 4*(E/0.25)
//   limitada entre 4 e 8 graus.
//
// LADO FIXO:
//
//   PEAK+ -> DOWN- -> PEAK-
//
//   A_MINUS = 8 graus.
//
// Lei continua:
//
//                  alpha
//   phiRef = A * -----------
//               |alphaPeak|
//
// Atualizada a cada 4 ms.
//
// NOVO NA 14C.1:
//
//   - ate 12 meios ciclos;
//   - mede dE do ciclo completo PEAK- -> PEAK-.
//
// LIMITES MANTIDOS:
//
//   E_STOP       = 0.25
//   PEAK_MAX     = 60 graus
//   velocidade   = FurutaConfig::MOTOR_MAX_SPEED_DEG_S
//   aceleracao   = FurutaConfig::MAX_ACCEL_DEG_S2
//
// ============================================================


// ============================================================
// CONFIGURACAO
// ============================================================

constexpr float SENSOR_DIRECTION_SIGN = 1.0F;


// Lado PEAK- -> PEAK+
constexpr float ADAPTIVE_AMPLITUDE_MAX_DEG = 8.0F;
constexpr float ADAPTIVE_AMPLITUDE_MIN_DEG = 4.0F;


// Lado PEAK+ -> PEAK-
constexpr float FIXED_AMPLITUDE_DEG = 8.0F;


// Energia experimental
constexpr float ENERGY_STOP = 0.25F;


// Limites
constexpr float MIN_SYNC_PEAK_DEG = 8.0F;
constexpr float MAX_PEAK_DEG = 60.0F;


// Mudanca principal da 14C.1
constexpr uint8_t MAX_HALF_CYCLES = 12;


// 12 meios ciclos sao cerca de 6 s.
// Mantemos pequena margem adicional.
constexpr uint32_t ACTIVE_TIMEOUT_US = 8000000UL;


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
    WAIT_SYNC_NEGATIVE_PEAK,
    ACTIVE,
    RETURNING_ZERO
};


enum class FinishReason : uint8_t
{
    NONE,

    ENERGY_REACHED,
    ANGLE_REACHED,
    MAX_HALF_CYCLES,
    TIMEOUT,
    EVENT_ERROR,
    MOTOR_ERROR
};


Mode mode =
    Mode::WAITING;


RunState runState =
    RunState::IDLE;


FinishReason finishReason =
    FinishReason::NONE;


// ============================================================
// REFERENCIA DO BRACO
// ============================================================

bool armReferenceDefined =
    false;


// ============================================================
// TEMPO
// ============================================================

uint32_t nextSampleUs =
    0;


uint32_t experimentStartUs =
    0;


uint32_t activeStartUs =
    0;


// ============================================================
// ESTADO DA LEI CONTINUA
// ============================================================

float peakAmplitudeRad =
    0.0F;


float currentAmplitudeDeg =
    ADAPTIVE_AMPLITUDE_MAX_DEG;


float currentPhiRefDeg =
    0.0F;


float previousPeakEnergy =
    0.0F;


uint8_t halfCycleCount =
    0;


bool expectedDownPositive =
    true;


bool expectedPeakPositive =
    true;


// ============================================================
// ENERGIA DE CICLO COMPLETO
//
// Como iniciamos em PEAK-, usamos PEAK- como referencia
// de energia de volta completa.
// ============================================================

float previousNegativePeakEnergy =
    0.0F;


bool previousNegativePeakEnergyValid =
    false;


uint8_t fullCycleCount =
    0;


// ============================================================
// BASELINE
// ============================================================

struct BaselineRecord
{
    bool valid;

    uint32_t peakUs;
    uint32_t detectedUs;

    float angleDeg;
    float energy;

    float firstAmplitudeDeg;
};


BaselineRecord baseline;


// ============================================================
// REGISTRO POR MEIO CICLO
// ============================================================

struct HalfCycleRecord
{
    bool used;

    bool adaptiveSide;

    float amplitudeUsedDeg;
    float amplitudeNextDeg;


    // DOWN

    bool downSeen;

    bool downPositive;

    uint32_t downUs;
    uint32_t downDetectedUs;

    float alphaDotDown;

    float phiDown;
    float phiRefDown;


    // PEAK

    bool peakSeen;

    bool peakPositive;

    uint32_t peakUs;
    uint32_t peakDetectedUs;

    float peakAngleDeg;

    float energy;
    float deltaEnergy;
    float ratio;


    // Ciclo completo
    bool cycleDeltaValid;

    float cycleDeltaEnergy;
};


HalfCycleRecord records[MAX_HALF_CYCLES];


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
// ENERGIA NORMALIZADA
//
// DOWN = 0
// TOP  = 1
//
// No pico:
// E = 0.5*(1-cos(alpha))
// ============================================================

float peakEnergy(
    float alphaRad
)
{
    return
        0.5F *
        (
            1.0F -
            cosf(alphaRad)
        );
}


// ============================================================
// AMPLITUDE ADAPTATIVA
//
// PEAK- -> DOWN+ -> PEAK+
//
// A(E) = 8 - 4*(E/0.25)
//
// limitada a 4...8 graus.
// ============================================================

float adaptiveAmplitudeFromEnergy(
    float energy
)
{
    const float normalizedEnergy =
        clampFloat(
            energy / ENERGY_STOP,
            0.0F,
            1.0F
        );


    const float amplitude =
        ADAPTIVE_AMPLITUDE_MAX_DEG
        -
        (
            ADAPTIVE_AMPLITUDE_MAX_DEG -
            ADAPTIVE_AMPLITUDE_MIN_DEG
        )
        *
        normalizedEnergy;


    return clampFloat(
        amplitude,
        ADAPTIVE_AMPLITUDE_MIN_DEG,
        ADAPTIVE_AMPLITUDE_MAX_DEG
    );
}


// ============================================================
// DEFINE A DO PROXIMO MEIO CICLO
//
// PEAK+:
//   proximo sera DOWN- -> PEAK-
//   usa 8 graus.
//
// PEAK-:
//   proximo sera DOWN+ -> PEAK+
//   usa A(E).
// ============================================================

float amplitudeAfterPeak(
    bool peakPositive,
    float energy
)
{
    if (peakPositive)
    {
        return FIXED_AMPLITUDE_DEG;
    }


    return adaptiveAmplitudeFromEnergy(
        energy
    );
}


// ============================================================
// REFERENCIA CONTINUA
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


    const float reference =
        currentAmplitudeDeg
        *
        alphaRad
        /
        peakAmplitudeRad;


    return clampFloat(
        reference,
        -currentAmplitudeDeg,
        +currentAmplitudeDeg
    );
}


// ============================================================
// RESET
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


    peakAmplitudeRad =
        0.0F;


    currentAmplitudeDeg =
        ADAPTIVE_AMPLITUDE_MAX_DEG;


    currentPhiRefDeg =
        0.0F;


    previousPeakEnergy =
        0.0F;


    halfCycleCount =
        0;


    expectedDownPositive =
        true;


    expectedPeakPositive =
        true;


    previousNegativePeakEnergy =
        0.0F;


    previousNegativePeakEnergyValid =
        false;


    fullCycleCount =
        0;


    finishReason =
        FinishReason::NONE;


    pendulumEvents.reset();
}


// ============================================================
// RESULTADO TEXTUAL
// ============================================================

void printFinishReason()
{
    switch (finishReason)
    {
        case FinishReason::ENERGY_REACHED:

            Serial.print(
                F("energy_reached")
            );

            break;


        case FinishReason::ANGLE_REACHED:

            Serial.print(
                F("angle_reached")
            );

            break;


        case FinishReason::MAX_HALF_CYCLES:

            Serial.print(
                F("max_half_cycles")
            );

            break;


        case FinishReason::TIMEOUT:

            Serial.print(
                F("timeout")
            );

            break;


        case FinishReason::EVENT_ERROR:

            Serial.print(
                F("event_error")
            );

            break;


        case FinishReason::MOTOR_ERROR:

            Serial.print(
                F("motor_error")
            );

            break;


        case FinishReason::NONE:
        default:

            Serial.print(
                F("none")
            );

            break;
    }
}


// ============================================================
// FINALIZA EXPERIMENTO
// ============================================================

void finishExperiment(
    FinishReason reason
)
{
    finishReason =
        reason;


    if (
        !motor.moveTo(0.0F)
    )
    {
        finishReason =
            FinishReason::MOTOR_ERROR;


        runState =
            RunState::IDLE;


        return;
    }


    runState =
        RunState::RETURNING_ZERO;
}


// ============================================================
// IMPRESSAO FINAL
// ============================================================

void printResult()
{
    Serial.println();


    Serial.println(
        F("# ===============================")
    );


    Serial.println(
        F("# FASE14C1_RESULT")
    );


    Serial.println(
        F("# STRATEGY=BIDIRECTIONAL_ASYMMETRIC")
    );


    Serial.println(
        F("# ADAPTIVE=PEAK-_DOWN+_PEAK+")
    );


    Serial.println(
        F("# FIXED=PEAK+_DOWN-_PEAK-")
    );


    Serial.println(
        F("# A_ADAPTIVE=8-4*(E/0.25),clamp[4,8]")
    );


    Serial.println(
        F("# A_FIXED=8.00")
    );


    Serial.println(
        F("# MAX_HALF_CYCLES=12")
    );


    Serial.println(
        F("# LAW=phiRef=A*alpha/abs(lastPeak)")
    );


    // ========================================================
    // BASELINE
    // ========================================================

    if (baseline.valid)
    {
        Serial.print(
            F("# BASELINE,PEAK-,t=")
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
            F(",A_first=")
        );


        Serial.print(
            baseline.firstAmplitudeDeg,
            2
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


        Serial.println();


        Serial.print(
            F("# HALF,")
        );


        Serial.print(
            i + 1
        );


        Serial.print(
            F(",side=")
        );


        Serial.print(
            r.adaptiveSide
                ?
                F("ADAPTIVE_TO_PEAK+")
                :
                F("FIXED_TO_PEAK-")
        );


        Serial.print(
            F(",A=")
        );


        Serial.println(
            r.amplitudeUsedDeg,
            2
        );


        // ----------------------------------------------------
        // DOWN
        // ----------------------------------------------------

        if (r.downSeen)
        {
            Serial.print(
                F("# ")
            );


            Serial.print(
                r.downPositive
                    ?
                    F("DOWN+")
                    :
                    F("DOWN-")
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
                F("# ")
            );


            Serial.print(
                r.peakPositive
                    ?
                    F("PEAK+")
                    :
                    F("PEAK-")
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
                F(",dE_half=")
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
                F(",A_used=")
            );


            Serial.print(
                r.amplitudeUsedDeg,
                2
            );


            Serial.print(
                F(",A_next=")
            );


            Serial.print(
                r.amplitudeNextDeg,
                2
            );


            // =================================================
            // NOVO: ganho de uma volta completa
            // =================================================

            if (r.cycleDeltaValid)
            {
                Serial.print(
                    F(",dE_cycle=")
                );


                Serial.print(
                    r.cycleDeltaEnergy,
                    6
                );
            }


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

    Serial.print(
        F("# RESULT=")
    );


    printFinishReason();


    Serial.println();


    Serial.print(
        F("# half_cycles=")
    );


    Serial.println(
        halfCycleCount
    );


    Serial.print(
        F("# full_cycles=")
    );


    Serial.println(
        fullCycleCount
    );


    if (baseline.valid)
    {
        Serial.print(
            F("# E_initial=")
        );


        Serial.println(
            baseline.energy,
            6
        );


        Serial.print(
            F("# E_final=")
        );


        Serial.println(
            previousPeakEnergy,
            6
        );


        if (
            baseline.energy >
            0.0F
        )
        {
            Serial.print(
                F("# total_ratio=")
            );


            Serial.println(
                previousPeakEnergy /
                baseline.energy,
                4
            );
        }
    }


    Serial.println(
        F("# ===============================")
    );


    Serial.println(
        F("A = novo ensaio 14C.1")
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


    const float energy =
        peakEnergy(
            event.alphaRad
        );


    // ========================================================
    // ESPERA PEAK- INICIAL
    // ========================================================

    if (
        runState ==
        RunState::WAIT_SYNC_NEGATIVE_PEAK
    )
    {
        // PEAK+ anterior ao PEAK- desejado e normal.
        if (positive)
        {
            return;
        }


        if (
            absAngleDeg <
            MIN_SYNC_PEAK_DEG
        )
        {
            finishExperiment(
                FinishReason::EVENT_ERROR
            );

            return;
        }


        if (
            energy >=
            ENERGY_STOP
        )
        {
            finishExperiment(
                FinishReason::ENERGY_REACHED
            );

            return;
        }


        if (
            absAngleDeg >=
            MAX_PEAK_DEG
        )
        {
            finishExperiment(
                FinishReason::ANGLE_REACHED
            );

            return;
        }


        baseline.valid =
            true;


        baseline.peakUs =
            event.timeUs;


        baseline.detectedUs =
            detectedUs;


        baseline.angleDeg =
            angleDeg;


        baseline.energy =
            energy;


        currentAmplitudeDeg =
            adaptiveAmplitudeFromEnergy(
                energy
            );


        baseline.firstAmplitudeDeg =
            currentAmplitudeDeg;


        peakAmplitudeRad =
            fabsf(
                event.alphaRad
            );


        previousPeakEnergy =
            energy;


        // Referencia do ciclo completo.
        previousNegativePeakEnergy =
            energy;


        previousNegativePeakEnergyValid =
            true;


        expectedDownPositive =
            true;


        expectedPeakPositive =
            true;


        activeStartUs =
            micros();


        runState =
            RunState::ACTIVE;


        return;
    }


    // ========================================================
    // CONTROLE ATIVO
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
        finishExperiment(
            FinishReason::EVENT_ERROR
        );

        return;
    }


    if (
        halfCycleCount >=
        MAX_HALF_CYCLES
    )
    {
        finishExperiment(
            FinishReason::MAX_HALF_CYCLES
        );

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
        finishExperiment(
            FinishReason::EVENT_ERROR
        );

        return;
    }


    // ========================================================
    // RESULTADO DO MEIO CICLO
    // ========================================================

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


    // ========================================================
    // NOVO NA 14C.1:
    // GANHO DE CICLO COMPLETO
    //
    // Medimos somente em PEAK-.
    // ========================================================

    if (!positive)
    {
        if (
            previousNegativePeakEnergyValid
        )
        {
            r.cycleDeltaValid =
                true;


            r.cycleDeltaEnergy =
                energy -
                previousNegativePeakEnergy;
        }


        previousNegativePeakEnergy =
            energy;


        previousNegativePeakEnergyValid =
            true;


        ++fullCycleCount;
    }


    // ========================================================
    // AMPLITUDE DO PROXIMO MEIO CICLO
    // ========================================================

    r.amplitudeNextDeg =
        amplitudeAfterPeak(
            positive,
            energy
        );


    previousPeakEnergy =
        energy;


    ++halfCycleCount;


    // ========================================================
    // PARADA
    // ========================================================

    if (
        energy >=
        ENERGY_STOP
    )
    {
        finishExperiment(
            FinishReason::ENERGY_REACHED
        );

        return;
    }


    if (
        absAngleDeg >=
        MAX_PEAK_DEG
    )
    {
        finishExperiment(
            FinishReason::ANGLE_REACHED
        );

        return;
    }


    if (
        halfCycleCount >=
        MAX_HALF_CYCLES
    )
    {
        finishExperiment(
            FinishReason::MAX_HALF_CYCLES
        );

        return;
    }


    // ========================================================
    // PREPARA PROXIMO MEIO CICLO
    // ========================================================

    peakAmplitudeRad =
        fabsf(
            event.alphaRad
        );


    currentAmplitudeDeg =
        r.amplitudeNextDeg;


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
        finishExperiment(
            FinishReason::EVENT_ERROR
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


    // Se o destino e PEAK+, estamos no lado adaptativo.
    r.adaptiveSide =
        expectedPeakPositive;


    r.amplitudeUsedDeg =
        currentAmplitudeDeg;


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


    // phi continua sendo inferido pela contagem de passos.
    r.phiDown =
        motor.currentPosition();


    r.phiRefDown =
        currentPhiRefDeg;
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
// MOTOR / ESTADO
// ============================================================

void updateMotorState()
{
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
            RunState::WAIT_SYNC_NEGATIVE_PEAK;


        Serial.println();


        Serial.print(
            F("# PREP_DONE,phi=")
        );


        Serial.println(
            motor.currentPosition(),
            2
        );


        Serial.println(
            F("Aguardando PEAK- para iniciar 14C.1.")
        );


        return;
    }


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
        F("A = iniciar Fase 14C.1")
    );
}


// ============================================================
// AQUISICAO + CONTROLE 250 Hz
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


    if (
        !pendulumVelocity.isReady()
    )
    {
        return;
    }


    // ========================================================
    // CONTROLE CONTINUO
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
            finishExperiment(
                FinishReason::TIMEOUT
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
            finishExperiment(
                FinishReason::MOTOR_ERROR
            );

            return;
        }
    }


    // ========================================================
    // EVENTOS
    // ========================================================

    if (
        runState !=
            RunState::WAIT_SYNC_NEGATIVE_PEAK
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
        event.type ==
        PendulumEventType::NONE
    )
    {
        return;
    }


    handleEvent(
        event
    );


    // Se um PEAK alterou A,
    // atualiza imediatamente a referencia.
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
            finishExperiment(
                FinishReason::MOTOR_ERROR
            );
        }
    }
}


// ============================================================
// INICIO
// ============================================================

void startTest()
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


    // Primeiro semiciclo:
    //
    // PEAK- -> DOWN+ -> PEAK+
    //
    // Portanto preposicionamos no lado negativo.
    if (
        !motor.moveTo(
            -ADAPTIVE_AMPLITUDE_MAX_DEG
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
        F("# PHASE14C1_ARMED")
    );


    Serial.println(
        F("# MAX_HALF_CYCLES=12")
    );


    Serial.println(
        F("# ENERGY_STOP=0.25")
    );


    Serial.println(
        F("# PEAK_LIMIT=60deg")
    );


    Serial.println(
        F("# PREPOSITION,target=-8.00")
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
        F("A=")
    );


    Serial.println(
        currentAmplitudeDeg,
        2
    );


    Serial.print(
        F("halfCycles=")
    );


    Serial.println(
        halfCycleCount
    );


    Serial.print(
        F("fullCycles=")
    );


    Serial.println(
        fullCycleCount
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


    // D sempre permitido.
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


    if (
        runState !=
        RunState::IDLE
    )
    {
        return;
    }


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


    if (
        c == 'T' ||
        c == 't'
    )
    {
        startCalibration();

        return;
    }


    if (
        c == 'A' ||
        c == 'a'
    )
    {
        startTest();

        return;
    }


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
        F("FASE 14C.1 - EXTENSAO DO SWING-UP")
    );


    Serial.println(
        F("ADAPTATIVO: PEAK- -> DOWN+ -> PEAK+")
    );


    Serial.println(
        F("FIXO 8deg: PEAK+ -> DOWN- -> PEAK-")
    );


    Serial.println(
        F("12 meios ciclos")
    );


    Serial.println(
        F("Parada: E>=0.25 ou |PEAK|>=60deg")
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
        F("A = iniciar")
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