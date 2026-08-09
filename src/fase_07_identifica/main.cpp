#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>

#include <PendulumPosition.h>
#include <PendulumVelocity.h>
#include <MotorVelocity.h>

#include <math.h>
#include <ctype.h>
#include <string.h>


// ============================================================
// FASE 07 — IDENTIFICACAO LOCAL
// Pendulo de Furuta
// ============================================================
//
// Modelo:
//
//      beta_ddot = a*beta + b*betaDot + c*u
//
// Estados:
//
//      beta      [rad]
//      betaDot   [rad/s]
//
// Entrada:
//
//      u = aceleracao comandada do braco [deg/s2]
//
// IMPORTANTE:
//
// Durante os 400 ms do experimento:
//      - nenhuma telemetria e enviada pela Serial;
//      - os dados sao armazenados em RAM;
//      - depois do fim do experimento o motor e parado;
//      - somente entao os dados sao impressos;
//      - depois ocorre RECENTER.
//
// ============================================================


// ============================================================
// SERIAL
// ============================================================

constexpr uint32_t SERIAL_BAUD =
    250000UL;


// ============================================================
// HARDWARE
// ============================================================

constexpr uint8_t STEP_PIN =
    6;

constexpr uint8_t DIR_PIN =
    8;

constexpr uint8_t ENABLE_PIN =
    4;


// ============================================================
// MOTOR
// ============================================================

constexpr uint16_t FULL_STEPS_PER_REVOLUTION =
    200;

constexpr uint8_t MICROSTEP_FACTOR =
    8;


constexpr float ARM_MINIMUM_DEGREES =
    -80.0F;

constexpr float ARM_MAXIMUM_DEGREES =
    80.0F;


constexpr float MOTOR_MAX_SPEED_DEG_S =
    180.0F;


// ============================================================
// AQUISICAO
// ============================================================
//
// Ts nominal:
//
//      4 ms
//
// fs nominal:
//
//      250 Hz
//
// ============================================================

constexpr uint32_t SAMPLE_PERIOD_US =
    4000UL;


constexpr uint32_t MAX_EXPERIMENT_DT_US =
    20000UL;


// ============================================================
// CONVERSAO
// ============================================================

constexpr float DEG_TO_RAD_LOCAL =
    0.01745329251994329577F;


// ============================================================
// REGIAO LOCAL
// ============================================================

constexpr float BETA_ABORT_DEGREES =
    10.0F;


constexpr float BETA_ABORT_RADIANS =
    BETA_ABORT_DEGREES
    *
    DEG_TO_RAD_LOCAL;


constexpr float ARM_IDENTIFICATION_LIMIT_DEGREES =
    10.0F;


// ============================================================
// TOPO
// ============================================================

constexpr float TOP_WINDOW_DEGREES =
    1.5F;


constexpr float TOP_WINDOW_RADIANS =
    TOP_WINDOW_DEGREES
    *
    DEG_TO_RAD_LOCAL;


constexpr float TOP_MAX_VELOCITY_RAD_S =
    0.25F;


constexpr uint32_t TOP_STABLE_TIME_US =
    400000UL;


// ============================================================
// DETECCAO DA SOLTURA
// ============================================================

constexpr float RELEASE_VELOCITY_RAD_S =
    0.10F;


constexpr float RELEASE_MAX_ANGLE_DEGREES =
    2.0F;


constexpr float RELEASE_MAX_ANGLE_RADIANS =
    RELEASE_MAX_ANGLE_DEGREES
    *
    DEG_TO_RAD_LOCAL;


// ============================================================
// EXCITACAO FORCED
// ============================================================
//
// G+
//
//      +600       0 ... 60 ms
//      -600      60 ... 180 ms
//      +600     180 ... 240 ms
//         0     depois de 240 ms
//
// G-
//
//      -600       0 ... 60 ms
//      +600      60 ... 180 ms
//      -600     180 ... 240 ms
//         0     depois de 240 ms
//
// ============================================================

constexpr float IDENTIFICATION_ACCELERATION_DEG_S2 =
    600.0F;


constexpr uint32_t FORCED_STAGE_1_END_US =
    60000UL;


constexpr uint32_t FORCED_STAGE_2_END_US =
    180000UL;


constexpr uint32_t FORCED_STAGE_3_END_US =
    240000UL;


// ============================================================
// JANELA DE DADOS
// ============================================================

constexpr uint32_t EXPERIMENT_DATA_DURATION_US =
    400000UL;


// ============================================================
// RECENTER
// ============================================================

constexpr float RECENTER_ACCELERATION_DEG_S2 =
    300.0F;


constexpr float RECENTER_MAX_SPEED_DEG_S =
    15.0F;


constexpr uint32_t RECENTER_TIMEOUT_US =
    2000000UL;


// ============================================================
// BUFFER DE AQUISICAO
// ============================================================
//
// O Nano possui apenas 2 kB de SRAM.
//
// Por isso guardamos apenas:
//
//      t
//      beta
//      betaDot
//
// u e reconstruido depois a partir do tempo.
//
// Cada amostra:
//
//      uint16_t t16us      = 2 bytes
//      int16_t beta1e5     = 2 bytes
//      int16_t betaDot1e4  = 2 bytes
//
// Total:
//
//      6 bytes/amostra
//
// 104 amostras:
//
//      624 bytes
//
// ============================================================

struct DataSample
{
    // Tempo relativo em unidades de 16 us.
    uint16_t t16us;

    // beta [rad] * 100000
    int16_t beta1e5;

    // betaDot [rad/s] * 10000
    int16_t betaDot1e4;
};


constexpr uint8_t MAX_BUFFER_SAMPLES =
    104;


DataSample dataBuffer[
    MAX_BUFFER_SAMPLES
];


uint8_t dataCount =
    0;


bool bufferOverflow =
    false;


// ============================================================
// SENSOR
// ============================================================

AS5600 as5600;


PendulumPosition pendulumPosition(
    as5600,
    1.0F
);


PendulumVelocity pendulumVelocity;


// ============================================================
// MOTOR
// ============================================================

MotorVelocity motor(
    STEP_PIN,
    DIR_PIN,
    ENABLE_PIN,

    FULL_STEPS_PER_REVOLUTION,
    MICROSTEP_FACTOR,

    ARM_MINIMUM_DEGREES,
    ARM_MAXIMUM_DEGREES,

    false
);


// ============================================================
// ESTADOS
// ============================================================

enum class ExperimentState
{
    IDLE,

    WAIT_TOP_FREE,

    WAIT_TOP_FORCED,

    RUN_FREE,

    RUN_FORCED,

    RECENTER
};


ExperimentState experimentState =
    ExperimentState::IDLE;


// ============================================================
// FLAGS
// ============================================================

bool armZeroDefined =
    false;


bool releaseArmed =
    false;


int8_t forcedDirection =
    1;


// ============================================================
// TEMPORIZACAO
// ============================================================

uint32_t nextSampleTimeUs =
    0;


uint32_t lastSampleTimeUs =
    0;


uint32_t topStableStartUs =
    0;


uint32_t experimentStartUs =
    0;


uint32_t recenterStartUs =
    0;


// ============================================================
// MEDIDAS
// ============================================================

float betaRadians =
    0.0F;


float betaDotRadiansPerSecond =
    0.0F;


float commandAccelerationDegS2 =
    0.0F;


// ============================================================
// SERIAL BUFFER
// ============================================================

constexpr uint8_t SERIAL_BUFFER_SIZE =
    24;


char serialBuffer[
    SERIAL_BUFFER_SIZE
];


uint8_t serialBufferIndex =
    0;


// ============================================================
// PROTOTIPOS
// ============================================================

void serviceSerial();

void handleCommand(
    char *command
);


void acquisitionTick(
    uint32_t nowUs
);


void serviceWaitingForTop(
    uint32_t nowUs
);


void startExperiment(
    uint32_t nowUs
);


void serviceExperiment(
    uint32_t nowUs,
    float actualDtSeconds,
    uint32_t actualDtUs
);


void recordSample(
    uint32_t nowUs
);


float commandedInputFromTime(
    uint32_t elapsedUs
);


void dumpData(
    const char *reason
);


void endDataAndPrepareRecenter(
    const char *reason,
    uint32_t nowUs
);


void serviceRecenter(
    uint32_t nowUs,
    float actualDtSeconds
);


void finishRecenter();


void recenterFailure(
    const char *reason
);


void armFreeExperiment();


void armForcedExperiment(
    int8_t direction
);


void definePendulumTop();

void defineArmZero();


void controlledAbort();

void emergencyAbort();


void printStatus();

void printHelp();

const char *stateName();


int16_t floatToInt16Scaled(
    float value,
    float scale
);


// ============================================================
// FLOAT -> INT16
// ============================================================

int16_t floatToInt16Scaled(
    float value,
    float scale
)
{
    float scaled =
        value
        *
        scale;


    if (
        scaled > 32767.0F
    )
    {
        scaled =
            32767.0F;
    }


    if (
        scaled < -32768.0F
    )
    {
        scaled =
            -32768.0F;
    }


    if (
        scaled >= 0.0F
    )
    {
        return static_cast<int16_t>(
            scaled + 0.5F
        );
    }


    return static_cast<int16_t>(
        scaled - 0.5F
    );
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(
        SERIAL_BAUD
    );


    Wire.begin();


    Wire.setClock(
        400000UL
    );


    // --------------------------------------------------------
    // SENSOR
    // --------------------------------------------------------

    pendulumPosition.begin();


    pendulumPosition.update();


    uint32_t nowUs =
        micros();


    betaRadians =
        pendulumPosition.betaRadians();


    pendulumVelocity.reset(
        betaRadians,
        nowUs
    );


    // --------------------------------------------------------
    // MOTOR
    // --------------------------------------------------------

    motor.begin(
        MOTOR_MAX_SPEED_DEG_S,
        2
    );


    armZeroDefined =
        false;


    // --------------------------------------------------------
    // TEMPORIZACAO
    // --------------------------------------------------------

    lastSampleTimeUs =
        micros();


    nextSampleTimeUs =
        lastSampleTimeUs
        +
        SAMPLE_PERIOD_US;


    // --------------------------------------------------------
    // CABECALHO
    // --------------------------------------------------------

    Serial.println();


    Serial.println(
        F("========================================")
    );


    Serial.println(
        F("FASE 07 - IDENTIFICACAO LOCAL")
    );


    Serial.println(
        F("Pendulo de Furuta")
    );


    Serial.println(
        F("========================================")
    );


    Serial.println();


    Serial.println(
        F("AQUISICAO SILENCIOSA EM RAM")
    );


    Serial.println(
        F("Ts nominal = 4 ms")
    );


    Serial.println(
        F("Serial = 250000 baud")
    );


    Serial.println();


    Serial.println(
        F("FORCED = +/-600 deg/s2")
    );


    Serial.println(
        F("Tempos = 60 / 120 / 60 ms")
    );


    Serial.println(
        F("Janela = 400 ms")
    );


    Serial.println(
        F("Release betaDot = 0.10 rad/s")
    );


    Serial.println(
        F("TOP_LOST se |beta| > 2 graus")
    );


    Serial.println(
        F("RECENTER automatico")
    );


    Serial.println();


    Serial.println(
        F("Motor iniciado DESABILITADO.")
    );


    if (
        pendulumPosition.magnetDetected()
    )
    {
        Serial.println(
            F("AS5600: ima detectado.")
        );
    }
    else
    {
        Serial.println(
            F("ATENCAO: ima nao detectado.")
        );
    }


    Serial.println();


    printHelp();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    motor.update();


    serviceSerial();


    motor.update();


    const uint32_t nowUs =
        micros();


    if (
        static_cast<int32_t>(
            nowUs - nextSampleTimeUs
        )
        >=
        0
    )
    {
        acquisitionTick(
            nowUs
        );


        nextSampleTimeUs +=
            SAMPLE_PERIOD_US;


        // ----------------------------------------------------
        // Se atrasou mais de um periodo, nao tenta "recuperar"
        // amostras antigas.
        // ----------------------------------------------------

        if (
            static_cast<int32_t>(
                micros() - nextSampleTimeUs
            )
            >=
            0
        )
        {
            nextSampleTimeUs =
                micros()
                +
                SAMPLE_PERIOD_US;
        }
    }


    motor.update();
}


// ============================================================
// AQUISICAO
// ============================================================

void acquisitionTick(
    uint32_t nowUs
)
{
    const uint32_t actualDtUs =
        nowUs
        -
        lastSampleTimeUs;


    const float actualDtSeconds =
        static_cast<float>(
            actualDtUs
        )
        *
        1.0e-6F;


    lastSampleTimeUs =
        nowUs;


    // --------------------------------------------------------
    // PENDULO
    // --------------------------------------------------------

    pendulumPosition.update();


    betaRadians =
        pendulumPosition.betaRadians();


    pendulumVelocity.update(
        betaRadians,
        nowUs
    );


    if (
        pendulumVelocity.isReady()
    )
    {
        betaDotRadiansPerSecond =
            pendulumVelocity.radiansPerSecond();
    }


    // --------------------------------------------------------
    // MAQUINA DE ESTADOS
    // --------------------------------------------------------

    switch (
        experimentState
    )
    {
        case ExperimentState::IDLE:
        {
            break;
        }


        case ExperimentState::WAIT_TOP_FREE:

        case ExperimentState::WAIT_TOP_FORCED:
        {
            serviceWaitingForTop(
                nowUs
            );

            break;
        }


        case ExperimentState::RUN_FREE:

        case ExperimentState::RUN_FORCED:
        {
            serviceExperiment(
                nowUs,
                actualDtSeconds,
                actualDtUs
            );

            break;
        }


        case ExperimentState::RECENTER:
        {
            serviceRecenter(
                nowUs,
                actualDtSeconds
            );

            break;
        }
    }
}


// ============================================================
// AGUARDAR TOPO / SOLTURA
// ============================================================

void serviceWaitingForTop(
    uint32_t nowUs
)
{
    if (
        !pendulumVelocity.isReady()
    )
    {
        return;
    }


    const float absBeta =
        fabsf(
            betaRadians
        );


    const float absBetaDot =
        fabsf(
            betaDotRadiansPerSecond
        );


    // ========================================================
    // ESPERAR ESTABILIDADE
    // ========================================================

    if (
        !releaseArmed
    )
    {
        const bool topStable =
            (
                absBeta
                <=
                TOP_WINDOW_RADIANS
            )
            &&
            (
                absBetaDot
                <=
                TOP_MAX_VELOCITY_RAD_S
            );


        if (
            topStable
        )
        {
            if (
                topStableStartUs == 0
            )
            {
                topStableStartUs =
                    nowUs;
            }


            if (
                (
                    nowUs
                    -
                    topStableStartUs
                )
                >=
                TOP_STABLE_TIME_US
            )
            {
                releaseArmed =
                    true;


                Serial.println();


                Serial.println(
                    F("TOP_READY")
                );


                Serial.println(
                    F("SOLTE O PENDULO.")
                );


                Serial.println(
                    F("Aguardando movimento...")
                );


                Serial.println();
            }
        }
        else
        {
            topStableStartUs =
                0;
        }


        return;
    }


    // ========================================================
    // TOP LOST
    // ========================================================

    if (
        absBeta
        >
        RELEASE_MAX_ANGLE_RADIANS
    )
    {
        releaseArmed =
            false;


        topStableStartUs =
            0;


        motor.stop();


        Serial.println();


        Serial.println(
            F("TOP_LOST")
        );


        Serial.print(
            F("beta = ")
        );


        Serial.print(
            betaRadians
            /
            DEG_TO_RAD_LOCAL,
            3
        );


        Serial.println(
            F(" deg")
        );


        Serial.println(
            F("Reposicione o pendulo.")
        );


        Serial.println();


        return;
    }


    // ========================================================
    // DETECTAR SOLTURA
    // ========================================================

    if (
        absBetaDot
        >=
        RELEASE_VELOCITY_RAD_S
    )
    {
        startExperiment(
            nowUs
        );
    }
}


// ============================================================
// INICIAR EXPERIMENTO
// ============================================================

void startExperiment(
    uint32_t nowUs
)
{
    motor.stop();


    experimentStartUs =
        nowUs;


    commandAccelerationDegS2 =
        0.0F;


    dataCount =
        0;


    bufferOverflow =
        false;


    if (
        experimentState
        ==
        ExperimentState::WAIT_TOP_FREE
    )
    {
        experimentState =
            ExperimentState::RUN_FREE;
    }
    else
    {
        experimentState =
            ExperimentState::RUN_FORCED;
    }


    // ========================================================
    // NAO IMPRIMIR NADA DAQUI ATE O FIM DO EXPERIMENTO
    // ========================================================
}


// ============================================================
// ARMAZENAR AMOSTRA
// ============================================================

void recordSample(
    uint32_t nowUs
)
{
    if (
        dataCount
        >=
        MAX_BUFFER_SAMPLES
    )
    {
        bufferOverflow =
            true;


        return;
    }


    DataSample &sample =
        dataBuffer[
            dataCount
        ];


    const uint32_t elapsedUs =
        nowUs
        -
        experimentStartUs;


    // ========================================================
    // TEMPO
    // ========================================================
    //
    // Unidade = 16 us.
    //
    // ========================================================

    uint32_t t16 =
        (
            elapsedUs + 8UL
        )
        /
        16UL;


    if (
        t16 > 65535UL
    )
    {
        t16 =
            65535UL;
    }


    sample.t16us =
        static_cast<uint16_t>(
            t16
        );


    // ========================================================
    // beta
    // ========================================================

    sample.beta1e5 =
        floatToInt16Scaled(
            betaRadians,
            100000.0F
        );


    // ========================================================
    // betaDot
    // ========================================================

    sample.betaDot1e4 =
        floatToInt16Scaled(
            betaDotRadiansPerSecond,
            10000.0F
        );


    dataCount++;
}


// ============================================================
// RECONSTRUIR U
// ============================================================
//
// u nao e armazenado no buffer.
//
// Como o perfil de entrada e conhecido, podemos reconstruir
// exatamente seu valor a partir do instante da amostra.
//
// ============================================================

float commandedInputFromTime(
    uint32_t elapsedUs
)
{
    if (
        experimentState
        ==
        ExperimentState::RUN_FREE
    )
    {
        return 0.0F;
    }


    const float excitation =
        static_cast<float>(
            forcedDirection
        )
        *
        IDENTIFICATION_ACCELERATION_DEG_S2;


    if (
        elapsedUs
        <
        FORCED_STAGE_1_END_US
    )
    {
        return excitation;
    }


    if (
        elapsedUs
        <
        FORCED_STAGE_2_END_US
    )
    {
        return -excitation;
    }


    if (
        elapsedUs
        <
        FORCED_STAGE_3_END_US
    )
    {
        return excitation;
    }


    return 0.0F;
}


// ============================================================
// EXECUTAR EXPERIMENTO
// ============================================================

void serviceExperiment(
    uint32_t nowUs,
    float actualDtSeconds,
    uint32_t actualDtUs
)
{
    const uint32_t elapsedUs =
        nowUs
        -
        experimentStartUs;


    // ========================================================
    // QUALIDADE TEMPORAL
    // ========================================================

    if (
        actualDtUs
        >
        MAX_EXPERIMENT_DT_US
    )
    {
        recordSample(
            nowUs
        );


        endDataAndPrepareRecenter(
            "SAMPLING_OVERRUN",
            nowUs
        );


        return;
    }


    // ========================================================
    // LIMITE BETA
    // ========================================================

    if (
        fabsf(
            betaRadians
        )
        >=
        BETA_ABORT_RADIANS
    )
    {
        recordSample(
            nowUs
        );


        endDataAndPrepareRecenter(
            "BETA_LIMIT",
            nowUs
        );


        return;
    }


    // ========================================================
    // LIMITE PHI
    // ========================================================

    if (
        fabsf(
            motor.currentPositionDegrees()
        )
        >=
        ARM_IDENTIFICATION_LIMIT_DEGREES
    )
    {
        recordSample(
            nowUs
        );


        endDataAndPrepareRecenter(
            "ARM_LIMIT",
            nowUs
        );


        return;
    }


    // ========================================================
    // BUFFER
    // ========================================================

    if (
        bufferOverflow
    )
    {
        endDataAndPrepareRecenter(
            "BUFFER_OVERFLOW",
            nowUs
        );


        return;
    }


    // ========================================================
    // FREE
    // ========================================================

    if (
        experimentState
        ==
        ExperimentState::RUN_FREE
    )
    {
        commandAccelerationDegS2 =
            0.0F;


        motor.commandAcceleration(
            0.0F,
            actualDtSeconds
        );
    }


    // ========================================================
    // FORCED
    // ========================================================

    else
    {
        const float excitation =
            static_cast<float>(
                forcedDirection
            )
            *
            IDENTIFICATION_ACCELERATION_DEG_S2;


        // ----------------------------------------------------
        // 0 ... 60 ms
        // ----------------------------------------------------

        if (
            elapsedUs
            <
            FORCED_STAGE_1_END_US
        )
        {
            commandAccelerationDegS2 =
                excitation;
        }


        // ----------------------------------------------------
        // 60 ... 180 ms
        // ----------------------------------------------------

        else if (
            elapsedUs
            <
            FORCED_STAGE_2_END_US
        )
        {
            commandAccelerationDegS2 =
                -excitation;
        }


        // ----------------------------------------------------
        // 180 ... 240 ms
        // ----------------------------------------------------

        else if (
            elapsedUs
            <
            FORCED_STAGE_3_END_US
        )
        {
            commandAccelerationDegS2 =
                excitation;
        }


        // ----------------------------------------------------
        // depois de 240 ms
        // ----------------------------------------------------

        else
        {
            commandAccelerationDegS2 =
                0.0F;
        }


        if (
            elapsedUs
            <
            FORCED_STAGE_3_END_US
        )
        {
            const bool accepted =
                motor.commandAcceleration(
                    commandAccelerationDegS2,
                    actualDtSeconds
                );


            if (
                !accepted
            )
            {
                recordSample(
                    nowUs
                );


                endDataAndPrepareRecenter(
                    "MOTOR_COMMAND_REJECTED",
                    nowUs
                );


                return;
            }
        }
        else
        {
            motor.stop();
        }
    }


    motor.update();


    // ========================================================
    // REGISTRAR AMOSTRA
    // ========================================================

    recordSample(
        nowUs
    );


    // ========================================================
    // FIM DO EXPERIMENTO
    // ========================================================

    if (
        elapsedUs
        >=
        EXPERIMENT_DATA_DURATION_US
    )
    {
        endDataAndPrepareRecenter(
            "DATA_WINDOW_END",
            nowUs
        );


        return;
    }


    if (
        bufferOverflow
    )
    {
        endDataAndPrepareRecenter(
            "BUFFER_OVERFLOW",
            nowUs
        );
    }
}


// ============================================================
// DESPEJAR DADOS
// ============================================================

void dumpData(
    const char *reason
)
{
    Serial.println();


    // ========================================================
    // TIPO DE ENSAIO
    // ========================================================

    if (
        experimentState
        ==
        ExperimentState::RUN_FREE
    )
    {
        Serial.println(
            F("# DATA_BEGIN FREE")
        );
    }
    else
    {
        if (
            forcedDirection > 0
        )
        {
            Serial.println(
                F("# DATA_BEGIN FORCED_POSITIVE")
            );
        }
        else
        {
            Serial.println(
                F("# DATA_BEGIN FORCED_NEGATIVE")
            );
        }
    }


    Serial.println(
        F("# t_us,beta,betaDot,u")
    );


    // ========================================================
    // DADOS
    // ========================================================

    for (
        uint8_t i = 0;
        i < dataCount;
        i++
    )
    {
        const DataSample &sample =
            dataBuffer[i];


        const uint32_t timeUs =
            static_cast<uint32_t>(
                sample.t16us
            )
            *
            16UL;


        // ----------------------------------------------------
        // t_us
        // ----------------------------------------------------

        Serial.print(
            timeUs
        );


        Serial.print(',');


        // ----------------------------------------------------
        // beta
        // ----------------------------------------------------

        Serial.print(
            static_cast<float>(
                sample.beta1e5
            )
            /
            100000.0F,
            6
        );


        Serial.print(',');


        // ----------------------------------------------------
        // betaDot
        // ----------------------------------------------------

        Serial.print(
            static_cast<float>(
                sample.betaDot1e4
            )
            /
            10000.0F,
            5
        );


        Serial.print(',');


        // ----------------------------------------------------
        // u reconstruido
        // ----------------------------------------------------

        const float u =
            commandedInputFromTime(
                timeUs
            );


        Serial.println(
            u,
            1
        );
    }


    // ========================================================
    // RESUMO
    // ========================================================

    Serial.println(
        F("# DATA_END")
    );


    Serial.print(
        F("# REASON=")
    );


    Serial.println(
        reason
    );


    Serial.print(
        F("# SAMPLE_COUNT=")
    );


    Serial.println(
        dataCount
    );


    Serial.print(
        F("# BUFFER_OVERFLOW=")
    );


    Serial.println(
        bufferOverflow
        ?
        F("YES")
        :
        F("NO")
    );


    // ========================================================
    // DT MEDIO
    // ========================================================

    if (
        dataCount >= 2
    )
    {
        const uint32_t firstUs =
            static_cast<uint32_t>(
                dataBuffer[0].t16us
            )
            *
            16UL;


        const uint32_t lastUs =
            static_cast<uint32_t>(
                dataBuffer[dataCount - 1].t16us
            )
            *
            16UL;


        const float meanDtUs =
            static_cast<float>(
                lastUs
                -
                firstUs
            )
            /
            static_cast<float>(
                dataCount - 1
            );


        Serial.print(
            F("# MEAN_DT_US=")
        );


        Serial.println(
            meanDtUs,
            2
        );
    }


    Serial.print(
        F("# PHI_DATA_END_DEG=")
    );


    Serial.println(
        motor.currentPositionDegrees(),
        3
    );


    Serial.print(
        F("# BETA_DATA_END_DEG=")
    );


    Serial.println(
        betaRadians
        /
        DEG_TO_RAD_LOCAL,
        3
    );


    Serial.println();
}


// ============================================================
// FINAL DOS DADOS
// ============================================================

void endDataAndPrepareRecenter(
    const char *reason,
    uint32_t nowUs
)
{
    // ========================================================
    // PRIMEIRO PARAMOS O MOTOR
    // ========================================================

    motor.stop();


    commandAccelerationDegS2 =
        0.0F;


    // ========================================================
    // SOMENTE DEPOIS USAMOS A SERIAL
    // ========================================================

    dumpData(
        reason
    );


    // ========================================================
    // JA ESTA EM ZERO?
    // ========================================================

    if (
        fabsf(
            motor.currentPositionDegrees()
        )
        <
        0.01F
    )
    {
        motor.setCurrentPosition(
            0.0F
        );


        experimentState =
            ExperimentState::IDLE;


        releaseArmed =
            false;


        topStableStartUs =
            0;


        Serial.println(
            F("Braco ja esta em phi=0.")
        );


        Serial.println(
            F("Pronto para novo ensaio.")
        );


        Serial.println();


        return;
    }


    // ========================================================
    // RECENTER
    // ========================================================

    experimentState =
        ExperimentState::RECENTER;


    recenterStartUs =
        nowUs;


    releaseArmed =
        false;


    topStableStartUs =
        0;


    Serial.println(
        F("# RECENTER_BEGIN")
    );


    Serial.print(
        F("# PHI_INITIAL_DEG=")
    );


    Serial.println(
        motor.currentPositionDegrees(),
        3
    );


    Serial.println(
        F("Retornando lentamente o braco para phi=0...")
    );
}


// ============================================================
// RECENTER
// ============================================================

void serviceRecenter(
    uint32_t nowUs,
    float actualDtSeconds
)
{
    if (
        (
            nowUs
            -
            recenterStartUs
        )
        >=
        RECENTER_TIMEOUT_US
    )
    {
        recenterFailure(
            "RECENTER_TIMEOUT"
        );


        return;
    }


    const float phiDegrees =
        motor.currentPositionDegrees();


    const float speedDegreesPerSecond =
        motor.speedReferenceDegreesPerSecond();


    // ========================================================
    // ZERO
    // ========================================================

    if (
        fabsf(
            phiDegrees
        )
        <
        0.01F
    )
    {
        motor.stop();


        motor.setCurrentPosition(
            0.0F
        );


        finishRecenter();


        return;
    }


    // ========================================================
    // SENTIDO
    // ========================================================

    float desiredDirection;


    if (
        phiDegrees > 0.0F
    )
    {
        desiredDirection =
            -1.0F;
    }
    else
    {
        desiredDirection =
            1.0F;
    }


    // ========================================================
    // CORRIGIR VELOCIDADE NO SENTIDO ERRADO
    // ========================================================

    if (
        speedDegreesPerSecond
        *
        desiredDirection
        <
        0.0F
    )
    {
        motor.commandAcceleration(
            desiredDirection
            *
            RECENTER_ACCELERATION_DEG_S2,

            actualDtSeconds
        );


        return;
    }


    // ========================================================
    // ACELERAR ATE VELOCIDADE DE RECENTER
    // ========================================================

    if (
        fabsf(
            speedDegreesPerSecond
        )
        <
        RECENTER_MAX_SPEED_DEG_S
    )
    {
        motor.commandAcceleration(
            desiredDirection
            *
            RECENTER_ACCELERATION_DEG_S2,

            actualDtSeconds
        );
    }
    else
    {
        motor.commandAcceleration(
            0.0F,
            actualDtSeconds
        );
    }
}


// ============================================================
// RECENTER CONCLUIDO
// ============================================================

void finishRecenter()
{
    motor.stop();


    motor.setCurrentPosition(
        0.0F
    );


    Serial.println(
        F("# RECENTER_END")
    );


    Serial.println(
        F("# PHI_FINAL_DEG=0.000")
    );


    Serial.println();


    Serial.println(
        F("Braco recentralizado.")
    );


    Serial.println(
        F("Pronto para novo ensaio.")
    );


    Serial.println();


    experimentState =
        ExperimentState::IDLE;


    releaseArmed =
        false;


    topStableStartUs =
        0;
}


// ============================================================
// RECENTER FALHOU
// ============================================================

void recenterFailure(
    const char *reason
)
{
    motor.stop();


    Serial.println();


    Serial.print(
        F("# ")
    );


    Serial.println(
        reason
    );


    Serial.print(
        F("# PHI_DEG=")
    );


    Serial.println(
        motor.currentPositionDegrees(),
        3
    );


    Serial.println(
        F("ERRO: RECENTER falhou.")
    );


    Serial.println(
        F("Referencia do braco perdida.")
    );


    Serial.println(
        F("Desabilite, reposicione e use Z.")
    );


    Serial.println();


    armZeroDefined =
        false;


    experimentState =
        ExperimentState::IDLE;


    releaseArmed =
        false;


    topStableStartUs =
        0;
}


// ============================================================
// ARMAR FREE
// ============================================================

void armFreeExperiment()
{
    if (
        experimentState
        !=
        ExperimentState::IDLE
    )
    {
        Serial.println(
            F("ERRO: sistema ocupado.")
        );


        return;
    }


    if (
        !pendulumPosition.topIsDefined()
    )
    {
        Serial.println(
            F("ERRO: defina T primeiro.")
        );


        return;
    }


    if (
        !armZeroDefined
    )
    {
        Serial.println(
            F("ERRO: defina Z primeiro.")
        );


        return;
    }


    if (
        !motor.isEnabled()
    )
    {
        Serial.println(
            F("ERRO: habilite com E.")
        );


        return;
    }


    if (
        fabsf(
            motor.currentPositionDegrees()
        )
        >
        0.5F
    )
    {
        Serial.println(
            F("ERRO: phi nao esta proximo de zero.")
        );


        return;
    }


    motor.stop();


    releaseArmed =
        false;


    topStableStartUs =
        0;


    experimentState =
        ExperimentState::WAIT_TOP_FREE;


    Serial.println();


    Serial.println(
        F("FREE ARMADO.")
    );


    Serial.println(
        F("Aguarde TOP_READY e solte.")
    );


    Serial.println();
}


// ============================================================
// ARMAR FORCED
// ============================================================

void armForcedExperiment(
    int8_t direction
)
{
    if (
        experimentState
        !=
        ExperimentState::IDLE
    )
    {
        Serial.println(
            F("ERRO: sistema ocupado.")
        );


        return;
    }


    if (
        !pendulumPosition.topIsDefined()
    )
    {
        Serial.println(
            F("ERRO: defina T primeiro.")
        );


        return;
    }


    if (
        !armZeroDefined
    )
    {
        Serial.println(
            F("ERRO: defina Z primeiro.")
        );


        return;
    }


    if (
        !motor.isEnabled()
    )
    {
        Serial.println(
            F("ERRO: habilite com E.")
        );


        return;
    }


    if (
        fabsf(
            motor.currentPositionDegrees()
        )
        >
        0.5F
    )
    {
        Serial.println(
            F("ERRO: phi nao esta proximo de zero.")
        );


        Serial.print(
            F("phi = ")
        );


        Serial.println(
            motor.currentPositionDegrees(),
            3
        );


        return;
    }


    forcedDirection =
        (
            direction >= 0
        )
        ?
        1
        :
        -1;


    motor.stop();


    releaseArmed =
        false;


    topStableStartUs =
        0;


    experimentState =
        ExperimentState::WAIT_TOP_FORCED;


    Serial.println();


    if (
        forcedDirection > 0
    )
    {
        Serial.println(
            F("FORCED POSITIVO ARMADO.")
        );


        Serial.println(
            F("+600 -> -600 -> +600 deg/s2")
        );
    }
    else
    {
        Serial.println(
            F("FORCED NEGATIVO ARMADO.")
        );


        Serial.println(
            F("-600 -> +600 -> -600 deg/s2")
        );
    }


    Serial.println(
        F("60 / 120 / 60 ms")
    );


    Serial.println(
        F("400 ms de aquisicao SILENCIOSA")
    );


    Serial.println(
        F("Dados serao impressos somente depois.")
    );


    Serial.println();


    Serial.println(
        F("Aguarde TOP_READY e solte.")
    );


    Serial.println();
}


// ============================================================
// DEFINIR TOPO
// ============================================================

void definePendulumTop()
{
    if (
        experimentState
        !=
        ExperimentState::IDLE
    )
    {
        Serial.println(
            F("ERRO: sistema ocupado.")
        );


        return;
    }


    motor.stop();


    if (
        !pendulumPosition.magnetDetected()
    )
    {
        Serial.println(
            F("ERRO: ima nao detectado.")
        );


        return;
    }


    Serial.println();


    Serial.println(
        F("Calibrando beta=0...")
    );


    Serial.println(
        F("Mantenha o pendulo na vertical.")
    );


    pendulumPosition.calibrateTop(
        32,
        2
    );


    pendulumPosition.update();


    const uint32_t nowUs =
        micros();


    betaRadians =
        pendulumPosition.betaRadians();


    pendulumVelocity.reset(
        betaRadians,
        nowUs
    );


    if (
        pendulumPosition.topIsDefined()
    )
    {
        Serial.println(
            F("Topo definido: beta=0.")
        );
    }
    else
    {
        Serial.println(
            F("ERRO ao definir topo.")
        );
    }


    Serial.println();
}


// ============================================================
// DEFINIR ZERO DO BRACO
// ============================================================

void defineArmZero()
{
    if (
        experimentState
        !=
        ExperimentState::IDLE
    )
    {
        Serial.println(
            F("ERRO: sistema ocupado.")
        );


        return;
    }


    if (
        motor.isEnabled()
    )
    {
        Serial.println(
            F("ERRO: desabilite antes de Z.")
        );


        return;
    }


    motor.setCurrentPosition(
        0.0F
    );


    armZeroDefined =
        true;


    Serial.println(
        F("Zero do braco definido: phi=0.")
    );
}


// ============================================================
// ABORT CONTROLADO
// ============================================================

void controlledAbort()
{
    motor.stop();


    commandAccelerationDegS2 =
        0.0F;


    experimentState =
        ExperimentState::IDLE;


    releaseArmed =
        false;


    topStableStartUs =
        0;


    dataCount =
        0;


    bufferOverflow =
        false;


    Serial.println(
        F("Ensaio interrompido.")
    );
}


// ============================================================
// EMERGENCY STOP
// ============================================================

void emergencyAbort()
{
    motor.emergencyStop();


    commandAccelerationDegS2 =
        0.0F;


    experimentState =
        ExperimentState::IDLE;


    releaseArmed =
        false;


    topStableStartUs =
        0;


    dataCount =
        0;


    bufferOverflow =
        false;


    armZeroDefined =
        false;


    Serial.println();


    Serial.println(
        F("EMERGENCY STOP.")
    );


    Serial.println(
        F("Motor desabilitado.")
    );


    Serial.println(
        F("Referencia perdida.")
    );


    Serial.println(
        F("Reposicione e use Z novamente.")
    );


    Serial.println();
}


// ============================================================
// SERIAL
// ============================================================

void serviceSerial()
{
    while (
        Serial.available() > 0
    )
    {
        const char character =
            static_cast<char>(
                Serial.read()
            );


        if (
            character == '\r'
        )
        {
            continue;
        }


        if (
            character == '\n'
        )
        {
            if (
                serialBufferIndex > 0
            )
            {
                serialBuffer[
                    serialBufferIndex
                ] =
                    '\0';


                handleCommand(
                    serialBuffer
                );


                serialBufferIndex =
                    0;
            }


            continue;
        }


        if (
            serialBufferIndex
            <
            SERIAL_BUFFER_SIZE - 1
        )
        {
            serialBuffer[
                serialBufferIndex
            ] =
                character;


            serialBufferIndex++;
        }
    }
}


// ============================================================
// COMANDOS
// ============================================================

void handleCommand(
    char *command
)
{
    while (
        *command != '\0'
        &&
        isspace(
            static_cast<unsigned char>(
                *command
            )
        )
    )
    {
        command++;
    }


    for (
        char *p = command;
        *p != '\0';
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


    if (
        strcmp(command, "T") == 0
        ||
        strcmp(command, "TOP") == 0
    )
    {
        definePendulumTop();

        return;
    }


    if (
        strcmp(command, "Z") == 0
        ||
        strcmp(command, "ZERO") == 0
    )
    {
        defineArmZero();

        return;
    }


    if (
        strcmp(command, "E") == 0
        ||
        strcmp(command, "ENABLE") == 0
    )
    {
        if (
            !armZeroDefined
        )
        {
            Serial.println(
                F("ERRO: defina Z primeiro.")
            );


            return;
        }


        motor.enable();


        Serial.println(
            F("Motor habilitado.")
        );


        return;
    }


    if (
        strcmp(command, "D") == 0
        ||
        strcmp(command, "DISABLE") == 0
    )
    {
        controlledAbort();


        motor.disable();


        armZeroDefined =
            false;


        Serial.println(
            F("Motor desabilitado.")
        );


        Serial.println(
            F("Referencia perdida.")
        );


        return;
    }


    if (
        strcmp(command, "F") == 0
        ||
        strcmp(command, "FREE") == 0
    )
    {
        armFreeExperiment();

        return;
    }


    if (
        strcmp(command, "G+") == 0
    )
    {
        armForcedExperiment(
            +1
        );


        return;
    }


    if (
        strcmp(command, "G-") == 0
    )
    {
        armForcedExperiment(
            -1
        );


        return;
    }


    if (
        strcmp(command, "G") == 0
        ||
        strcmp(command, "FORCED") == 0
    )
    {
        Serial.println(
            F("Use G+ ou G-.")
        );


        return;
    }


    if (
        strcmp(command, "STOP") == 0
    )
    {
        controlledAbort();

        return;
    }


    if (
        strcmp(command, "X") == 0
    )
    {
        emergencyAbort();

        return;
    }


    if (
        strcmp(command, "S") == 0
        ||
        strcmp(command, "STATUS") == 0
    )
    {
        printStatus();

        return;
    }


    if (
        strcmp(command, "H") == 0
        ||
        strcmp(command, "HELP") == 0
        ||
        strcmp(command, "?") == 0
    )
    {
        printHelp();

        return;
    }


    Serial.println(
        F("Comando desconhecido. Use H.")
    );
}


// ============================================================
// STATUS
// ============================================================

void printStatus()
{
    Serial.println();


    Serial.println(
        F("---------- STATUS ----------")
    );


    Serial.print(
        F("Estado: ")
    );


    Serial.println(
        stateName()
    );


    Serial.print(
        F("Topo: ")
    );


    Serial.println(
        pendulumPosition.topIsDefined()
        ?
        F("DEFINIDO")
        :
        F("NAO DEFINIDO")
    );


    Serial.print(
        F("Zero braco: ")
    );


    Serial.println(
        armZeroDefined
        ?
        F("DEFINIDO")
        :
        F("NAO DEFINIDO")
    );


    Serial.print(
        F("Motor: ")
    );


    Serial.println(
        motor.isEnabled()
        ?
        F("HABILITADO")
        :
        F("DESABILITADO")
    );


    Serial.print(
        F("beta [deg]: ")
    );


    Serial.println(
        betaRadians
        /
        DEG_TO_RAD_LOCAL,
        3
    );


    Serial.print(
        F("betaDot [rad/s]: ")
    );


    Serial.println(
        betaDotRadiansPerSecond,
        4
    );


    Serial.print(
        F("phi [deg]: ")
    );


    Serial.println(
        motor.currentPositionDegrees(),
        3
    );


    Serial.print(
        F("phiDotCmd [deg/s]: ")
    );


    Serial.println(
        motor.speedReferenceDegreesPerSecond(),
        3
    );


    Serial.print(
        F("Buffer samples: ")
    );


    Serial.println(
        dataCount
    );


    Serial.println(
        F("----------------------------")
    );


    Serial.println();
}


// ============================================================
// NOME DO ESTADO
// ============================================================

const char *stateName()
{
    switch (
        experimentState
    )
    {
        case ExperimentState::IDLE:
            return "IDLE";


        case ExperimentState::WAIT_TOP_FREE:
            return "WAIT_TOP_FREE";


        case ExperimentState::WAIT_TOP_FORCED:
            return "WAIT_TOP_FORCED";


        case ExperimentState::RUN_FREE:
            return "RUN_FREE";


        case ExperimentState::RUN_FORCED:
            return "RUN_FORCED";


        case ExperimentState::RECENTER:
            return "RECENTER";
    }


    return "UNKNOWN";
}


// ============================================================
// AJUDA
// ============================================================

void printHelp()
{
    Serial.println(
        F("COMANDOS:")
    );


    Serial.println(
        F("T      = definir beta=0")
    );


    Serial.println(
        F("Z      = definir phi=0")
    );


    Serial.println(
        F("E      = habilitar motor")
    );


    Serial.println(
        F("D      = desabilitar motor")
    );


    Serial.println(
        F("F      = FREE")
    );


    Serial.println(
        F("G+     = FORCED positivo")
    );


    Serial.println(
        F("G-     = FORCED negativo")
    );


    Serial.println(
        F("STOP   = interromper")
    );


    Serial.println(
        F("X      = emergency stop")
    );


    Serial.println(
        F("S      = status")
    );


    Serial.println(
        F("H      = ajuda")
    );


    Serial.println();


    Serial.println(
        F("FORCED = +/-600 deg/s2")
    );


    Serial.println(
        F("Tempos = 60 / 120 / 60 ms")
    );


    Serial.println(
        F("Janela = 400 ms")
    );


    Serial.println(
        F("Durante o ensaio: SERIAL SILENCIOSA")
    );


    Serial.println(
        F("Depois: dump dos dados + RECENTER")
    );


    Serial.println();
}