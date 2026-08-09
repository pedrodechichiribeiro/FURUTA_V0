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
// FASE 08 — CONTROLE PD
// Pendulo de Furuta
//
// REFERENCIA DO PENDULO:
//
// A referencia angular NAO e mais definida manualmente no topo.
//
// O comando T deve ser executado com o pendulo:
//
//      - livre;
//      - parado;
//      - pendurado para baixo.
//
// Essa posicao e definida como referencia inferior.
//
// A vertical superior e calculada automaticamente como:
//
//      TOP = DOWN + 180 graus
//
// Na coordenada usada pelo controlador:
//
//      beta = 0        -> vertical superior
//      beta = +/-PI    -> vertical inferior
//
// ============================================================
//
// MODELO LOCAL IDENTIFICADO:
//
// betaDDot = 21.7*beta
//          -3.28*betaDot
//          -0.75*u
//
// beta     [rad]
// betaDot  [rad/s]
// u        [rad/s2]
//
// ============================================================
//
// CONTROLE:
//
//      u = Kp*beta + Kd*betaDot
//
//      Kp = 60.93
//      Kd = 8.96
//
// ============================================================
//
// CAPTURA:
//
// 1. B arma WAIT_CAPTURE.
//
// 2. Usuario leva manualmente o pendulo perto da vertical.
//
// 3. CAPTURE_READY quando:
//
//      |beta|    <= 1.5 deg
//      |betaDot| <= 0.25 rad/s
//
//    por 80 ms.
//
// 4. Depois de CAPTURE_READY:
//
//      soltura detectada quando
//
//      |betaDot| >= 0.05 rad/s
//
// 5. Se antes da soltura:
//
//      |beta| > 1.75 deg
//
//    ocorre CAPTURE_LOST.
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
// TEMPORIZACAO
// ============================================================

constexpr uint32_t CONTROL_PERIOD_US =
    4000UL;


constexpr uint32_t MAX_CONTROL_DT_US =
    20000UL;


// ============================================================
// CONSTANTES
// ============================================================

constexpr float DEG_TO_RAD_LOCAL =
    0.01745329251994329577F;

constexpr float RAD_TO_DEG_LOCAL =
    57.295779513082320876F;

constexpr float PI_LOCAL =
    3.14159265358979323846F;

constexpr float TWO_PI_LOCAL =
    6.28318530717958647692F;


// ============================================================
// MODELO IDENTIFICADO
// ============================================================

constexpr float MODEL_A =
    21.7F;

constexpr float MODEL_B =
    -3.28F;

constexpr float MODEL_C =
    -0.75F;


// ============================================================
// GANHOS PD
// ============================================================

constexpr float KP =
    60.93F;

constexpr float KD =
    8.96F;


// ============================================================
// SATURACAO
// ============================================================

constexpr float CONTROL_MAX_ACCEL_DEG_S2 =
    600.0F;


constexpr float CONTROL_MAX_ACCEL_RAD_S2 =
    CONTROL_MAX_ACCEL_DEG_S2
    *
    DEG_TO_RAD_LOCAL;


// ============================================================
// CAPTURA
// ============================================================
//
// Modificados apos o primeiro teste com referencia DOWN.
//
// A captura anterior iniciava o BALANCE somente quando beta
// ja estava aproximadamente em -2.6 graus.
//
// Agora:
//
//      READY    <= 1.5 deg
//      RELEASE  >= 0.05 rad/s
//      LOST     > 1.75 deg
//
// ============================================================

constexpr float CAPTURE_READY_BETA_DEG =
    1.5F;


constexpr float CAPTURE_READY_BETA_RAD =
    CAPTURE_READY_BETA_DEG
    *
    DEG_TO_RAD_LOCAL;


constexpr float CAPTURE_READY_BETADOT_RAD_S =
    0.25F;


// Tempo minimo perto do topo para confirmar READY.

constexpr uint32_t CAPTURE_READY_TIME_US =
    80000UL;


// Velocidade para detectar a soltura.
//
// O estimador apresentou niveis tipicos:
//
//      0.0411
//      0.0685
//      0.0822
//      0.1096
//
// Com 0.05 esperamos detectar normalmente ja em ~0.0685.

constexpr float RELEASE_VELOCITY_RAD_S =
    0.05F;


// Depois de READY nao permitimos grande afastamento
// antes de entregar o sistema ao PD.

constexpr float CAPTURE_RELEASE_BETA_DEG =
    1.75F;


constexpr float CAPTURE_RELEASE_BETA_RAD =
    CAPTURE_RELEASE_BETA_DEG
    *
    DEG_TO_RAD_LOCAL;


// ============================================================
// ABORT
// ============================================================

constexpr float BETA_ABORT_DEG =
    8.0F;


constexpr float BETA_ABORT_RAD =
    BETA_ABORT_DEG
    *
    DEG_TO_RAD_LOCAL;


constexpr float ARM_BALANCE_LIMIT_DEG =
    80.0F;


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

enum class ControlState
{
    IDLE,

    WAIT_CAPTURE,

    WAIT_RELEASE,

    BALANCE
};


ControlState controlState =
    ControlState::IDLE;


// ============================================================
// REFERENCIAS
// ============================================================

bool armZeroDefined =
    false;


// A biblioteca PendulumPosition sera calibrada fisicamente
// com o pendulo PARA BAIXO.
//
// Portanto sua referencia interna passa a representar DOWN.
//
// O main.cpp faz posteriormente o deslocamento de PI rad.

bool pendulumDownReferenceDefined =
    false;


// ============================================================
// TEMPOS
// ============================================================

uint32_t nextControlTimeUs =
    0;


uint32_t lastControlTimeUs =
    0;


uint32_t captureStableStartUs =
    0;


uint32_t balanceStartUs =
    0;


// ============================================================
// MEDIDAS
// ============================================================

float betaRadians =
    0.0F;


float betaDotRadiansPerSecond =
    0.0F;


// ============================================================
// CONTROLE
// ============================================================

float controlRawRadS2 =
    0.0F;


float controlLimitedRadS2 =
    0.0F;


float controlCommandDegS2 =
    0.0F;


// ============================================================
// DIAGNOSTICO
// ============================================================

float maximumAbsBetaRad =
    0.0F;


float maximumAbsBetaDotRadS =
    0.0F;


float maximumAbsControlDegS2 =
    0.0F;


uint32_t saturationCounter =
    0;


uint32_t controlSampleCounter =
    0;


// ============================================================
// SERIAL
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

float wrapToPi(
    float angle
);


float calculateBetaFromDownReference();


float limitFloat(
    float value,
    float minimum,
    float maximum
);


void serviceSerial();


void handleCommand(
    char *command
);


void controlTick(
    uint32_t nowUs
);


void serviceWaitCapture(
    uint32_t nowUs
);


void serviceWaitRelease(
    uint32_t nowUs
);


void startBalance(
    uint32_t nowUs
);


void serviceBalance(
    uint32_t nowUs,
    float dtSeconds,
    uint32_t dtUs
);


void abortBalance(
    const char *reason
);


void stopControl();


void definePendulumDownReference();


void defineArmZero();


void armBalance();


void emergencyStop();


void printStatus();


void printHelp();


void printBalanceSummary(
    const char *reason
);


const char *stateName();


// ============================================================
// WRAP ANGULAR
// ============================================================

float wrapToPi(
    float angle
)
{
    while (
        angle >
        PI_LOCAL
    )
    {
        angle -=
            TWO_PI_LOCAL;
    }


    while (
        angle <=
        -PI_LOCAL
    )
    {
        angle +=
            TWO_PI_LOCAL;
    }


    return angle;
}


// ============================================================
// CALCULO DE beta
// ============================================================
//
// A referencia interna da biblioteca:
//
//      beta_library = 0
//
// corresponde ao pendulo PARA BAIXO.
//
// Queremos:
//
//      beta = 0
//
// no TOPO.
//
// Portanto:
//
//      beta = wrap(beta_library - PI)
//
// ============================================================

float calculateBetaFromDownReference()
{
    if (
        !pendulumDownReferenceDefined
    )
    {
        return 0.0F;
    }


    const float betaFromDown =
        pendulumPosition.betaRadians();


    return wrapToPi(
        betaFromDown
        -
        PI_LOCAL
    );
}


// ============================================================
// LIMITADOR
// ============================================================

float limitFloat(
    float value,
    float minimum,
    float maximum
)
{
    if (
        value >
        maximum
    )
    {
        return maximum;
    }


    if (
        value <
        minimum
    )
    {
        return minimum;
    }


    return value;
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
        0.0F;


    betaDotRadiansPerSecond =
        0.0F;


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


    pendulumDownReferenceDefined =
        false;


    // --------------------------------------------------------
    // TIMING
    // --------------------------------------------------------

    lastControlTimeUs =
        micros();


    nextControlTimeUs =
        lastControlTimeUs
        +
        CONTROL_PERIOD_US;


    // --------------------------------------------------------
    // CABECALHO
    // --------------------------------------------------------

    Serial.println();


    Serial.println(
        F("========================================")
    );


    Serial.println(
        F("FASE 08 - CONTROLE PD")
    );


    Serial.println(
        F("Referencia pelo ponto inferior")
    );


    Serial.println(
        F("Captura automatica da vertical")
    );


    Serial.println(
        F("========================================")
    );


    Serial.println();


    Serial.println(
        F("Modelo:")
    );


    Serial.println(
        F("betaDDot = 21.7*beta -3.28*betaDot -0.75*u")
    );


    Serial.println();


    Serial.print(
        F("Kp = ")
    );


    Serial.println(
        KP,
        2
    );


    Serial.print(
        F("Kd = ")
    );


    Serial.println(
        KD,
        2
    );


    Serial.println();


    Serial.println(
        F("u = Kp*beta + Kd*betaDot")
    );


    Serial.println(
        F("Ts controle = 4 ms")
    );


    Serial.println(
        F("Saturacao = +/-600 deg/s2")
    );


    Serial.println(
        F("Abort beta = +/-8 deg")
    );


    Serial.println(
        F("Abort phi = +/-80 deg")
    );


    Serial.println();


    Serial.println(
        F("CAPTURA:")
    );


    Serial.print(
        F("READY beta = +/-")
    );


    Serial.print(
        CAPTURE_READY_BETA_DEG,
        2
    );


    Serial.println(
        F(" deg")
    );


    Serial.print(
        F("RELEASE betaDot = ")
    );


    Serial.print(
        RELEASE_VELOCITY_RAD_S,
        3
    );


    Serial.println(
        F(" rad/s")
    );


    Serial.print(
        F("CAPTURE_LOST beta = +/-")
    );


    Serial.print(
        CAPTURE_RELEASE_BETA_DEG,
        2
    );


    Serial.println(
        F(" deg")
    );


    Serial.println();


    Serial.println(
        F("IMPORTANTE:")
    );


    Serial.println(
        F("T deve ser executado com o pendulo")
    );


    Serial.println(
        F("LIVRE, PARADO e PARA BAIXO.")
    );


    Serial.println(
        F("O topo sera calculado a 180 graus.")
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
    // runSpeed precisa ser atendido frequentemente.

    motor.update();


    serviceSerial();


    motor.update();


    const uint32_t nowUs =
        micros();


    if (
        static_cast<int32_t>(
            nowUs
            -
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
            CONTROL_PERIOD_US;


        // Nao tentamos recuperar multiplos ciclos atrasados.

        if (
            static_cast<int32_t>(
                micros()
                -
                nextControlTimeUs
            )
            >=
            0
        )
        {
            nextControlTimeUs =
                micros()
                +
                CONTROL_PERIOD_US;
        }
    }


    motor.update();
}


// ============================================================
// TICK PRINCIPAL
// ============================================================

void controlTick(
    uint32_t nowUs
)
{
    const uint32_t dtUs =
        nowUs
        -
        lastControlTimeUs;


    const float dtSeconds =
        static_cast<float>(
            dtUs
        )
        *
        1.0e-6F;


    lastControlTimeUs =
        nowUs;


    // --------------------------------------------------------
    // SENSOR
    // --------------------------------------------------------

    pendulumPosition.update();


    if (
        pendulumDownReferenceDefined
    )
    {
        betaRadians =
            calculateBetaFromDownReference();


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
    }


    // --------------------------------------------------------
    // MAQUINA DE ESTADOS
    // --------------------------------------------------------

    switch (
        controlState
    )
    {
        case ControlState::IDLE:
        {
            break;
        }


        case ControlState::WAIT_CAPTURE:
        {
            serviceWaitCapture(
                nowUs
            );

            break;
        }


        case ControlState::WAIT_RELEASE:
        {
            serviceWaitRelease(
                nowUs
            );

            break;
        }


        case ControlState::BALANCE:
        {
            serviceBalance(
                nowUs,
                dtSeconds,
                dtUs
            );

            break;
        }
    }
}


// ============================================================
// WAIT_CAPTURE
// ============================================================

void serviceWaitCapture(
    uint32_t nowUs
)
{
    if (
        !pendulumVelocity.isReady()
    )
    {
        captureStableStartUs =
            0;


        return;
    }


    const bool betaOK =
        fabsf(
            betaRadians
        )
        <=
        CAPTURE_READY_BETA_RAD;


    const bool betaDotOK =
        fabsf(
            betaDotRadiansPerSecond
        )
        <=
        CAPTURE_READY_BETADOT_RAD_S;


    const bool phiOK =
        fabsf(
            motor.currentPositionDegrees()
        )
        <=
        5.0F;


    if (
        betaOK
        &&
        betaDotOK
        &&
        phiOK
    )
    {
        if (
            captureStableStartUs
            ==
            0
        )
        {
            captureStableStartUs =
                nowUs;
        }


        if (
            (
                nowUs
                -
                captureStableStartUs
            )
            >=
            CAPTURE_READY_TIME_US
        )
        {
            controlState =
                ControlState::WAIT_RELEASE;


            captureStableStartUs =
                0;


            Serial.println();


            Serial.println(
                F("CAPTURE_READY")
            );


            Serial.println(
                F("Topo reconhecido.")
            );


            Serial.print(
                F("beta [deg] = ")
            );


            Serial.println(
                betaRadians
                *
                RAD_TO_DEG_LOCAL,
                3
            );


            Serial.print(
                F("betaDot [rad/s] = ")
            );


            Serial.println(
                betaDotRadiansPerSecond,
                4
            );


            Serial.println();


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
        captureStableStartUs =
            0;
    }
}


// ============================================================
// WAIT_RELEASE
// ============================================================

void serviceWaitRelease(
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


    // --------------------------------------------------------
    // Perdeu a regiao de captura antes de detectar soltura.
    // --------------------------------------------------------

    if (
        absBeta
        >
        CAPTURE_RELEASE_BETA_RAD
    )
    {
        motor.stop();


        controlState =
            ControlState::WAIT_CAPTURE;


        captureStableStartUs =
            0;


        Serial.println();


        Serial.println(
            F("CAPTURE_LOST")
        );


        Serial.print(
            F("beta [deg] = ")
        );


        Serial.println(
            betaRadians
            *
            RAD_TO_DEG_LOCAL,
            3
        );


        Serial.println(
            F("Reposicione perto da vertical.")
        );


        Serial.println();


        return;
    }


    // --------------------------------------------------------
    // Detectar soltura.
    // --------------------------------------------------------

    if (
        fabsf(
            betaDotRadiansPerSecond
        )
        >=
        RELEASE_VELOCITY_RAD_S
    )
    {
        startBalance(
            nowUs
        );
    }
}


// ============================================================
// START BALANCE
// ============================================================

void startBalance(
    uint32_t nowUs
)
{
    motor.stop();


    balanceStartUs =
        nowUs;


    maximumAbsBetaRad =
        fabsf(
            betaRadians
        );


    maximumAbsBetaDotRadS =
        fabsf(
            betaDotRadiansPerSecond
        );


    maximumAbsControlDegS2 =
        0.0F;


    saturationCounter =
        0;


    controlSampleCounter =
        0;


    controlRawRadS2 =
        0.0F;


    controlLimitedRadS2 =
        0.0F;


    controlCommandDegS2 =
        0.0F;


    controlState =
        ControlState::BALANCE;


    Serial.println();


    Serial.println(
        F("========================================")
    );


    Serial.println(
        F("BALANCE ATIVO")
    );


    Serial.println(
        F("========================================")
    );


    Serial.print(
        F("beta inicial [deg] = ")
    );


    Serial.println(
        betaRadians
        *
        RAD_TO_DEG_LOCAL,
        3
    );


    Serial.print(
        F("betaDot inicial [rad/s] = ")
    );


    Serial.println(
        betaDotRadiansPerSecond,
        4
    );


    Serial.println();


    // O Serial acima e bloqueante.
    //
    // Atualizamos o timestamp para impedir que esse tempo
    // apareca como dt muito grande no primeiro ciclo PD.

    lastControlTimeUs =
        micros();


    nextControlTimeUs =
        lastControlTimeUs
        +
        CONTROL_PERIOD_US;
}


// ============================================================
// BALANCE
// ============================================================

void serviceBalance(
    uint32_t nowUs,
    float dtSeconds,
    uint32_t dtUs
)
{
    // ========================================================
    // TIMING
    // ========================================================

    if (
        dtUs
        >
        MAX_CONTROL_DT_US
    )
    {
        abortBalance(
            "CONTROL_OVERRUN"
        );


        return;
    }


    // ========================================================
    // SENSOR
    // ========================================================

    if (
        !pendulumVelocity.isReady()
    )
    {
        abortBalance(
            "VELOCITY_NOT_READY"
        );


        return;
    }


    // ========================================================
    // ESTATISTICAS DA MEDIDA ATUAL
    //
    // Atualizamos ANTES dos testes de abort.
    //
    // Isso corrige a pequena inconsistencia observada
    // anteriormente entre BETA_FINAL e MAX_ABS_BETA.
    // ========================================================

    const float absBeta =
        fabsf(
            betaRadians
        );


    if (
        absBeta
        >
        maximumAbsBetaRad
    )
    {
        maximumAbsBetaRad =
            absBeta;
    }


    const float absBetaDot =
        fabsf(
            betaDotRadiansPerSecond
        );


    if (
        absBetaDot
        >
        maximumAbsBetaDotRadS
    )
    {
        maximumAbsBetaDotRadS =
            absBetaDot;
    }


    // ========================================================
    // LIMITE DO PENDULO
    // ========================================================

    if (
        absBeta
        >=
        BETA_ABORT_RAD
    )
    {
        abortBalance(
            "BETA_LIMIT"
        );


        return;
    }


    // ========================================================
    // LIMITE DO BRACO
    // ========================================================

    const float phiDegrees =
        motor.currentPositionDegrees();


    if (
        fabsf(
            phiDegrees
        )
        >=
        ARM_BALANCE_LIMIT_DEG
    )
    {
        abortBalance(
            "ARM_LIMIT"
        );


        return;
    }


    // ========================================================
    // PD
    //
    // u [rad/s2]
    //
    // u = Kp*beta + Kd*betaDot
    //
    // MODEL_C < 0
    //
    // Portanto ganhos positivos fornecem a realimentacao
    // estabilizante para a convencao identificada.
    // ========================================================

    controlRawRadS2 =
        KP
        *
        betaRadians
        +
        KD
        *
        betaDotRadiansPerSecond;


    // ========================================================
    // SATURACAO
    // ========================================================

    controlLimitedRadS2 =
        limitFloat(
            controlRawRadS2,
            -CONTROL_MAX_ACCEL_RAD_S2,
            CONTROL_MAX_ACCEL_RAD_S2
        );


    if (
        fabsf(
            controlRawRadS2
            -
            controlLimitedRadS2
        )
        >
        0.0001F
    )
    {
        saturationCounter++;
    }


    // ========================================================
    // rad/s2 -> deg/s2
    // ========================================================

    controlCommandDegS2 =
        controlLimitedRadS2
        *
        RAD_TO_DEG_LOCAL;


    const float absControl =
        fabsf(
            controlCommandDegS2
        );


    if (
        absControl
        >
        maximumAbsControlDegS2
    )
    {
        maximumAbsControlDegS2 =
            absControl;
    }


    // ========================================================
    // MOTOR
    // ========================================================

    const bool accepted =
        motor.commandAcceleration(
            controlCommandDegS2,
            dtSeconds
        );


    if (
        !accepted
    )
    {
        abortBalance(
            "MOTOR_COMMAND_REJECTED"
        );


        return;
    }


    motor.update();


    controlSampleCounter++;
}


// ============================================================
// ABORT
// ============================================================

void abortBalance(
    const char *reason
)
{
    motor.stop();


    controlRawRadS2 =
        0.0F;


    controlLimitedRadS2 =
        0.0F;


    controlCommandDegS2 =
        0.0F;


    printBalanceSummary(
        reason
    );


    controlState =
        ControlState::IDLE;


    captureStableStartUs =
        0;


    Serial.println(
        F("BALANCE encerrado.")
    );


    Serial.println(
        F("Motor continua habilitado, velocidade zerada.")
    );


    Serial.println(
        F("Use B para novo teste ou D para desabilitar.")
    );


    Serial.println();
}


// ============================================================
// STOP
// ============================================================

void stopControl()
{
    motor.stop();


    if (
        controlState
        ==
        ControlState::BALANCE
    )
    {
        printBalanceSummary(
            "USER_STOP"
        );
    }


    controlState =
        ControlState::IDLE;


    captureStableStartUs =
        0;


    controlRawRadS2 =
        0.0F;


    controlLimitedRadS2 =
        0.0F;


    controlCommandDegS2 =
        0.0F;


    Serial.println(
        F("Controle parado.")
    );
}


// ============================================================
// RESUMO
// ============================================================

void printBalanceSummary(
    const char *reason
)
{
    const uint32_t elapsedUs =
        micros()
        -
        balanceStartUs;


    Serial.println();


    Serial.println(
        F("# BALANCE_END")
    );


    Serial.print(
        F("# REASON=")
    );


    Serial.println(
        reason
    );


    Serial.print(
        F("# TIME_MS=")
    );


    Serial.println(
        elapsedUs
        /
        1000UL
    );


    Serial.print(
        F("# SAMPLES=")
    );


    Serial.println(
        controlSampleCounter
    );


    Serial.print(
        F("# BETA_FINAL_DEG=")
    );


    Serial.println(
        betaRadians
        *
        RAD_TO_DEG_LOCAL,
        3
    );


    Serial.print(
        F("# BETADOT_FINAL_RAD_S=")
    );


    Serial.println(
        betaDotRadiansPerSecond,
        4
    );


    Serial.print(
        F("# PHI_FINAL_DEG=")
    );


    Serial.println(
        motor.currentPositionDegrees(),
        3
    );


    Serial.print(
        F("# MAX_ABS_BETA_DEG=")
    );


    Serial.println(
        maximumAbsBetaRad
        *
        RAD_TO_DEG_LOCAL,
        3
    );


    Serial.print(
        F("# MAX_ABS_BETADOT_RAD_S=")
    );


    Serial.println(
        maximumAbsBetaDotRadS,
        4
    );


    Serial.print(
        F("# MAX_ABS_U_DEG_S2=")
    );


    Serial.println(
        maximumAbsControlDegS2,
        1
    );


    Serial.print(
        F("# SATURATION_COUNT=")
    );


    Serial.println(
        saturationCounter
    );


    Serial.println();
}


// ============================================================
// ARMAR BALANCE
// ============================================================

void armBalance()
{
    if (
        controlState
        !=
        ControlState::IDLE
    )
    {
        Serial.println(
            F("ERRO: sistema ocupado.")
        );


        return;
    }


    if (
        !pendulumDownReferenceDefined
    )
    {
        Serial.println(
            F("ERRO: calibre a referencia DOWN com T.")
        );


        return;
    }


    if (
        !armZeroDefined
    )
    {
        Serial.println(
            F("ERRO: defina phi=0 com Z.")
        );


        return;
    }


    if (
        !motor.isEnabled()
    )
    {
        Serial.println(
            F("ERRO: habilite motor com E.")
        );


        return;
    }


    if (
        fabsf(
            motor.currentPositionDegrees()
        )
        >
        5.0F
    )
    {
        Serial.println(
            F("ERRO: braco fora da regiao inicial.")
        );


        return;
    }


    motor.stop();


    captureStableStartUs =
        0;


    controlState =
        ControlState::WAIT_CAPTURE;


    Serial.println();


    Serial.println(
        F("BALANCE ARMADO.")
    );


    Serial.println(
        F("Leve o pendulo manualmente para perto da vertical.")
    );


    Serial.println();


    Serial.print(
        F("CAPTURE_READY se |beta| <= ")
    );


    Serial.print(
        CAPTURE_READY_BETA_DEG,
        2
    );


    Serial.println(
        F(" deg")
    );


    Serial.print(
        F("e |betaDot| <= ")
    );


    Serial.print(
        CAPTURE_READY_BETADOT_RAD_S,
        2
    );


    Serial.println(
        F(" rad/s")
    );


    Serial.print(
        F("por ")
    );


    Serial.print(
        CAPTURE_READY_TIME_US
        /
        1000UL
    );


    Serial.println(
        F(" ms.")
    );


    Serial.println();


    Serial.println(
        F("Quando aparecer CAPTURE_READY, solte.")
    );


    Serial.println();
}


// ============================================================
// REFERENCIA INFERIOR
// ============================================================

void definePendulumDownReference()
{
    if (
        controlState
        !=
        ControlState::IDLE
    )
    {
        Serial.println(
            F("ERRO: pare o controle antes de T.")
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
        F("Calibrando referencia inferior...")
    );


    Serial.println(
        F("Deixe o pendulo LIVRE, PARADO e PARA BAIXO.")
    );


    // Utilizamos a rotina ja validada da biblioteca de posicao.
    //
    // Ela acredita estar definindo o "top".
    // Aqui, deliberadamente, essa referencia sera DOWN.
    //
    // O deslocamento de PI e feito depois no main.cpp.

    pendulumPosition.calibrateTop(
        32,
        2
    );


    pendulumPosition.update();


    pendulumDownReferenceDefined =
        pendulumPosition.topIsDefined();


    if (
        !pendulumDownReferenceDefined
    )
    {
        Serial.println(
            F("ERRO ao definir referencia inferior.")
        );


        return;
    }


    betaRadians =
        calculateBetaFromDownReference();


    const uint32_t nowUs =
        micros();


    pendulumVelocity.reset(
        betaRadians,
        nowUs
    );


    betaDotRadiansPerSecond =
        0.0F;


    Serial.println();


    Serial.println(
        F("Referencia inferior definida.")
    );


    Serial.println(
        F("TOP calculado automaticamente a 180 graus.")
    );


    Serial.println();


    Serial.print(
        F("beta atual esperado [deg] ~= +/-180: ")
    );


    Serial.println(
        betaRadians
        *
        RAD_TO_DEG_LOCAL,
        3
    );


    Serial.println();


    Serial.println(
        F("Nao execute T novamente no topo.")
    );


    Serial.println();
}


// ============================================================
// ZERO DO BRACO
// ============================================================

void defineArmZero()
{
    if (
        controlState
        !=
        ControlState::IDLE
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
            F("ERRO: desabilite o motor antes de Z.")
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
// EMERGENCY STOP
// ============================================================

void emergencyStop()
{
    motor.emergencyStop();


    controlState =
        ControlState::IDLE;


    controlRawRadS2 =
        0.0F;


    controlLimitedRadS2 =
        0.0F;


    controlCommandDegS2 =
        0.0F;


    captureStableStartUs =
        0;


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
        F("Referencia do braco perdida.")
    );


    Serial.println();
}


// ============================================================
// SERIAL
// ============================================================

void serviceSerial()
{
    while (
        Serial.available()
        >
        0
    )
    {
        const char character =
            static_cast<char>(
                Serial.read()
            );


        if (
            character
            ==
            '\r'
        )
        {
            continue;
        }


        if (
            character
            ==
            '\n'
        )
        {
            if (
                serialBufferIndex
                >
                0
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


    // --------------------------------------------------------
    // DOWN
    // --------------------------------------------------------

    if (
        strcmp(command, "T") == 0
        ||
        strcmp(command, "DOWN") == 0
    )
    {
        definePendulumDownReference();


        return;
    }


    // --------------------------------------------------------
    // ZERO
    // --------------------------------------------------------

    if (
        strcmp(command, "Z") == 0
        ||
        strcmp(command, "ZERO") == 0
    )
    {
        defineArmZero();


        return;
    }


    // --------------------------------------------------------
    // ENABLE
    // --------------------------------------------------------

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


    // --------------------------------------------------------
    // DISABLE
    // --------------------------------------------------------

    if (
        strcmp(command, "D") == 0
        ||
        strcmp(command, "DISABLE") == 0
    )
    {
        stopControl();


        motor.disable();


        armZeroDefined =
            false;


        Serial.println(
            F("Motor desabilitado.")
        );


        Serial.println(
            F("Referencia do braco perdida.")
        );


        return;
    }


    // --------------------------------------------------------
    // BALANCE
    // --------------------------------------------------------

    if (
        strcmp(command, "B") == 0
        ||
        strcmp(command, "BALANCE") == 0
    )
    {
        armBalance();


        return;
    }


    // --------------------------------------------------------
    // STOP
    // --------------------------------------------------------

    if (
        strcmp(command, "STOP") == 0
    )
    {
        stopControl();


        return;
    }


    // --------------------------------------------------------
    // EMERGENCY
    // --------------------------------------------------------

    if (
        strcmp(command, "X") == 0
    )
    {
        emergencyStop();


        return;
    }


    // --------------------------------------------------------
    // STATUS
    // --------------------------------------------------------

    if (
        strcmp(command, "S") == 0
        ||
        strcmp(command, "STATUS") == 0
    )
    {
        printStatus();


        return;
    }


    // --------------------------------------------------------
    // HELP
    // --------------------------------------------------------

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
    // Atualiza a medida imediatamente antes de imprimir.

    pendulumPosition.update();


    if (
        pendulumDownReferenceDefined
    )
    {
        betaRadians =
            calculateBetaFromDownReference();
    }


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
        F("Referencia DOWN: ")
    );


    Serial.println(
        pendulumDownReferenceDefined
        ?
        F("DEFINIDA")
        :
        F("NAO DEFINIDA")
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
        *
        RAD_TO_DEG_LOCAL,
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
        F("phiDot cmd [deg/s]: ")
    );


    Serial.println(
        motor.speedReferenceDegreesPerSecond(),
        3
    );


    Serial.print(
        F("u [deg/s2]: ")
    );


    Serial.println(
        controlCommandDegS2,
        2
    );


    Serial.println(
        F("----------------------------")
    );


    Serial.println();
}


// ============================================================
// STATE NAME
// ============================================================

const char *stateName()
{
    switch (
        controlState
    )
    {
        case ControlState::IDLE:
        {
            return "IDLE";
        }


        case ControlState::WAIT_CAPTURE:
        {
            return "WAIT_CAPTURE";
        }


        case ControlState::WAIT_RELEASE:
        {
            return "WAIT_RELEASE";
        }


        case ControlState::BALANCE:
        {
            return "BALANCE";
        }
    }


    return "UNKNOWN";
}


// ============================================================
// HELP
// ============================================================

void printHelp()
{
    Serial.println(
        F("COMANDOS:")
    );


    Serial.println(
        F("T       = calibrar com pendulo PARA BAIXO")
    );


    Serial.println(
        F("Z       = definir phi=0")
    );


    Serial.println(
        F("E       = habilitar motor")
    );


    Serial.println(
        F("D       = desabilitar motor")
    );


    Serial.println(
        F("B       = armar captura + BALANCE")
    );


    Serial.println(
        F("STOP    = parar controle")
    );


    Serial.println(
        F("X       = emergency stop")
    );


    Serial.println(
        F("S       = status")
    );


    Serial.println(
        F("H       = ajuda")
    );


    Serial.println();


    Serial.println(
        F("CALIBRACAO:")
    );


    Serial.println(
        F("1. Pendulo livre e parado para baixo.")
    );


    Serial.println(
        F("2. Envie T.")
    );


    Serial.println(
        F("3. O topo sera calculado a 180 graus.")
    );


    Serial.println();


    Serial.println(
        F("CAPTURA:")
    );


    Serial.println(
        F("1. Envie B.")
    );


    Serial.println(
        F("2. Leve o pendulo perto do topo.")
    );


    Serial.println(
        F("3. Aguarde CAPTURE_READY.")
    );


    Serial.println(
        F("4. Solte o pendulo.")
    );


    Serial.println();


    Serial.print(
        F("READY beta +/-")
    );


    Serial.print(
        CAPTURE_READY_BETA_DEG,
        2
    );


    Serial.println(
        F(" deg")
    );


    Serial.print(
        F("RELEASE betaDot ")
    );


    Serial.print(
        RELEASE_VELOCITY_RAD_S,
        3
    );


    Serial.println(
        F(" rad/s")
    );


    Serial.print(
        F("CAPTURE_LOST beta +/-")
    );


    Serial.print(
        CAPTURE_RELEASE_BETA_DEG,
        2
    );


    Serial.println(
        F(" deg")
    );


    Serial.println();


    Serial.print(
        F("Kp = ")
    );


    Serial.println(
        KP,
        2
    );


    Serial.print(
        F("Kd = ")
    );


    Serial.println(
        KD,
        2
    );


    Serial.println();


    Serial.println(
        F("Abort beta = +/-8 deg.")
    );


    Serial.println(
        F("Abort phi = +/-80 deg.")
    );


    Serial.println(
        F("u limitado a +/-600 deg/s2.")
    );


    Serial.println();
}