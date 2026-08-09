#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>

#include <PendulumPosition.h>
#include <PendulumVelocity.h>
#include <MotorPosition.h>

#include <math.h>


// ============================================================
// FASE 12B.4
//
// Objetivo:
//
//   PEAK-
//      -> DOWN+
//      -> motor: 0 -> +8 graus
//      -> espera motor TERMINAR
//      -> espera PEAK+ posterior
//      -> mede Eafter
//      -> retorna motor para 0
//
// Mantemos os mesmos parametros da 12B.3.
// Mudamos apenas o lado/sinal do ensaio.
// ============================================================


// ============================================================
// AQUISICAO
// ============================================================

constexpr uint32_t SAMPLE_PERIOD_US = 4000UL;   // 250 Hz


// ============================================================
// HARDWARE DO MOTOR
// ============================================================

constexpr uint8_t STEP_PIN   = 6;
constexpr uint8_t DIR_PIN    = 8;
constexpr uint8_t ENABLE_PIN = 4;

constexpr uint16_t FULL_STEPS_PER_REV = 200;
constexpr uint8_t MICROSTEP = 8;


// ============================================================
// LIMITES DO BRACO
// ============================================================

constexpr float ARM_MIN_DEG = -80.0F;
constexpr float ARM_MAX_DEG =  80.0F;


// ============================================================
// MOTOR
// ============================================================

constexpr float MICROSTEPS_PER_DEG =
    (FULL_STEPS_PER_REV * MICROSTEP) / 360.0F;

constexpr float MOTOR_MAX_SPEED_DEG_S = 180.0F;
constexpr float MOTOR_ACCEL_DEG_S2    = 600.0F;

constexpr float MOTOR_MAX_SPEED_STEPS_S =
    MOTOR_MAX_SPEED_DEG_S * MICROSTEPS_PER_DEG;

constexpr float MOTOR_ACCEL_STEPS_S2 =
    MOTOR_ACCEL_DEG_S2 * MICROSTEPS_PER_DEG;


// ============================================================
// MOVIMENTO EXPERIMENTAL
//
// Agora:
//
//      DOWN+ -> +8 graus
// ============================================================

constexpr float IMPULSE_TARGET_DEG = +8.0F;


// ============================================================
// DETECCAO DO PENDULO
// ============================================================

constexpr float DOWN_ZONE_DEG      = 3.0F;
constexpr float PEAK_SPEED_RAD_S   = 0.20F;
constexpr float MIN_PEAK_ANGLE_DEG = 5.0F;


// ============================================================
// CALIBRACAO DOWN
// ============================================================

constexpr uint32_t DOWN_CALIBRATION_TIME_MS = 3000UL;
constexpr float MAX_CALIBRATION_SPAN_DEG = 20.0F;


// ============================================================
// OBJETOS
// ============================================================

AS5600 as5600;

PendulumPosition position(
    as5600,
    1.0F
);

PendulumVelocity velocity;

MotorPosition motor(
    STEP_PIN,
    DIR_PIN,
    ENABLE_PIN,
    FULL_STEPS_PER_REV,
    MICROSTEP,
    ARM_MIN_DEG,
    ARM_MAX_DEG
);


// ============================================================
// ESTADOS
// ============================================================

enum class Mode
{
    WAITING,
    CALIBRATING,
    READY
};


enum class TestState
{
    IDLE,

    WAIT_BEFORE_PEAK,

    WAIT_DOWN,

    WAIT_IMPULSE_END,

    WAIT_AFTER_PEAK,

    RETURNING_ZERO
};


Mode mode = Mode::WAITING;
TestState testState = TestState::IDLE;


// ============================================================
// REFERENCIA DO BRACO
// ============================================================

bool armReferenceDefined = false;


// ============================================================
// TEMPO
// ============================================================

uint32_t nextSampleUs = 0;
uint32_t startUs = 0;

uint32_t impulseStartUs = 0;
uint32_t impulseEndUs = 0;


// ============================================================
// REFERENCIA DOWN
// ============================================================

float downOffsetRad = 0.0F;


// ============================================================
// CALIBRACAO DOWN
// ============================================================

uint32_t calibrationStartUs = 0;
uint32_t calibrationSamples = 0;

float sumSin = 0.0F;
float sumCos = 0.0F;

float calibrationMinDeg = 0.0F;
float calibrationMaxDeg = 0.0F;


// ============================================================
// DETECTOR DE DOWN
// ============================================================

bool downPositiveArmed = false;
bool downNegativeArmed = false;


// ============================================================
// DETECTOR DE PEAK
// ============================================================

bool peakPositiveArmed = false;
bool peakNegativeArmed = false;

float maxAlphaDeg = 0.0F;
float minAlphaDeg = 0.0F;


// ============================================================
// DADOS DO ENSAIO
// ============================================================

float energyBefore = 0.0F;
float triggerAlphaDot = 0.0F;


// ============================================================
// FUNCOES AUXILIARES
// ============================================================

float wrapToPi(float angle)
{
    while (angle >= PI)
    {
        angle -= TWO_PI;
    }

    while (angle < -PI)
    {
        angle += TWO_PI;
    }

    return angle;
}


float correctedAlpha()
{
    return wrapToPi(
        position.betaRadians()
        -
        downOffsetRad
    );
}


// ============================================================
// ENERGIA NORMALIZADA NO PICO
// ============================================================

float peakEnergy(float alphaDeg)
{
    const float alphaRad =
        alphaDeg * DEG_TO_RAD;

    return 0.5F *
           (1.0F - cosf(alphaRad));
}


// ============================================================
// REINICIA DETECTORES
// ============================================================

void resetEventDetectors()
{
    downPositiveArmed = false;
    downNegativeArmed = false;

    peakPositiveArmed = false;
    peakNegativeArmed = false;

    maxAlphaDeg = 0.0F;
    minAlphaDeg = 0.0F;
}


// ============================================================
// FINALIZA ENSAIO
// ============================================================

void finishTest(
    float energyAfter,
    float peakAfterDeg,
    uint32_t nowUs
)
{
    const float deltaEnergy =
        energyAfter - energyBefore;


    const float deltaPercent =
        energyBefore > 0.0F
            ?
            100.0F *
            deltaEnergy /
            energyBefore
            :
            0.0F;


    Serial.println();


    Serial.print(
        F("# AFTER,PEAK+,E=")
    );

    Serial.print(
        energyAfter,
        6
    );


    Serial.print(
        F(",angle=")
    );

    Serial.println(
        peakAfterDeg,
        2
    );


    Serial.print(
        F("# RESULT,Ebefore=")
    );

    Serial.print(
        energyBefore,
        6
    );


    Serial.print(
        F(",Eafter=")
    );

    Serial.print(
        energyAfter,
        6
    );


    Serial.print(
        F(",dE=")
    );

    Serial.print(
        deltaEnergy,
        6
    );


    Serial.print(
        F(",dE_percent=")
    );

    Serial.print(
        deltaPercent,
        2
    );


    Serial.print(
        F(",triggerAlphaDot=")
    );

    Serial.print(
        triggerAlphaDot,
        4
    );


    Serial.print(
        F(",phi=")
    );

    Serial.print(
        motor.currentPosition(),
        2
    );


    Serial.print(
        F(",motorMoving=")
    );

    Serial.println(
        motor.isMoving() ? 1 : 0
    );


    Serial.println(
        F("# TEST_END")
    );


    // ========================================================
    // RETORNO PARA ZERO
    //
    // Somente depois de medir Eafter.
    // ========================================================

    if (
        motor.moveTo(0.0F)
    )
    {
        Serial.println(
            F("# ARM_RETURN,target=0.0")
        );


        testState =
            TestState::RETURNING_ZERO;
    }
    else
    {
        Serial.println(
            F("# ARM_RETURN_ERROR")
        );


        testState =
            TestState::IDLE;
    }
}


// ============================================================
// PROCESSA PEAK
// ============================================================

void handlePeak(
    bool positive,
    float peakDeg,
    uint32_t nowUs
)
{
    const float energy =
        peakEnergy(peakDeg);


    // ========================================================
    // TELEMETRIA
    // ========================================================

    Serial.print(
        F("# PEAK,")
    );

    Serial.print(
        positive ? '+' : '-'
    );


    Serial.print(
        F(",t=")
    );

    Serial.print(
        (nowUs - startUs) / 1000UL
    );


    Serial.print(
        F(",angle=")
    );

    Serial.print(
        peakDeg,
        2
    );


    Serial.print(
        F(",energy=")
    );

    Serial.println(
        energy,
        6
    );


    // ========================================================
    // BEFORE
    //
    // Agora o ensaio comeca somente em PEAK-
    // ========================================================

    if (
        testState ==
        TestState::WAIT_BEFORE_PEAK
    )
    {
        // Ignora PEAK+
        if (positive)
        {
            return;
        }


        energyBefore =
            energy;


        Serial.print(
            F("# BEFORE,PEAK-,E=")
        );

        Serial.println(
            energyBefore,
            6
        );


        testState =
            TestState::WAIT_DOWN;


        return;
    }


    // ========================================================
    // PICO DURANTE MOVIMENTO DO MOTOR
    //
    // Nao pode ser usado como AFTER.
    // ========================================================

    if (
        testState ==
        TestState::WAIT_IMPULSE_END
    )
    {
        Serial.print(
            F("# PEAK_IGNORED,")
        );

        Serial.print(
            positive ? "PEAK+" : "PEAK-"
        );

        Serial.println(
            F(",reason=motor_moving")
        );


        return;
    }


    // ========================================================
    // AFTER
    //
    // Agora aceitamos apenas PEAK+
    // posterior ao termino do movimento.
    // ========================================================

    if (
        testState ==
        TestState::WAIT_AFTER_PEAK
    )
    {
        // Ignora PEAK-
        if (!positive)
        {
            return;
        }


        if (motor.isMoving())
        {
            Serial.println(
                F("# PEAK_IGNORED,reason=motor_moving")
            );

            return;
        }


        finishTest(
            energy,
            peakDeg,
            nowUs
        );
    }
}


// ============================================================
// PROCESSA DOWN
// ============================================================

void handleDown(
    bool positive,
    float alphaDot,
    uint32_t nowUs
)
{
    Serial.print(
        F("# DOWN,")
    );

    Serial.print(
        positive ? '+' : '-'
    );


    Serial.print(
        F(",t=")
    );

    Serial.print(
        (nowUs - startUs) / 1000UL
    );


    Serial.print(
        F(",alphaDot=")
    );

    Serial.println(
        alphaDot,
        4
    );


    if (
        testState !=
        TestState::WAIT_DOWN
    )
    {
        return;
    }


    // ========================================================
    // Ensaio 12B.4:
    //
    // PEAK- -> DOWN+
    //
    // Portanto ignoramos DOWN-
    // ========================================================

    if (!positive)
    {
        return;
    }


    // ========================================================
    // VALIDACOES
    // ========================================================

    if (!armReferenceDefined)
    {
        Serial.println(
            F("# IMPULSE_ERROR,arm_reference_lost")
        );

        testState =
            TestState::IDLE;

        return;
    }


    if (!motor.isEnabled())
    {
        Serial.println(
            F("# IMPULSE_ERROR,motor_disabled")
        );

        testState =
            TestState::IDLE;

        return;
    }


    if (motor.isMoving())
    {
        Serial.println(
            F("# IMPULSE_ERROR,motor_already_moving")
        );

        testState =
            TestState::IDLE;

        return;
    }


    // ========================================================
    // TRIGGER
    // ========================================================

    triggerAlphaDot =
        alphaDot;


    impulseStartUs =
        nowUs;


    Serial.print(
        F("# TRIGGER,DOWN+,t=")
    );

    Serial.print(
        (nowUs - startUs) / 1000UL
    );


    Serial.print(
        F(",alphaDot=")
    );

    Serial.print(
        alphaDot,
        4
    );


    Serial.print(
        F(",phi=")
    );

    Serial.println(
        motor.currentPosition(),
        2
    );


    // ========================================================
    // MOVIMENTO REAL
    //
    // 0 -> +8 graus
    // ========================================================

    if (
        !motor.moveTo(
            IMPULSE_TARGET_DEG
        )
    )
    {
        Serial.println(
            F("# IMPULSE_ERROR,move_rejected")
        );

        testState =
            TestState::IDLE;

        return;
    }


    Serial.print(
        F("# IMPULSE,target=")
    );

    Serial.print(
        IMPULSE_TARGET_DEG,
        2
    );


    Serial.print(
        F(",phi0=")
    );

    Serial.println(
        motor.currentPosition(),
        2
    );


    testState =
        TestState::WAIT_IMPULSE_END;
}


// ============================================================
// DETECCAO DE EVENTOS
// ============================================================

void detectEvents(
    float alpha,
    float alphaDot,
    uint32_t nowUs
)
{
    const float alphaDeg =
        alpha * RAD_TO_DEG;


    // ========================================================
    // DOWN
    // ========================================================

    if (
        alphaDeg <= -DOWN_ZONE_DEG
    )
    {
        downPositiveArmed = true;
    }


    if (
        alphaDeg >= DOWN_ZONE_DEG
    )
    {
        downNegativeArmed = true;
    }


    // DOWN+
    if (
        downPositiveArmed &&
        alphaDeg >= 0.0F &&
        alphaDot > 0.0F
    )
    {
        downPositiveArmed =
            false;


        handleDown(
            true,
            alphaDot,
            nowUs
        );
    }


    // DOWN-
    if (
        downNegativeArmed &&
        alphaDeg <= 0.0F &&
        alphaDot < 0.0F
    )
    {
        downNegativeArmed =
            false;


        handleDown(
            false,
            alphaDot,
            nowUs
        );
    }


    // ========================================================
    // PEAK+
    // ========================================================

    if (
        alphaDeg >
        MIN_PEAK_ANGLE_DEG
        &&
        alphaDot >
        PEAK_SPEED_RAD_S
    )
    {
        if (!peakPositiveArmed)
        {
            peakPositiveArmed =
                true;


            maxAlphaDeg =
                alphaDeg;
        }


        if (
            alphaDeg >
            maxAlphaDeg
        )
        {
            maxAlphaDeg =
                alphaDeg;
        }
    }


    if (
        peakPositiveArmed &&
        alphaDot <= 0.0F
    )
    {
        peakPositiveArmed =
            false;


        handlePeak(
            true,
            maxAlphaDeg,
            nowUs
        );
    }


    // ========================================================
    // PEAK-
    // ========================================================

    if (
        alphaDeg <
        -MIN_PEAK_ANGLE_DEG
        &&
        alphaDot <
        -PEAK_SPEED_RAD_S
    )
    {
        if (!peakNegativeArmed)
        {
            peakNegativeArmed =
                true;


            minAlphaDeg =
                alphaDeg;
        }


        if (
            alphaDeg <
            minAlphaDeg
        )
        {
            minAlphaDeg =
                alphaDeg;
        }
    }


    if (
        peakNegativeArmed &&
        alphaDot >= 0.0F
    )
    {
        peakNegativeArmed =
            false;


        handlePeak(
            false,
            minAlphaDeg,
            nowUs
        );
    }
}


// ============================================================
// INICIA CALIBRACAO DE DOWN
// ============================================================

void startCalibration()
{
    Serial.println();


    if (
        !position.calibrateTop(
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


    const uint32_t now =
        micros();


    velocity.reset(
        position.betaRadians(),
        now
    );


    sumSin = 0.0F;
    sumCos = 0.0F;

    calibrationSamples = 0;

    calibrationMinDeg =
        1000.0F;

    calibrationMaxDeg =
        -1000.0F;


    calibrationStartUs =
        now;


    nextSampleUs =
        now + SAMPLE_PERIOD_US;


    mode =
        Mode::CALIBRATING;


    testState =
        TestState::IDLE;


    resetEventDetectors();


    Serial.println(
        F("Calibrando centro de DOWN por 3 s...")
    );
}


// ============================================================
// PROCESSA CALIBRACAO
// ============================================================

void processCalibration()
{
    if (!position.update())
    {
        return;
    }


    const uint32_t now =
        micros();


    const float alpha =
        position.betaRadians();


    const float alphaDeg =
        alpha * RAD_TO_DEG;


    sumSin +=
        sinf(alpha);


    sumCos +=
        cosf(alpha);


    ++calibrationSamples;


    if (
        alphaDeg <
        calibrationMinDeg
    )
    {
        calibrationMinDeg =
            alphaDeg;
    }


    if (
        alphaDeg >
        calibrationMaxDeg
    )
    {
        calibrationMaxDeg =
            alphaDeg;
    }


    if (
        now - calibrationStartUs
        <
        DOWN_CALIBRATION_TIME_MS
        * 1000UL
    )
    {
        return;
    }


    const float spanDeg =
        calibrationMaxDeg -
        calibrationMinDeg;


    if (
        spanDeg >
        MAX_CALIBRATION_SPAN_DEG
    )
    {
        Serial.print(
            F("CALIBRACAO REJEITADA. Span=")
        );


        Serial.print(
            spanDeg,
            2
        );


        Serial.println(
            F(" deg")
        );


        mode =
            Mode::WAITING;


        return;
    }


    // ========================================================
    // CENTRO MEDIO
    // ========================================================

    downOffsetRad =
        atan2f(
            sumSin,
            sumCos
        );


    Serial.print(
        F("DOWN estimado = ")
    );


    Serial.print(
        downOffsetRad *
        RAD_TO_DEG,
        3
    );


    Serial.println(
        F(" deg")
    );


    Serial.print(
        F("Span observado = ")
    );


    Serial.print(
        spanDeg,
        2
    );


    Serial.println(
        F(" deg")
    );


    // ========================================================
    // REINICIA VELOCIDADE
    // ========================================================

    const float alphaCorrected =
        correctedAlpha();


    velocity.reset(
        alphaCorrected,
        now
    );


    resetEventDetectors();


    startUs =
        now;


    nextSampleUs =
        now + SAMPLE_PERIOD_US;


    mode =
        Mode::READY;


    testState =
        TestState::IDLE;


    Serial.println(
        F("DOWN definido.")
    );


    Serial.println(
        F("A = armar ensaio 12B.4")
    );
}


// ============================================================
// AQUISICAO
// ============================================================

void acquireSample()
{
    if (!position.update())
    {
        return;
    }


    const uint32_t now =
        micros();


    const float alpha =
        correctedAlpha();


    velocity.update(
        alpha,
        now
    );


    if (!velocity.isReady())
    {
        return;
    }


    // ========================================================
    // SEM ENSAIO ARMADO:
    //
    // sensor e velocidade continuam atualizados,
    // mas nao registramos eventos.
    // ========================================================

    if (
        testState == TestState::IDLE
        ||
        testState == TestState::RETURNING_ZERO
    )
    {
        return;
    }


    detectEvents(
        alpha,
        velocity.radiansPerSecond(),
        now
    );
}


// ============================================================
// MONITORA TERMINO DO IMPULSO
// ============================================================

void updateImpulseState()
{
    if (
        testState !=
        TestState::WAIT_IMPULSE_END
    )
    {
        return;
    }


    if (motor.isMoving())
    {
        return;
    }


    impulseEndUs =
        micros();


    const float durationMs =
        (impulseEndUs - impulseStartUs)
        / 1000.0F;


    Serial.print(
        F("# IMPULSE_DONE,t=")
    );


    Serial.print(
        (impulseEndUs - startUs)
        / 1000UL
    );


    Serial.print(
        F(",duration_ms=")
    );


    Serial.print(
        durationMs,
        1
    );


    Serial.print(
        F(",phi=")
    );


    Serial.println(
        motor.currentPosition(),
        2
    );


    testState =
        TestState::WAIT_AFTER_PEAK;
}


// ============================================================
// STATUS
// ============================================================

void printStatus()
{
    Serial.println();


    Serial.print(
        F("Motor: ")
    );


    Serial.println(
        motor.isEnabled()
            ?
            F("ENABLED")
            :
            F("DISABLED")
    );


    Serial.print(
        F("Referencia braco: ")
    );


    Serial.println(
        armReferenceDefined
            ?
            F("OK")
            :
            F("NAO DEFINIDA")
    );


    Serial.print(
        F("phi = ")
    );


    Serial.print(
        motor.currentPosition(),
        2
    );


    Serial.println(
        F(" deg")
    );


    Serial.print(
        F("target = ")
    );


    Serial.print(
        motor.targetPosition(),
        2
    );


    Serial.println(
        F(" deg")
    );


    Serial.print(
        F("moving = ")
    );


    Serial.println(
        motor.isMoving()
            ?
            F("SIM")
            :
            F("NAO")
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


    // --------------------------------------------------------
    // CALIBRAR DOWN
    // --------------------------------------------------------

    if (
        c == 'T' ||
        c == 't'
    )
    {
        startCalibration();

        return;
    }


    // --------------------------------------------------------
    // ZERO DO BRACO
    // --------------------------------------------------------

    if (
        c == 'Z' ||
        c == 'z'
    )
    {
        if (motor.isMoving())
        {
            Serial.println(
                F("Z recusado: motor em movimento.")
            );

            return;
        }


        motor.setCurrentPosition(
            0.0F
        );


        armReferenceDefined =
            true;


        Serial.println(
            F("Braco: posicao atual definida como phi=0.")
        );


        return;
    }


    // --------------------------------------------------------
    // ENABLE
    // --------------------------------------------------------

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


    // --------------------------------------------------------
    // DISABLE
    // --------------------------------------------------------

    if (
        c == 'D' ||
        c == 'd'
    )
    {
        motor.stop();

        motor.disable();


        armReferenceDefined =
            false;


        testState =
            TestState::IDLE;


        resetEventDetectors();


        Serial.println(
            F("Motor desabilitado.")
        );


        Serial.println(
            F("Referencia do braco perdida.")
        );


        return;
    }


    // --------------------------------------------------------
    // STATUS
    // --------------------------------------------------------

    if (
        c == 'S' ||
        c == 's'
    )
    {
        printStatus();

        return;
    }


    // --------------------------------------------------------
    // ARMAR ENSAIO
    // --------------------------------------------------------

    if (
        c == 'A' ||
        c == 'a'
    )
    {
        if (
            mode !=
            Mode::READY
        )
        {
            Serial.println(
                F("Ensaio recusado: calibre DOWN com T.")
            );

            return;
        }


        if (!armReferenceDefined)
        {
            Serial.println(
                F("Ensaio recusado: defina phi=0 com Z.")
            );

            return;
        }


        if (!motor.isEnabled())
        {
            Serial.println(
                F("Ensaio recusado: habilite motor com E.")
            );

            return;
        }


        if (motor.isMoving())
        {
            Serial.println(
                F("Ensaio recusado: motor em movimento.")
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
                F("Ensaio recusado: braco nao esta em phi=0.")
            );

            return;
        }


        // ====================================================
        // ENSAIO NOVO:
        // detectores comecam limpos
        // ====================================================

        resetEventDetectors();


        energyBefore = 0.0F;
        triggerAlphaDot = 0.0F;


        testState =
            TestState::WAIT_BEFORE_PEAK;


        Serial.println();


        Serial.println(
            F("# TEST_ARMED")
        );


        Serial.println(
            F("12B.4: PEAK- -> DOWN+ -> +8 deg -> fim do movimento -> PEAK+")
        );


        return;
    }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(
        250000
    );


    Wire.begin();


    Wire.setClock(
        400000UL
    );


    // ========================================================
    // MOTOR
    // ========================================================

    motor.begin(
        MOTOR_MAX_SPEED_STEPS_S,
        MOTOR_ACCEL_STEPS_S2
    );


    // ========================================================
    // SENSOR
    // ========================================================

    if (!position.begin())
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
        F("FASE 12B.4 - IMPULSO REAL DOWN+")
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
        F("A = armar ensaio")
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
    // Motor precisa de update continuo.

    motor.update();


    // Verifica fim da manobra.

    updateImpulseState();


    // Comandos seriais.

    processSerial();


    // ========================================================
    // RETORNO AO ZERO
    // ========================================================

    if (
        testState ==
        TestState::RETURNING_ZERO
    )
    {
        if (!motor.isMoving())
        {
            Serial.print(
                F("# ARM_RETURN_DONE,phi=")
            );


            Serial.println(
                motor.currentPosition(),
                2
            );


            testState =
                TestState::IDLE;


            resetEventDetectors();


            Serial.println(
                F("A = armar novo ensaio")
            );
        }
    }


    // ========================================================
    // AQUISICAO
    // ========================================================

    if (
        mode ==
        Mode::WAITING
    )
    {
        return;
    }


    const uint32_t now =
        micros();


    if (
        static_cast<int32_t>(
            now - nextSampleUs
        )
        <
        0
    )
    {
        return;
    }


    nextSampleUs +=
        SAMPLE_PERIOD_US;


    if (
        mode ==
        Mode::CALIBRATING
    )
    {
        processCalibration();
    }
    else if (
        mode ==
        Mode::READY
    )
    {
        acquireSample();
    }
}