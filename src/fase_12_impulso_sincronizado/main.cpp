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
// FASE 12B.4 - REGRESSAO APOS REFATORACAO
//
// Ensaio validado:
//   PEAK- -> DOWN+ -> braco 0 -> +8 graus
//   -> espera fim do movimento
//   -> PEAK+ -> mede energia
//   -> retorna braco a zero
//
// A deteccao de eventos e a referencia DOWN foram extraidas
// para bibliotecas reutilizaveis antes da Fase 13.
// ============================================================

constexpr float SENSOR_DIRECTION_SIGN = 1.0F;
constexpr float IMPULSE_TARGET_DEG = 8.0F;

AS5600 as5600;

PendulumPosition pendulumPosition(
    as5600,
    SENSOR_DIRECTION_SIGN
);

PendulumVelocity pendulumVelocity;
PendulumEvents pendulumEvents;
PendulumDownReference downReference(3000UL, 20.0F);

MotorPosition motor(
    FurutaConfig::STEP_PIN,
    FurutaConfig::DIR_PIN,
    FurutaConfig::ENABLE_PIN,
    FurutaConfig::FULL_STEPS_PER_REVOLUTION,
    FurutaConfig::MICROSTEP_FACTOR,
    FurutaConfig::ARM_MIN_DEG,
    FurutaConfig::ARM_MAX_DEG
);

enum class Mode : uint8_t
{
    WAITING,
    CALIBRATING_DOWN,
    READY
};

enum class TestState : uint8_t
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

bool armReferenceDefined = false;

uint32_t nextSampleUs = 0;
uint32_t startUs = 0;
uint32_t impulseStartUs = 0;

float energyBefore = 0.0F;
float triggerAlphaDot = 0.0F;

// ============================================================
// ENERGIA NORMALIZADA NO PICO
// ============================================================

float peakEnergy(float alphaRad)
{
    return 0.5F * (1.0F - cosf(alphaRad));
}

// ============================================================
// RESET DO ENSAIO
// ============================================================

void resetExperimentDetection()
{
    pendulumEvents.reset();
    energyBefore = 0.0F;
    triggerAlphaDot = 0.0F;
}

// ============================================================
// FINALIZACAO
// ============================================================

void finishTest(
    float energyAfter,
    float peakAfterRad
)
{
    const float deltaEnergy =
        energyAfter - energyBefore;

    const float deltaPercent =
        energyBefore > 0.0F
            ? 100.0F * deltaEnergy / energyBefore
            : 0.0F;

    Serial.println();
    Serial.print(F("# AFTER,PEAK+,E="));
    Serial.print(energyAfter, 6);
    Serial.print(F(",angle="));
    Serial.println(peakAfterRad * RAD_TO_DEG, 2);

    Serial.print(F("# RESULT,Ebefore="));
    Serial.print(energyBefore, 6);
    Serial.print(F(",Eafter="));
    Serial.print(energyAfter, 6);
    Serial.print(F(",dE="));
    Serial.print(deltaEnergy, 6);
    Serial.print(F(",dE_percent="));
    Serial.print(deltaPercent, 2);
    Serial.print(F(",triggerAlphaDot="));
    Serial.print(triggerAlphaDot, 4);
    Serial.print(F(",phi="));
    Serial.print(motor.currentPosition(), 2);
    Serial.print(F(",motorMoving="));
    Serial.println(motor.isMoving() ? 1 : 0);

    Serial.println(F("# TEST_END"));

    if (motor.moveTo(0.0F))
    {
        Serial.println(F("# ARM_RETURN,target=0.0"));
        testState = TestState::RETURNING_ZERO;
    }
    else
    {
        Serial.println(F("# ARM_RETURN_ERROR"));
        testState = TestState::IDLE;
    }
}

// ============================================================
// EVENTOS DO PENDULO
// ============================================================

void handlePendulumEvent(const PendulumEvent &event)
{
    const uint32_t tMs =
        (event.timeUs - startUs) / 1000UL;

    switch (event.type)
    {
        case PendulumEventType::PEAK_NEGATIVE:
        {
            const float energy = peakEnergy(event.alphaRad);

            Serial.print(F("# PEAK,-,t="));
            Serial.print(tMs);
            Serial.print(F(",angle="));
            Serial.print(event.alphaRad * RAD_TO_DEG, 2);
            Serial.print(F(",energy="));
            Serial.println(energy, 6);

            if (testState == TestState::WAIT_BEFORE_PEAK)
            {
                energyBefore = energy;

                Serial.print(F("# BEFORE,PEAK-,E="));
                Serial.println(energyBefore, 6);

                testState = TestState::WAIT_DOWN;
            }

            break;
        }

        case PendulumEventType::DOWN_POSITIVE:
        {
            Serial.print(F("# DOWN,+,t="));
            Serial.print(tMs);
            Serial.print(F(",alphaDot="));
            Serial.println(event.alphaDotRadS, 4);

            if (testState != TestState::WAIT_DOWN)
            {
                break;
            }

            if (
                !armReferenceDefined
                || !motor.isEnabled()
                || motor.isMoving()
            )
            {
                Serial.println(F("# IMPULSE_ERROR,motor_state"));
                testState = TestState::IDLE;
                break;
            }

            triggerAlphaDot = event.alphaDotRadS;
            impulseStartUs = micros();

            Serial.print(F("# TRIGGER,DOWN+,t="));
            Serial.print(tMs);
            Serial.print(F(",alphaDot="));
            Serial.print(triggerAlphaDot, 4);
            Serial.print(F(",phi="));
            Serial.println(motor.currentPosition(), 2);

            if (!motor.moveTo(IMPULSE_TARGET_DEG))
            {
                Serial.println(F("# IMPULSE_ERROR,move_rejected"));
                testState = TestState::IDLE;
                break;
            }

            Serial.print(F("# IMPULSE,target="));
            Serial.print(IMPULSE_TARGET_DEG, 2);
            Serial.print(F(",phi0="));
            Serial.println(motor.currentPosition(), 2);

            testState = TestState::WAIT_IMPULSE_END;
            break;
        }

        case PendulumEventType::PEAK_POSITIVE:
        {
            const float energy = peakEnergy(event.alphaRad);

            Serial.print(F("# PEAK,+,t="));
            Serial.print(tMs);
            Serial.print(F(",angle="));
            Serial.print(event.alphaRad * RAD_TO_DEG, 2);
            Serial.print(F(",energy="));
            Serial.println(energy, 6);

            if (
                testState == TestState::WAIT_IMPULSE_END
                || motor.isMoving()
            )
            {
                Serial.println(F("# PEAK_IGNORED,reason=motor_moving"));
                break;
            }

            if (testState == TestState::WAIT_AFTER_PEAK)
            {
                finishTest(energy, event.alphaRad);
            }

            break;
        }

        case PendulumEventType::DOWN_NEGATIVE:
        {
            // Informativo somente durante ensaio armado.
            Serial.print(F("# DOWN,-,t="));
            Serial.print(tMs);
            Serial.print(F(",alphaDot="));
            Serial.println(event.alphaDotRadS, 4);
            break;
        }

        case PendulumEventType::NONE:
        default:
            break;
    }
}

// ============================================================
// CALIBRACAO DOWN
// ============================================================

void startDownCalibration()
{
    Serial.println();

    if (!pendulumPosition.calibrateTop(32, 2))
    {
        Serial.println(F("ERRO na leitura inicial do AS5600."));
        return;
    }

    const uint32_t nowUs = micros();

    pendulumVelocity.reset(
        pendulumPosition.betaRadians(),
        nowUs
    );

    downReference.start(nowUs);
    nextSampleUs = nowUs + FurutaConfig::CONTROL_PERIOD_US;

    resetExperimentDetection();
    testState = TestState::IDLE;
    mode = Mode::CALIBRATING_DOWN;

    Serial.println(F("Calibrando centro de DOWN por 3 s..."));
}

void updateDownCalibration(uint32_t nowUs)
{
    if (!pendulumPosition.update())
    {
        return;
    }

    const bool finished = downReference.update(
        pendulumPosition.betaRadians(),
        nowUs
    );

    if (!finished)
    {
        return;
    }

    if (!downReference.isReady())
    {
        Serial.print(F("CALIBRACAO REJEITADA. Span="));
        Serial.print(downReference.observedSpanDeg(), 2);
        Serial.println(F(" deg"));

        mode = Mode::WAITING;
        return;
    }

    Serial.print(F("DOWN estimado = "));
    Serial.print(downReference.offsetDeg(), 3);
    Serial.println(F(" deg"));

    Serial.print(F("Span observado = "));
    Serial.print(downReference.observedSpanDeg(), 2);
    Serial.println(F(" deg"));

    const float alpha = downReference.correctedAngleRad(
        pendulumPosition.betaRadians()
    );

    pendulumVelocity.reset(alpha, nowUs);
    pendulumEvents.reset();

    startUs = nowUs;
    nextSampleUs = nowUs + FurutaConfig::CONTROL_PERIOD_US;

    mode = Mode::READY;
    testState = TestState::IDLE;

    Serial.println(F("DOWN definido."));
    Serial.println(F("A = armar ensaio 12B.4 refatorado"));
}

// ============================================================
// AQUISICAO
// ============================================================

void acquireSample(uint32_t nowUs)
{
    if (!pendulumPosition.update())
    {
        return;
    }

    const float alpha = downReference.correctedAngleRad(
        pendulumPosition.betaRadians()
    );

    pendulumVelocity.update(alpha, nowUs);

    if (!pendulumVelocity.isReady())
    {
        return;
    }

    // Fora de um ensaio armado, sensor e velocidade continuam
    // atualizados, mas nao geramos telemetria de eventos.
    if (
        testState == TestState::IDLE
        || testState == TestState::RETURNING_ZERO
    )
    {
        return;
    }

    const PendulumEvent event = pendulumEvents.update(
        alpha,
        pendulumVelocity.radiansPerSecond(),
        nowUs
    );

    if (event.type != PendulumEventType::NONE)
    {
        handlePendulumEvent(event);
    }
}

// ============================================================
// MOTOR / ESTADOS AUXILIARES
// ============================================================

void updateMotorState()
{
    if (testState == TestState::WAIT_IMPULSE_END)
    {
        if (!motor.isMoving())
        {
            const uint32_t nowUs = micros();

            Serial.print(F("# IMPULSE_DONE,t="));
            Serial.print((nowUs - startUs) / 1000UL);
            Serial.print(F(",duration_ms="));
            Serial.print((nowUs - impulseStartUs) / 1000.0F, 1);
            Serial.print(F(",phi="));
            Serial.println(motor.currentPosition(), 2);

            testState = TestState::WAIT_AFTER_PEAK;
        }
    }

    if (testState == TestState::RETURNING_ZERO)
    {
        if (!motor.isMoving())
        {
            Serial.print(F("# ARM_RETURN_DONE,phi="));
            Serial.println(motor.currentPosition(), 2);

            testState = TestState::IDLE;
            pendulumEvents.reset();

            Serial.println(F("A = armar novo ensaio"));
        }
    }
}

// ============================================================
// SERIAL
// ============================================================

void printStatus()
{
    Serial.println();
    Serial.print(F("Motor: "));
    Serial.println(motor.isEnabled() ? F("ENABLED") : F("DISABLED"));
    Serial.print(F("Referencia braco: "));
    Serial.println(armReferenceDefined ? F("OK") : F("NAO DEFINIDA"));
    Serial.print(F("phi = "));
    Serial.print(motor.currentPosition(), 2);
    Serial.println(F(" deg"));
    Serial.print(F("moving = "));
    Serial.println(motor.isMoving() ? F("SIM") : F("NAO"));
}

void processSerial()
{
    if (!Serial.available())
    {
        return;
    }

    const char c = Serial.read();

    if (c == 'T' || c == 't')
    {
        startDownCalibration();
        return;
    }

    if (c == 'Z' || c == 'z')
    {
        if (motor.isMoving())
        {
            Serial.println(F("Z recusado: motor em movimento."));
            return;
        }

        motor.setCurrentPosition(0.0F);
        armReferenceDefined = true;
        Serial.println(F("Braco: posicao atual definida como phi=0."));
        return;
    }

    if (c == 'E' || c == 'e')
    {
        motor.enable();
        Serial.println(F("Motor habilitado."));
        return;
    }

    if (c == 'D' || c == 'd')
    {
        motor.stop();
        motor.disable();
        armReferenceDefined = false;
        testState = TestState::IDLE;
        pendulumEvents.reset();

        Serial.println(F("Motor desabilitado. Referencia do braco perdida."));
        return;
    }

    if (c == 'S' || c == 's')
    {
        printStatus();
        return;
    }

    if (c == 'A' || c == 'a')
    {
        if (mode != Mode::READY)
        {
            Serial.println(F("Ensaio recusado: calibre DOWN com T."));
            return;
        }

        if (!armReferenceDefined)
        {
            Serial.println(F("Ensaio recusado: defina phi=0 com Z."));
            return;
        }

        if (!motor.isEnabled())
        {
            Serial.println(F("Ensaio recusado: habilite motor com E."));
            return;
        }

        if (motor.isMoving())
        {
            Serial.println(F("Ensaio recusado: motor em movimento."));
            return;
        }

        if (fabsf(motor.currentPosition()) > 0.5F)
        {
            Serial.println(F("Ensaio recusado: braco nao esta em phi=0."));
            return;
        }

        resetExperimentDetection();
        testState = TestState::WAIT_BEFORE_PEAK;

        Serial.println();
        Serial.println(F("# TEST_ARMED"));
        Serial.println(F("12B.4 refatorado: PEAK- -> DOWN+ -> +8 deg -> PEAK+"));
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

    motor.beginDegrees(
        FurutaConfig::MOTOR_MAX_SPEED_DEG_S,
        FurutaConfig::MAX_ACCEL_DEG_S2
    );

    if (!pendulumPosition.begin())
    {
        Serial.println(F("ERRO: AS5600 nao encontrado."));
        while (true)
        {
            delay(1000);
        }
    }

    Serial.println();
    Serial.println(F("FASE 12B.4 - REGRESSAO APOS REFATORACAO"));
    Serial.println(F("Z = definir phi=0"));
    Serial.println(F("E = habilitar motor"));
    Serial.println(F("D = desabilitar motor"));
    Serial.println(F("T = calibrar DOWN"));
    Serial.println(F("A = armar ensaio"));
    Serial.println(F("S = status"));
}

void loop()
{
    motor.update();
    updateMotorState();
    processSerial();

    if (mode == Mode::WAITING)
    {
        return;
    }

    const uint32_t nowUs = micros();

    if (static_cast<int32_t>(nowUs - nextSampleUs) < 0)
    {
        return;
    }

    nextSampleUs += FurutaConfig::CONTROL_PERIOD_US;

    if (mode == Mode::CALIBRATING_DOWN)
    {
        updateDownCalibration(nowUs);
    }
    else if (mode == Mode::READY)
    {
        acquireSample(nowUs);
    }
}
