#include "FurutaSystem.h"

#include <math.h>
#include <ctype.h>
#include <string.h>


// ============================================================
// CONSTRUTOR
// ============================================================

FurutaSystem::FurutaSystem()
    : as5600_(),

      pendulumPosition_(
          as5600_,
          1.0F
      ),

      pendulumVelocity_(),

      motor_(
          FurutaConfig::STEP_PIN,
          FurutaConfig::DIR_PIN,
          FurutaConfig::ENABLE_PIN,

          FurutaConfig::FULL_STEPS_PER_REVOLUTION,
          FurutaConfig::MICROSTEP_FACTOR,

          FurutaConfig::ARM_MIN_DEG,
          FurutaConfig::ARM_MAX_DEG,

          false
      ),

      controller_(
          FurutaConfig::K_PHI,
          FurutaConfig::K_PHI_DOT,
          FurutaConfig::K_BETA,
          FurutaConfig::K_BETA_DOT,
          FurutaConfig::MAX_ACCEL_RAD_S2
      ),

      telemetry_(
          FurutaConfig::TELEMETRY_PERIOD_US
      ),

      controlState_(
          ControlState::IDLE
      ),

      armZeroDefined_(
          false
      ),

      pendulumDownReferenceDefined_(
          false
      ),

      state_{
          0.0F,
          0.0F,
          0.0F,
          0.0F
      },

      control_{
          0.0F,
          0.0F,
          0.0F,
          0.0F,
          0.0F,
          0.0F,
          false
      },

      statistics_{},

      finalSnapshot_{},

      nextControlTimeUs_(
          0
      ),

      lastControlTimeUs_(
          0
      ),

      captureStableStartUs_(
          0
      ),

      balanceStartUs_(
          0
      ),

      serialBufferIndex_(
          0
      )
{
}


// ============================================================
// BEGIN
// ============================================================

void FurutaSystem::begin()
{
    Serial.begin(
        FurutaConfig::SERIAL_BAUD
    );

    Wire.begin();

    Wire.setClock(
        400000UL
    );


    // Sensor.
    pendulumPosition_.begin();

    pendulumPosition_.update();

    const uint32_t nowUs =
        micros();

    pendulumVelocity_.reset(
        0.0F,
        nowUs
    );


    // Motor.
    motor_.begin(
        FurutaConfig::MOTOR_MAX_SPEED_DEG_S,
        2
    );


    resetControlSignal();


    // Timing inicial.
    lastControlTimeUs_ =
        micros();

    nextControlTimeUs_ =
        lastControlTimeUs_
        +
        FurutaConfig::CONTROL_PERIOD_US;


    // Cabecalho.
    Serial.println();

    Serial.println(
        F("========================================")
    );

    Serial.println(
        F("FASE 10 - INTEGRACAO FINAL REFATORADA")
    );

    Serial.println(
        F("========================================")
    );

    Serial.println();

    Serial.println(
        F("x = [phi phiDot beta betaDot]^T")
    );

    Serial.println(
        F("u = K*x")
    );

    Serial.println();

    Serial.println(
        F("Controle: 250 Hz")
    );

    Serial.println(
        F("Telemetria: 25 Hz")
    );

    Serial.println(
        F("CSV em unidades SI")
    );

    Serial.println();

    Serial.print(
        F("Tempo de teste [s]: ")
    );

    Serial.println(
        FurutaConfig::BALANCE_TEST_TIME_US
        /
        1000000UL
    );

    Serial.println();

    if (pendulumPosition_.magnetDetected())
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
// UPDATE PRINCIPAL
// ============================================================

void FurutaSystem::update()
{
    // runSpeed deve ser chamado frequentemente.
    motor_.update();

    serviceSerial();

    motor_.update();


    const uint32_t nowUs =
        micros();


    if (
        static_cast<int32_t>(
            nowUs
            -
            nextControlTimeUs_
        )
        >=
        0
    )
    {
        controlTick(
            nowUs
        );


        nextControlTimeUs_ +=
            FurutaConfig::CONTROL_PERIOD_US;


        // Nao executa rajada de ticks atrasados.
        if (
            static_cast<int32_t>(
                micros()
                -
                nextControlTimeUs_
            )
            >=
            0
        )
        {
            nextControlTimeUs_ =
                micros()
                +
                FurutaConfig::CONTROL_PERIOD_US;
        }
    }


    motor_.update();
}


// ============================================================
// UTILITARIOS
// ============================================================

float FurutaSystem::wrapToPi(
    float angle
) const
{
    while (
        angle
        >
        FurutaConfig::PI_LOCAL
    )
    {
        angle -=
            FurutaConfig::TWO_PI_LOCAL;
    }


    while (
        angle
        <=
        -FurutaConfig::PI_LOCAL
    )
    {
        angle +=
            FurutaConfig::TWO_PI_LOCAL;
    }


    return angle;
}


float FurutaSystem::betaFromDownReference()
{
    if (!pendulumDownReferenceDefined_)
    {
        return 0.0F;
    }


    return wrapToPi(
        pendulumPosition_.betaRadians()
        -
        FurutaConfig::PI_LOCAL
    );
}


void FurutaSystem::resetControlSignal()
{
    control_.phiTermRadS2 =
        0.0F;

    control_.phiDotTermRadS2 =
        0.0F;

    control_.betaTermRadS2 =
        0.0F;

    control_.betaDotTermRadS2 =
        0.0F;

    control_.rawRadS2 =
        0.0F;

    control_.appliedRadS2 =
        0.0F;

    control_.saturated =
        false;
}


// ============================================================
// ESTATISTICAS
// ============================================================

void FurutaSystem::resetStatistics()
{
    statistics_.maxAbsBetaRad =
        fabsf(
            state_.beta
        );

    statistics_.maxAbsBetaDotRadS =
        fabsf(
            state_.betaDot
        );

    statistics_.maxAbsPhiDeg =
        fabsf(
            state_.phi
            *
            FurutaConfig::RAD_TO_DEG_LOCAL
        );

    statistics_.maxAbsPhiDotDegS =
        fabsf(
            state_.phiDot
            *
            FurutaConfig::RAD_TO_DEG_LOCAL
        );

    statistics_.maxAbsControlDegS2 =
        0.0F;

    statistics_.maxAbsPhiTermDegS2 =
        0.0F;

    statistics_.maxAbsPhiDotTermDegS2 =
        0.0F;

    statistics_.maxAbsBetaTermDegS2 =
        0.0F;

    statistics_.maxAbsBetaDotTermDegS2 =
        0.0F;

    statistics_.saturationCount =
        0;

    statistics_.sampleCount =
        0;
}


void FurutaSystem::updateStateStatistics()
{
    const float absBeta =
        fabsf(
            state_.beta
        );

    const float absBetaDot =
        fabsf(
            state_.betaDot
        );

    const float absPhiDeg =
        fabsf(
            state_.phi
            *
            FurutaConfig::RAD_TO_DEG_LOCAL
        );

    const float absPhiDotDegS =
        fabsf(
            state_.phiDot
            *
            FurutaConfig::RAD_TO_DEG_LOCAL
        );


    if (
        absBeta
        >
        statistics_.maxAbsBetaRad
    )
    {
        statistics_.maxAbsBetaRad =
            absBeta;
    }


    if (
        absBetaDot
        >
        statistics_.maxAbsBetaDotRadS
    )
    {
        statistics_.maxAbsBetaDotRadS =
            absBetaDot;
    }


    if (
        absPhiDeg
        >
        statistics_.maxAbsPhiDeg
    )
    {
        statistics_.maxAbsPhiDeg =
            absPhiDeg;
    }


    if (
        absPhiDotDegS
        >
        statistics_.maxAbsPhiDotDegS
    )
    {
        statistics_.maxAbsPhiDotDegS =
            absPhiDotDegS;
    }
}


void FurutaSystem::updateControlStatistics()
{
    const float absControlDegS2 =
        fabsf(
            control_.appliedRadS2
            *
            FurutaConfig::RAD_TO_DEG_LOCAL
        );

    const float absPhiTermDegS2 =
        fabsf(
            control_.phiTermRadS2
            *
            FurutaConfig::RAD_TO_DEG_LOCAL
        );

    const float absPhiDotTermDegS2 =
        fabsf(
            control_.phiDotTermRadS2
            *
            FurutaConfig::RAD_TO_DEG_LOCAL
        );

    const float absBetaTermDegS2 =
        fabsf(
            control_.betaTermRadS2
            *
            FurutaConfig::RAD_TO_DEG_LOCAL
        );

    const float absBetaDotTermDegS2 =
        fabsf(
            control_.betaDotTermRadS2
            *
            FurutaConfig::RAD_TO_DEG_LOCAL
        );


    if (
        absControlDegS2
        >
        statistics_.maxAbsControlDegS2
    )
    {
        statistics_.maxAbsControlDegS2 =
            absControlDegS2;
    }


    if (
        absPhiTermDegS2
        >
        statistics_.maxAbsPhiTermDegS2
    )
    {
        statistics_.maxAbsPhiTermDegS2 =
            absPhiTermDegS2;
    }


    if (
        absPhiDotTermDegS2
        >
        statistics_.maxAbsPhiDotTermDegS2
    )
    {
        statistics_.maxAbsPhiDotTermDegS2 =
            absPhiDotTermDegS2;
    }


    if (
        absBetaTermDegS2
        >
        statistics_.maxAbsBetaTermDegS2
    )
    {
        statistics_.maxAbsBetaTermDegS2 =
            absBetaTermDegS2;
    }


    if (
        absBetaDotTermDegS2
        >
        statistics_.maxAbsBetaDotTermDegS2
    )
    {
        statistics_.maxAbsBetaDotTermDegS2 =
            absBetaDotTermDegS2;
    }


    if (control_.saturated)
    {
        statistics_.saturationCount++;
    }
}


// ============================================================
// AQUISICAO DO VETOR DE ESTADO
// ============================================================

void FurutaSystem::acquireState(
    uint32_t nowUs
)
{
    pendulumPosition_.update();


    if (pendulumDownReferenceDefined_)
    {
        state_.beta =
            betaFromDownReference();


        pendulumVelocity_.update(
            state_.beta,
            nowUs
        );


        if (pendulumVelocity_.isReady())
        {
            state_.betaDot =
                pendulumVelocity_.radiansPerSecond();
        }
    }


    state_.phi =
        motor_.currentPositionDegrees()
        *
        FurutaConfig::DEG_TO_RAD_LOCAL;


    state_.phiDot =
        motor_.speedReferenceDegreesPerSecond()
        *
        FurutaConfig::DEG_TO_RAD_LOCAL;
}


// ============================================================
// TICK DE CONTROLE
// ============================================================

void FurutaSystem::controlTick(
    uint32_t nowUs
)
{
    const uint32_t dtUs =
        nowUs
        -
        lastControlTimeUs_;


    const float dtSeconds =
        static_cast<float>(
            dtUs
        )
        *
        1.0e-6F;


    lastControlTimeUs_ =
        nowUs;


    acquireState(
        nowUs
    );


    switch (controlState_)
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
            serviceWaitRelease();

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
// CAPTURA
// ============================================================

void FurutaSystem::serviceWaitCapture(
    uint32_t nowUs
)
{
    if (!pendulumVelocity_.isReady())
    {
        captureStableStartUs_ =
            0;

        return;
    }


    const bool betaOK =
        fabsf(
            state_.beta
        )
        <=
        FurutaConfig::CAPTURE_READY_BETA_RAD;


    const bool betaDotOK =
        fabsf(
            state_.betaDot
        )
        <=
        FurutaConfig::CAPTURE_READY_BETA_DOT_RAD_S;


    const bool phiOK =
        fabsf(
            state_.phi
            *
            FurutaConfig::RAD_TO_DEG_LOCAL
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
            captureStableStartUs_
            ==
            0
        )
        {
            captureStableStartUs_ =
                nowUs;
        }


        if (
            nowUs
            -
            captureStableStartUs_
            >=
            FurutaConfig::CAPTURE_READY_TIME_US
        )
        {
            controlState_ =
                ControlState::WAIT_RELEASE;


            captureStableStartUs_ =
                0;


            Serial.println();

            Serial.println(
                F("CAPTURE_READY")
            );


            Serial.print(
                F("beta [deg] = ")
            );

            Serial.println(
                state_.beta
                *
                FurutaConfig::RAD_TO_DEG_LOCAL,
                3
            );


            Serial.print(
                F("betaDot [rad/s] = ")
            );

            Serial.println(
                state_.betaDot,
                4
            );


            Serial.println();

            Serial.println(
                F("SOLTE O PENDULO.")
            );

            Serial.println();
        }
    }
    else
    {
        captureStableStartUs_ =
            0;
    }
}


void FurutaSystem::serviceWaitRelease()
{
    if (!pendulumVelocity_.isReady())
    {
        return;
    }


    if (
        fabsf(
            state_.beta
        )
        >
        FurutaConfig::CAPTURE_RELEASE_BETA_RAD
    )
    {
        motor_.stop();


        controlState_ =
            ControlState::WAIT_CAPTURE;


        captureStableStartUs_ =
            0;


        Serial.println();

        Serial.println(
            F("CAPTURE_LOST")
        );

        Serial.print(
            F("beta [deg] = ")
        );

        Serial.println(
            state_.beta
            *
            FurutaConfig::RAD_TO_DEG_LOCAL,
            3
        );

        Serial.println(
            F("Reposicione perto da vertical.")
        );

        Serial.println();

        return;
    }


    if (
        fabsf(
            state_.betaDot
        )
        >=
        FurutaConfig::RELEASE_VELOCITY_RAD_S
    )
    {
        startBalance();
    }
}


// ============================================================
// START BALANCE
// ============================================================

void FurutaSystem::startBalance()
{
    motor_.stop();


    // O motor foi parado: phiDot comandado volta a zero.
    state_.phiDot =
        0.0F;


    resetControlSignal();

    resetStatistics();


    Serial.println();

    Serial.println(
        F("========================================")
    );

    Serial.println(
        F("BALANCE - FASE 10")
    );

    Serial.println(
        F("========================================")
    );


    Serial.print(
        F("beta inicial [deg] = ")
    );

    Serial.println(
        state_.beta
        *
        FurutaConfig::RAD_TO_DEG_LOCAL,
        3
    );


    Serial.print(
        F("betaDot inicial [rad/s] = ")
    );

    Serial.println(
        state_.betaDot,
        4
    );


    Serial.print(
        F("phi inicial [deg] = ")
    );

    Serial.println(
        state_.phi
        *
        FurutaConfig::RAD_TO_DEG_LOCAL,
        3
    );


    Serial.print(
        F("phiDot inicial [deg/s] = ")
    );

    Serial.println(
        state_.phiDot
        *
        FurutaConfig::RAD_TO_DEG_LOCAL,
        3
    );


    // Cabecalho antes de iniciar o relogio do BALANCE.
    telemetry_.printSessionHeader(
        Serial
    );


    const uint32_t startUs =
        micros();


    balanceStartUs_ =
        startUs;


    lastControlTimeUs_ =
        startUs;


    nextControlTimeUs_ =
        startUs
        +
        FurutaConfig::CONTROL_PERIOD_US;


    telemetry_.start(
        startUs
    );


    controlState_ =
        ControlState::BALANCE;
}


// ============================================================
// BALANCE
// ============================================================

void FurutaSystem::serviceBalance(
    uint32_t nowUs,
    float dtSeconds,
    uint32_t dtUs
)
{
    if (
        dtUs
        >
        FurutaConfig::MAX_CONTROL_DT_US
    )
    {
        endBalance(
            "CONTROL_OVERRUN"
        );

        return;
    }


    if (!pendulumVelocity_.isReady())
    {
        endBalance(
            "VELOCITY_NOT_READY"
        );

        return;
    }


    updateStateStatistics();


    // --------------------------------------------------------
    // Limites
    // --------------------------------------------------------

    if (
        fabsf(
            state_.beta
        )
        >=
        FurutaConfig::BETA_ABORT_RAD
    )
    {
        endBalance(
            "BETA_LIMIT"
        );

        return;
    }


    if (
        fabsf(
            state_.phi
            *
            FurutaConfig::RAD_TO_DEG_LOCAL
        )
        >=
        FurutaConfig::ARM_ABORT_DEG
    )
    {
        endBalance(
            "ARM_LIMIT"
        );

        return;
    }


    if (
        nowUs
        -
        balanceStartUs_
        >=
        FurutaConfig::BALANCE_TEST_TIME_US
    )
    {
        endBalance(
            "TIME_LIMIT"
        );

        return;
    }


    // --------------------------------------------------------
    // u = Kx
    // --------------------------------------------------------

    control_ =
        controller_.compute(
            state_
        );


    updateControlStatistics();


    // --------------------------------------------------------
    // Atuador
    // --------------------------------------------------------

    const float commandDegS2 =
        control_.appliedRadS2
        *
        FurutaConfig::RAD_TO_DEG_LOCAL;


    const bool accepted =
        motor_.commandAcceleration(
            commandDegS2,
            dtSeconds
        );


    if (!accepted)
    {
        endBalance(
            "MOTOR_COMMAND_REJECTED"
        );

        return;
    }


    motor_.update();


    statistics_.sampleCount++;


    // --------------------------------------------------------
    // Telemetria: 25 Hz
    // --------------------------------------------------------

    telemetry_.update(
        nowUs,
        state_,
        control_,
        Serial
    );
}


// ============================================================
// ENCERRAMENTO
// ============================================================

void FurutaSystem::endBalance(
    const char *reason
)
{
    // Snapshot ANTES do stop.
    finalSnapshot_.elapsedUs =
        micros()
        -
        balanceStartUs_;


    finalSnapshot_.state =
        state_;


    finalSnapshot_.controlRawRadS2 =
        control_.rawRadS2;


    finalSnapshot_.controlAppliedRadS2 =
        control_.appliedRadS2;


    motor_.stop();


    telemetry_.end(
        Serial
    );


    printBalanceSummary(
        reason
    );


    resetControlSignal();


    controlState_ =
        ControlState::IDLE;


    captureStableStartUs_ =
        0;


    Serial.println(
        F("BALANCE encerrado.")
    );

    Serial.println(
        F("Motor continua habilitado e parado.")
    );

    Serial.println(
        F("Use D antes de reposicionar o braco manualmente.")
    );

    Serial.println();
}


void FurutaSystem::stopControl()
{
    if (
        controlState_
        ==
        ControlState::BALANCE
    )
    {
        endBalance(
            "USER_STOP"
        );

        return;
    }


    telemetry_.stop();

    motor_.stop();

    resetControlSignal();


    controlState_ =
        ControlState::IDLE;


    captureStableStartUs_ =
        0;


    Serial.println(
        F("Controle parado.")
    );
}


// ============================================================
// RESUMO
// ============================================================

void FurutaSystem::printBalanceSummary(
    const char *reason
)
{
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
        finalSnapshot_.elapsedUs
        /
        1000UL
    );


    Serial.print(
        F("# SAMPLES=")
    );

    Serial.println(
        statistics_.sampleCount
    );


    Serial.print(
        F("# BETA_FINAL_DEG=")
    );

    Serial.println(
        finalSnapshot_.state.beta
        *
        FurutaConfig::RAD_TO_DEG_LOCAL,
        3
    );


    Serial.print(
        F("# BETADOT_FINAL_RAD_S=")
    );

    Serial.println(
        finalSnapshot_.state.betaDot,
        4
    );


    Serial.print(
        F("# PHI_FINAL_DEG=")
    );

    Serial.println(
        finalSnapshot_.state.phi
        *
        FurutaConfig::RAD_TO_DEG_LOCAL,
        3
    );


    Serial.print(
        F("# PHIDOT_FINAL_DEG_S=")
    );

    Serial.println(
        finalSnapshot_.state.phiDot
        *
        FurutaConfig::RAD_TO_DEG_LOCAL,
        3
    );


    Serial.print(
        F("# U_RAW_FINAL_RAD_S2=")
    );

    Serial.println(
        finalSnapshot_.controlRawRadS2,
        4
    );


    Serial.print(
        F("# U_FINAL_RAD_S2=")
    );

    Serial.println(
        finalSnapshot_.controlAppliedRadS2,
        4
    );


    Serial.print(
        F("# MAX_ABS_BETA_DEG=")
    );

    Serial.println(
        statistics_.maxAbsBetaRad
        *
        FurutaConfig::RAD_TO_DEG_LOCAL,
        3
    );


    Serial.print(
        F("# MAX_ABS_BETADOT_RAD_S=")
    );

    Serial.println(
        statistics_.maxAbsBetaDotRadS,
        4
    );


    Serial.print(
        F("# MAX_ABS_PHI_DEG=")
    );

    Serial.println(
        statistics_.maxAbsPhiDeg,
        3
    );


    Serial.print(
        F("# MAX_ABS_PHIDOT_DEG_S=")
    );

    Serial.println(
        statistics_.maxAbsPhiDotDegS,
        3
    );


    Serial.print(
        F("# MAX_ABS_U_DEG_S2=")
    );

    Serial.println(
        statistics_.maxAbsControlDegS2,
        1
    );


    Serial.println();

    Serial.println(
        F("# COMPONENTES DO CONTROLE")
    );


    Serial.print(
        F("# MAX_ABS_TERM_PHI_DEG_S2=")
    );

    Serial.println(
        statistics_.maxAbsPhiTermDegS2,
        1
    );


    Serial.print(
        F("# MAX_ABS_TERM_PHIDOT_DEG_S2=")
    );

    Serial.println(
        statistics_.maxAbsPhiDotTermDegS2,
        1
    );


    Serial.print(
        F("# MAX_ABS_TERM_BETA_DEG_S2=")
    );

    Serial.println(
        statistics_.maxAbsBetaTermDegS2,
        1
    );


    Serial.print(
        F("# MAX_ABS_TERM_BETADOT_DEG_S2=")
    );

    Serial.println(
        statistics_.maxAbsBetaDotTermDegS2,
        1
    );


    Serial.print(
        F("# SATURATION_COUNT=")
    );

    Serial.println(
        statistics_.saturationCount
    );


    Serial.println();
}


// ============================================================
// ARMAR BALANCE
// ============================================================

void FurutaSystem::armBalance()
{
    if (
        controlState_
        !=
        ControlState::IDLE
    )
    {
        Serial.println(
            F("ERRO: sistema ocupado.")
        );

        return;
    }


    if (!pendulumDownReferenceDefined_)
    {
        Serial.println(
            F("ERRO: calibre DOWN com T.")
        );

        return;
    }


    if (!armZeroDefined_)
    {
        Serial.println(
            F("ERRO: defina phi=0 com Z.")
        );

        return;
    }


    if (!motor_.isEnabled())
    {
        Serial.println(
            F("ERRO: habilite o motor com E.")
        );

        return;
    }


    if (
        fabsf(
            motor_.currentPositionDegrees()
        )
        >
        5.0F
    )
    {
        Serial.println(
            F("ERRO: braco deve iniciar perto de phi=0.")
        );

        return;
    }


    motor_.stop();

    resetControlSignal();


    captureStableStartUs_ =
        0;


    controlState_ =
        ControlState::WAIT_CAPTURE;


    Serial.println();

    Serial.println(
        F("BALANCE FASE 10 ARMADO.")
    );

    Serial.println(
        F("Leve o pendulo perto da vertical.")
    );

    Serial.println();


    Serial.print(
        F("CAPTURE_READY: |beta| <= ")
    );

    Serial.print(
        FurutaConfig::CAPTURE_READY_BETA_DEG,
        2
    );

    Serial.println(
        F(" deg")
    );


    Serial.print(
        F("|betaDot| <= ")
    );

    Serial.print(
        FurutaConfig::CAPTURE_READY_BETA_DOT_RAD_S,
        2
    );

    Serial.println(
        F(" rad/s")
    );


    Serial.println();

    Serial.println(
        F("Quando aparecer CAPTURE_READY, solte.")
    );

    Serial.println();
}


// ============================================================
// REFERENCIA DOWN
// ============================================================

void FurutaSystem::definePendulumDownReference()
{
    if (
        controlState_
        !=
        ControlState::IDLE
    )
    {
        Serial.println(
            F("ERRO: pare o controle antes de T.")
        );

        return;
    }


    if (motor_.isEnabled())
    {
        Serial.println(
            F("ERRO: desabilite o motor antes de T.")
        );

        return;
    }


    if (!pendulumPosition_.magnetDetected())
    {
        Serial.println(
            F("ERRO: ima nao detectado.")
        );

        return;
    }


    motor_.stop();


    Serial.println();

    Serial.println(
        F("Calibrando referencia inferior...")
    );

    Serial.println(
        F("Pendulo deve estar LIVRE, PARADO e PARA BAIXO.")
    );


    // A biblioteca chama calibrateTop().
    // Aqui usamos DOWN como referencia interna.
    pendulumPosition_.calibrateTop(
        32,
        2
    );


    pendulumPosition_.update();


    pendulumDownReferenceDefined_ =
        pendulumPosition_.topIsDefined();


    if (!pendulumDownReferenceDefined_)
    {
        Serial.println(
            F("ERRO ao definir referencia inferior.")
        );

        return;
    }


    state_.beta =
        betaFromDownReference();


    state_.betaDot =
        0.0F;


    const uint32_t nowUs =
        micros();


    pendulumVelocity_.reset(
        state_.beta,
        nowUs
    );


    Serial.println();

    Serial.println(
        F("Referencia inferior definida.")
    );

    Serial.println(
        F("TOP = DOWN + 180 graus.")
    );


    Serial.print(
        F("beta atual [deg] ~= +/-180: ")
    );

    Serial.println(
        state_.beta
        *
        FurutaConfig::RAD_TO_DEG_LOCAL,
        3
    );


    Serial.println();
}


// ============================================================
// ZERO DO BRACO
// ============================================================

void FurutaSystem::defineArmZero()
{
    if (
        controlState_
        !=
        ControlState::IDLE
    )
    {
        Serial.println(
            F("ERRO: sistema ocupado.")
        );

        return;
    }


    if (motor_.isEnabled())
    {
        Serial.println(
            F("ERRO: desabilite o motor antes de Z.")
        );

        return;
    }


    motor_.setCurrentPosition(
        0.0F
    );


    state_.phi =
        0.0F;


    state_.phiDot =
        0.0F;


    armZeroDefined_ =
        true;


    Serial.println(
        F("Zero do braco definido: phi=0.")
    );
}


// ============================================================
// EMERGENCY STOP
// ============================================================

void FurutaSystem::emergencyStop()
{
    telemetry_.stop();

    motor_.emergencyStop();

    resetControlSignal();


    controlState_ =
        ControlState::IDLE;


    captureStableStartUs_ =
        0;


    armZeroDefined_ =
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

void FurutaSystem::serviceSerial()
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
                serialBufferIndex_
                >
                0
            )
            {
                serialBuffer_[
                    serialBufferIndex_
                ] =
                    '\0';


                handleCommand(
                    serialBuffer_
                );


                serialBufferIndex_ =
                    0;
            }

            continue;
        }


        if (
            serialBufferIndex_
            <
            FurutaConfig::SERIAL_BUFFER_SIZE
            -
            1
        )
        {
            serialBuffer_[
                serialBufferIndex_
            ] =
                character;


            serialBufferIndex_++;
        }
    }
}


void FurutaSystem::handleCommand(
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


    // Durante BALANCE evitamos mensagens longas.
    if (
        controlState_
        ==
        ControlState::BALANCE
    )
    {
        if (
            strcmp(
                command,
                "STOP"
            )
            ==
            0
        )
        {
            stopControl();

            return;
        }


        if (
            strcmp(
                command,
                "X"
            )
            ==
            0
        )
        {
            emergencyStop();

            return;
        }


        Serial.println(
            F("BALANCE ativo: use STOP ou X.")
        );

        return;
    }


    if (
        strcmp(command, "T") == 0
        ||
        strcmp(command, "DOWN") == 0
    )
    {
        definePendulumDownReference();

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
        if (!armZeroDefined_)
        {
            Serial.println(
                F("ERRO: defina Z primeiro.")
            );

            return;
        }


        motor_.enable();


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
        stopControl();


        motor_.disable();


        // Sem encoder independente no braco.
        armZeroDefined_ =
            false;


        Serial.println(
            F("Motor desabilitado.")
        );

        Serial.println(
            F("Referencia do braco perdida.")
        );

        return;
    }


    if (
        strcmp(command, "B") == 0
        ||
        strcmp(command, "BALANCE") == 0
    )
    {
        armBalance();

        return;
    }


    if (
        strcmp(command, "STOP") == 0
    )
    {
        stopControl();

        return;
    }


    if (
        strcmp(command, "X") == 0
    )
    {
        emergencyStop();

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

void FurutaSystem::printStatus()
{
    acquireState(
        micros()
    );


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
        pendulumDownReferenceDefined_
        ?
        F("DEFINIDA")
        :
        F("NAO DEFINIDA")
    );


    Serial.print(
        F("Zero braco: ")
    );

    Serial.println(
        armZeroDefined_
        ?
        F("DEFINIDO")
        :
        F("NAO DEFINIDO")
    );


    Serial.print(
        F("Motor: ")
    );

    Serial.println(
        motor_.isEnabled()
        ?
        F("HABILITADO")
        :
        F("DESABILITADO")
    );


    Serial.print(
        F("x = [")
    );

    Serial.print(
        state_.phi,
        5
    );

    Serial.print(
        F(", ")
    );

    Serial.print(
        state_.phiDot,
        5
    );

    Serial.print(
        F(", ")
    );

    Serial.print(
        state_.beta,
        5
    );

    Serial.print(
        F(", ")
    );

    Serial.print(
        state_.betaDot,
        5
    );

    Serial.println(
        F("]")
    );


    Serial.print(
        F("uRaw [rad/s2]: ")
    );

    Serial.println(
        control_.rawRadS2,
        4
    );


    Serial.print(
        F("u [rad/s2]: ")
    );

    Serial.println(
        control_.appliedRadS2,
        4
    );


    Serial.println(
        F("----------------------------")
    );

    Serial.println();
}


// ============================================================
// HELP
// ============================================================

void FurutaSystem::printHelp()
{
    Serial.println(
        F("COMANDOS:")
    );

    Serial.println(
        F("T       = calibrar pendulo PARA BAIXO")
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
        F("SEQUENCIA: D -> Z -> T -> E -> B")
    );

    Serial.println();

    Serial.println(
        F("CSV:")
    );

    Serial.println(
        F("t_ms,phi,phiDot,beta,betaDot,uRaw,u")
    );

    Serial.println(
        F("Estados em rad/rad/s; controle em rad/s2.")
    );

    Serial.println();
}


// ============================================================
// NOME DO ESTADO
// ============================================================

const char *FurutaSystem::stateName() const
{
    switch (controlState_)
    {
        case ControlState::IDLE:
            return "IDLE";

        case ControlState::WAIT_CAPTURE:
            return "WAIT_CAPTURE";

        case ControlState::WAIT_RELEASE:
            return "WAIT_RELEASE";

        case ControlState::BALANCE:
            return "BALANCE";
    }

    return "UNKNOWN";
}
