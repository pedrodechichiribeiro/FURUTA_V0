#include "FurutaRevisionSystem.h"
#include <EEPROM.h>

namespace
{
    constexpr int RAW_EEPROM_BASE = 960;
    constexpr uint8_t RAW_HISTORY_CAPACITY = 16;
    constexpr uint8_t RAW_MAGIC = 0xA7;
    constexpr uint8_t RAW_VERSION = 1;

    constexpr int RAW_ADDR_MAGIC = RAW_EEPROM_BASE + 0;
    constexpr int RAW_ADDR_VERSION = RAW_EEPROM_BASE + 1;
    constexpr int RAW_ADDR_COUNT = RAW_EEPROM_BASE + 2;
    constexpr int RAW_ADDR_NEXT = RAW_EEPROM_BASE + 3;
    constexpr int RAW_ADDR_DATA = RAW_EEPROM_BASE + 4;
}

// ============================================================
// CONSTRUTOR
// ============================================================

FurutaRevisionSystem::FurutaRevisionSystem(const RevisionSettings &settings)
    : settings_(settings),
      as5600_(),
      pendulumPosition_(as5600_, 1.0F),
      pendulumVelocity_(),
      downVelocity_(),
      downReference_(
          RevisionConfig::DOWN_CALIBRATION_TIME_MS,
          RevisionConfig::DOWN_MAXIMUM_SPAN_DEG
      ),
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
      telemetry_(FurutaConfig::TELEMETRY_PERIOD_US),
      controlState_(ControlState::IDLE),
      state_{0.0F, 0.0F, 0.0F, 0.0F},
      control_{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, false},
      statistics_{},
      armZeroDefined_(false),
      downReferenceDefined_(false),
      downCalibrationActive_(false),
      downCollecting_(false),
      topDiagnosticActive_(false),
      topMeasureActive_(false),
      topCollecting_(false),
      topStableStartUs_(0),
      topCollectStartUs_(0),
      topSampleCount_(0),
      topSumBetaRad_(0.0F),
      topMinBetaRad_(0.0F),
      topMaxBetaRad_(0.0F),
      downAnchorAbsoluteRad_(0.0F),
      downStableStartUs_(0),
      downCalibrationSamples_(0),
      downCalibrationReadErrors_(0),
      consecutiveSensorErrors_(0),
      totalSensorErrors_(0),
      nextControlTimeUs_(0),
      lastControlTimeUs_(0),
      captureStableStartUs_(0),
      releaseConfirmCount_(0),
      balanceStartUs_(0),
      nextTopDiagnosticUs_(0),
      balanceInitialState_{0.0F, 0.0F, 0.0F, 0.0F},
      debugSampleCount_(0),
      debugWriteIndex_(0),
      debugDecimationCounter_(0),
      serialBufferIndex_(0)
{
}

// ============================================================
// BEGIN / LOOP
// ============================================================

void FurutaRevisionSystem::begin()
{
    Serial.begin(FurutaConfig::SERIAL_BAUD);
    Wire.begin();
    Wire.setClock(400000UL);

    const bool sensorConnected = pendulumPosition_.begin();
    const bool firstReadOK = pendulumPosition_.update();
    const uint32_t nowUs = micros();

    pendulumVelocity_.reset(0.0F, nowUs);
    motor_.begin(settings_.motorMaxSpeedDegS, 2);
    resetControlSignal();
    lastControlTimeUs_ = nowUs;
    nextControlTimeUs_ = nowUs + settings_.controlPeriodUs;

    applyFixedDownReference();

    Serial.println(F("REV10-v16-SERIAL"));
    Serial.print(F("AS5600=")); Serial.print(sensorConnected ? 1 : 0);
    Serial.print(F(",READ=")); Serial.print(firstReadOK ? 1 : 0);
    Serial.print(F(",MAG=")); Serial.println(pendulumPosition_.magnetDetected() ? 1 : 0);
    rawHistoryInit();
    Serial.print(F("DOWN,FIXED,RAW=")); Serial.print(settings_.rawReferenceCounts);
    Serial.print(F(",OFF=")); Serial.println(settings_.topReferenceOffsetDeg, 3);
    printParameters();
    printHelp();
}

void FurutaRevisionSystem::update()
{
    // runSpeed precisa de chamadas frequentes.
    motor_.update();
    serviceSerial();
    motor_.update();

    const uint32_t nowUs = micros();

    if (static_cast<int32_t>(nowUs - nextControlTimeUs_) >= 0)
    {
        const bool wasBalancing = (controlState_ == ControlState::BALANCE);
        const uint32_t tickStartUs = nowUs;

        controlTick(nowUs);

        nextControlTimeUs_ += settings_.controlPeriodUs;

        // Mede o custo real do tick. Durante a v9 nao ha telemetria
        // Serial dentro de BALANCE; estes numeros mostram se a malha
        // consegue sustentar o periodo nominal de 4 ms.
        const uint32_t afterTickUs = micros();

        const bool stillBalancing =
            wasBalancing && (controlState_ == ControlState::BALANCE);

        if (stillBalancing)
        {
            const uint32_t tickExecUs = afterTickUs - tickStartUs;
            if (tickExecUs > statistics_.maxTickExecUs)
                statistics_.maxTickExecUs = tickExecUs;
        }

        // Nao recupera atraso em rajada. Conta quantos slots nominais
        // precisaram ser abandonados antes de resincronizar.
        if (static_cast<int32_t>(afterTickUs - nextControlTimeUs_) >= 0)
        {
            if (stillBalancing)
            {
                const uint32_t overdueUs = afterTickUs - nextControlTimeUs_;
                statistics_.missedScheduleSlots +=
                    1UL + overdueUs / settings_.controlPeriodUs;
            }

            nextControlTimeUs_ = afterTickUs + settings_.controlPeriodUs;
        }
    }

    motor_.update();
}

// ============================================================
// ANGULOS
// ============================================================

float FurutaRevisionSystem::wrapToPi(float angleRad)
{
    while (angleRad >= FurutaConfig::PI_LOCAL)
    {
        angleRad -= FurutaConfig::TWO_PI_LOCAL;
    }

    while (angleRad < -FurutaConfig::PI_LOCAL)
    {
        angleRad += FurutaConfig::TWO_PI_LOCAL;
    }

    return angleRad;
}

float FurutaRevisionSystem::rawRelativeToDownAnchor() const
{
    return wrapToPi(
        pendulumPosition_.absoluteAngleRadians()
        - downAnchorAbsoluteRad_
    );
}

float FurutaRevisionSystem::betaFromRobustDownReference() const
{
    if (!downReferenceDefined_)
    {
        return 0.0F;
    }

    const float relativeAngle = rawRelativeToDownAnchor();
    const float angleFromDown = downReference_.correctedAngleRad(relativeAngle);

    // Referencia geometrica: TOP = DOWN + pi.
    // Nesta fase de revisao aplicamos um offset experimental de -1 grau.
    // Convencao: se o beta geometrico for -1 grau, o beta corrigido sera 0.
    const float topOffsetRad =
        settings_.topReferenceOffsetDeg * FurutaConfig::DEG_TO_RAD_LOCAL;

    return wrapToPi(
        angleFromDown
        - FurutaConfig::PI_LOCAL
        - topOffsetRad
    );
}

void FurutaRevisionSystem::applyFixedDownReference()
{
    // Equivalente a uma calibracao DOWN cujo valor medio absoluto fosse
    // exatamente rawReferenceCounts. O offset interno de downReference_
    // permanece zero; portanto downAnchorAbsoluteRad_ e a referencia efetiva.
    downAnchorAbsoluteRad_ =
        static_cast<float>(settings_.rawReferenceCounts & 0x0FFF)
        * (FurutaConfig::TWO_PI_LOCAL / 4096.0F);

    downReferenceDefined_ = true;
    downCalibrationActive_ = false;
    downCollecting_ = false;

    if (pendulumPosition_.update())
    {
        state_.beta = betaFromRobustDownReference();
        state_.betaDot = 0.0F;
        pendulumVelocity_.reset(state_.beta, micros());
    }
}

// ============================================================
// AQUISICAO
// ============================================================

bool FurutaRevisionSystem::acquireState(uint32_t nowUs)
{
    const bool readOK = pendulumPosition_.update();

    if (!readOK)
    {
        ++totalSensorErrors_;
        ++consecutiveSensorErrors_;
        return false;
    }

    consecutiveSensorErrors_ = 0;

    if (downReferenceDefined_)
    {
        state_.beta = betaFromRobustDownReference();
        pendulumVelocity_.update(state_.beta, nowUs);

        if (pendulumVelocity_.isReady())
        {
            state_.betaDot = pendulumVelocity_.radiansPerSecond();
        }
    }

    state_.phi =
        motor_.currentPositionDegrees()
        * FurutaConfig::DEG_TO_RAD_LOCAL;

    state_.phiDot =
        motor_.speedReferenceDegreesPerSecond()
        * FurutaConfig::DEG_TO_RAD_LOCAL;

    return true;
}

// ============================================================
// CALIBRACAO DOWN ROBUSTA
// ============================================================

void FurutaRevisionSystem::startDownCalibration()
{
    if (controlState_ != ControlState::IDLE || downCalibrationActive_)
    {
        Serial.println(F("ERR,T,BUSY"));
        return;
    }

    motor_.stop();

    if (!pendulumPosition_.magnetDetected())
    {
        Serial.println(F("ERR,T,MAG"));
        return;
    }

    if (!pendulumPosition_.update())
    {
        Serial.println(F("ERR,T,AS5600"));
        return;
    }

    topDiagnosticActive_ = false;
    topMeasureActive_ = false;
    topCollecting_ = false;
    downReferenceDefined_ = false;

    const uint32_t nowUs = micros();
    downAnchorAbsoluteRad_ = pendulumPosition_.absoluteAngleRadians();
    downStableStartUs_ = 0;
    downCalibrationSamples_ = 0;
    downCalibrationReadErrors_ = 0;
    downCollecting_ = false;
    downVelocity_.reset(0.0F, nowUs);
    downCalibrationActive_ = true;

    Serial.print(F("T,WAIT,HOLD="));
    Serial.println(motor_.isEnabled() ? 1 : 0);
}

void FurutaRevisionSystem::serviceDownCalibration(
    uint32_t nowUs,
    bool sensorReadingValid
)
{
    if (!downCalibrationActive_) return;

    if (!sensorReadingValid)
    {
        ++downCalibrationReadErrors_;
        downStableStartUs_ = 0;
        if (downCollecting_)
        {
            downCollecting_ = false;
            Serial.println(F("T,RESTART,READ"));
        }
        return;
    }

    const float relative = rawRelativeToDownAnchor();
    downVelocity_.update(relative, nowUs);

    if (!downVelocity_.isReady()) return;

    const float absSpeed = fabsf(downVelocity_.radiansPerSecond());

    // Etapa 1: exige estabilidade continua antes de iniciar a media DOWN.
    if (!downCollecting_)
    {
        if (absSpeed > RevisionConfig::DOWN_STABLE_SPEED_RAD_S)
        {
            downStableStartUs_ = 0;
            return;
        }

        if (downStableStartUs_ == 0)
        {
            downStableStartUs_ = nowUs;
            return;
        }

        if ((nowUs - downStableStartUs_) < RevisionConfig::DOWN_STABLE_TIME_US)
            return;

        // Reancora no instante em que a coleta realmente comeca.
        downAnchorAbsoluteRad_ = pendulumPosition_.absoluteAngleRadians();
        downVelocity_.reset(0.0F, nowUs);
        downReference_.start(nowUs);
        downCalibrationSamples_ = 0;
        downCalibrationReadErrors_ = 0;
        downCollecting_ = true;
        Serial.println(F("T,COLLECT"));
        return;
    }

    // Etapa 2: se voltar a se mover, descarta a janela inteira e espera de novo.
    if (absSpeed > RevisionConfig::DOWN_COLLECT_SPEED_RAD_S)
    {
        downCollecting_ = false;
        downStableStartUs_ = 0;
        downAnchorAbsoluteRad_ = pendulumPosition_.absoluteAngleRadians();
        downVelocity_.reset(0.0F, nowUs);
        downCalibrationSamples_ = 0;
        Serial.println(F("T,RESTART,MOVE"));
        return;
    }

    ++downCalibrationSamples_;

    if (downReference_.update(rawRelativeToDownAnchor(), nowUs))
        finishDownCalibration();
}

void FurutaRevisionSystem::finishDownCalibration()
{
    downCollecting_ = false;

    // Uma janela ruim nao encerra T: ela e descartada e o firmware
    // volta automaticamente a esperar o pendulo assentar.
    if (!downReference_.isReady() || downReference_.wasRejected())
    {
        downReferenceDefined_ = false;
        downStableStartUs_ = 0;
        const uint32_t nowUs = micros();
        downAnchorAbsoluteRad_ = pendulumPosition_.absoluteAngleRadians();
        downVelocity_.reset(0.0F, nowUs);
        downCalibrationSamples_ = 0;
        Serial.print(F("T,RESTART,SPAN="));
        Serial.println(downReference_.observedSpanDeg(), 3);
        return;
    }

    if (downCalibrationReadErrors_ > 0)
    {
        downReferenceDefined_ = false;
        downStableStartUs_ = 0;
        downCalibrationReadErrors_ = 0;
        const uint32_t nowUs = micros();
        downAnchorAbsoluteRad_ = pendulumPosition_.absoluteAngleRadians();
        downVelocity_.reset(0.0F, nowUs);
        Serial.println(F("T,RESTART,READ"));
        return;
    }

    downCalibrationActive_ = false;
    downReferenceDefined_ = true;
    state_.beta = betaFromRobustDownReference();
    state_.betaDot = 0.0F;
    pendulumVelocity_.reset(state_.beta, micros());

    // Referencia absoluta em contagens AS5600 para comparar T repetidos.
    float downAbsRad = downAnchorAbsoluteRad_ + downReference_.offsetRad();
    while (downAbsRad < 0.0F) downAbsRad += FurutaConfig::TWO_PI_LOCAL;
    while (downAbsRad >= FurutaConfig::TWO_PI_LOCAL) downAbsRad -= FurutaConfig::TWO_PI_LOCAL;
    uint16_t downRaw = static_cast<uint16_t>(
        downAbsRad * (4096.0F / FurutaConfig::TWO_PI_LOCAL) + 0.5F
    );
    downRaw &= 0x0FFF;

    Serial.print(F("T,OK,N=")); Serial.print(downCalibrationSamples_);
    Serial.print(F(",SPAN=")); Serial.print(downReference_.observedSpanDeg(), 3);
    Serial.print(F(",RAW=")); Serial.println(downRaw);

    if (settings_.rawHistoryEnabled)
        rawHistoryAppend(downRaw);
}

// ============================================================
// DIAGNOSTICO DO TOP
// ============================================================

void FurutaRevisionSystem::toggleTopDiagnostic()
{
    if (controlState_ != ControlState::IDLE || downCalibrationActive_ || topMeasureActive_)
    {
        Serial.println(F("ERR,V,BUSY"));
        return;
    }
    if (!downReferenceDefined_)
    {
        Serial.println(F("ERR,V,T_FIRST"));
        return;
    }

    motor_.stop();
    topDiagnosticActive_ = !topDiagnosticActive_;

    if (topDiagnosticActive_)
    {
        nextTopDiagnosticUs_ = micros();
        pendulumVelocity_.reset(state_.beta, micros());
        Serial.print(F("V,START,HOLD=")); Serial.println(motor_.isEnabled() ? 1 : 0);
        Serial.println(F("beta_deg,betaDot,raw"));
    }
    else
    {
        Serial.println(F("V,END"));
    }
}

void FurutaRevisionSystem::serviceTopDiagnostic(uint32_t nowUs)
{
    if (!topDiagnosticActive_)
    {
        return;
    }

    if (static_cast<int32_t>(nowUs - nextTopDiagnosticUs_) < 0)
    {
        return;
    }

    nextTopDiagnosticUs_ = nowUs + RevisionConfig::TOP_DIAGNOSTIC_PERIOD_US;

    Serial.print(state_.beta * FurutaConfig::RAD_TO_DEG_LOCAL, 3);
    Serial.print(',');
    Serial.print(state_.betaDot, 4);
    Serial.print(',');
    Serial.println(pendulumPosition_.raw());
}


// ============================================================
// MEDICAO COMPACTA DO TOP (O)
// ============================================================

void FurutaRevisionSystem::resetTopWindow()
{
    topSampleCount_ = 0;
    topSumBetaRad_ = 0.0F;
    topMinBetaRad_ = 1000.0F;
    topMaxBetaRad_ = -1000.0F;
}

void FurutaRevisionSystem::toggleTopMeasurement()
{
    if (topMeasureActive_)
    {
        topMeasureActive_ = false;
        topCollecting_ = false;
        topStableStartUs_ = 0;
        Serial.println(F("O,CANCEL"));
        return;
    }

    if (controlState_ != ControlState::IDLE || downCalibrationActive_)
    {
        Serial.println(F("ERR,BUSY"));
        return;
    }

    if (!downReferenceDefined_)
    {
        Serial.println(F("ERRO: use T primeiro."));
        return;
    }

    motor_.stop();                    // ENABLE pode permanecer ativo: HOLD
    topDiagnosticActive_ = false;
    topMeasureActive_ = true;
    topCollecting_ = false;
    topStableStartUs_ = 0;
    resetTopWindow();

    state_.beta = betaFromRobustDownReference();
    state_.betaDot = 0.0F;
    pendulumVelocity_.reset(state_.beta, micros());

    Serial.println(F("O,ARMED: segure TOP imovel"));
}

void FurutaRevisionSystem::serviceTopMeasurement(uint32_t nowUs)
{
    if (!topMeasureActive_ || !pendulumVelocity_.isReady()) return;

    const float betaDeg = state_.beta * FurutaConfig::RAD_TO_DEG_LOCAL;
    const float absBetaDeg = fabsf(betaDeg);
    const float absSpeed = fabsf(state_.betaDot);

    if (!topCollecting_)
    {
        if (
            absBetaDeg > RevisionConfig::TOP_POSITION_MAX_DEG
            || absSpeed > RevisionConfig::TOP_STABLE_SPEED_RAD_S
        )
        {
            topStableStartUs_ = 0;
            return;
        }

        if (topStableStartUs_ == 0)
        {
            topStableStartUs_ = nowUs;
            return;
        }

        if (nowUs - topStableStartUs_ < RevisionConfig::TOP_STABLE_TIME_US)
            return;

        topCollecting_ = true;
        topCollectStartUs_ = nowUs;
        resetTopWindow();
        Serial.println(F("O,COLLECT"));
    }

    if (
        absBetaDeg > RevisionConfig::TOP_POSITION_MAX_DEG
        || absSpeed > RevisionConfig::TOP_COLLECT_SPEED_RAD_S
    )
    {
        topCollecting_ = false;
        topStableStartUs_ = 0;
        resetTopWindow();
        Serial.println(F("O,MOVED"));
        return;
    }

    ++topSampleCount_;
    topSumBetaRad_ += state_.beta;
    if (state_.beta < topMinBetaRad_) topMinBetaRad_ = state_.beta;
    if (state_.beta > topMaxBetaRad_) topMaxBetaRad_ = state_.beta;

    if (nowUs - topCollectStartUs_ < RevisionConfig::TOP_MEASURE_TIME_US)
        return;

    topMeasureActive_ = false;
    topCollecting_ = false;
    topStableStartUs_ = 0;

    if (topSampleCount_ == 0)
    {
        Serial.println(F("O,REJECT,NO_SAMPLES"));
        return;
    }

    const float meanBeta =
        topSumBetaRad_ / static_cast<float>(topSampleCount_);
    const float spanRad = topMaxBetaRad_ - topMinBetaRad_;
    const float spanDeg = spanRad * FurutaConfig::RAD_TO_DEG_LOCAL;

    if (spanDeg > RevisionConfig::TOP_MAX_SPAN_DEG)
    {
        Serial.print(F("O,REJECT,SPAN="));
        Serial.println(spanDeg, 3);
        return;
    }

    Serial.print(F("O,OK,n=")); Serial.print(topSampleCount_);
    Serial.print(F(",beta=")); Serial.print(meanBeta * FurutaConfig::RAD_TO_DEG_LOCAL, 4);
    Serial.print(F(",span=")); Serial.println(spanDeg, 4);
}

// ============================================================
// TICK DE CONTROLE
// ============================================================

void FurutaRevisionSystem::controlTick(uint32_t nowUs)
{
    const uint32_t dtUs = nowUs - lastControlTimeUs_;
    const float dtSeconds = static_cast<float>(dtUs) * 1.0e-6F;
    lastControlTimeUs_ = nowUs;

    const bool sensorOK = acquireState(nowUs);

    // A calibracao usa somente leituras validas.
    serviceDownCalibration(nowUs, sensorOK);

    if (!sensorOK)
    {
        if (
            controlState_ == ControlState::BALANCE
            && consecutiveSensorErrors_ >= RevisionConfig::MAX_CONSECUTIVE_SENSOR_ERRORS
        )
        {
            endBalance("AS5600_READ_ERROR");
        }
        return;
    }

    serviceTopMeasurement(nowUs);
    serviceTopDiagnostic(nowUs);

    switch (controlState_)
    {
        case ControlState::IDLE:
            break;

        case ControlState::WAIT_CAPTURE:
            serviceWaitCapture(nowUs);
            break;

        case ControlState::WAIT_RELEASE:
            serviceWaitRelease(nowUs);
            break;

        case ControlState::BALANCE:
            serviceBalance(nowUs, dtSeconds, dtUs);
            break;
    }
}

// ============================================================
// CAPTURA / SOLTURA
// ============================================================

void FurutaRevisionSystem::serviceWaitCapture(uint32_t nowUs)
{
    if (!pendulumVelocity_.isReady())
    {
        captureStableStartUs_ = 0;
        releaseConfirmCount_ = 0;
        return;
    }

    const float absBetaDeg =
        fabsf(state_.beta * FurutaConfig::RAD_TO_DEG_LOCAL);

    const bool betaOK =
        absBetaDeg <= settings_.captureBetaMaxDeg;

    const bool betaDotOK =
        fabsf(state_.betaDot)
        <= settings_.captureBetaDotMaxRadS;

    const bool phiOK =
        fabsf(state_.phi * FurutaConfig::RAD_TO_DEG_LOCAL)
        <= settings_.captureArmMaxDeg;

    if (betaOK && betaDotOK && phiOK)
    {
        if (captureStableStartUs_ == 0)
        {
            captureStableStartUs_ = nowUs;
        }

        if (
            nowUs - captureStableStartUs_
            >= settings_.captureStableTimeUs
        )
        {
            controlState_ = ControlState::WAIT_RELEASE;
            captureStableStartUs_ = 0;
            releaseConfirmCount_ = 0;

            Serial.print(F("CAPTURE_READY,beta="));
            Serial.print(state_.beta * FurutaConfig::RAD_TO_DEG_LOCAL, 3);
            Serial.print(F(",bd=")); Serial.println(state_.betaDot, 4);
        }
    }
    else
    {
        captureStableStartUs_ = 0;
        releaseConfirmCount_ = 0;
    }
}

void FurutaRevisionSystem::serviceWaitRelease(uint32_t nowUs)
{
    if (!pendulumVelocity_.isReady())
    {
        releaseConfirmCount_ = 0;
        return;
    }

    const float absBetaDeg =
        fabsf(state_.beta * FurutaConfig::RAD_TO_DEG_LOCAL);

    // Primeiro confirmamos a soltura pela velocidade. Isso e deliberado:
    // betaDot vem de uma regressao de 7 amostras e responde com atraso.
    // Se testarmos o angulo antes, podemos declarar CAPTURE_LOST justamente
    // quando a soltura real esta sendo reconhecida pelo estimador.
    if (fabsf(state_.betaDot) >= settings_.releaseVelocityRadS)
    {
        if (releaseConfirmCount_ < settings_.releaseConfirmSamples)
            ++releaseConfirmCount_;

        if (releaseConfirmCount_ >= settings_.releaseConfirmSamples)
        {
            // Nenhuma impressao serial longa aqui: assume o controle imediatamente.
            startBalance(nowUs);
            return;
        }
    }
    else
    {
        releaseConfirmCount_ = 0;
    }

    // Somente se a soltura AINDA nao foi confirmada verificamos se o pendulo
    // se afastou demais. A janela de 1,20 deg serve apenas para WAIT_RELEASE;
    // CAPTURE_READY continua exigindo +/-0,40 deg por 150 ms.
    if (absBetaDeg > settings_.releaseBetaMaxDeg)
    {
        motor_.stop();
        controlState_ = ControlState::WAIT_CAPTURE;
        captureStableStartUs_ = 0;
        releaseConfirmCount_ = 0;

        Serial.println(F("CAPTURE_LOST"));
        return;
    }
}

// ============================================================
// BALANCE
// ============================================================

void FurutaRevisionSystem::startBalance(uint32_t nowUs)
{
    motor_.stop();

    // A referencia de velocidade foi zerada pelo stop().
    state_.phiDot = 0.0F;
    balanceInitialState_ = state_;

    resetControlSignal();
    resetStatistics();
    resetDebugTrace();

    balanceStartUs_ = nowUs;
    releaseConfirmCount_ = 0;

    // IMPORTANTE: nao reprogramar nextControlTimeUs_ aqui.
    // startBalance() e chamado de dentro do controlTick(); ao retornar,
    // update() ja soma exatamente um CONTROL_PERIOD_US ao agendamento.
    // A versao anterior tambem fazia now+4 ms aqui e depois somava mais
    // 4 ms em update(), criando um primeiro intervalo de ~8 ms.
    lastControlTimeUs_ = nowUs;

    // Nenhuma telemetria Serial durante BALANCE.
    // Somente o resumo final sera impresso.
    telemetry_.stop();
    controlState_ = ControlState::BALANCE;
}

void FurutaRevisionSystem::serviceBalance(
    uint32_t nowUs,
    float dtSeconds,
    uint32_t dtUs
)
{
    if (dtUs > settings_.maxControlDtUs)
    {
        endBalance("CONTROL_OVERRUN");
        return;
    }

    if (!pendulumVelocity_.isReady())
    {
        endBalance("VELOCITY_NOT_READY");
        return;
    }

    if (fabsf(state_.beta) >= (settings_.betaAbortDeg * FurutaConfig::DEG_TO_RAD_LOCAL))
    {
        endBalance("BETA_LIMIT");
        return;
    }

    if (
        fabsf(state_.phi * FurutaConfig::RAD_TO_DEG_LOCAL)
        >= settings_.armAbortDeg
    )
    {
        endBalance("ARM_LIMIT");
        return;
    }

    if (nowUs - balanceStartUs_ >= settings_.balanceTestTimeUs)
    {
        endBalance("TIME_LIMIT");
        return;
    }

    // Mesma lei de quatro estados, agora calculada com os ganhos
    // correntes de settings_, que podem ser alterados pela Serial em IDLE.
    control_ = computeRuntimeControl();

    // Registra somente em RAM, de forma decimada. Nenhuma Serial aqui.
    recordDebugTrace(nowUs);

    const float commandDegS2 =
        control_.appliedRadS2 * FurutaConfig::RAD_TO_DEG_LOCAL;

    const bool accepted = motor_.commandAcceleration(commandDegS2, dtSeconds);

    if (!accepted)
    {
        endBalance("MOTOR_COMMAND_REJECTED");
        return;
    }

    motor_.update();
    updateStatistics(dtUs);

    // V9: intencionalmente NENHUMA Serial aqui.
}

ControlSignal FurutaRevisionSystem::computeRuntimeControl() const
{
    ControlSignal signal;

    signal.phiTermRadS2 = settings_.kPhi * state_.phi;
    signal.phiDotTermRadS2 = settings_.kPhiDot * state_.phiDot;
    signal.betaTermRadS2 = settings_.kBeta * state_.beta;
    signal.betaDotTermRadS2 = settings_.kBetaDot * state_.betaDot;

    signal.rawRadS2 =
        signal.phiTermRadS2
        + signal.phiDotTermRadS2
        + signal.betaTermRadS2
        + signal.betaDotTermRadS2;

    const float limitRadS2 =
        fabsf(settings_.maxAccelDegS2) * FurutaConfig::DEG_TO_RAD_LOCAL;

    if (signal.rawRadS2 > limitRadS2)
        signal.appliedRadS2 = limitRadS2;
    else if (signal.rawRadS2 < -limitRadS2)
        signal.appliedRadS2 = -limitRadS2;
    else
        signal.appliedRadS2 = signal.rawRadS2;

    signal.saturated =
        fabsf(signal.rawRadS2 - signal.appliedRadS2) > 0.0001F;

    return signal;
}

void FurutaRevisionSystem::endBalance(const char *reason)
{
    motor_.stop();
    telemetry_.stop();

    // O motor ja esta parado: agora e seguro despejar o trace pela Serial.
    dumpDebugTrace();
    printBalanceSummary(reason);

    resetControlSignal();
    controlState_ = ControlState::IDLE;
    captureStableStartUs_ = 0;
    releaseConfirmCount_ = 0;

    Serial.println(F("BALANCE,END,HOLD"));
}

void FurutaRevisionSystem::stopControl(bool printMessage)
{
    if (controlState_ == ControlState::BALANCE)
    {
        endBalance("USER_STOP");
        return;
    }

    telemetry_.stop();
    motor_.stop();
    resetControlSignal();
    controlState_ = ControlState::IDLE;
    captureStableStartUs_ = 0;
    releaseConfirmCount_ = 0;
    topMeasureActive_ = false;
    topCollecting_ = false;
    downCalibrationActive_ = false;
    downCollecting_ = false;
    downStableStartUs_ = 0;

    if (printMessage)
    {
        Serial.println(F("STOP,OK"));
    }
}

// ============================================================
// BRACO / MOTOR
// ============================================================

void FurutaRevisionSystem::defineArmZero()
{
    if (controlState_ != ControlState::IDLE || downCalibrationActive_)
    {
        Serial.println(F("ERR,BUSY"));
        return;
    }

    if (motor_.isEnabled())
    {
        Serial.println(F("ERR,Z,DISABLE"));
        return;
    }

    motor_.setCurrentPosition(0.0F);
    state_.phi = 0.0F;
    state_.phiDot = 0.0F;
    armZeroDefined_ = true;

    Serial.println(F("Z,OK"));
}

void FurutaRevisionSystem::enableMotor()
{
    if (downCalibrationActive_)
    {
        Serial.println(F("ERRO: aguarde a calibracao DOWN."));
        return;
    }

    if (!armZeroDefined_)
    {
        Serial.println(F("ERR,E,Z_FIRST"));
        return;
    }

    motor_.stop();
    motor_.enable();
    Serial.println(F("E,OK,HOLD"));
    if (topDiagnosticActive_)
    {
        Serial.println(F("V,ACTIVE"));
    }
}

void FurutaRevisionSystem::disableMotor()
{
    stopControl(false);
    motor_.disable();
    armZeroDefined_ = false;
    topDiagnosticActive_ = false;
    topMeasureActive_ = false;
    topCollecting_ = false;

    Serial.println(F("D,OK"));
    Serial.println(F("Z,LOST"));
}

void FurutaRevisionSystem::armBalance()
{
    if (controlState_ != ControlState::IDLE || downCalibrationActive_ || topMeasureActive_)
    {
        Serial.println(F("ERR,BUSY"));
        return;
    }

    if (!downReferenceDefined_)
    {
        Serial.println(F("ERR,B,T_FIRST"));
        return;
    }

    if (!armZeroDefined_)
    {
        Serial.println(F("ERR,B,Z_FIRST"));
        return;
    }

    if (!motor_.isEnabled())
    {
        Serial.println(F("ERR,B,E_FIRST"));
        return;
    }

    if (
        fabsf(motor_.currentPositionDegrees())
        > settings_.captureArmMaxDeg
    )
    {
        Serial.println(F("ERR,B,PHI"));
        return;
    }

    motor_.stop();
    resetControlSignal();
    captureStableStartUs_ = 0;
    releaseConfirmCount_ = 0;
    topDiagnosticActive_ = false;

    Serial.println(F("B,ARMED,BASE_CAPTURE,TAIL_RAM"));

    // Evita usar historico de betaDot de uma operacao anterior.
    state_.beta = betaFromRobustDownReference();
    state_.betaDot = 0.0F;
    pendulumVelocity_.reset(state_.beta, micros());

    controlState_ = ControlState::WAIT_CAPTURE;
}

void FurutaRevisionSystem::emergencyStop()
{
    telemetry_.stop();
    motor_.emergencyStop();
    resetControlSignal();

    controlState_ = ControlState::IDLE;
    captureStableStartUs_ = 0;
    releaseConfirmCount_ = 0;
    armZeroDefined_ = false;
    topDiagnosticActive_ = false;
    topMeasureActive_ = false;
    topCollecting_ = false;
    downCalibrationActive_ = false;
    downCollecting_ = false;
    downStableStartUs_ = 0;

    Serial.println(F("X,EMERGENCY,Z_LOST"));
}

// ============================================================
// ESTATISTICAS
// ============================================================

void FurutaRevisionSystem::resetControlSignal()
{
    control_.phiTermRadS2 = 0.0F;
    control_.phiDotTermRadS2 = 0.0F;
    control_.betaTermRadS2 = 0.0F;
    control_.betaDotTermRadS2 = 0.0F;
    control_.rawRadS2 = 0.0F;
    control_.appliedRadS2 = 0.0F;
    control_.saturated = false;
}

void FurutaRevisionSystem::resetStatistics()
{
    statistics_.maxAbsBetaRad = fabsf(state_.beta);
    statistics_.maxAbsBetaDotRadS = fabsf(state_.betaDot);
    statistics_.maxAbsPhiDeg = fabsf(state_.phi * FurutaConfig::RAD_TO_DEG_LOCAL);
    statistics_.maxAbsPhiDotDegS = fabsf(state_.phiDot * FurutaConfig::RAD_TO_DEG_LOCAL);
    statistics_.maxAbsControlDegS2 = 0.0F;
    statistics_.saturationCount = 0;
    statistics_.sampleCount = 0;
    statistics_.sensorErrorCount = 0;
    statistics_.maxControlDtUs = 0;
    statistics_.minControlDtUs = 0xFFFFFFFFUL;
    statistics_.sumControlDtUs = 0;
    statistics_.maxTickExecUs = 0;
    statistics_.missedScheduleSlots = 0;
    statistics_.sumPhiDeg = 0.0F;
    statistics_.sumBetaDeg = 0.0F;
    statistics_.sumControlDegS2 = 0.0F;
}

void FurutaRevisionSystem::updateStatistics(uint32_t dtUs)
{
    const float absBeta = fabsf(state_.beta);
    const float absBetaDot = fabsf(state_.betaDot);
    const float absPhiDeg = fabsf(state_.phi * FurutaConfig::RAD_TO_DEG_LOCAL);
    const float absPhiDotDegS = fabsf(state_.phiDot * FurutaConfig::RAD_TO_DEG_LOCAL);
    const float absUDegS2 = fabsf(control_.appliedRadS2 * FurutaConfig::RAD_TO_DEG_LOCAL);

    if (absBeta > statistics_.maxAbsBetaRad) statistics_.maxAbsBetaRad = absBeta;
    if (absBetaDot > statistics_.maxAbsBetaDotRadS) statistics_.maxAbsBetaDotRadS = absBetaDot;
    if (absPhiDeg > statistics_.maxAbsPhiDeg) statistics_.maxAbsPhiDeg = absPhiDeg;
    if (absPhiDotDegS > statistics_.maxAbsPhiDotDegS) statistics_.maxAbsPhiDotDegS = absPhiDotDegS;
    if (absUDegS2 > statistics_.maxAbsControlDegS2) statistics_.maxAbsControlDegS2 = absUDegS2;

    if (control_.saturated) ++statistics_.saturationCount;
    ++statistics_.sampleCount;

    if (dtUs > statistics_.maxControlDtUs) statistics_.maxControlDtUs = dtUs;
    if (dtUs < statistics_.minControlDtUs) statistics_.minControlDtUs = dtUs;
    statistics_.sumControlDtUs += dtUs;

    statistics_.sumPhiDeg += state_.phi * FurutaConfig::RAD_TO_DEG_LOCAL;
    statistics_.sumBetaDeg += state_.beta * FurutaConfig::RAD_TO_DEG_LOCAL;
    statistics_.sumControlDegS2 +=
        control_.appliedRadS2 * FurutaConfig::RAD_TO_DEG_LOCAL;

    statistics_.sensorErrorCount = totalSensorErrors_;
}

// ============================================================
// TRACE DE DIAGNOSTICO EM RAM
// ============================================================

int16_t FurutaRevisionSystem::toInt16Scaled(float value, float scale)
{
    float scaled = value * scale;
    if (scaled > 32767.0F) scaled = 32767.0F;
    if (scaled < -32768.0F) scaled = -32768.0F;
    return static_cast<int16_t>(scaled >= 0.0F ? scaled + 0.5F : scaled - 0.5F);
}

void FurutaRevisionSystem::resetDebugTrace()
{
    debugSampleCount_ = 0;
    debugWriteIndex_ = 0;
    debugDecimationCounter_ = 0;
}

void FurutaRevisionSystem::recordDebugTrace(uint32_t nowUs)
{
    (void)nowUs;

    if (debugDecimationCounter_ != 0)
    {
        ++debugDecimationCounter_;
        if (debugDecimationCounter_ >= RevisionConfig::DEBUG_DECIMATION)
            debugDecimationCounter_ = 0;
        return;
    }

    DebugSample &sample = debugSamples_[debugWriteIndex_];

    const float phiDeg = state_.phi * FurutaConfig::RAD_TO_DEG_LOCAL;
    const float betaDeg = state_.beta * FurutaConfig::RAD_TO_DEG_LOCAL;
    const float uDegS2 = control_.appliedRadS2 * FurutaConfig::RAD_TO_DEG_LOCAL;

    sample.phiCdeg = toInt16Scaled(phiDeg, 100.0F);
    sample.betaMdeg = toInt16Scaled(betaDeg, 1000.0F);
    sample.betaDot5e3 = toInt16Scaled(state_.betaDot, 5000.0F);
    sample.uDegS2 = toInt16Scaled(uDegS2, 1.0F);

    debugWriteIndex_ = static_cast<uint8_t>(
        (debugWriteIndex_ + 1U) % RevisionConfig::DEBUG_MAX_SAMPLES
    );

    if (debugSampleCount_ < RevisionConfig::DEBUG_MAX_SAMPLES)
        ++debugSampleCount_;

    debugDecimationCounter_ = 1;
}

void FurutaRevisionSystem::dumpDebugTrace()
{
    Serial.println(F("#TAIL_BEGIN"));
    Serial.println(F("#t_to_end_ms,phi_deg,beta_deg,betaDot,u_deg_s2"));

    if (debugSampleCount_ == 0)
    {
        Serial.println(F("#TAIL_END"));
        return;
    }

    const uint8_t oldestIndex =
        (debugSampleCount_ < RevisionConfig::DEBUG_MAX_SAMPLES)
        ? 0
        : debugWriteIndex_;

    const uint32_t tracePeriodMs =
        (settings_.controlPeriodUs * RevisionConfig::DEBUG_DECIMATION) / 1000UL;

    for (uint8_t logical = 0; logical < debugSampleCount_; ++logical)
    {
        const uint8_t physical = static_cast<uint8_t>(
            (oldestIndex + logical) % RevisionConfig::DEBUG_MAX_SAMPLES
        );

        const DebugSample &sample = debugSamples_[physical];

        const float phiDeg = static_cast<float>(sample.phiCdeg) / 100.0F;
        const float betaDeg = static_cast<float>(sample.betaMdeg) / 1000.0F;
        const float betaDot = static_cast<float>(sample.betaDot5e3) / 5000.0F;
        const int16_t uDegS2 = sample.uDegS2;

        // Ultima amostra ~= 0 ms; anteriores aparecem com tempo negativo.
        const int32_t samplesToEnd =
            static_cast<int32_t>(debugSampleCount_ - 1U - logical);
        const int32_t tToEndMs =
            -samplesToEnd * static_cast<int32_t>(tracePeriodMs);

        Serial.print(tToEndMs);
        Serial.print(','); Serial.print(phiDeg, 2);
        Serial.print(','); Serial.print(betaDeg, 3);
        Serial.print(','); Serial.print(betaDot, 4);
        Serial.print(','); Serial.println(uDegS2);
    }

    Serial.println(F("#TAIL_END"));
}


// ============================================================
// HISTORICO RAW — EEPROM
// ============================================================

void FurutaRevisionSystem::rawHistoryInit()
{
    if (!settings_.rawHistoryEnabled) return;

    const bool valid =
        EEPROM.read(RAW_ADDR_MAGIC) == RAW_MAGIC &&
        EEPROM.read(RAW_ADDR_VERSION) == RAW_VERSION &&
        EEPROM.read(RAW_ADDR_COUNT) <= RAW_HISTORY_CAPACITY &&
        EEPROM.read(RAW_ADDR_NEXT) < RAW_HISTORY_CAPACITY;

    if (valid) return;

    EEPROM.update(RAW_ADDR_MAGIC, RAW_MAGIC);
    EEPROM.update(RAW_ADDR_VERSION, RAW_VERSION);
    EEPROM.update(RAW_ADDR_COUNT, 0);
    EEPROM.update(RAW_ADDR_NEXT, 0);
}

uint16_t FurutaRevisionSystem::rawHistoryReadValue(uint8_t logicalIndex) const
{
    const uint8_t count = EEPROM.read(RAW_ADDR_COUNT);
    const uint8_t next = EEPROM.read(RAW_ADDR_NEXT);

    if (logicalIndex >= count || count == 0) return 0;

    const uint8_t start =
        (count < RAW_HISTORY_CAPACITY)
        ? 0
        : next;

    const uint8_t physicalIndex =
        static_cast<uint8_t>((start + logicalIndex) % RAW_HISTORY_CAPACITY);

    const int address = RAW_ADDR_DATA + static_cast<int>(physicalIndex) * 2;
    const uint16_t low = EEPROM.read(address);
    const uint16_t high = EEPROM.read(address + 1);
    return static_cast<uint16_t>(low | (high << 8));
}

void FurutaRevisionSystem::rawHistoryAppend(uint16_t raw)
{
    rawHistoryInit();

    uint8_t count = EEPROM.read(RAW_ADDR_COUNT);
    uint8_t next = EEPROM.read(RAW_ADDR_NEXT);

    const int address = RAW_ADDR_DATA + static_cast<int>(next) * 2;
    EEPROM.update(address, static_cast<uint8_t>(raw & 0xFF));
    EEPROM.update(address + 1, static_cast<uint8_t>((raw >> 8) & 0xFF));

    next = static_cast<uint8_t>((next + 1) % RAW_HISTORY_CAPACITY);
    if (count < RAW_HISTORY_CAPACITY) ++count;

    EEPROM.update(RAW_ADDR_COUNT, count);
    EEPROM.update(RAW_ADDR_NEXT, next);

    uint16_t minRaw = 4095;
    uint16_t maxRaw = 0;
    uint32_t sumRaw = 0;

    for (uint8_t i = 0; i < count; ++i)
    {
        const uint16_t value = rawHistoryReadValue(i);
        if (value < minRaw) minRaw = value;
        if (value > maxRaw) maxRaw = value;
        sumRaw += value;
    }

    const int16_t delta =
        static_cast<int16_t>(raw) -
        static_cast<int16_t>(settings_.rawReferenceCounts);

    Serial.print(F("RAWLOG,n=")); Serial.print(count);
    Serial.print(F(",last=")); Serial.print(raw);
    Serial.print(F(",dref="));
    if (delta >= 0) Serial.print('+');
    Serial.print(delta);
    Serial.print(F(",min=")); Serial.print(minRaw);
    Serial.print(F(",max=")); Serial.print(maxRaw);
    Serial.print(F(",mean="));
    Serial.println(static_cast<float>(sumRaw) / static_cast<float>(count), 2);
}

void FurutaRevisionSystem::printRawHistory()
{
    if (!settings_.rawHistoryEnabled)
    {
        Serial.println(F("RAWLOG,OFF"));
        return;
    }

    rawHistoryInit();
    const uint8_t count = EEPROM.read(RAW_ADDR_COUNT);

    Serial.print(F("RAW,HIST,n=")); Serial.print(count);
    Serial.print(F(",ref=")); Serial.print(settings_.rawReferenceCounts);
    Serial.print(F(",values="));

    if (count == 0)
    {
        Serial.println(F("-"));
        return;
    }

    for (uint8_t i = 0; i < count; ++i)
    {
        if (i > 0) Serial.print('/');
        Serial.print(rawHistoryReadValue(i));
    }
    Serial.println();
}

// ============================================================
// PARAMETROS / STATUS
// ============================================================

void FurutaRevisionSystem::printParameters()
{
    Serial.print(F("P,K="));
    Serial.print(settings_.kPhi, 6); Serial.print(',');
    Serial.print(settings_.kPhiDot, 6); Serial.print(',');
    Serial.print(settings_.kBeta, 6); Serial.print(',');
    Serial.print(settings_.kBetaDot, 6);
    Serial.print(F(",Ts=")); Serial.print(settings_.controlPeriodUs);
    Serial.print(F(",V=")); Serial.print(settings_.motorMaxSpeedDegS, 0);
    Serial.print(F(",A=")); Serial.print(settings_.maxAccelDegS2, 0);
    Serial.print(F(",MS=")); Serial.print(FurutaConfig::MICROSTEP_FACTOR);
    Serial.print(F(",OFF=")); Serial.print(settings_.topReferenceOffsetDeg, 3);
    Serial.print(F(",CAP=")); Serial.print(settings_.captureBetaMaxDeg, 2);
    Serial.print('/'); Serial.print(settings_.captureBetaDotMaxRadS, 2);
    Serial.print(F(",REL=")); Serial.print(settings_.releaseBetaMaxDeg, 2);
    Serial.print('/'); Serial.print(settings_.releaseVelocityRadS, 2);
    Serial.print(F(",DBG=")); Serial.print(RevisionConfig::DEBUG_MAX_SAMPLES);
    Serial.print('/'); Serial.print(RevisionConfig::DEBUG_DECIMATION);
    Serial.print(F(",RAWFIX=")); Serial.print(settings_.rawReferenceCounts);
    Serial.print(F(",RLOG=")); Serial.println(settings_.rawHistoryEnabled ? 1 : 0);
}

void FurutaRevisionSystem::printStatus()
{
    const bool ok = acquireState(micros());
    Serial.print(F("S,st=")); Serial.print(static_cast<uint8_t>(controlState_));
    Serial.print(F(",sen=")); Serial.print(ok ? 1 : 0);
    Serial.print(F(",down=")); Serial.print(downReferenceDefined_ ? 1 : 0);
    Serial.print(F(",zero=")); Serial.print(armZeroDefined_ ? 1 : 0);
    Serial.print(F(",mot=")); Serial.print(motor_.isEnabled() ? 1 : 0);
    Serial.print(F(",b=")); Serial.print(state_.beta * FurutaConfig::RAD_TO_DEG_LOCAL, 3);
    Serial.print(F(",bd=")); Serial.print(state_.betaDot, 4);
    Serial.print(F(",phi=")); Serial.print(state_.phi * FurutaConfig::RAD_TO_DEG_LOCAL, 2);
    Serial.print(F(",raw=")); Serial.print(pendulumPosition_.raw());
    Serial.print(F(",err=")); Serial.println(totalSensorErrors_);
}

void FurutaRevisionSystem::printBalanceSummary(const char *reason)
{
    const uint32_t elapsedUs = micros() - balanceStartUs_;
    const uint32_t elapsedMs = elapsedUs / 1000UL;
    const uint32_t dtAvgUs =
        (statistics_.sampleCount > 0)
        ? (statistics_.sumControlDtUs / statistics_.sampleCount)
        : 0UL;

    Serial.print(F("END,")); Serial.print(reason);
    Serial.print(F(",ms=")); Serial.print(elapsedMs);
    Serial.print(F(",n=")); Serial.print(statistics_.sampleCount);
    Serial.print(F(",dtavg=")); Serial.print(dtAvgUs);
    Serial.print(F(",dtmin="));
    Serial.print(statistics_.minControlDtUs == 0xFFFFFFFFUL ? 0UL : statistics_.minControlDtUs);
    Serial.print(F(",dtmax=")); Serial.print(statistics_.maxControlDtUs);
    Serial.print(F(",execmax=")); Serial.print(statistics_.maxTickExecUs);
    Serial.print(F(",miss=")); Serial.print(statistics_.missedScheduleSlots);
    Serial.print(F(",b0=")); Serial.print(balanceInitialState_.beta * FurutaConfig::RAD_TO_DEG_LOCAL, 3);
    Serial.print(F(",bd0=")); Serial.print(balanceInitialState_.betaDot, 4);
    Serial.print(F(",bmax=")); Serial.print(statistics_.maxAbsBetaRad * FurutaConfig::RAD_TO_DEG_LOCAL, 3);
    Serial.print(F(",phimax=")); Serial.print(statistics_.maxAbsPhiDeg, 2);
    Serial.print(F(",umax=")); Serial.print(statistics_.maxAbsControlDegS2, 1);
    Serial.print(F(",bdmax=")); Serial.print(statistics_.maxAbsBetaDotRadS, 3);
    Serial.print(F(",pdmax=")); Serial.print(statistics_.maxAbsPhiDotDegS, 1);
    Serial.print(F(",sat=")); Serial.print(statistics_.saturationCount);

    if (statistics_.sampleCount > 0)
    {
        const float invN = 1.0F / static_cast<float>(statistics_.sampleCount);
        Serial.print(F(",phimean=")); Serial.print(statistics_.sumPhiDeg * invN, 2);
        Serial.print(F(",bmean=")); Serial.print(statistics_.sumBetaDeg * invN, 3);
        Serial.print(F(",umean=")); Serial.print(statistics_.sumControlDegS2 * invN, 1);
    }

    Serial.print(F(",phiend="));
    Serial.print(state_.phi * FurutaConfig::RAD_TO_DEG_LOCAL, 2);
    Serial.print(F(",err=")); Serial.println(totalSensorErrors_);
}

// ============================================================
// SERIAL
// ============================================================

void FurutaRevisionSystem::serviceSerial()
{
    while (Serial.available() > 0)
    {
        const char c = static_cast<char>(Serial.read());

        if (c == '\r') continue;

        if (c == '\n')
        {
            if (serialBufferIndex_ > 0)
            {
                serialBuffer_[serialBufferIndex_] = '\0';
                handleCommand(serialBuffer_);
                serialBufferIndex_ = 0;
            }
            continue;
        }

        if (serialBufferIndex_ < RevisionConfig::SERIAL_BUFFER_SIZE - 1)
        {
            serialBuffer_[serialBufferIndex_++] = c;
        }
    }
}

void FurutaRevisionSystem::handleCommand(char *command)
{
    while (*command && isspace(static_cast<unsigned char>(*command))) ++command;

    char *end = command + strlen(command);
    while (end > command && isspace(static_cast<unsigned char>(end[-1])))
        *--end = '\0';

    if (*command == '\0') return;

    for (char *p = command; *p; ++p)
        *p = static_cast<char>(toupper(static_cast<unsigned char>(*p)));

    if (controlState_ == ControlState::BALANCE)
    {
        if (command[0] == 'X' && command[1] == '\0') emergencyStop();
        else if (strcmp(command, "STOP") == 0) stopControl();
        return;
    }

    if (topMeasureActive_)
    {
        if (command[0] == 'O' && command[1] == '\0') toggleTopMeasurement();
        else if (command[0] == 'X' && command[1] == '\0') emergencyStop();
        else if (strcmp(command, "STOP") == 0) stopControl();
        return;
    }

    if (handleTuningCommand(command)) return;

    if (command[1] == '\0')
    {
        switch (command[0])
        {
            case 'T':
                Serial.print(F("T,SKIP,FIXED_RAW="));
                Serial.println(settings_.rawReferenceCounts);
                return;
            case 'V': toggleTopDiagnostic(); return;
            case 'O': toggleTopMeasurement(); return;
            case 'Z': defineArmZero(); return;
            case 'E': enableMotor(); return;
            case 'D': disableMotor(); return;
            case 'B': armBalance(); return;
            case 'X': emergencyStop(); return;
            case 'S': printStatus(); return;
            case 'P': printParameters(); return;
            case 'R': printRawHistory(); return;
            case 'H': case '?': printHelp(); return;
        }
    }

    if (strcmp(command, "STOP") == 0) { stopControl(); return; }
    Serial.println(F("ERR,CMD"));
}

bool FurutaRevisionSystem::parseOneFloat(const char *text, float &value)
{
    const char *p = text;
    while (*p && isspace(static_cast<unsigned char>(*p))) ++p;

    float sign = 1.0F;
    if (*p == '-') { sign = -1.0F; ++p; }
    else if (*p == '+') { ++p; }

    bool hasDigit = false;
    float result = 0.0F;

    while (*p >= '0' && *p <= '9')
    {
        hasDigit = true;
        result = result * 10.0F + static_cast<float>(*p - '0');
        ++p;
    }

    if (*p == '.')
    {
        ++p;
        float scale = 0.1F;
        while (*p >= '0' && *p <= '9')
        {
            hasDigit = true;
            result += static_cast<float>(*p - '0') * scale;
            scale *= 0.1F;
            ++p;
        }
    }

    while (*p && isspace(static_cast<unsigned char>(*p))) ++p;
    if (!hasDigit || *p != '\0') return false;

    value = sign * result;
    return true;
}

bool FurutaRevisionSystem::parseFourFloats(
    const char *text,
    float &a,
    float &b,
    float &c,
    float &d
)
{
    float *out[4] = {&a, &b, &c, &d};
    const char *p = text;

    for (uint8_t i = 0; i < 4; ++i)
    {
        while (*p && isspace(static_cast<unsigned char>(*p))) ++p;
        if (*p == '\0') return false;

        float sign = 1.0F;
        if (*p == '-') { sign = -1.0F; ++p; }
        else if (*p == '+') { ++p; }

        bool hasDigit = false;
        float result = 0.0F;

        while (*p >= '0' && *p <= '9')
        {
            hasDigit = true;
            result = result * 10.0F + static_cast<float>(*p - '0');
            ++p;
        }

        if (*p == '.')
        {
            ++p;
            float scale = 0.1F;
            while (*p >= '0' && *p <= '9')
            {
                hasDigit = true;
                result += static_cast<float>(*p - '0') * scale;
                scale *= 0.1F;
                ++p;
            }
        }

        if (!hasDigit) return false;
        *out[i] = sign * result;

        if (i < 3)
        {
            if (!isspace(static_cast<unsigned char>(*p))) return false;
        }
    }

    while (*p && isspace(static_cast<unsigned char>(*p))) ++p;
    return *p == '\0';
}

bool FurutaRevisionSystem::setSingleRuntimeParameter(
    const char *name,
    float value
)
{
    // Faixas deliberadamente conservadoras para evitar comandos absurdos.
    // Nao elevamos AMAX acima do limite atualmente usado nos ensaios.
    if (strcmp(name, "KPHI") == 0)
    {
        if (value < 0.0F || value > 5.0F) return false;
        settings_.kPhi = value;
    }
    else if (strcmp(name, "KPD") == 0)
    {
        if (value < 0.0F || value > 10.0F) return false;
        settings_.kPhiDot = value;
    }
    else if (strcmp(name, "KB") == 0)
    {
        if (value < 0.0F || value > 150.0F) return false;
        settings_.kBeta = value;
    }
    else if (strcmp(name, "KBD") == 0)
    {
        if (value < 0.0F || value > 20.0F) return false;
        settings_.kBetaDot = value;
    }
    else if (strcmp(name, "AMAX") == 0)
    {
        if (value < 50.0F || value > 600.0F) return false;
        settings_.maxAccelDegS2 = value;
    }
    else
    {
        return false;
    }

    Serial.print(F("SET,")); Serial.print(name);
    Serial.print('='); Serial.println(value, 6);
    return true;
}

bool FurutaRevisionSystem::handleTuningCommand(char *command)
{
    // Nunca alteramos parametros com a maquina armada ou controlando.
    if (controlState_ != ControlState::IDLE)
        return false;

    if (strncmp(command, "K ", 2) == 0)
    {
        float k1, k2, k3, k4;
        if (!parseFourFloats(command + 2, k1, k2, k3, k4))
        {
            Serial.println(F("ERR,K,FORMAT"));
            return true;
        }

        if (
            k1 < 0.0F || k1 > 5.0F ||
            k2 < 0.0F || k2 > 10.0F ||
            k3 < 0.0F || k3 > 150.0F ||
            k4 < 0.0F || k4 > 20.0F
        )
        {
            Serial.println(F("ERR,K,RANGE"));
            return true;
        }

        settings_.kPhi = k1;
        settings_.kPhiDot = k2;
        settings_.kBeta = k3;
        settings_.kBetaDot = k4;

        Serial.print(F("SET,K="));
        Serial.print(settings_.kPhi, 6); Serial.print(',');
        Serial.print(settings_.kPhiDot, 6); Serial.print(',');
        Serial.print(settings_.kBeta, 6); Serial.print(',');
        Serial.println(settings_.kBetaDot, 6);
        return true;
    }

    const char *names[] = {"KPHI", "KPD", "KB", "KBD", "AMAX"};
    for (uint8_t i = 0; i < 5; ++i)
    {
        const size_t len = strlen(names[i]);
        if (strncmp(command, names[i], len) == 0 && command[len] == ' ')
        {
            float value;
            if (!parseOneFloat(command + len + 1, value))
            {
                Serial.print(F("ERR,")); Serial.print(names[i]);
                Serial.println(F(",FORMAT"));
                return true;
            }

            if (!setSingleRuntimeParameter(names[i], value))
            {
                Serial.print(F("ERR,")); Serial.print(names[i]);
                Serial.println(F(",RANGE"));
            }
            return true;
        }
    }

    return false;
}

void FurutaRevisionSystem::printHelp()
{
    Serial.println(F("H:D Z E B | P S R | K <kp kpd kb kbd> | KPHI/KPD/KB/KBD <v> | AMAX <v> | T=skip | STOP X"));
}
