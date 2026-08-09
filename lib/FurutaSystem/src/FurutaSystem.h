#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>

#include <MotorVelocity.h>
#include <PendulumPosition.h>
#include <PendulumVelocity.h>

#include <StateVector.h>
#include <FourStateController.h>
#include <BalanceTelemetry.h>

#include "FurutaConfig.h"

class FurutaSystem
{
public:
    FurutaSystem();

    void begin();

    void update();

private:
    enum class ControlState
    {
        IDLE,
        WAIT_CAPTURE,
        WAIT_RELEASE,
        BALANCE
    };

    struct BalanceStatistics
    {
        float maxAbsBetaRad;
        float maxAbsBetaDotRadS;

        float maxAbsPhiDeg;
        float maxAbsPhiDotDegS;

        float maxAbsControlDegS2;

        float maxAbsPhiTermDegS2;
        float maxAbsPhiDotTermDegS2;
        float maxAbsBetaTermDegS2;
        float maxAbsBetaDotTermDegS2;

        uint32_t saturationCount;
        uint32_t sampleCount;
    };

    struct BalanceSnapshot
    {
        uint32_t elapsedUs;

        StateVector state;

        float controlRawRadS2;
        float controlAppliedRadS2;
    };


    // Hardware e bibliotecas.
    AS5600 as5600_;

    PendulumPosition pendulumPosition_;

    PendulumVelocity pendulumVelocity_;

    MotorVelocity motor_;

    FourStateController controller_;

    BalanceTelemetry telemetry_;


    // Estado.
    ControlState controlState_;

    bool armZeroDefined_;

    bool pendulumDownReferenceDefined_;

    StateVector state_;

    ControlSignal control_;

    BalanceStatistics statistics_;

    BalanceSnapshot finalSnapshot_;


    // Timing.
    uint32_t nextControlTimeUs_;

    uint32_t lastControlTimeUs_;

    uint32_t captureStableStartUs_;

    uint32_t balanceStartUs_;


    // Serial.
    char serialBuffer_[
        FurutaConfig::SERIAL_BUFFER_SIZE
    ];

    uint8_t serialBufferIndex_;


    // Utilitarios.
    float wrapToPi(float angle) const;

    float betaFromDownReference();

    void resetControlSignal();

    void resetStatistics();

    void updateStateStatistics();

    void updateControlStatistics();

    void acquireState(uint32_t nowUs);


    // Controle.
    void controlTick(uint32_t nowUs);

    void serviceWaitCapture(uint32_t nowUs);

    void serviceWaitRelease();

    void startBalance();

    void serviceBalance(
        uint32_t nowUs,
        float dtSeconds,
        uint32_t dtUs
    );

    void endBalance(const char *reason);

    void stopControl();


    // Referencias e comandos.
    void definePendulumDownReference();

    void defineArmZero();

    void armBalance();

    void emergencyStop();


    // Serial.
    void serviceSerial();

    void handleCommand(char *command);

    void printStatus();

    void printHelp();

    void printBalanceSummary(
        const char *reason
    );

    const char *stateName() const;
};
