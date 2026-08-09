#pragma once

#include <Arduino.h>
#include <StateVector.h>
#include <FourStateController.h>

class BalanceTelemetry
{
public:
    explicit BalanceTelemetry(uint32_t periodUs);

    void printSessionHeader(HardwareSerial &serial);

    void start(uint32_t startTimeUs);

    void update(
        uint32_t nowUs,
        const StateVector &state,
        const ControlSignal &control,
        HardwareSerial &serial
    );

    void end(HardwareSerial &serial);

    void stop();

private:
    uint32_t periodUs_;
    uint32_t startTimeUs_;
    uint32_t nextTimeUs_;
    bool active_;
};
