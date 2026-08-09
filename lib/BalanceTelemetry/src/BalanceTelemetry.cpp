#include "BalanceTelemetry.h"

BalanceTelemetry::BalanceTelemetry(uint32_t periodUs)
    : periodUs_(periodUs),
      startTimeUs_(0),
      nextTimeUs_(0),
      active_(false)
{
}

void BalanceTelemetry::printSessionHeader(
    HardwareSerial &serial
)
{
    serial.println();
    serial.println(F("# TELEMETRY_BEGIN"));
    serial.println(
        F("# t_ms,phi_rad,phiDot_rad_s,beta_rad,betaDot_rad_s,uRaw_rad_s2,u_rad_s2")
    );
}

void BalanceTelemetry::start(uint32_t startTimeUs)
{
    startTimeUs_ =
        startTimeUs;

    // Primeira linha apos um periodo completo de telemetria.
    nextTimeUs_ =
        startTimeUs_
        +
        periodUs_;

    active_ =
        true;
}

void BalanceTelemetry::update(
    uint32_t nowUs,
    const StateVector &state,
    const ControlSignal &control,
    HardwareSerial &serial
)
{
    if (!active_)
    {
        return;
    }

    if (
        static_cast<int32_t>(
            nowUs - nextTimeUs_
        )
        <
        0
    )
    {
        return;
    }

    const uint32_t elapsedMs =
        (nowUs - startTimeUs_)
        /
        1000UL;

    // CSV compacto em unidades SI.
    serial.print(elapsedMs);
    serial.print(',');

    serial.print(state.phi, 5);
    serial.print(',');

    serial.print(state.phiDot, 5);
    serial.print(',');

    serial.print(state.beta, 5);
    serial.print(',');

    serial.print(state.betaDot, 5);
    serial.print(',');

    serial.print(control.rawRadS2, 4);
    serial.print(',');

    serial.println(control.appliedRadS2, 4);

    nextTimeUs_ +=
        periodUs_;

    // Nao tenta despejar amostras atrasadas em rajada.
    if (
        static_cast<int32_t>(
            nowUs - nextTimeUs_
        )
        >=
        0
    )
    {
        nextTimeUs_ =
            nowUs
            +
            periodUs_;
    }
}

void BalanceTelemetry::end(
    HardwareSerial &serial
)
{
    if (!active_)
    {
        return;
    }

    active_ =
        false;

    serial.println(F("# TELEMETRY_END"));
    serial.println();
}

void BalanceTelemetry::stop()
{
    active_ =
        false;
}
