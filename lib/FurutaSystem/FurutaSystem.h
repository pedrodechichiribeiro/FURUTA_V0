#pragma once

namespace FurutaConfig
{
    constexpr uint32_t SERIAL_BAUD = 250000UL;

    constexpr uint8_t STEP_PIN   = 6;
    constexpr uint8_t DIR_PIN    = 8;
    constexpr uint8_t ENABLE_PIN = 4;

    constexpr uint16_t FULL_STEPS_PER_REVOLUTION = 200;
    constexpr uint8_t MICROSTEP_FACTOR = 8;

    constexpr float ARM_MIN_DEG = -80.0F;
    constexpr float ARM_MAX_DEG =  80.0F;

    constexpr float MOTOR_MAX_SPEED_DEG_S = 180.0F;

    constexpr uint32_t CONTROL_PERIOD_US   = 4000UL;
    constexpr uint32_t TELEMETRY_PERIOD_US = 40000UL;

    constexpr float DEG_TO_RAD_LOCAL =
        0.01745329251994329577F;

    constexpr float RAD_TO_DEG_LOCAL =
        57.295779513082320876F;

    constexpr float K_PHI      = 0.77419355F;
    constexpr float K_PHI_DOT  = 2.31978594F;
    constexpr float K_BETA     = 95.71078856F;
    constexpr float K_BETA_DOT = 14.31971458F;

    constexpr float MAX_ACCEL_DEG_S2 = 600.0F;

    constexpr float BETA_ABORT_DEG = 8.0F;
    constexpr float ARM_ABORT_DEG  = 75.0F;
}