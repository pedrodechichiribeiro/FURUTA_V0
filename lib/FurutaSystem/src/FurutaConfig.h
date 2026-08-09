#pragma once

#include <Arduino.h>

namespace FurutaConfig
{
    // ========================================================
    // Serial
    // ========================================================

    constexpr uint32_t SERIAL_BAUD =
        250000UL;


    // ========================================================
    // Hardware
    // ========================================================

    constexpr uint8_t STEP_PIN =
        6;

    constexpr uint8_t DIR_PIN =
        8;

    constexpr uint8_t ENABLE_PIN =
        4;


    // ========================================================
    // Motor
    // ========================================================

    constexpr uint16_t FULL_STEPS_PER_REVOLUTION =
        200;

    constexpr uint8_t MICROSTEP_FACTOR =
        8;

    constexpr float ARM_MIN_DEG =
        -80.0F;

    constexpr float ARM_MAX_DEG =
        80.0F;

    constexpr float MOTOR_MAX_SPEED_DEG_S =
        180.0F;


    // ========================================================
    // Temporizacao
    // ========================================================

    constexpr uint32_t CONTROL_PERIOD_US =
        4000UL;       // 250 Hz

    constexpr uint32_t TELEMETRY_PERIOD_US =
        40000UL;      // 25 Hz

    constexpr uint32_t MAX_CONTROL_DT_US =
        20000UL;

    // Primeiro teste da refatoracao: 100 s.
   
    // 100000000UL
    constexpr uint32_t BALANCE_TEST_TIME_US =
       100000000UL;


    // ========================================================
    // Conversoes
    //
    // O sufixo LOCAL evita colisao com macros Arduino:
    // DEG_TO_RAD, RAD_TO_DEG e TWO_PI.
    // ========================================================

    constexpr float DEG_TO_RAD_LOCAL =
        0.01745329251994329577F;

    constexpr float RAD_TO_DEG_LOCAL =
        57.295779513082320876F;

    constexpr float PI_LOCAL =
        3.14159265358979323846F;

    constexpr float TWO_PI_LOCAL =
        6.28318530717958647692F;


    // ========================================================
    // Ganhos congelados — Fase 09
    // ========================================================

    constexpr float K_PHI =
        0.77419355F;

    constexpr float K_PHI_DOT =
        2.31978594F;

    constexpr float K_BETA =
        95.71078856F;

    constexpr float K_BETA_DOT =
        14.31971458F;


    // ========================================================
    // Saturacao
    // ========================================================

    constexpr float MAX_ACCEL_DEG_S2 =
        600.0F;

    constexpr float MAX_ACCEL_RAD_S2 =
        MAX_ACCEL_DEG_S2
        *
        DEG_TO_RAD_LOCAL;


    // ========================================================
    // Captura
    // ========================================================

    constexpr float CAPTURE_READY_BETA_DEG =
        1.5F;

    constexpr float CAPTURE_READY_BETA_RAD =
        CAPTURE_READY_BETA_DEG
        *
        DEG_TO_RAD_LOCAL;

    constexpr float CAPTURE_READY_BETA_DOT_RAD_S =
        0.25F;

    constexpr uint32_t CAPTURE_READY_TIME_US =
        80000UL;

    constexpr float RELEASE_VELOCITY_RAD_S =
        0.05F;

    constexpr float CAPTURE_RELEASE_BETA_DEG =
        1.75F;

    constexpr float CAPTURE_RELEASE_BETA_RAD =
        CAPTURE_RELEASE_BETA_DEG
        *
        DEG_TO_RAD_LOCAL;


    // ========================================================
    // Seguranca
    // ========================================================

    constexpr float BETA_ABORT_DEG =
        8.0F;

    constexpr float BETA_ABORT_RAD =
        BETA_ABORT_DEG
        *
        DEG_TO_RAD_LOCAL;

    constexpr float ARM_ABORT_DEG =
        75.0F;


    // ========================================================
    // Parser serial
    // ========================================================

    constexpr uint8_t SERIAL_BUFFER_SIZE =
        24;
}
