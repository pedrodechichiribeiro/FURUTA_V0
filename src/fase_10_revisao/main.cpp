#include <Arduino.h>
#include "FurutaRevisionSystem.h"

// ============================================================
// PARAMETROS DA FASE 10 REVISAO
// Edite SOMENTE este bloco durante os ensaios.
// Nenhuma biblioteca precisa ser alterada.
// ============================================================

const RevisionSettings SETTINGS = {
    // Ganhos: u = Kphi*phi + KphiDot*phiDot + Kbeta*beta + KbetaDot*betaDot
    0.77419355F,  // K_PHI
    2.31978594F,  // K_PHI_DOT
    95.71078856F, // K_BETA
    7.0F,         // K_BETA_DOT  (75% do valor original 14.31971458)

    // Malha / atuador
    4000UL,  // CONTROL_PERIOD_US
    180.0F,  // MOTOR_MAX_SPEED_DEG_S
    600.0F,  // MAX_ACCEL_DEG_S2
    20000UL, // MAX_CONTROL_DT_US

    // Referencia vertical
    -1.0F, // TOP_REFERENCE_OFFSET_DEG

    // CAPTURA
    5.0F,     // phi max [deg]
    0.80F,    // beta max [deg]       antes: 0.40
    0.15F,    // betaDot max [rad/s]  antes: 0.10
    100000UL, // estabilidade [us]    antes: 150000

    // SOLTURA
    1.80F, // beta max [deg]       antes: 1.20
    0.15F, // betaDot soltura [rad/s]
    2,     // amostras consecutivas

    // Abort / duracao
    8.0F,       // BETA_ABORT_DEG
    75.0F,      // ARM_ABORT_DEG
    100000000UL // BALANCE_TEST_TIME_US = 100 s
};

FurutaRevisionSystem furuta(SETTINGS);

void setup()
{
    furuta.begin();
}

void loop()
{
    furuta.update();
}
