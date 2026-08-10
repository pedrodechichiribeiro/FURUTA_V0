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

    // CAPTURA — restaurada da fase_10_integracao_final
    5.0F,    // phi max [deg]
    1.50F,   // beta max [deg]
    0.25F,   // betaDot max [rad/s]
    80000UL, // estabilidade = 80 ms

    // SOLTURA — restaurada da fase_10_integracao_final
    1.75F, // beta max antes de CAPTURE_LOST [deg]
    0.05F, // betaDot para reconhecer soltura [rad/s]
    1,     // uma amostra basta

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
