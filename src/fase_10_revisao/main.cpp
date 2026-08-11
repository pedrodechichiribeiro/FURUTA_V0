#include <Arduino.h>
#include "FurutaRevisionSystem.h"

RevisionSettings SETTINGS = {
    // Ganhos
    0.77419355F,     // K_PHI
    2.31978594F,     // K_PHI_DOT
    95.71078856F,    // K_BETA
    14.31971458F,    // K_BETA_DOT

    // Malha / atuador
    4000UL,
    180.0F,
    600.0F,
    20000UL,

    // Offset adicional do TOP
    0.0F,

    // Captura
    5.0F,
    1.50F,
    0.25F,
    80000UL,

    // Soltura
    1.75F,
    0.05F,
    1,

    // Abort / duração
    8.0F,
    75.0F,
    100000000UL,

    // DOWN medido
    2901,
    true
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