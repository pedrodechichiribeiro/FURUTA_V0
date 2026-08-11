#include <Arduino.h>
#include <Wire.h>
#include <new>
#include "FurutaRevisionSystem.h"

// ============================================================
// REV10 v17b — calibracao direta no TOP, com memoria sobreposta.
//
// Objetivo:
// - calibrar beta=0 diretamente na vertical;
// - manter o motor DESABILITADO durante a calibracao manual;
// - depois reutilizar a MESMA RAM para FurutaRevisionSystem;
// - nao alterar FurutaRevisionSystem.h/.cpp nem qualquer lib/.
// ============================================================

namespace TopCal
{
    constexpr uint32_t TS_US       = 4000UL;      // 250 Hz
    constexpr uint32_t TIME_US     = 3000000UL;   // 3 s
    constexpr int16_t  MAX_SPAN_CT = 12;          // ~1,05 grau
}

// Estes sao os parametros que o FurutaRevisionSystem recebera DEPOIS
// que a referencia do TOP for medida.
RevisionSettings SETTINGS = {
    // Ganhos
    0.77419355F,    // K_PHI
    2.31978594F,    // K_PHI_DOT
    95.71078856F,   // K_BETA
    6.00000000F,    // K_BETA_DOT

    // Malha / atuador
    4000UL,
    180.0F,
    600.0F,
    20000UL,

    // O TOP medido define beta=0 diretamente.
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

    // Abort / duracao
    8.0F,
    75.0F,
    100000000UL,

    // Valor provisório; sera substituido apos a calibracao TOP.
    2888,
    true
};

// Durante a fase de calibracao, este pequeno contexto ocupa o MESMO
// bloco de memoria que depois sera usado pelo FurutaRevisionSystem.
// Assim nao mantemos dois objetos AS5600 nem dois grandes conjuntos
// de estado simultaneamente na SRAM do Nano.
struct CalibrationContext
{
    AS5600 sensor;
    bool collecting;
    uint32_t t0Us;
    uint32_t nextUs;
    uint16_t anchor;
    uint16_t n;
    uint16_t readErr;
    int32_t sumDelta;
    int16_t dMin;
    int16_t dMax;
};

static_assert(sizeof(CalibrationContext) <= sizeof(FurutaRevisionSystem),
              "CalibrationContext maior que FurutaRevisionSystem");
static_assert(alignof(CalibrationContext) <= alignof(FurutaRevisionSystem),
              "Alinhamento incompativel");

alignas(FurutaRevisionSystem)
static uint8_t systemMemory[sizeof(FurutaRevisionSystem)];

static CalibrationContext *cal = nullptr;
static FurutaRevisionSystem *furuta = nullptr;

static int16_t rawDelta(uint16_t raw, uint16_t ref)
{
    int16_t d = (int16_t)raw - (int16_t)ref;
    if (d > 2048) d -= 4096;
    else if (d < -2048) d += 4096;
    return d;
}

static void startCalibration()
{
    if (cal == nullptr || cal->collecting) return;

    const uint16_t raw = cal->sensor.readAngle() & 0x0FFFU;
    if (cal->sensor.lastError() != AS5600_OK)
    {
        Serial.println(F("TOP,ERR,READ"));
        return;
    }

    cal->anchor = raw;
    cal->n = 0;
    cal->readErr = 0;
    cal->sumDelta = 0;
    cal->dMin = 0;
    cal->dMax = 0;
    cal->t0Us = micros();
    cal->nextUs = cal->t0Us;
    cal->collecting = true;

    Serial.println(F("TOP,COLLECT"));
}

static void enterControl(uint16_t topRaw)
{
    // A referencia equivalente DOWN fica exatamente 180 graus do TOP.
    const uint16_t downEq = (topRaw + 2048U) & 0x0FFFU;

    SETTINGS.rawReferenceCounts = downEq;
    SETTINGS.topReferenceOffsetDeg = 0.0F;

    Serial.print(F("TOP,REF,RAW_TOP="));
    Serial.print(topRaw);
    Serial.print(F(",RAW_DOWN_EQ="));
    Serial.println(downEq);
    Serial.flush();

    // O contexto de calibracao deixa de existir. A mesma RAM passa a
    // conter o controlador completo.
    cal->~CalibrationContext();
    cal = nullptr;

    furuta = new (systemMemory) FurutaRevisionSystem(SETTINGS);
    furuta->begin();

    Serial.println(F("READY:D Z E B"));
}

static void finishCalibration()
{
    cal->collecting = false;

    if (cal->n == 0 || cal->readErr != 0)
    {
        Serial.println(F("TOP,REJECT,READ"));
        return;
    }

    const int16_t span = cal->dMax - cal->dMin;
    if (span > TopCal::MAX_SPAN_CT)
    {
        Serial.print(F("TOP,REJECT,SPAN_C="));
        Serial.println(span);
        return;
    }

    // Media das leituras, arredondada para a contagem mais proxima.
    int32_t meanDelta = cal->sumDelta;
    if (meanDelta >= 0) meanDelta += (int32_t)cal->n / 2;
    else                meanDelta -= (int32_t)cal->n / 2;
    meanDelta /= (int32_t)cal->n;

    int32_t top = (int32_t)cal->anchor + meanDelta;
    if (top < 0) top += 4096;
    else if (top >= 4096) top -= 4096;

    const uint16_t topRaw = (uint16_t)top;

    Serial.print(F("TOP,OK,N="));
    Serial.print(cal->n);
    Serial.print(F(",SPAN_C="));
    Serial.print(span);
    Serial.print(F(",RAW_TOP="));
    Serial.println(topRaw);
    Serial.flush();

    enterControl(topRaw);
}

static void serviceCalibration()
{
    if (cal == nullptr || !cal->collecting) return;

    const uint32_t now = micros();
    if ((int32_t)(now - cal->nextUs) < 0) return;

    cal->nextUs += TopCal::TS_US;
    if ((int32_t)(now - cal->nextUs) >= 0)
        cal->nextUs = now + TopCal::TS_US;

    const uint16_t raw = cal->sensor.readAngle() & 0x0FFFU;
    if (cal->sensor.lastError() != AS5600_OK)
    {
        ++cal->readErr;
    }
    else
    {
        const int16_t d = rawDelta(raw, cal->anchor);

        if (cal->n == 0)
            cal->dMin = cal->dMax = d;
        else
        {
            if (d < cal->dMin) cal->dMin = d;
            if (d > cal->dMax) cal->dMax = d;
        }

        cal->sumDelta += d;
        ++cal->n;
    }

    if ((uint32_t)(now - cal->t0Us) >= TopCal::TIME_US)
        finishCalibration();
}

void setup()
{
    // Enquanto existe contato manual com o pendulo, o motor fica OFF.
    pinMode(FurutaConfig::ENABLE_PIN, OUTPUT);
    digitalWrite(FurutaConfig::ENABLE_PIN, HIGH);

    Serial.begin(FurutaConfig::SERIAL_BAUD);
    Wire.begin();
    Wire.setClock(400000UL);

    // Constrói SOMENTE o pequeno contexto de calibracao dentro da RAM
    // que futuramente recebera FurutaRevisionSystem.
    cal = new (systemMemory) CalibrationContext{};
    cal->collecting = false;

    const bool sensorOK = cal->sensor.begin();

    Serial.println(F("REV10-v17c-TOPCAL"));
    Serial.print(F("AS5600="));
    Serial.println(sensorOK ? 1 : 0);
    Serial.println(F("TOP: segure e envie C"));
}

void loop()
{
    if (furuta != nullptr)
    {
        furuta->update();
        return;
    }

    // Antes do controle, somente C/c e aceito.
    if (cal != nullptr && !cal->collecting)
    {
        while (Serial.available() > 0)
        {
            const char c = (char)Serial.read();
            if (c == 'C' || c == 'c')
            {
                startCalibration();
                break;
            }
        }
    }

    serviceCalibration();
}