#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>
#include <PendulumPosition.h>
#include <PendulumVelocity.h>
#include <math.h>


constexpr uint32_t SAMPLE_PERIOD_US = 4000UL;   // 250 Hz

constexpr uint8_t STEP_PIN   = 6;
constexpr uint8_t ENABLE_PIN = 4;

constexpr float DOWN_ZONE_DEG      = 3.0F;
constexpr float PEAK_SPEED_RAD_S   = 0.20F;
constexpr float MIN_PEAK_ANGLE_DEG = 5.0F;

constexpr uint32_t DOWN_CALIBRATION_TIME_MS = 3000UL;
constexpr float MAX_CALIBRATION_SPAN_DEG = 20.0F;


AS5600 as5600;
PendulumPosition position(as5600, 1.0F);
PendulumVelocity velocity;


// ============================================================
// ESTADO GERAL
// ============================================================

enum class Mode
{
    WAITING,
    CALIBRATING_DOWN,
    RUNNING
};

Mode mode = Mode::WAITING;

uint32_t nextSampleUs = 0;
uint32_t startUs = 0;


// ============================================================
// REFERENCIA DOWN
// ============================================================

float downOffsetRad = 0.0F;


// ============================================================
// CALIBRACAO DE DOWN
// ============================================================

uint32_t calibrationStartUs = 0;

float sumSin = 0.0F;
float sumCos = 0.0F;

float calibrationMinDeg = 0.0F;
float calibrationMaxDeg = 0.0F;

uint32_t calibrationSamples = 0;


// ============================================================
// DETECTORES DE EVENTOS
// ============================================================

bool downArmedPositive = false;
bool downArmedNegative = false;

bool peakPositiveArmed = false;
bool peakNegativeArmed = false;

float maxAlphaDeg = 0.0F;
float minAlphaDeg = 0.0F;


// ============================================================
// CARACTERIZACAO
// ============================================================

uint32_t lastPeakPositiveUs = 0;

float periodMean = 0.0F;
uint32_t periodCount = 0;


// ------------------------------------------------------------
// NOVO:
// energia anterior separada para cada lado
// ------------------------------------------------------------

float previousPositiveEnergy = -1.0F;
float previousNegativeEnergy = -1.0F;


// ============================================================
// WRAP ANGULAR
// ============================================================

float wrapToPi(float angle)
{
    while (angle >= PI)
        angle -= TWO_PI;

    while (angle < -PI)
        angle += TWO_PI;

    return angle;
}


// ============================================================
// ALPHA CORRIGIDO
// ============================================================

float correctedAlpha()
{
    return wrapToPi(
        position.betaRadians() - downOffsetRad
    );
}


// ============================================================
// ENERGIA NORMALIZADA NO PICO
// ============================================================

float peakEnergy(float alphaDeg)
{
    const float alphaRad =
        alphaDeg * DEG_TO_RAD;

    return 0.5F * (1.0F - cosf(alphaRad));
}


// ============================================================
// RELATORIO DE PICO
// ============================================================

void reportPeak(
    bool positive,
    float peakDeg,
    uint32_t nowUs
)
{
    const float energy =
        peakEnergy(peakDeg);


    Serial.print(F("# PEAK,"));
    Serial.print(positive ? '+' : '-');

    Serial.print(F(",t="));
    Serial.print(
        (nowUs - startUs) / 1000UL
    );

    Serial.print(F(",angle="));
    Serial.print(peakDeg, 2);

    Serial.print(F(",energy="));
    Serial.print(energy, 6);


    // ========================================================
    // RATIO ENTRE PICOS DO MESMO LADO
    // ========================================================

    if (positive)
    {
        if (previousPositiveEnergy > 0.0F)
        {
            Serial.print(F(",ratio+="));
            Serial.print(
                energy / previousPositiveEnergy,
                5
            );
        }

        previousPositiveEnergy = energy;
    }
    else
    {
        if (previousNegativeEnergy > 0.0F)
        {
            Serial.print(F(",ratio-="));
            Serial.print(
                energy / previousNegativeEnergy,
                5
            );
        }

        previousNegativeEnergy = energy;
    }


    // ========================================================
    // PERIODO ENTRE PEAK+
    // ========================================================

    if (positive)
    {
        if (lastPeakPositiveUs != 0)
        {
            const float period =
                (nowUs - lastPeakPositiveUs)
                * 1.0e-6F;

            ++periodCount;

            periodMean +=
                (period - periodMean)
                /
                periodCount;

            const float omega0 =
                TWO_PI / periodMean;


            Serial.print(F(",T="));
            Serial.print(period, 4);

            Serial.print(F(",Tmean="));
            Serial.print(periodMean, 4);

            Serial.print(F(",omega0="));
            Serial.print(omega0, 4);
        }

        lastPeakPositiveUs = nowUs;
    }


    Serial.println();
}


// ============================================================
// DETECCAO DE EVENTOS
// ============================================================

void detectEvents(
    float alpha,
    float alphaDot,
    uint32_t nowUs
)
{
    const float alphaDeg =
        alpha * RAD_TO_DEG;


    // ========================================================
    // DOWN
    // ========================================================

    if (alphaDeg < -DOWN_ZONE_DEG)
        downArmedPositive = true;

    if (alphaDeg > DOWN_ZONE_DEG)
        downArmedNegative = true;


    if (
        downArmedPositive &&
        alphaDeg >= 0.0F &&
        alphaDot > 0.0F
    )
    {
        Serial.print(F("# DOWN+,"));
        Serial.println(
            (nowUs - startUs) / 1000UL
        );

        downArmedPositive = false;
    }


    if (
        downArmedNegative &&
        alphaDeg <= 0.0F &&
        alphaDot < 0.0F
    )
    {
        Serial.print(F("# DOWN-,"));
        Serial.println(
            (nowUs - startUs) / 1000UL
        );

        downArmedNegative = false;
    }


    // ========================================================
    // PEAK+
    // ========================================================

    if (
        alphaDeg > MIN_PEAK_ANGLE_DEG &&
        alphaDot > PEAK_SPEED_RAD_S
    )
    {
        if (!peakPositiveArmed)
        {
            peakPositiveArmed = true;
            maxAlphaDeg = alphaDeg;
        }

        if (alphaDeg > maxAlphaDeg)
            maxAlphaDeg = alphaDeg;
    }


    if (
        peakPositiveArmed &&
        alphaDot <= 0.0F
    )
    {
        reportPeak(
            true,
            maxAlphaDeg,
            nowUs
        );

        peakPositiveArmed = false;
    }


    // ========================================================
    // PEAK-
    // ========================================================

    if (
        alphaDeg < -MIN_PEAK_ANGLE_DEG &&
        alphaDot < -PEAK_SPEED_RAD_S
    )
    {
        if (!peakNegativeArmed)
        {
            peakNegativeArmed = true;
            minAlphaDeg = alphaDeg;
        }

        if (alphaDeg < minAlphaDeg)
            minAlphaDeg = alphaDeg;
    }


    if (
        peakNegativeArmed &&
        alphaDot >= 0.0F
    )
    {
        reportPeak(
            false,
            minAlphaDeg,
            nowUs
        );

        peakNegativeArmed = false;
    }
}


// ============================================================
// INICIA CALIBRACAO DE DOWN
// ============================================================

void startDownCalibration()
{
    Serial.println();

    Serial.println(
        F("Observe: pequena oscilacao em torno de DOWN e permitida.")
    );


    if (!position.calibrateTop(32, 2))
    {
        Serial.println(
            F("ERRO na leitura inicial do AS5600.")
        );
        return;
    }


    const uint32_t now =
        micros();


    velocity.reset(
        position.betaRadians(),
        now
    );


    sumSin = 0.0F;
    sumCos = 0.0F;

    calibrationSamples = 0;

    calibrationMinDeg = 1000.0F;
    calibrationMaxDeg = -1000.0F;

    calibrationStartUs = now;

    nextSampleUs =
        now + SAMPLE_PERIOD_US;


    mode =
        Mode::CALIBRATING_DOWN;


    Serial.println(
        F("Calibrando centro de DOWN por 3 s...")
    );
}


// ============================================================
// PROCESSA CALIBRACAO
// ============================================================

void processDownCalibration()
{
    if (!position.update())
        return;


    const uint32_t now =
        micros();


    const float alpha =
        position.betaRadians();

    const float alphaDeg =
        alpha * RAD_TO_DEG;


    sumSin += sinf(alpha);
    sumCos += cosf(alpha);

    ++calibrationSamples;


    if (alphaDeg < calibrationMinDeg)
        calibrationMinDeg = alphaDeg;

    if (alphaDeg > calibrationMaxDeg)
        calibrationMaxDeg = alphaDeg;


    const uint32_t elapsedMs =
        (now - calibrationStartUs) / 1000UL;


    if (
        elapsedMs < DOWN_CALIBRATION_TIME_MS
    )
        return;


    const float spanDeg =
        calibrationMaxDeg
        -
        calibrationMinDeg;


    if (
        spanDeg > MAX_CALIBRATION_SPAN_DEG
    )
    {
        Serial.print(
            F("CALIBRACAO REJEITADA. Span = ")
        );

        Serial.print(spanDeg, 2);

        Serial.println(F(" deg"));

        Serial.println(
            F("Deixe o pendulo oscilar menos e pressione T novamente.")
        );

        mode =
            Mode::WAITING;

        return;
    }


    // ========================================================
    // CENTRO MEDIO CIRCULAR
    // ========================================================

    downOffsetRad =
        atan2f(
            sumSin / calibrationSamples,
            sumCos / calibrationSamples
        );


    Serial.print(
        F("DOWN estimado = ")
    );

    Serial.print(
        downOffsetRad * RAD_TO_DEG,
        3
    );

    Serial.println(F(" deg"));


    Serial.print(
        F("Span observado = ")
    );

    Serial.print(spanDeg, 2);

    Serial.println(F(" deg"));


    // ========================================================
    // REINICIA VELOCIDADE NO NOVO ZERO
    // ========================================================

    const float alphaCorrected =
        correctedAlpha();


    velocity.reset(
        alphaCorrected,
        now
    );


    // ========================================================
    // REINICIA CARACTERIZACAO
    // ========================================================

    startUs = now;

    nextSampleUs =
        now + SAMPLE_PERIOD_US;


    downArmedPositive = false;
    downArmedNegative = false;

    peakPositiveArmed = false;
    peakNegativeArmed = false;


    lastPeakPositiveUs = 0;

    periodMean = 0.0F;
    periodCount = 0;


    // NOVO
    previousPositiveEnergy = -1.0F;
    previousNegativeEnergy = -1.0F;


    mode =
        Mode::RUNNING;


    Serial.println(
        F("DOWN definido pelo centro da oscilacao.")
    );

    Serial.println(
        F("Agora provoque a oscilacao de teste.")
    );
}


// ============================================================
// AQUISICAO NORMAL
// ============================================================

void acquireSample()
{
    if (!position.update())
        return;


    const uint32_t now =
        micros();


    const float alpha =
        correctedAlpha();


    velocity.update(
        alpha,
        now
    );


    if (!velocity.isReady())
        return;


    detectEvents(
        alpha,
        velocity.radiansPerSecond(),
        now
    );
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(250000);

    Wire.begin();
    Wire.setClock(400000UL);


    pinMode(STEP_PIN, OUTPUT);
    digitalWrite(STEP_PIN, LOW);

    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);


    if (!position.begin())
    {
        Serial.println(
            F("ERRO: AS5600 nao encontrado.")
        );

        while (true)
            delay(1000);
    }


    Serial.println();

    Serial.println(
        F("FASE 11C - CARACTERIZACAO DA OSCILACAO")
    );

    Serial.println(
        F("T = calibrar DOWN pela oscilacao media")
    );
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    if (Serial.available())
    {
        const char c =
            Serial.read();


        if (c == 'T' || c == 't')
        {
            startDownCalibration();
        }
    }


    if (mode == Mode::WAITING)
        return;


    const uint32_t now =
        micros();


    if (
        static_cast<int32_t>(
            now - nextSampleUs
        ) < 0
    )
        return;


    nextSampleUs +=
        SAMPLE_PERIOD_US;


    if (
        mode == Mode::CALIBRATING_DOWN
    )
    {
        processDownCalibration();
    }
    else if (
        mode == Mode::RUNNING
    )
    {
        acquireSample();
    }
}