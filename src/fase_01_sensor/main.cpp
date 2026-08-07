#include <Arduino.h>
#include <ctype.h>

#include <PendulumPosition.h>
#include <PendulumVelocity.h>

// ============================================================
// Fase 01 - Posição e velocidade do pêndulo
// ============================================================

constexpr unsigned long SERIAL_BAUD_RATE = 115200;
constexpr unsigned long I2C_CLOCK_HZ = 100000UL;

constexpr uint32_t SAMPLE_PERIOD_US = 5000UL;
constexpr uint32_t PRINT_PERIOD_MS = 50UL;

// Pinos da placa reaproveitada
constexpr uint8_t STEP_PIN   = 6;
constexpr uint8_t DIR_PIN    = 8;
constexpr uint8_t ENABLE_PIN = 4;

// ============================================================
// Bibliotecas locais
// ============================================================

PendulumPosition positionSensor;

/*
 * Constante de tempo inicial do filtro:
 *
 * tau = 30 ms.
 */
PendulumVelocity velocityEstimator(0.030F);

// ============================================================
// Temporização
// ============================================================

uint32_t previousSampleTimeUs = 0;
uint32_t previousPrintTimeMs = 0;

// ============================================================
// Protótipos
// ============================================================

void processSerialCommands();
void defineLowerReference();
void printTelemetry();
void printHelp();
void printSensorStatus();

// ============================================================
// Setup
// ============================================================

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    // Mantém o A4988 desabilitado.
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH);

    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);

    digitalWrite(STEP_PIN, LOW);
    digitalWrite(DIR_PIN, LOW);

    Serial.begin(SERIAL_BAUD_RATE);
    delay(800);

    Serial.println();
    Serial.println(F("=========================================="));
    Serial.println(F("Pendulo de Furuta"));
    Serial.println(F("Fase 01 - Posicao e velocidade"));
    Serial.println(F("Motor desabilitado"));
    Serial.println(F("=========================================="));

    const bool sensorStarted =
        positionSensor.begin(
            I2C_CLOCK_HZ,
            AS5600_CLOCK_WISE
        );

    if (!sensorStarted) {
        Serial.println(
            F("ERRO: AS5600 nao respondeu em 0x36.")
        );

        while (true) {
            digitalWrite(
                LED_BUILTIN,
                !digitalRead(LED_BUILTIN)
            );

            delay(250);
        }
    }

    Serial.println(F("AS5600 conectado em 0x36."));

    printSensorStatus();

    /*
     * Inicializa o estimador usando a posição contínua
     * fornecida pela biblioteca de posição.
     */
    velocityEstimator.begin(
        positionSensor.continuousAngleRad(),
        micros()
    );

    previousSampleTimeUs = micros();
    previousPrintTimeMs = millis();

    printHelp();

    digitalWrite(LED_BUILTIN, LOW);
}

// ============================================================
// Loop principal
// ============================================================

void loop()
{
    processSerialCommands();

    const uint32_t currentTimeUs = micros();

    if (
        static_cast<uint32_t>(
            currentTimeUs - previousSampleTimeUs
        ) >= SAMPLE_PERIOD_US
    ) {
        previousSampleTimeUs = currentTimeUs;

        /*
         * Primeiro atualizamos a posição.
         *
         * Depois usamos exatamente essa posição para
         * atualizar a velocidade.
         */
        if (positionSensor.update()) {
            velocityEstimator.update(
                positionSensor.continuousAngleRad(),
                currentTimeUs
            );
        }
    }

    const uint32_t currentTimeMs = millis();

    if (
        static_cast<uint32_t>(
            currentTimeMs - previousPrintTimeMs
        ) >= PRINT_PERIOD_MS
    ) {
        previousPrintTimeMs = currentTimeMs;

        printTelemetry();
    }
}

// ============================================================
// Comandos seriais
// ============================================================

void processSerialCommands()
{
    while (Serial.available() > 0) {
        const char received =
            static_cast<char>(Serial.read());

        const char command =
            static_cast<char>(
                toupper(received)
            );

        switch (command) {
            case 'Z':
                defineLowerReference();
                break;

            case 'P':
                Serial.println();
                Serial.print(F("Referencia definida: "));
                Serial.println(
                    positionSensor.isReferenceDefined()
                        ? F("SIM")
                        : F("NAO")
                );

                Serial.print(F("RAW atual: "));
                Serial.println(
                    positionSensor.rawAngle()
                );

                Serial.print(F("Posicao acumulada: "));
                Serial.println(
                    positionSensor.cumulativeCounts()
                );

                Serial.println();
                break;

            case 'M':
                printSensorStatus();
                break;

            case 'H':
            case '?':
                printHelp();
                break;

            case '\r':
            case '\n':
            case ' ':
                break;

            default:
                Serial.println(
                    F("Comando desconhecido. Envie H.")
                );
                break;
        }
    }
}

// ============================================================
// Definição da posição inferior
// ============================================================

void defineLowerReference()
{
    /*
     * O pêndulo deve estar parado e apontando para baixo.
     */
    positionSensor.defineLowerReference();

    /*
     * Reinicia o estimador para eliminar qualquer
     * velocidade residual da calibração.
     */
    velocityEstimator.reset(
        positionSensor.continuousAngleRad(),
        micros()
    );

    Serial.println();
    Serial.println(F("=========================================="));
    Serial.println(F("REFERENCIA INFERIOR REGISTRADA"));
    Serial.println(F("Posicao inferior = 0 rad"));
    Serial.println(F("Posicao vertical = PI rad"));
    Serial.println(F("Erro na vertical = 0 rad"));
    Serial.println(F("=========================================="));
    Serial.println();
}

// ============================================================
// Diagnóstico do sensor e do ímã
// ============================================================

void printSensorStatus()
{
    Serial.println();
    Serial.println(F("Diagnostico do AS5600:"));

    if (!positionSensor.magnetDetected()) {
        Serial.println(F("Ima nao detectado."));
        return;
    }

    Serial.println(F("Ima detectado."));

    Serial.print(F("Magnitude: "));
    Serial.println(
        positionSensor.magneticMagnitude()
    );

    if (positionSensor.magnetTooWeak()) {
        Serial.println(F("Campo magnetico fraco."));
    } else if (positionSensor.magnetTooStrong()) {
        Serial.println(F("Campo magnetico forte."));
    } else {
        Serial.println(F("Campo magnetico adequado."));
    }

    Serial.println();
}

// ============================================================
// Telemetria
// ============================================================

void printTelemetry()
{
    Serial.print(F("calibrado:"));
    Serial.print(
        positionSensor.isReferenceDefined() ? 1 : 0
    );

    Serial.print(F(" raw:"));
    Serial.print(
        positionSensor.rawAngle()
    );

    Serial.print(F(" alpha_deg:"));
    Serial.print(
        positionSensor.angleFromLowerWrappedRad() *
        RAD_TO_DEG,
        2
    );

    Serial.print(F(" alpha_continua_deg:"));
    Serial.print(
        positionSensor.angleFromLowerUnwrappedRad() *
        RAD_TO_DEG,
        2
    );

    Serial.print(F(" erro_vertical_deg:"));
    Serial.print(
        positionSensor.equilibriumErrorRad() *
        RAD_TO_DEG,
        2
    );

    Serial.print(F(" erro_vertical_rad:"));
    Serial.print(
        positionSensor.equilibriumErrorRad(),
        4
    );

    Serial.print(F(" velocidade_raw_rad_s:"));
    Serial.print(
        velocityEstimator.rawVelocityRadS(),
        4
    );

    Serial.print(F(" velocidade_filtrada_rad_s:"));
    Serial.print(
        velocityEstimator.filteredVelocityRadS(),
        4
    );

    Serial.print(F(" dt_ms:"));
    Serial.println(
        velocityEstimator.sampleTimeSeconds() *
        1000.0F,
        3
    );
}

// ============================================================
// Ajuda
// ============================================================

void printHelp()
{
    Serial.println();
    Serial.println(F("Comandos:"));
    Serial.println(
        F("Z = registrar a posicao inferior")
    );
    Serial.println(
        F("P = mostrar posicao e referencia")
    );
    Serial.println(
        F("M = diagnosticar ima e campo magnetico")
    );
    Serial.println(
        F("H = mostrar ajuda")
    );
    Serial.println();
    Serial.println(
        F("Deixe o pendulo parado para baixo e envie Z.")
    );
    Serial.println();
}