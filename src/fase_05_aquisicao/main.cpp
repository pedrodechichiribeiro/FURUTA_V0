#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>

#include <PendulumPosition.h>
#include <PendulumVelocity.h>


// ============================================================
// CONFIGURAÇÃO
// ============================================================

// 4 ms -> 250 Hz
constexpr uint32_t SAMPLE_PERIOD_US =
    4000UL;


// Sentido positivo adotado para beta.
//
// Se futuramente verificarmos que a convenção precisa
// ser invertida, basta trocar para -1.0F.
constexpr float SENSOR_DIRECTION_SIGN =
    1.0F;


// Imprime uma linha a cada duas aquisições.
//
// Aquisição:   250 Hz
// Telemetria:  125 Hz
constexpr uint8_t TELEMETRY_DECIMATION =
    2;


// ============================================================
// OBJETOS
// ============================================================

AS5600 as5600;


PendulumPosition pendulumPosition(
    as5600,
    SENSOR_DIRECTION_SIGN
);


PendulumVelocity pendulumVelocity;


// ============================================================
// ESTADO
// ============================================================

uint32_t nextSampleUs = 0;

uint32_t sampleCounter = 0;
uint32_t missedSamples = 0;
uint32_t readErrors = 0;

bool acquisitionEnabled = false;
bool telemetryEnabled = false;


// ============================================================
// AJUDA
// ============================================================

void printHelp()
{
    Serial.println();

    Serial.println(
        F("FASE 05 - AQUISICAO")
    );

    Serial.println(
        F("--------------------------------")
    );

    Serial.println(
        F("T = definir referencia beta = 0")
    );

    Serial.println(
        F("L = ligar/desligar telemetria")
    );

    Serial.println(
        F("S = status")
    );

    Serial.println(
        F("H = ajuda")
    );

    Serial.println();
}


// ============================================================
// STATUS
// ============================================================

void printStatus()
{
    Serial.println();

    Serial.println(
        F("--------------------------------")
    );


    Serial.print(
        F("AS5600 conectado: ")
    );

    Serial.println(
        as5600.isConnected()
            ? F("SIM")
            : F("NAO")
    );


    Serial.print(
        F("Referencia definida: ")
    );

    Serial.println(
        pendulumPosition.topIsDefined()
            ? F("SIM")
            : F("NAO")
    );


    Serial.print(
        F("Estimador velocidade pronto: ")
    );

    Serial.println(
        pendulumVelocity.isReady()
            ? F("SIM")
            : F("NAO")
    );


    if (pendulumPosition.topIsDefined())
    {
        Serial.print(
            F("beta [rad]: ")
        );

        Serial.println(
            pendulumPosition.betaRadians(),
            6
        );


        Serial.print(
            F("beta_dot [rad/s]: ")
        );

        Serial.println(
            pendulumVelocity.radiansPerSecond(),
            5
        );


        Serial.print(
            F("Ts real [us]: ")
        );

        Serial.println(
            pendulumVelocity.samplePeriodSeconds()
            * 1.0e6F,
            0
        );
    }


    Serial.print(
        F("Amostras: ")
    );

    Serial.println(
        sampleCounter
    );


    Serial.print(
        F("Amostras perdidas: ")
    );

    Serial.println(
        missedSamples
    );


    Serial.print(
        F("Erros I2C: ")
    );

    Serial.println(
        readErrors
    );


    Serial.println(
        F("--------------------------------")
    );
}


// ============================================================
// DEFINIÇÃO DA REFERÊNCIA
// ============================================================

void defineReference()
{
    Serial.println();

    Serial.println(
        F("Mantenha o pendulo parado...")
    );


    if (!pendulumPosition.calibrateTop(32, 2))
    {
        Serial.println(
            F("ERRO: referencia nao definida.")
        );

        acquisitionEnabled =
            false;

        return;
    }


    const uint32_t now =
        micros();


    // Toda redefinição de beta = 0 também reinicia
    // completamente o estimador de velocidade.
    pendulumVelocity.reset(
        pendulumPosition.betaRadians(),
        now
    );


    sampleCounter = 0;
    missedSamples = 0;
    readErrors = 0;


    nextSampleUs =
        now +
        SAMPLE_PERIOD_US;


    acquisitionEnabled =
        true;


    Serial.print(
        F("OK. beta = 0 em RAW = ")
    );

    Serial.println(
        pendulumPosition.topRaw()
    );
}


// ============================================================
// TELEMETRIA
// ============================================================

void printTelemetry(
    uint32_t sampleTimeUs
)
{
    // CSV:
    //
    // 1  t_us
    // 2  raw
    // 3  beta
    // 4  betaDot
    // 5  dt_us


    Serial.print(
        sampleTimeUs
    );


    Serial.print(',');

    Serial.print(
        pendulumPosition.raw()
    );


    Serial.print(',');

    Serial.print(
        pendulumPosition.betaRadians(),
        6
    );


    Serial.print(',');

    Serial.print(
        pendulumVelocity.radiansPerSecond(),
        5
    );


    Serial.print(',');

    Serial.println(
        pendulumVelocity.samplePeriodSeconds()
        * 1.0e6F,
        0
    );
}


// ============================================================
// AQUISIÇÃO
// ============================================================

void acquireSample()
{
    // --------------------------------------------------------
    // 1. Lê posição
    // --------------------------------------------------------

    if (!pendulumPosition.update())
    {
        ++readErrors;

        return;
    }


    const uint32_t sampleTimeUs =
        micros();


    // --------------------------------------------------------
    // 2. Atualiza velocidade
    // --------------------------------------------------------

    pendulumVelocity.update(
        pendulumPosition.betaRadians(),
        sampleTimeUs
    );


    // --------------------------------------------------------
    // 3. Conta amostra
    // --------------------------------------------------------

    ++sampleCounter;


    // --------------------------------------------------------
    // 4. Telemetria
    // --------------------------------------------------------

    if (
        telemetryEnabled &&
        pendulumVelocity.isReady() &&
        (
            sampleCounter %
            TELEMETRY_DECIMATION
            == 0
        )
    )
    {
        printTelemetry(
            sampleTimeUs
        );
    }
}


// ============================================================
// SERIAL
// ============================================================

void processSerial()
{
    if (!Serial.available())
    {
        return;
    }


    char command =
        Serial.read();


    if (
        command == '\n' ||
        command == '\r'
    )
    {
        return;
    }


    if (
        command >= 'a' &&
        command <= 'z'
    )
    {
        command -= 32;
    }


    switch (command)
    {
        case 'T':

            defineReference();

            break;


        case 'L':

            telemetryEnabled =
                !telemetryEnabled;


            if (telemetryEnabled)
            {
                Serial.println(
                    F(
                        "t_us,raw,beta,"
                        "betaDot,dt_us"
                    )
                );
            }
            else
            {
                Serial.println(
                    F("Telemetria desligada.")
                );
            }

            break;


        case 'S':

            printStatus();

            break;


        case 'H':

            printHelp();

            break;


        default:

            Serial.println(
                F("Comando desconhecido. Use H.")
            );

            break;
    }
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);


    Wire.begin();

    Wire.setClock(
        400000UL
    );


    delay(100);


    Serial.println();

    Serial.println(
        F("FASE 05 - AQUISICAO")
    );


    if (!pendulumPosition.begin())
    {
        Serial.println(
            F("ERRO: AS5600 nao encontrado.")
        );


        while (true)
        {
            delay(1000);
        }
    }


    Serial.println(
        F("AS5600 conectado.")
    );


    printHelp();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    processSerial();


    if (!acquisitionEnabled)
    {
        return;
    }


    const uint32_t now =
        micros();


    const int32_t lateness =
        static_cast<int32_t>(
            now -
            nextSampleUs
        );


    if (lateness < 0)
    {
        return;
    }


    // --------------------------------------------------------
    // Verifica se algum período foi perdido
    // --------------------------------------------------------

    if (
        static_cast<uint32_t>(lateness)
        >= SAMPLE_PERIOD_US
    )
    {
        missedSamples +=
            static_cast<uint32_t>(lateness)
            /
            SAMPLE_PERIOD_US;


        nextSampleUs =
            now +
            SAMPLE_PERIOD_US;
    }
    else
    {
        // Mantém a grade temporal de 4 ms.
        nextSampleUs +=
            SAMPLE_PERIOD_US;
    }


    acquireSample();
}