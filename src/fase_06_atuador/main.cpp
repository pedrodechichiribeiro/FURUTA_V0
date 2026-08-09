#include <Arduino.h>

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include <MotorPosition.h>

// ============================================================
// FASE 06 — CARACTERIZAÇÃO DO ATUADOR
//
// Objetivos:
//
// 1. Determinar velocidade máxima confiável.
// 2. Determinar aceleração máxima confiável.
// 3. Verificar perda de passos.
// 4. Verificar comportamento nas reversões.
// 5. Definir limites seguros para o futuro PD.
//
// Teste automático:
//
//      0° -> +60° -> -60° -> 0°
//
// ============================================================

// ============================================================
// HARDWARE
// ============================================================

constexpr uint8_t STEP_PIN = 6;
constexpr uint8_t DIR_PIN = 8;
constexpr uint8_t ENABLE_PIN = 4;

// ============================================================
// MOTOR
// ============================================================

constexpr uint16_t FULL_STEPS_PER_REVOLUTION =
    200;

// Configuração consolidada do projeto:
// microstepping 1/8.
constexpr uint8_t MICROSTEP_FACTOR =
    8;

// ============================================================
// LIMITES MECÂNICOS
// ============================================================

constexpr float MINIMUM_ANGLE_DEGREES =
    -80.0F;

constexpr float MAXIMUM_ANGLE_DEGREES =
    +80.0F;

// Amplitude do teste.
//
// Continua bem dentro dos limites +/-80 graus.
constexpr float TEST_ANGLE_DEGREES =
    60.0F;

// ============================================================
// CONFIGURAÇÃO INICIAL
// ============================================================

constexpr float DEFAULT_SPEED_DEG_S =
    180.0F;

constexpr float DEFAULT_ACCEL_DEG_S2 =
    700.0F;

// Limites administrativos desta fase.
//
// Não significam que o hardware consegue atingir estes valores.
// Apenas impedem comandos absurdos durante os testes.
constexpr float TEST_MAX_SPEED_DEG_S =
    720.0F;

constexpr float TEST_MAX_ACCEL_DEG_S2 =
    3000.0F;

// ============================================================
// SERIAL
// ============================================================

constexpr size_t SERIAL_BUFFER_SIZE =
    48;

char serialBuffer[SERIAL_BUFFER_SIZE];

size_t serialLength = 0;

bool serialOverflow = false;

// ============================================================
// MOTOR
// ============================================================

MotorPosition motor(
    STEP_PIN,
    DIR_PIN,
    ENABLE_PIN,
    FULL_STEPS_PER_REVOLUTION,
    MICROSTEP_FACTOR,
    MINIMUM_ANGLE_DEGREES,
    MAXIMUM_ANGLE_DEGREES);

// ============================================================
// REFERÊNCIA
// ============================================================

bool zeroDefined =
    false;

// ============================================================
// MÁQUINA DE ESTADOS DO TESTE
// ============================================================

enum class TestState : uint8_t
{
    IDLE,

    GO_POSITIVE,

    GO_NEGATIVE,

    RETURN_ZERO
};

TestState testState =
    TestState::IDLE;

bool testActive =
    false;

uint16_t requestedCycles =
    0;

uint16_t completedCycles =
    0;

uint32_t segmentStartMs =
    0;

// ============================================================
// UTILIDADES
// ============================================================

void printSeparator()
{
    Serial.println(
        F("----------------------------------------"));
}

// ============================================================
// AJUDA
// ============================================================

void printHelp()
{
    printSeparator();

    Serial.println(
        F("FASE 06 - CARACTERIZACAO DO ATUADOR"));

    Serial.println();

    Serial.println(
        F("Comandos:"));

    Serial.println(
        F("  Z          define posicao atual como 0 grau"));

    Serial.println(
        F("  E          habilita A4988"));

    Serial.println(
        F("  D          desabilita A4988"));

    Serial.println(
        F("  P <graus>  movimento manual"));

    Serial.println(
        F("  V <deg/s>  velocidade maxima"));

    Serial.println(
        F("  A <deg/s2> aceleracao"));

    Serial.println(
        F("  T <n>      executa n ciclos"));

    Serial.println(
        F("  STOP       parada controlada"));

    Serial.println(
        F("  X          emergencia"));

    Serial.println(
        F("  S          status"));

    Serial.println(
        F("  H          ajuda"));

    Serial.println();

    Serial.println(
        F("Teste automatico:"));

    Serial.println(
        F("0 -> +60 -> -60 -> 0"));

    printSeparator();
}

// ============================================================
// STATUS
// ============================================================

void printStatus()
{
    printSeparator();

    Serial.print(
        F("Driver: "));

    Serial.println(
        motor.isEnabled()
            ? F("HABILITADO")
            : F("DESABILITADO"));

    Serial.print(
        F("Zero: "));

    Serial.println(
        zeroDefined
            ? F("DEFINIDO")
            : F("NAO DEFINIDO"));

    Serial.print(
        F("Movimento: "));

    Serial.println(
        motor.isMoving()
            ? F("SIM")
            : F("NAO"));

    Serial.print(
        F("Posicao estimada [deg]: "));

    Serial.println(
        motor.currentPosition(),
        2);

    Serial.print(
        F("Destino [deg]: "));

    Serial.println(
        motor.targetPosition(),
        2);

    Serial.print(
        F("Velocidade configurada [deg/s]: "));

    Serial.println(
        motor.maxSpeedDegrees(),
        1);

    Serial.print(
        F("Aceleracao configurada [deg/s2]: "));

    Serial.println(
        motor.accelerationDegrees(),
        1);

    Serial.print(
        F("Teste automatico: "));

    Serial.println(
        testActive
            ? F("ATIVO")
            : F("PARADO"));

    if (testActive)
    {
        Serial.print(
            F("Ciclo: "));

        Serial.print(
            completedCycles + 1);

        Serial.print(
            '/');

        Serial.println(
            requestedCycles);
    }

    printSeparator();
}

// ============================================================
// PARSE FLOAT
// ============================================================

bool parseFloatArgument(
    const char *command,
    float &value)
{
    const char *argument =
        command + 1;

    while (
        *argument != '\0' &&
        isspace(
            static_cast<unsigned char>(
                *argument)))
    {
        ++argument;
    }

    if (*argument == '\0')
    {
        return false;
    }

    char *endPointer =
        nullptr;

    value =
        static_cast<float>(
            strtod(
                argument,
                &endPointer));

    if (
        endPointer ==
        argument)
    {
        return false;
    }

    while (
        *endPointer != '\0' &&
        isspace(
            static_cast<unsigned char>(
                *endPointer)))
    {
        ++endPointer;
    }

    return *endPointer == '\0';
}

// ============================================================
// PARSE INTEIRO
// ============================================================

bool parseIntegerArgument(
    const char *command,
    uint16_t &value)
{
    const char *argument =
        command + 1;

    while (
        *argument != '\0' &&
        isspace(
            static_cast<unsigned char>(
                *argument)))
    {
        ++argument;
    }

    if (*argument == '\0')
    {
        return false;
    }

    char *endPointer =
        nullptr;

    const long result =
        strtol(
            argument,
            &endPointer,
            10);

    if (
        endPointer == argument ||
        result <= 0 ||
        result > 200)
    {
        return false;
    }

    while (
        *endPointer != '\0' &&
        isspace(
            static_cast<unsigned char>(
                *endPointer)))
    {
        ++endPointer;
    }

    if (*endPointer != '\0')
    {
        return false;
    }

    value =
        static_cast<uint16_t>(
            result);

    return true;
}

// ============================================================
// INICIAR SEGMENTO
// ============================================================

bool startSegment(
    float targetDegrees,
    TestState newState)
{
    if (
        !motor.moveTo(
            targetDegrees))
    {
        Serial.println(
            F("ERRO ao iniciar segmento."));

        testActive =
            false;

        testState =
            TestState::IDLE;

        return false;
    }

    testState =
        newState;

    segmentStartMs =
        millis();

    return true;
}

// ============================================================
// RESULTADO DE UM SEGMENTO
// ============================================================

void printSegmentFinished()
{
    const uint32_t elapsedMs =
        millis() -
        segmentStartMs;

    Serial.print(
        F("SEGMENTO: alvo="));

    Serial.print(
        motor.currentPosition(),
        2);

    Serial.print(
        F(" deg, tempo="));

    Serial.print(
        elapsedMs);

    Serial.println(
        F(" ms"));
}

// ============================================================
// INICIAR TESTE
// ============================================================

void startAutomaticTest(
    uint16_t cycles)
{
    if (!zeroDefined)
    {
        Serial.println(
            F("ERRO: defina o zero com Z."));

        return;
    }

    if (!motor.isEnabled())
    {
        Serial.println(
            F("ERRO: habilite o motor com E."));

        return;
    }

    if (motor.isMoving())
    {
        Serial.println(
            F("ERRO: motor ainda esta em movimento."));

        return;
    }

    if (
        fabs(
            motor.currentPosition()) >
        1.0F)
    {
        Serial.println(
            F("ERRO: coloque o braco em 0 grau antes de T."));

        return;
    }

    requestedCycles =
        cycles;

    completedCycles =
        0;

    testActive =
        true;

    Serial.println();

    Serial.print(
        F("TESTE INICIADO: "));

    Serial.print(
        cycles);

    Serial.println(
        F(" ciclos"));

    Serial.print(
        F("V = "));

    Serial.print(
        motor.maxSpeedDegrees(),
        1);

    Serial.println(
        F(" deg/s"));

    Serial.print(
        F("A = "));

    Serial.print(
        motor.accelerationDegrees(),
        1);

    Serial.println(
        F(" deg/s2"));

    startSegment(
        TEST_ANGLE_DEGREES,
        TestState::GO_POSITIVE);
}

// ============================================================
// SERVIÇO DO TESTE AUTOMÁTICO
// ============================================================

void serviceAutomaticTest()
{
    if (!testActive)
    {
        return;
    }

    // Enquanto o motor estiver se movendo,
    // não há mudança de estado.
    if (motor.isMoving())
    {
        return;
    }

    printSegmentFinished();

    switch (testState)
    {

        // ----------------------------------------------------
        // +60 atingido
        // ----------------------------------------------------

    case TestState::GO_POSITIVE:

        startSegment(
            -TEST_ANGLE_DEGREES,
            TestState::GO_NEGATIVE);

        break;

        // ----------------------------------------------------
        // -60 atingido
        // ----------------------------------------------------

    case TestState::GO_NEGATIVE:

        startSegment(
            0.0F,
            TestState::RETURN_ZERO);

        break;

        // ----------------------------------------------------
        // 0 atingido
        // ----------------------------------------------------

    case TestState::RETURN_ZERO:

        ++completedCycles;

        Serial.print(
            F("CICLO "));

        Serial.print(
            completedCycles);

        Serial.print(
            '/');

        Serial.println(
            requestedCycles);

        if (
            completedCycles >=
            requestedCycles)
        {
            testActive =
                false;

            testState =
                TestState::IDLE;

            Serial.println();

            Serial.println(
                F("*** TESTE CONCLUIDO ***"));

            Serial.println(
                F("Confira FISICAMENTE a marca de zero."));

            return;
        }

        startSegment(
            TEST_ANGLE_DEGREES,
            TestState::GO_POSITIVE);

        break;

    default:

        testActive =
            false;

        testState =
            TestState::IDLE;

        break;
    }
}

// ============================================================
// PROCESSAMENTO DE COMANDO
// ============================================================

void processCommand(
    char *command)
{
    // Remove espaços finais.

    size_t length =
        strlen(command);

    while (
        length > 0 &&
        isspace(
            static_cast<unsigned char>(
                command[length - 1])))
    {
        command[--length] = '\0';
    }

    // Converte para maiúsculas.

    for (
        size_t i = 0;
        command[i] != '\0';
        ++i)
    {
        command[i] =
            static_cast<char>(
                toupper(
                    static_cast<unsigned char>(
                        command[i])));
    }

    // ========================================================
    // HELP
    // ========================================================

    if (
        strcmp(command, "H") == 0 ||
        strcmp(command, "?") == 0)
    {
        printHelp();

        return;
    }

    // ========================================================
    // STATUS
    // ========================================================

    if (
        strcmp(command, "S") == 0)
    {
        printStatus();

        return;
    }

    // ========================================================
    // ZERO
    // ========================================================

    if (
        strcmp(command, "Z") == 0)
    {
        if (motor.isMoving())
        {
            Serial.println(
                F("ERRO: motor em movimento."));

            return;
        }

        if (motor.isEnabled())
        {
            Serial.println(
                F("ERRO: desabilite o motor antes de Z."));

            return;
        }

        motor.setCurrentPosition(
            0.0F);

        zeroDefined =
            true;

        Serial.println(
            F("OK: posicao atual definida como 0 grau."));

        return;
    }

    // ========================================================
    // ENABLE
    // ========================================================

    if (
        strcmp(command, "E") == 0)
    {
        if (!zeroDefined)
        {
            Serial.println(
                F("ERRO: defina primeiro Z."));

            return;
        }

        motor.enable();

        Serial.println(
            F("Motor habilitado."));

        return;
    }

    // ========================================================
    // DISABLE
    // ========================================================

    if (
        strcmp(command, "D") == 0)
    {
        if (motor.isMoving())
        {
            Serial.println(
                F("ERRO: envie STOP e aguarde."));

            return;
        }

        motor.disable();

        // Após liberar o braço, ele pode ser movido
        // manualmente. A referência passa a ser suspeita.
        zeroDefined =
            false;

        Serial.println(
            F("Motor liberado."));

        Serial.println(
            F("Referencia perdida. Defina Z novamente."));

        return;
    }

    // ========================================================
    // STOP
    // ========================================================

    if (
        strcmp(command, "STOP") == 0)
    {
        testActive =
            false;

        testState =
            TestState::IDLE;

        motor.stop();

        Serial.println(
            F("Parada controlada solicitada."));

        return;
    }

    // ========================================================
    // EMERGÊNCIA
    // ========================================================

    if (
        strcmp(command, "X") == 0)
    {
        testActive =
            false;

        testState =
            TestState::IDLE;

        motor.emergencyStop();

        zeroDefined =
            false;

        Serial.println(
            F("EMERGENCIA: motor desabilitado."));

        Serial.println(
            F("Referencia perdida."));

        return;
    }

    // ========================================================
    // POSIÇÃO
    // ========================================================

    if (
        command[0] == 'P')
    {
        float target =
            0.0F;

        if (
            !parseFloatArgument(
                command,
                target))
        {
            Serial.println(
                F("Use: P <graus>"));

            return;
        }

        if (
            testActive ||
            motor.isMoving())
        {
            Serial.println(
                F("ERRO: aguarde o movimento atual."));

            return;
        }

        if (
            !motor.moveTo(
                target))
        {
            Serial.println(
                F("ERRO: destino invalido ou motor desabilitado."));

            return;
        }

        Serial.print(
            F("Destino = "));

        Serial.print(
            target,
            2);

        Serial.println(
            F(" deg"));

        return;
    }

    // ========================================================
    // VELOCIDADE
    // ========================================================

    if (
        command[0] == 'V')
    {
        float speed =
            0.0F;

        if (
            !parseFloatArgument(
                command,
                speed))
        {
            Serial.println(
                F("Use: V <graus/s>"));

            return;
        }

        if (
            motor.isMoving() ||
            testActive)
        {
            Serial.println(
                F("ERRO: altere V somente com motor parado."));

            return;
        }

        if (
            speed <= 0.0F ||
            speed >
                TEST_MAX_SPEED_DEG_S)
        {
            Serial.println(
                F("ERRO: velocidade fora da faixa da fase."));

            return;
        }

        motor.setMaxSpeedDegrees(
            speed);

        Serial.print(
            F("V = "));

        Serial.print(
            speed,
            1);

        Serial.println(
            F(" deg/s"));

        return;
    }

    // ========================================================
    // ACELERAÇÃO
    // ========================================================

    if (
        command[0] == 'A')
    {
        float acceleration =
            0.0F;

        if (
            !parseFloatArgument(
                command,
                acceleration))
        {
            Serial.println(
                F("Use: A <graus/s2>"));

            return;
        }

        if (
            motor.isMoving() ||
            testActive)
        {
            Serial.println(
                F("ERRO: altere A somente com motor parado."));

            return;
        }

        if (
            acceleration <= 0.0F ||
            acceleration >
                TEST_MAX_ACCEL_DEG_S2)
        {
            Serial.println(
                F("ERRO: aceleracao fora da faixa da fase."));

            return;
        }

        motor.setAccelerationDegrees(
            acceleration);

        Serial.print(
            F("A = "));

        Serial.print(
            acceleration,
            1);

        Serial.println(
            F(" deg/s2"));

        return;
    }

    // ========================================================
    // TESTE AUTOMÁTICO
    // ========================================================

    if (
        command[0] == 'T')
    {
        uint16_t cycles =
            0;

        if (
            !parseIntegerArgument(
                command,
                cycles))
        {
            Serial.println(
                F("Use: T <numero de ciclos>"));

            return;
        }

        startAutomaticTest(
            cycles);

        return;
    }

    Serial.println(
        F("Comando desconhecido. Use H."));
}

// ============================================================
// SERIAL
// ============================================================

void serviceSerial()
{
    while (
        Serial.available() > 0)
    {
        const char received =
            static_cast<char>(
                Serial.read());

        if (
            received == '\r')
        {
            continue;
        }

        if (
            received == '\n')
        {
            if (serialOverflow)
            {
                Serial.println(
                    F("ERRO: comando muito longo."));
            }
            else
            {
                serialBuffer[serialLength] = '\0';

                processCommand(
                    serialBuffer);
            }

            serialLength =
                0;

            serialOverflow =
                false;

            continue;
        }

        if (
            serialLength <
            SERIAL_BUFFER_SIZE - 1)
        {
            serialBuffer[serialLength++] =
                received;
        }
        else
        {
            serialOverflow =
                true;
        }
    }
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(
        115200);

    motor.beginDegrees(
        DEFAULT_SPEED_DEG_S,
        DEFAULT_ACCEL_DEG_S2);

    delay(200);

    Serial.println();

    Serial.println(
        F("FASE 06 - CARACTERIZACAO DO ATUADOR"));

    Serial.println(
        F("Motor iniciado DESABILITADO."));

    printHelp();
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    // AccelStepper precisa ser atendida continuamente.
    motor.update();

    serviceAutomaticTest();

    serviceSerial();

    // Segunda chamada reduz o intervalo entre run().
    motor.update();
}