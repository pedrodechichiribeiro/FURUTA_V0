#include <Arduino.h>
#include <MotorPosition.h>

// ============================================================
// Hardware
// ============================================================

const uint8_t STEP_PIN   = 6;
const uint8_t DIR_PIN    = 8;
const uint8_t ENABLE_PIN = 4;

const uint16_t PASSOS_POR_VOLTA = 200;
const uint8_t MICRO_PASSO = 8;

// ============================================================
// Limites
// ============================================================

// Limites físicos aproximados: -90° a +90°.
// Mantemos 10° de margem em cada lado.
const float ANGULO_MINIMO = -80.0F;
const float ANGULO_MAXIMO =  80.0F;

// Amplitude do ensaio automático.
const float ANGULO_TESTE = 60.0F;

// Valores iniciais conservadores.
float velocidadeAtual = 180.0F;   // graus/s
float aceleracaoAtual = 450.0F;   // graus/s²

// Limites máximos permitidos pelo programa de teste.
const float VELOCIDADE_LIMITE = 900.0F;
const float ACELERACAO_LIMITE = 3000.0F;

// ============================================================
// Motor
// ============================================================

MotorPosition motor(
    STEP_PIN,
    DIR_PIN,
    ENABLE_PIN,
    PASSOS_POR_VOLTA,
    MICRO_PASSO,
    ANGULO_MINIMO,
    ANGULO_MAXIMO
);

// ============================================================
// Estado
// ============================================================

String comando;

bool referenciaDefinida = false;
bool motorHabilitado = false;
bool testeAtivo = false;

uint16_t ciclosSolicitados = 0;
uint16_t ciclosConcluidos = 0;

enum class EtapaTeste
{
    PARADO,
    ZERO_INICIAL,
    POSITIVO,
    NEGATIVO,
    ZERO_FINAL
};

EtapaTeste etapaTeste = EtapaTeste::PARADO;

// ============================================================
// Teste automático
// ============================================================

void iniciarTeste(uint16_t ciclos)
{
    if (!referenciaDefinida)
    {
        Serial.println("Teste recusado: defina o zero com Z.");
        return;
    }

    if (!motorHabilitado)
    {
        Serial.println("Teste recusado: habilite o motor com E.");
        return;
    }

    if (motor.isMoving())
    {
        Serial.println("Teste recusado: motor ainda em movimento.");
        return;
    }

    if (ciclos == 0)
    {
        Serial.println("Quantidade de ciclos invalida.");
        return;
    }

    ciclosSolicitados = ciclos;
    ciclosConcluidos = 0;
    testeAtivo = true;

    etapaTeste = EtapaTeste::ZERO_INICIAL;
    motor.moveTo(0.0F);

    Serial.println();
    Serial.print("Teste iniciado: ");
    Serial.print(ciclosSolicitados);
    Serial.println(" ciclos");

    Serial.print("Velocidade: ");
    Serial.print(velocidadeAtual);
    Serial.println(" graus/s");

    Serial.print("Aceleracao: ");
    Serial.print(aceleracaoAtual);
    Serial.println(" graus/s2");
}

void atualizarTeste()
{
    if (!testeAtivo || motor.isMoving())
    {
        return;
    }

    switch (etapaTeste)
    {
        case EtapaTeste::ZERO_INICIAL:
            motor.moveTo(ANGULO_TESTE);
            etapaTeste = EtapaTeste::POSITIVO;
            break;

        case EtapaTeste::POSITIVO:
            motor.moveTo(-ANGULO_TESTE);
            etapaTeste = EtapaTeste::NEGATIVO;
            break;

        case EtapaTeste::NEGATIVO:
            motor.moveTo(0.0F);
            etapaTeste = EtapaTeste::ZERO_FINAL;
            break;

        case EtapaTeste::ZERO_FINAL:
            ciclosConcluidos++;

            Serial.print("Ciclo concluido: ");
            Serial.print(ciclosConcluidos);
            Serial.print("/");
            Serial.println(ciclosSolicitados);

            if (ciclosConcluidos >= ciclosSolicitados)
            {
                testeAtivo = false;
                etapaTeste = EtapaTeste::PARADO;

                Serial.println();
                Serial.println("Teste concluido.");
                Serial.println(
                    "Verifique fisicamente se o braco retornou ao zero."
                );
            }
            else
            {
                motor.moveTo(ANGULO_TESTE);
                etapaTeste = EtapaTeste::POSITIVO;
            }
            break;

        case EtapaTeste::PARADO:
            break;
    }
}

// ============================================================
// Status
// ============================================================

void mostrarStatus()
{
    Serial.println();
    Serial.println("--------- STATUS ---------");

    Serial.print("Referencia definida: ");
    Serial.println(referenciaDefinida ? "SIM" : "NAO");

    Serial.print("Motor habilitado: ");
    Serial.println(motorHabilitado ? "SIM" : "NAO");

    Serial.print("Posicao calculada: ");
    Serial.print(motor.currentPosition(), 2);
    Serial.println(" graus");

    Serial.print("Velocidade configurada: ");
    Serial.print(velocidadeAtual, 1);
    Serial.println(" graus/s");

    Serial.print("Aceleracao configurada: ");
    Serial.print(aceleracaoAtual, 1);
    Serial.println(" graus/s2");

    Serial.print("Teste automatico: ");
    Serial.println(testeAtivo ? "ATIVO" : "PARADO");

    Serial.println("--------------------------");
    Serial.println();
}

// ============================================================
// Comandos
// ============================================================

void executarComando(String texto)
{
    texto.trim();
    texto.toUpperCase();

    if (texto.length() == 0)
    {
        return;
    }

    if (texto == "E")
    {
        if (!referenciaDefinida)
        {
            Serial.println("Defina o centro com Z antes de habilitar.");
            return;
        }

        motor.enable();
        motorHabilitado = true;

        Serial.println("Motor habilitado.");
    }
    else if (texto == "D" || texto == "L")
    {
        testeAtivo = false;
        etapaTeste = EtapaTeste::PARADO;

        motor.disable();

        motorHabilitado = false;
        referenciaDefinida = false;

        Serial.println("Motor liberado.");
        Serial.println("A referencia foi perdida.");
    }
    else if (texto == "Z")
    {
        if (motor.isMoving())
        {
            Serial.println("Pare o motor antes de definir o zero.");
            return;
        }

        motor.setCurrentPosition(0.0F);
        referenciaDefinida = true;

        Serial.println("Posicao atual definida como zero.");
    }
    else if (texto == "STOP")
    {
        testeAtivo = false;
        etapaTeste = EtapaTeste::PARADO;

        motor.stop();

        Serial.println("Teste cancelado e parada iniciada.");
    }
    else if (texto == "S")
    {
        mostrarStatus();
    }
    else if (texto.startsWith("V "))
    {
        const float novaVelocidade =
            texto.substring(2).toFloat();

        if (
            novaVelocidade <= 0.0F ||
            novaVelocidade > VELOCIDADE_LIMITE
        )
        {
            Serial.println("Velocidade invalida.");
            Serial.println("Faixa permitida: 1 a 900 graus/s.");
            return;
        }

        velocidadeAtual = novaVelocidade;
        motor.setMaxSpeed(velocidadeAtual);

        Serial.print("Velocidade configurada: ");
        Serial.print(velocidadeAtual);
        Serial.println(" graus/s");
    }
    else if (texto.startsWith("A "))
    {
        const float novaAceleracao =
            texto.substring(2).toFloat();

        if (
            novaAceleracao <= 0.0F ||
            novaAceleracao > ACELERACAO_LIMITE
        )
        {
            Serial.println("Aceleracao invalida.");
            Serial.println("Faixa permitida: 1 a 3000 graus/s2.");
            return;
        }

        aceleracaoAtual = novaAceleracao;
        motor.setAcceleration(aceleracaoAtual);

        Serial.print("Aceleracao configurada: ");
        Serial.print(aceleracaoAtual);
        Serial.println(" graus/s2");
    }
    else if (texto.startsWith("T "))
    {
        const uint16_t ciclos =
            texto.substring(2).toInt();

        iniciarTeste(ciclos);
    }
    else if (texto.startsWith("P "))
    {
        if (testeAtivo)
        {
            Serial.println("Comando recusado: teste automatico ativo.");
            return;
        }

        if (!referenciaDefinida || !motorHabilitado)
        {
            Serial.println("Defina Z e habilite com E.");
            return;
        }

        const float angulo =
            texto.substring(2).toFloat();

        if (!motor.moveTo(angulo))
        {
            Serial.println("Posicao invalida ou fora dos limites.");
            return;
        }

        Serial.print("Movendo para ");
        Serial.print(angulo);
        Serial.println(" graus");
    }
    else
    {
        Serial.println("Comando desconhecido.");
    }
}

// ============================================================
// Serial
// ============================================================

void lerSerial()
{
    while (Serial.available())
    {
        const char caractere = Serial.read();

        if (caractere == '\n' || caractere == '\r')
        {
            if (comando.length() > 0)
            {
                executarComando(comando);
                comando = "";
            }
        }
        else if (comando.length() < 32)
        {
            comando += caractere;
        }
    }
}

// ============================================================
// Setup
// ============================================================

void setup()
{
    Serial.begin(115200);
    delay(500);

    comando.reserve(32);

    motor.begin(
        velocidadeAtual,
        aceleracaoAtual
    );

    Serial.println();
    Serial.println("Fase 04 - Limites reais do motor");
    Serial.println();
    Serial.println("Comandos:");
    Serial.println("L ou D    = liberar motor");
    Serial.println("Z         = definir centro como zero");
    Serial.println("E         = habilitar motor");
    Serial.println("P <graus> = movimento manual");
    Serial.println("V <valor> = velocidade em graus/s");
    Serial.println("A <valor> = aceleracao em graus/s2");
    Serial.println("T <ciclos> = teste automatico");
    Serial.println("STOP      = cancelar teste");
    Serial.println("S         = mostrar status");
}

// ============================================================
// Loop
// ============================================================

void loop()
{
    motor.update();
    atualizarTeste();
    lerSerial();
}