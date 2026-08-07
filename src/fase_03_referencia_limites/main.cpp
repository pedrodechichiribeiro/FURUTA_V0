#include <Arduino.h>
#include <MotorPosition.h>

const uint8_t STEP_PIN   = 6;
const uint8_t DIR_PIN    = 8;
const uint8_t ENABLE_PIN = 4;

const uint16_t PASSOS_POR_VOLTA = 200;
const uint8_t MICRO_PASSO = 8;

// Limites mecânicos reais.
const float LIMITE_FISICO_MINIMO = -90.0F;
const float LIMITE_FISICO_MAXIMO =  90.0F;

// Limites operacionais, deixando 10° de segurança.
const float ANGULO_MINIMO = -80.0F;
const float ANGULO_MAXIMO =  80.0F;

const float VELOCIDADE_MAXIMA = 360.0F;  // graus/s
const float ACELERACAO = 900.0F;         // graus/s²

MotorPosition motor(
    STEP_PIN,
    DIR_PIN,
    ENABLE_PIN,
    PASSOS_POR_VOLTA,
    MICRO_PASSO,
    ANGULO_MINIMO,
    ANGULO_MAXIMO
);

String comando;

// Somente fica verdadeira depois do comando Z.
bool referenciaDefinida = false;

void mostrarStatus()
{
    const float posicao = motor.currentPosition();

    Serial.println();
    Serial.println("----- STATUS -----");

    Serial.print("Referencia definida: ");
    Serial.println(referenciaDefinida ? "SIM" : "NAO");

    Serial.print("Motor habilitado: ");
    Serial.println(motor.isEnabled() ? "SIM" : "NAO");

    Serial.print("Posicao atual: ");
    Serial.print(posicao, 2);
    Serial.println(" graus");

    if (referenciaDefinida)
    {
        Serial.print("Distancia ate limite negativo: ");
        Serial.print(posicao - ANGULO_MINIMO, 2);
        Serial.println(" graus");

        Serial.print("Distancia ate limite positivo: ");
        Serial.print(ANGULO_MAXIMO - posicao, 2);
        Serial.println(" graus");
    }

    Serial.print("Limites operacionais: ");
    Serial.print(ANGULO_MINIMO, 1);
    Serial.print(" a ");
    Serial.print(ANGULO_MAXIMO, 1);
    Serial.println(" graus");

    Serial.print("Limites fisicos: ");
    Serial.print(LIMITE_FISICO_MINIMO, 1);
    Serial.print(" a ");
    Serial.print(LIMITE_FISICO_MAXIMO, 1);
    Serial.println(" graus");

    Serial.println("------------------");
    Serial.println();
}

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
            Serial.println("Motor nao habilitado.");
            Serial.println("Posicione o braco no centro e envie Z.");
            return;
        }

        motor.enable();
        Serial.println("Motor habilitado");
    }
    else if (texto == "D" || texto == "L")
    {
        motor.disable();

        // O braço poderá ser movimentado manualmente.
        // A posição armazenada deixa de ser confiável.
        referenciaDefinida = false;

        Serial.println("Motor liberado");
        Serial.println("A referencia foi perdida.");
        Serial.println("Posicione no centro e envie Z.");
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

        Serial.println("Centro registrado como 0 grau.");
        Serial.println("Agora envie E para habilitar o motor.");
    }
    else if (texto == "STOP")
    {
        motor.stop();
        Serial.println("Parada com desaceleracao iniciada");
    }
    else if (texto == "S")
    {
        mostrarStatus();
    }
    else if (texto.startsWith("P "))
    {
        if (!referenciaDefinida)
        {
            Serial.println("Movimento recusado: referencia desconhecida.");
            Serial.println("Posicione o braco no centro e envie Z.");
            return;
        }

        if (!motor.isEnabled())
        {
            Serial.println("Movimento recusado: motor desabilitado.");
            Serial.println("Envie E para habilitar.");
            return;
        }

        const float angulo = texto.substring(2).toFloat();

        if (angulo < ANGULO_MINIMO || angulo > ANGULO_MAXIMO)
        {
            Serial.print("Posicao fora dos limites operacionais: ");
            Serial.print(ANGULO_MINIMO, 1);
            Serial.print(" a ");
            Serial.print(ANGULO_MAXIMO, 1);
            Serial.println(" graus");
            return;
        }

        if (!motor.moveTo(angulo))
        {
            Serial.println("Comando de movimento recusado.");
            return;
        }

        Serial.print("Movendo para: ");
        Serial.print(angulo, 2);
        Serial.println(" graus");
    }
    else
    {
        Serial.println("Comando desconhecido");
    }
}

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

void setup()
{
    Serial.begin(115200);
    delay(500);

    comando.reserve(32);

    motor.begin(
        VELOCIDADE_MAXIMA,
        ACELERACAO
    );

    referenciaDefinida = false;

    Serial.println();
    Serial.println("Fase 03 - Referencia e limites do braco");
    Serial.println("Limites operacionais: -80 a +80 graus");
    Serial.println();
    Serial.println("Comandos:");
    Serial.println("L ou D    = liberar motor e perder referencia");
    Serial.println("Z         = definir posicao atual como zero");
    Serial.println("E         = habilitar motor");
    Serial.println("P <graus> = mover entre -80 e +80");
    Serial.println("STOP      = parar");
    Serial.println("S         = mostrar status");
    Serial.println();
    Serial.println("Procedimento inicial: L, posicionar no centro, Z, E");
}

void loop()
{
    motor.update();
    lerSerial();
}