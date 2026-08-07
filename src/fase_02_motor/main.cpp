#include <Arduino.h>
#include <MotorPosition.h>

const uint8_t STEP_PIN   = 6;
const uint8_t DIR_PIN    = 8;
const uint8_t ENABLE_PIN = 4;

const uint16_t PASSOS_POR_VOLTA = 200;
const uint8_t MICRO_PASSO = 8;

// Posição definida em torno do centro:
const float ANGULO_MINIMO = -90.0F;
const float ANGULO_MAXIMO =  90.0F;

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

void executarComando(String texto)
{
    texto.trim();
    texto.toUpperCase();

    if (texto == "E")
    {
        motor.enable();
        Serial.println("Motor habilitado");
    }
    else if (texto == "D" || texto == "L")
    {
        motor.disable();

        Serial.println("Motor liberado");
        Serial.println("Posicione manualmente e depois envie Z");
    }
    else if (texto == "Z")
    {
        motor.setCurrentPosition(0.0F);
        Serial.println("Posicao atual definida como 0 grau");
    }
    else if (texto == "STOP")
    {
        motor.stop();
        Serial.println("Parando motor");
    }
    else if (texto == "S")
    {
        Serial.print("Posicao atual: ");
        Serial.print(motor.currentPosition(), 2);
        Serial.println(" graus");
    }
    else if (texto.startsWith("P "))
    {
        const float angulo = texto.substring(2).toFloat();

        if (!motor.moveTo(angulo))
        {
            Serial.println(
                "Posicao invalida, fora de -90 a +90, "
                "ou motor desabilitado"
            );
        }
        else
        {
            Serial.print("Movendo para: ");
            Serial.print(angulo, 2);
            Serial.println(" graus");
        }
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
        else
        {
            comando += caractere;
        }
    }
}

void setup()
{
    Serial.begin(115200);

    motor.begin(
        VELOCIDADE_MAXIMA,
        ACELERACAO
    );

    Serial.println();
    Serial.println("Comandos:");
    Serial.println("L ou D   = liberar motor");
    Serial.println("Z        = definir posicao atual como zero");
    Serial.println("E        = habilitar motor");
    Serial.println("P <graus> = mover entre -90 e +90");
    Serial.println("STOP     = parar");
    Serial.println("S        = mostrar posicao");
}

void loop()
{
    motor.update();
    lerSerial();
}