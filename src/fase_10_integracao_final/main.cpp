#include <Arduino.h>
#include <FurutaSystem.h>

FurutaSystem furuta;

void setup()
{
    furuta.begin();
}

void loop()
{
    furuta.update();
}