#include <Arduino.h>
#include <LoRa.h>
#include <SPI.h>
void setup()
{
  Serial.begin(9600);
  while (!Serial)
  {
    delay(10);
  }

  SPI1.setSCK(10);
  SPI1.setCS(13);
  SPI1.setRX(12);
  SPI1.setTX(11);
  SPI1.begin();
  pinMode(13, OUTPUT);
  // LoRa Initialization
  LoRa.setPins(13, 9, 8);
  LoRa.setSPI(SPI1);
  LoRa.setGain(2);
  LoRa.dumpRegisters(Serial);
  if (!LoRa.begin(915E6))
  {
    Serial.println("LoRa init falied !");
    return;
  }
  Serial.print("test");
}

void loop()
{
}