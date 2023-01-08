#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

const int board_SPI_SCK = 2;
const int board_SPI_TX = 3;
const int board_SPI_RX = 4;

const int csPin = 19;
const int resetPin = 18;
const int irqPin = 20;

void setup()
{
  Serial.begin(9600);
  delay(3000);
  SPI.setTX(board_SPI_TX);
  SPI.setRX(board_SPI_RX);
  SPI.setSCK(board_SPI_SCK);
  SPI.begin();
  LoRa.setSPI(SPI);
  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(915E6))
  {
    Serial.println("LoRa init failed. Check your connections.");
    while (true)
    {
    }
  }
}

void loop()
{
  // wait until the radio is ready to send a packet
  while (LoRa.beginPacket() == 0)
  {
    Serial.print("waiting for radio ... ");
    delay(100);
  }

  Serial.print("Sending packet non-blocking: ");

  LoRa.beginPacket();
  LoRa.print("{");
  LoRa.print("100");
  LoRa.endPacket(true);
}