#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
const int board_SPI_SCK = 2;
const int board_SPI_TX = 3;
const int board_SPI_RX = 4;
const int ssPin = 10;   // define the slave select pin
const int resetPin = 9; // define the reset pin
const int dio0Pin = 2;  // define the DIO0 pin

void setup()
{
  // initialize the LoRa module with the custom pins:
  LoRa.setPins(ssPin, resetPin, dio0Pin);
  LoRa.begin(915E6);

  // set the frequency and transmit power of the LoRa module:
  LoRa.setFrequency(915E6);
  LoRa.setTxPower(20);
}

void loop()
{
  // prepare the data to be sent:
  LoRa.beginPacket();
  LoRa.print("Hello, world!");
  LoRa.endPacket();

  // wait for a second before sending the next packet:
  delay(1000);
}