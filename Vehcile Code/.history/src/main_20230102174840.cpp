#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
const int board_SPI_SCK = 2;
const int board_SPI_TX = 3;
const int board_SPI_RX = 4;
int counter = 0;
// LoRa
const int RFM_CS = 19;  // RFM95 chip select pin
const int RFM_RST = 18; // RFM95 reset pin
const int RFM_IQR = 20; // RFM95 IQR pin
void setup()
{
  // initialize the LoRa module with the custom pins:
  // SPI.setTX(3);
  // SPI.setRX(4);
  // SPI.setSCK(2);
  SPI.begin();
  LoRa.setSPI(SPI);
  LoRa.setPins(RFM_CS, RFM_RST, RFM_IQR);
  if (!LoRa.begin(915E6))
  {
    delay(2000);
    Serial.println("LoRa init failed, check connections.");
    return;
  }
  LoRa.dumpRegisters(Serial);
  // set the frequency and transmit power of the LoRa module:
  // LoRa.setFrequency(915E6);
  // LoRa.setTxPower(20);
}

void loop()
{
   Serial.print("Sending packet: ");
  Serial.println(counter);

  // send packet
  LoRa.beginPacket();
  LoRa.print("hello ");
  LoRa.print(counter);
  LoRa.endPacket();

  counter++;

  delay(5000);
}