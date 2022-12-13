#include <Arduino.h>
#include <LoRa.h>
#include <SPI.h>
const int board_SPI_SCK = 2;
const int board_SPI_TX = 3;
const int board_SPI_RX = 4;
// LoRa
const int RFM_CS = 19;          // RFM95 chip select pin
const int RFM_RST = 18;         // RFM95 reset pin
const int RFM_IQR = 20;         // RFM95 IQR pin
void setup()
{
  Serial.begin(9600); 
  while (!Serial)
  {
    delay(10);
  }
  // SPI Initialization
  SPI.setRX(board_SPI_RX);
  SPI.setTX(board_SPI_TX);
  SPI.setSCK(board_SPI_SCK);
  SPI.begin();

  // LoRa Initialization
  LoRa.setPins(RFM_CS, RFM_RST, RFM_IQR);
  LoRa.setSPI(SPI);
  while (!LoRa.begin(915E6))
  {
    Serial.println("LoRa init falied !");
    return;
  }
  Serial.println("LoRa init");
}

void loop()
{
}