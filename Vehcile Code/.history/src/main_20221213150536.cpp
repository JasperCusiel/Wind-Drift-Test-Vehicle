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