#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>
#define RF95_FREQ 915.0
RH_RF95 rf95(19, 20);
const int board_SPI_SCK = 2;
const int board_SPI_TX = 3;
const int board_SPI_RX = 4;
// // LoRa
// const int RFM_CS = 19;  // RFM95 chip select pin
// const int RFM_RST = 18; // RFM95 reset pin
// const int RFM_IQR = 20; // RFM95 IQR pin
void setup()
{
  Serial.begin(9600);
  while (!Serial)
  {
    delay(10);
  }
  // manual reset
  pinMode(18, OUTPUT);
  digitalWrite(18, LOW);
  delay(10);
  digitalWrite(18, HIGH);
  delay(10);
  // SPI Initialization
  SPI.setRX(board_SPI_RX);
  SPI.setTX(board_SPI_TX);
  SPI.setSCK(board_SPI_SCK);
  SPI.begin();

  while (!rf95.init())
  {
    Serial.println("LoRa radio init failed");
    while (1)
      ;
  }
  Serial.println("LoRa radio init OK !");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ))
  {
    Serial.println("setFrequency failed");
    while (1)
      ;
  }

  // // LoRa Initialization
  // LoRa.setPins(RFM_CS, RFM_RST, RFM_IQR);
  // LoRa.setSPI(SPI);
  // if (!LoRa.begin(915E6))
  // {
  //   Serial.println("LoRa init falied !");
  //   return;
  // }
  // Serial.println("LoRa init");
}

void loop()
{
}