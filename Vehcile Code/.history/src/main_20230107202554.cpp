#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

const int board_SPI_SCK = 2;
const int board_SPI_TX = 3;
const int board_SPI_RX = 4;

const int csPin = 19;
const int resetPin = 18;
const int irqPin = 20;

byte localAddress = 0xBB;
byte destinationAddress = 0xAA;
long lastSendTime = 0;
int interval = 2000;
int count = 0;

void sendMessage(String outgoing)
{
  char message[] = "{100,200}";
  LoRa.beginPacket();
  LoRa.write(message, sizeof(message));
  LoRa.endPacket();
}

void setup()
{
  Serial.begin(9600);
  delay(3000);
  Serial.println("Start LoRa duplex");
  Serial.print("Local address: ");
  Serial.println(String(localAddress, HEX));
  Serial.print("Destination address: ");
  Serial.print(String(destinationAddress, HEX));
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
  char message[] = "{100,200}";

  Serial.print("Sending packet non-blocking: ");

  // send in async / non-blocking mode
  LoRa.beginPacket();
  LoRa.write(message, sizeof(message));
  LoRa.endPacket(true); // true = async / non-blocking mode
}