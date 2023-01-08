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

void receiveMessage(int packetSize)
{
  if (packetSize == 0)
    return;

  int recipient = LoRa.read();
  byte sender = LoRa.read();
  byte incomingLength = LoRa.read();

  String incoming = "";

  while (LoRa.available())
  {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length())
  {
    Serial.println("Error: Message length does not match length");
    return;
  }

  if (recipient != localAddress)
  {
    Serial.println("Error: Recipient address does not match local address");
    return;
  }

  Serial.print("Received data " + incoming);
  Serial.print(" from 0x" + String(sender, HEX));
  Serial.println(" to 0x" + String(recipient, HEX));
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
  if (millis() - lastSendTime > interval)
  {
    String sensorData = String(count++);
    sendMessage(sensorData);

    Serial.print("Sending data " + sensorData);
    Serial.print(" from source 0x" + String(localAddress, HEX));
    Serial.println(" to destination 0x" + String(destinationAddress, HEX));

    lastSendTime = millis();
    interval = random(2000) + 1000;
  }

  receiveMessage(LoRa.parsePacket());
}