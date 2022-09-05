#include <Arduino.h>
#include <LoRa.h>
#include <SPI.h>

//Uncomment to enable debugging
#define DEBUGGING

int counter = 0;

void setup() {
  #ifdef DEBUGGING
    Serial.begin(9600);
    while (!Serial);
    Serial.println("LoRa Sender non-blocking Callback");
  #endif
  if (!LoRa.begin(915E6)) {
    #ifdef DEBUGGING
      Serial.println("Starting LoRa failed!");
    #endif
    while (1);
  }

  LoRa.onTxDone(onTxDone);
}

void loop() {
  if (runEvery(5000)) { // repeat every 5000 millis
    #ifdef DEBUGGING
      Serial.print("Sending packet non-blocking: ");
      Serial.println(counter);
    #endif
    
    // send in async / non-blocking mode
    LoRa.beginPacket();
    LoRa.print("hello ");
    LoRa.print(counter);
    LoRa.endPacket(true); // true = async / non-blocking mode

    counter++;
  }
}

void onTxDone() {
  #ifdef DEBUGGING
    Serial.println("TxDone");
  #endif
}

boolean runEvery(unsigned long interval)
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}