#include <Wire.h>
#include "SparkFun_MS5637_Arduino_Library.h"
#include "pico/stdlib.h"
#include "Arduino.h"

// Soft Power Switch 
int POWER_BUTTON = 4;
int FAST_OFF = 5;
long powerPressedStartTime = 0;
int debounceDelay = 20;
// Status LED
int LED_RED = 4;
int LED_BLUE = 4;
int LED_GREEN = 4;
// MS5637 Altimeter
MS5637 barometricSensor;
// GPS

// LoRa 


void setup(void) {
  //Power Switch Input Setup
  pinMode(POWER_BUTTON, INPUT_PULLUP);
  powerPressedStartTime = millis();
  while (digitalRead(POWER_BUTTON) == LOW)
  {
    //Wait for user to stop pressing button before 
    delay(100);
    if (millis() - powerPressedStartTime > 500)
      break;
  }

  if (millis() - powerPressedStartTime < 500)
  {
    fastPowerDown();
  }

  // Display Red LED to indiacate system is on and not data logging
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, HIGH);

  powerPressedStartTime = 0; //Reset var to return to normal 'on' state

  //Turn on 

  //Serial Initialization
  Serial.begin(9600);
  Serial.println("Begin");
  
  //I2C Start
  Wire1.setSDA(6);
  Wire1.setSCL(7);
  Wire1.begin();
  barometricSensor.begin(Wire1);
  



}

void loop(void) {
  
  if (barometricSensor.isConnected())
  {
    Serial.print("Good");

    float temperature = barometricSensor.getTemperature();
    float pressure = barometricSensor.getPressure();

    Serial.print(" Temp=");
    Serial.print(temperature, 1);
    Serial.print("(C)");

    Serial.print(" Press=");
    Serial.print(pressure, 3);
    Serial.print("(hPa)");

    Serial.println();
  }
  else
  {
    Serial.println("Not connected");
  }
  delay(50);
}

void fastPowerDown()
{
  pinMode(POWER_BUTTON, OUTPUT);
  digitalWrite(POWER_BUTTON, LOW);
  pinMode(FAST_OFF, OUTPUT);
  digitalWrite(FAST_OFF, LOW);
  powerPressedStartTime = millis();
  while (1)
  {
    delay(1);
  }
}