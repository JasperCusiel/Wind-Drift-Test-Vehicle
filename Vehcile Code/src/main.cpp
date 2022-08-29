#include <Wire.h>
#include "SparkFun_MS5637_Arduino_Library.h"
#include "pico/stdlib.h"
#include "Arduino.h"
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>

// RP2040 Board
int SDA = 6;
int SCL = 7;

// MAX17048 Battery Fuel Gauge
SFE_MAX1704X lipo(MAX1704X_MAX17048); 
double voltage = 0;
double soc = 0;
bool alert;

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
int SPI_CLK = 2;
int MOSI = 3;
int MISO = 4;
int GPS_CS = 5;
int TX = 0;
int RX = 1;
int RST = 13;
int PWM = 16;
int INT = 17;
int AN = 26;

// LoRa 
int RADIO_FREQ_MHZ = 915;
int RFM_CS = 1;
int RFM_RST = 1;
int RFM_IQR = 1;

// Micro SD Card
int SDO = 12;
int SDI = 15;
int CLK = 14;
int DATA_1 = 11;
int DATA_2 = 10;
int DATA_3 = 9;


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
  Wire1.setSDA(SDA);
  Wire1.setSCL(SCL);
  Wire1.begin();
  barometricSensor.begin(Wire1);

  // MAX17048 Battery Fuel Gauge start
  if (lipo.begin() == false) // Connect to the MAX17043 using non-standard wire port
  {
    Serial.println(F("MAX17048 not detected. Please check wiring. Freezing."));
    while (1)
      ;
  }
  lipo.quickStart();
  lipo.setThreshold(20);
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