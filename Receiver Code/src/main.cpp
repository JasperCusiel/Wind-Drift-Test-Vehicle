#include <Arduino.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include <SparkFun_MS5637_Arduino_Library.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
#include <SPI.h>
#include <LoRa.h>
#include <arduino-sht.h>
#include <Adafruit_TinyUSB.h>
#include <SDFS.h>

#define DEBUGGING
// RP2040 Board
const int board_SPI_SCK = 2;
const int board_SPI_TX = 3;
const int board_SPI_RX = 4;

const int board_SPI1_SCK = 14;
const int board_SPI1_TX = 12;
const int board_SPI1_RX = 15;

const int board_SDA = 6;
const int board_SCL = 7;

// Status LED
const int LED_GREEN = 26;
const int LED_RED = 27;
const int LED_BLUE = 8;
int ledState = LOW;
unsigned long previousMillis = 0;

// MS5637 Altimeter
MS5637 altimeter;
float startingPressure = 0.0;

// Soft Power Switch
const int powerBtnSense = 16;
long powerPressedStartTime = 0;
int debounceDelay = 20;

// MAX17048 Battery Fuel Gauge
SFE_MAX1704X lipo(MAX1704X_MAX17048);
double voltage = 0;
double soc = 0;
bool alert;

// SHT30 Temperature and Humidity Sensor
SHTSensor SHT30;

// LoRa
const int RFM_CS = 19;          // RFM95 chip select pin
const int RFM_RST = 18;         // RFM95 reset pin
const int RFM_IQR = 20;         // RFM95 IQR pin
byte localAddress = 0xAA;       // address of this device
byte destinationAddress = 0xBB; // destination to send to
byte msgCount = 0;              // count of out going messages
int count = 0;
long lastSendTime = 0; // last send time

// Ublox Neo M9N module
SFE_UBLOX_GNSS GNSS;

// SDFS Micro SD
const int chipSelect = 9;
SDFSConfig c1;
SdFs sd;

// USB Mass Storage object
Adafruit_USBD_MSC usb_msc;

// Set to true when PC write to flash
bool fs_changed;

void PowerDown()
{
  pinMode(powerBtnSense, OUTPUT);
  digitalWrite(powerBtnSense, LOW);
}

void setup()
{
  SPI1.setRX(board_SPI1_RX);
  SPI1.setTX(board_SPI1_TX);
  SPI1.setSCK(board_SPI1_SCK);
  SPI1.begin();
  c1.setCSPin(9);
  c1.setSPI(SPI1);
  SDFS.setConfig(c1);
  sd.begin();
  // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
  // usb_msc.setID("Adafruit", "SD Card", "1.0");
  // usb_msc.setUnitReady(false);
  // usb_msc.begin();

  Serial.begin(115200);
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Adafruit TinyUSB Mass Storage SD Card example");
  while (true)
  {
    int cardSize = int(sd.vol()->fatType());
    Serial.print("Volume is FAT:   ");
    Serial.println(cardSize);
    delay(1000);
  }

  // Set disk size, SD block size is always 512
  // usb_msc.setCapacity(cardSize, 512);

  // MSC is ready for read/write
  // usb_msc.setUnitReady(true);

  pinMode(powerBtnSense, INPUT_PULLUP);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_BLUE, HIGH);
  Serial.println("Begin");

  // I2C Initialization
  Wire1.setSDA(board_SDA);
  Wire1.setSCL(board_SCL);
  Wire1.begin();

  /*/ SPI Initialization
  SPI.setRX(board_SPI_RX);
  SPI.setTX(board_SPI_TX);
  SPI.setSCK(board_SPI_SCK);
  SPI.begin();*/

  // Altimeter Initialization
  altimeter.begin(Wire1);
  altimeter.setResolution(ms5637_resolution_osr_8192);

  // MAX17048 Battery Fuel Gauge start
  while (lipo.begin(Wire1) == false) // Connect to the MAX17043 using non-standard wire port
  {
    Serial.println(F("MAX17048 not detected."));
  }
  lipo.quickStart();
  lipo.setThreshold(20);

  // SHT30 Temperature and Humidity Sensor Initalization

  while (!SHT30.init(Wire1))
  {
    Serial.print("SHT30 error");
  }
  // LoRa Initialization
  LoRa.setPins(RFM_CS, RFM_RST, RFM_IQR);
  LoRa.setSPI(SPI);
  while (!LoRa.begin(915E6))
  {
    Serial.println("LoRa init failed, check connections.");
  }

  /*/ SD card initialization
  if (!SD.begin(9, SPI1))
  {
    Serial.println("SD failed to begin");
  }
  Serial.println("SD ok!");
  file = SD.open("test.txt", FILE_WRITE);
  // if the file opened okay, write to it:
  if (file)
  {
    Serial.print("Writing to test.txt...");
    file.println("testing 1, 2, 3.");
    // close the file:
    file.close();
    Serial.println("done.");
  }
  else
  {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

  // re-open the file for reading:
  file = SD.open("test.txt");
  if (file)
  {
    Serial.println("test.txt:");

    // read from the file until there's nothing else in it:
    while (file.available())
    {
      Serial.write(file.read());
    }
    // close the file:
    file.close();
  }
  else
  {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }*/
}
void loop()
{
  if (digitalRead(powerBtnSense) == LOW && powerPressedStartTime == 0)
  {
    digitalWrite(LED_BLUE, LOW);
    delay(debounceDelay);
    if (digitalRead(powerBtnSense) == LOW)
    {
      powerPressedStartTime = millis();
    }
  }
  else if (digitalRead(powerBtnSense) == LOW && powerPressedStartTime > 0)
  {
    digitalWrite(LED_BLUE, LOW);
    delay(debounceDelay);
    if (digitalRead(powerBtnSense) == LOW)
    {
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= 500)
      {
        // save the last time you blinked the LED
        previousMillis = currentMillis;
        // if the LED is off turn it on and vice-versa:
        if (ledState == LOW)
        {
          ledState = HIGH;
        }
        else
        {
          ledState = LOW;
        }
        digitalWrite(LED_BLUE, ledState);
      }
      if ((millis() - powerPressedStartTime) > 2000)
      {
        PowerDown();
      }
    }
  }
  else if (digitalRead(powerBtnSense) == HIGH && powerPressedStartTime > 0)
  {
    powerPressedStartTime = 0;
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_BLUE, HIGH);
  }
  /*
  float timeToCrunch = millis();
  float altitudeDelta = 0.0;
  float currentPressure = barometricSensor.getPressure();
  for (int x = 0; x < 16; x++)
    altitudeDelta += barometricSensor.altitudeChange(currentPressure, startingPressure);
  altitudeDelta /= (float)16;
  Serial.print("  time to sample " + String((millis() - timeToCrunch)) + "ms  ");
  Serial.print("Pressure=");
  Serial.print(currentPressure, 3);
  Serial.print("(hP or mbar)");

  Serial.print(" Change in Altitude=");
  Serial.print(altitudeDelta, 1);
  Serial.print("m");
  Serial.println();*/

  /*/ Print the variables:
  Serial.print("Voltage: ");
  Serial.print(lipo.getVoltage()); // Print the battery voltage
  Serial.print("V");

  Serial.print(" Percentage: ");
  Serial.print(lipo.getSOC(), 2); // Print the battery state of charge with 2 decimal places
  Serial.print("%");

  Serial.print(" Change Rate: ");
  Serial.print(lipo.getChangeRate(), 2); // Print the battery change rate with 2 decimal places
  Serial.print("%/hr");

  Serial.print(" Alert: ");
  Serial.print(lipo.getAlert()); // Print the generic alert flag

  Serial.print(" Voltage High Alert: ");
  Serial.print(lipo.isVoltageHigh()); // Print the alert flag

  Serial.print(" Voltage Low Alert: ");
  Serial.print(lipo.isVoltageLow()); // Print the alert flag

  Serial.print(" Empty Alert: ");
  Serial.print(lipo.isLow()); // Print the alert flag

  Serial.print(" SOC 1% Change Alert: ");
  Serial.print(lipo.isChange()); // Print the alert flag

  Serial.print(" Hibernating: ");
  Serial.print(lipo.isHibernating()); // Print the alert flag

  Serial.println();
  delay(1000);*/
}