#include <Arduino.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include <SparkFun_MS5637_Arduino_Library.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <SPI.h>
#include <arduino-sht.h>
#include <Adafruit_TinyUSB.h>
#include <LoRa.h>
#include <OneButton.h>
#include <SdFat.h>
#include <SdFatConfig.h>

// Status LED
const int LED_GREEN = 26;
const int LED_RED = 28;
const int LED_BLUE = 27;
// Global variables to store the current state and time of the LED
const int ON_TIME = 250;   // LED on time in milliseconds
const int OFF_TIME = 2500; // LED off time in milliseconds
// Time (in milliseconds) to hold the button to initiate power down
const int POWER_DOWN_TIME = 6000;

// Timestamp of the last time the LED state was updated
unsigned long lastUpdateTime = 0;

// Timestamp of the start of the long press
unsigned long longPressStartTime = 0;

unsigned long previousMillis = 0; // Stores the last time the LED was updated
int ledState = LOW;               // Stores the current state of the LED
volatile bool powerButtonPressed = false;

// Buttons
const int powerButtonPin = 21;
const int bootSelectButtonPin = 22;
OneButton powerButton(powerButtonPin);
OneButton bootSelectButton(bootSelectButtonPin);

// MS5637 Altimeter
MS5637 altimeter;
const float SEA_LEVEL_PRESSURE = 101325; // Sea level pressure in pascals

// Soft Power Switch
const int powerBtnSense = 16;
long powerPressedStartTime = 0;
int debounceDelay = 20;

// MAX17048 Battery Fuel Gauge
SFE_MAX1704X lipo(MAX1704X_MAX17048);
// SHT30 Temperature and Humidity Sensor
SHTSensor SHT30;
float temp, humidity;

// LoRa Setup
const int csPin = 19;
const int resetPin = 18;
const int irqPin = 20;
const int MAX_MESSAGE_LENGTH = 100;
char message[MAX_MESSAGE_LENGTH];
int logCount = 0;               // counter to keep track of number of logs made
const int LOGS_BEFORE_SEND = 5; // number of logs to make before sending lora data ie send lora data every five seconds

// Ublox Neo M9N module
SFE_UBLOX_GNSS GNSS;
const int ppsPin = 17;

// USB Mass Storage object
Adafruit_USBD_MSC usb_msc;
// File system on SD Card
#define SPI_CLOCK SD_SCK_MHZ(20)
#define SD_CONFIG SdSpiConfig(PIN_SPI1_SS, SHARED_SPI, SPI_CLOCK, &SPI1)
SdFat sd;
// string to buffer output
bool fileCreated = false;
char loraBuffer[250];

// Create a new file with a number one higher than the highest numbered file
char newFileName[13];

int mode = 0;
float batterySOC = 0.0;
unsigned long previousLogMillis = 0;
unsigned long currentLogMillis = 0;
const int DATA_LOG_INTERVAL = 1000; // interval in milliseconds between data logs
unsigned long lastTime = 0;         // Simple local timer. Limits amount if I2C traffic to u-blox module.
unsigned long startTime = 0;        // Used to calc the actual update rate.
unsigned long updateCount = 0;      // Used to calc the actual update rate.

void iluminateErrorLed()
{
  Serial.println("error");
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);
}
//====================================================================================
//                                    SD card callbacks
//====================================================================================

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
int32_t msc_read_cb(uint32_t lba, void *buffer, uint32_t bufsize)
{
  bool rc;

#if SD_FAT_VERSION >= 20000
  rc = sd.card()->readSectors(lba, (uint8_t *)buffer, bufsize / 512);
#else
  rc = sd.card()->readBlocks(lba, (uint8_t *)buffer, bufsize / 512);
#endif

  return rc ? bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and
// return number of written bytes (must be multiple of block size)
int32_t msc_write_cb(uint32_t lba, uint8_t *buffer, uint32_t bufsize)
{
  bool rc;

#if SD_FAT_VERSION >= 20000
  rc = sd.card()->writeSectors(lba, buffer, bufsize / 512);
#else
  rc = sd.card()->writeBlocks(lba, buffer, bufsize / 512);
#endif

  return rc ? bufsize : -1;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
void msc_flush_cb(void)
{
#if SD_FAT_VERSION >= 20000
  sd.card()->syncDevice();
#else
  sd.card()->syncBlocks();
#endif

  // clear file system's cache to force refresh
  sd.cacheClear();
}

void start_usb_mass_storage()
{
  // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
  usb_msc.setID("Adafruit", "SD Card", "1.0");

  // Set read write callback
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);

  // Still initialize MSC but tell usb stack that MSC is not ready to read/write
  // If we don't initialize, board will be enumerated as CDC only
  usb_msc.setUnitReady(false);
  usb_msc.begin();

  Serial.begin(9600);
  Serial.println("Adafruit TinyUSB Mass Storage SD Card example");

  Serial.print("\nInitializing SD card ... ");
  Serial.print("CS = ");
  Serial.println(PIN_SPI1_SS);

  if (!sd.begin(SD_CONFIG))
  {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    iluminateErrorLed();
    while (1)
    {
    }
  }

  // Size in blocks (512 bytes)
#if SD_FAT_VERSION >= 20000
  uint32_t block_count = sd.card()->sectorCount();
#else
  uint32_t block_count = sd.card()->cardSize();
#endif

  Serial.print("Volume size (MB):  ");
  Serial.println((block_count / 2) / 1024);

  // Set disk size, SD block size is always 512
  usb_msc.setCapacity(block_count, 512);

  // MSC is ready for read/write
  usb_msc.setUnitReady(true);
}

//====================================================================================
//                                  Data Logging
//====================================================================================

bool createDataLoggingFile()
{
  // Create a new file with a number one higher than the highest numbered file
  int highestFileNumber = 0; // Initialize the highest file number to 0
  SdFile root;
  root.openRoot(sd.vol());
  SdFile file;
  while (file.openNext(&root, O_READ))
  {
    // Get the file's name
    char fileName[13];
    file.getName(fileName, 13);
    // Extract the file number from the file's name
    int fileNumber = atoi(fileName);
    // Update the highest file number if necessary
    if (fileNumber > highestFileNumber)
    {
      highestFileNumber = fileNumber;
    }
    file.close();
  }
  sprintf(newFileName, "%d.csv", highestFileNumber + 1);
  if (!sd.exists(newFileName))
  {
    SdFile newFile;
    if (newFile.open(newFileName, O_CREAT | O_WRITE))
    {
      Serial.print("Created new file: ");
      Serial.println(newFileName);
      newFile.print("Time UTC (H:M:S),Time Valid (0 = Invalid 1 = Valid),Longitude (DD°),Latitude (DD°),GPS Altitude (m),GPS Ground Speed (m/s),GPS Track Over Ground (deg°),Satellites In View, Fix Type (0 = No Fix 3 = 3D 4 = GNSS 5 = Time Fix), Primary Temperature (C°), Humidity (RH%), Altimeter Temperature (C°), Altitude Relative To Sea Level (1013.25 mBar) (m), Battery Percentage, Battery Discharge Rate (%/h), Time Since Power On (ms)");
      newFile.println();
      newFile.close();
      return true;
    }
    else
    {
      Serial.println("Error creating new file!");
      return false;
    }
  }
  else
  {
    return false;
  }
}
float getAltitude()
{
  // Get the atmospheric pressure in pascals
  float pressure = (altimeter.getPressure() * 100);
  // Calculate the altitude above sea level in meters using the barometric formula
  float altitude = (1 - pow(pressure / SEA_LEVEL_PRESSURE, 0.1903)) * 44330.8;
  return altitude;
}

void logGPSData()
{
  // SdFile dataFile;
  // if (dataFile.open(newFileName, FILE_WRITE))
  // {

  //   dataFile.println(dataBuffer);
  //   dataFile.close();
  //   digitalWrite(LED_GREEN, HIGH);
  //   logCount++;
  //   if (logCount == 5)
  //   {
  //     sprintf(loraBuffer, "%d:%d:%d,%.4f,%.4f,%.0f,%.1f,%.1f,%.1f,%.1f,%.1f", hour, min, sec, gpsLatitude, gpsLongitude, altimeterAltitude, gpsGroundSpeed, gpsHeading, externalTemp, externalHumidity, lipoStateOfCharge);
  //     LoRa.beginPacket();
  //     LoRa.write((const uint8_t *)loraBuffer, strlen(loraBuffer));
  //     LoRa.endPacket(true); // true = async / non-blocking mode
  //     Serial.println("sent lora");
  //     Serial.println(strlen(loraBuffer));
  //     Serial.println(loraBuffer);
  //     logCount = 0;
  //   }
  // }
  // else
  // {
  //   iluminateErrorLed();
  //   Serial.println("error");
  //   while (1)
  //   {
  //   }
  // }
  int currentTime = millis();
  char dataBuffer[250];
  // add a new line to the dataBuffer
  // int hour = GNSS.getHour();
  // int min = GNSS.getMinute();
  // int sec = GNSS.getSecond();
  // int timeValid = GNSS.getTimeValid();
  long gpsLongitude = (GNSS.getLongitude());
  long gpsLatitude = (GNSS.getLatitude());
  // float gpsAltitude = (GNSS.getAltitude());
  // float gpsGroundSpeed = (GNSS.getGroundSpeed());
  // float gpsHeading = (GNSS.getHeading());
  // int satelitesInView = GNSS.getSIV();
  // int fixType = GNSS.getFixType();
  // SHT30.readSample();
  // float externalTemp = SHT30.getTemperature();
  // float externalHumidity = SHT30.getHumidity();
  // float altimeterTemp = altimeter.getTemperature();
  // float altimeterAltitude = getAltitude();
  // float lipoStateOfCharge = lipo.getSOC();
  // float lipoDischargeRate = lipo.getChangeRate();
  unsigned long timestamp = millis();

  // sprintf(dataBuffer, "%d:%d:%d,%d,%.4f,%.4f,%.2f,%.2f,%.2f,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d", hour, min, sec, timeValid, gpsLatitude, gpsLongitude, gpsAltitude, gpsGroundSpeed, gpsHeading, satelitesInView, fixType, externalTemp, externalHumidity, altimeterTemp, altimeterAltitude, lipoStateOfCharge, lipoDischargeRate, timestamp);
  int time2 = millis();
  Serial.println(time2 - currentTime);
}
void buttonDoubleClick()
{
  Serial.println("double clicked");
  mode = 1;
}
// Function to blink the LED
void blinkLED()
{
  // Get the current time
  unsigned long currentMillis = millis();

  // Check if it's time to change the state of the LED
  if (currentMillis - previousMillis > (ledState ? ON_TIME : OFF_TIME))
  {
    // Toggle the state of the LED
    ledState = !ledState;
    digitalWrite(LED_BLUE, ledState);
    // Update the previous time to be the current time
    previousMillis = currentMillis;
  }
}
void slowPowerDown()
{
  noInterrupts();
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);
  pinMode(powerButtonPin, OUTPUT);
  digitalWrite(powerButtonPin, LOW);
  while (1)
  {
  }
}

// This function will be called repeatedly while the button is held
void handleLongPress()
{
  powerButtonPressed = true;
  // If this is the first time the function is called, record the start time
  if (longPressStartTime == 0)
  {
    longPressStartTime = millis();
  }

  // If the button has been held for longer than POWER_DOWN_TIME, perform the power down sequence
  if (millis() - longPressStartTime >= POWER_DOWN_TIME)
  {
    slowPowerDown();
  }
  // Get the current time
  unsigned long currentMillis = millis();

  // Check if it's time to change the state of the LED
  if (currentMillis - previousMillis > (ledState ? 500 : 500))
  {
    // Toggle the state of the LED
    ledState = !ledState;
    digitalWrite(LED_RED, ledState);
    digitalWrite(LED_BLUE, LOW);

    // Update the previous time to be the current time
    previousMillis = currentMillis;
  }
}

// This function will be called when the button is released after a long press
void handleLongPressStop()
{
  // Turn off the LED
  digitalWrite(LED_RED, LOW);

  // Reset the long press start time
  longPressStartTime = 0;
  powerButtonPressed = false;
}

//====================================================================================
//                                    Setup
//====================================================================================

void setup()
{
  // rp2040.idleOtherCore();
  // pinMode(LED_BLUE, OUTPUT);
  // pinMode(LED_RED, OUTPUT);
  // pinMode(LED_GREEN, OUTPUT);
  // pinMode(bootSelectButtonPin, INPUT_PULLUP);
  // int bootButtonReading = digitalRead(bootSelectButtonPin);
  // if (bootButtonReading == LOW)
  // {
  //   digitalWrite(LED_GREEN, HIGH);
  //   digitalWrite(LED_BLUE, HIGH);
  //   start_usb_mass_storage();
  //   while (1)
  //   {
  //   }
  // }
  // rp2040.resumeOtherCore();
  // digitalWrite(LED_BLUE, HIGH);
  // digitalWrite(LED_RED, HIGH);
  // Serial.begin(9600);
  // Wire1.begin();
  // Wire1.setClock(400000);

  // // Altimeter Initialization
  // if (!altimeter.begin(Wire1))
  // {
  //   iluminateErrorLed();
  //   while (1)
  //   {
  //     Serial.println("MS5637 sensor did not respond. Please check wiring.");
  //   }
  // }
  // // Set the resolution of the sensor to the highest level of resolution: 0.016 mbar
  // altimeter.setResolution(ms5637_resolution_osr_8192);
  // // MAX17048 Battery Fuel Gauge start
  // if (!lipo.begin(Wire1)) // Connect to the MAX17043 using non-standard wire port
  // {
  //   iluminateErrorLed();
  //   while (1)
  //   {
  //     Serial.println(F("MAX17048 not detected."));
  //   }
  // }
  // lipo.setThreshold(20);

  // // SHT30 Temperature and Humidity Sensor Initalization
  // if (!SHT30.init(Wire1))
  // {
  //   iluminateErrorLed();
  //   while (1)
  //   {
  //     Serial.print("SHT30 error");
  //   }
  // }
  // SHT30.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM); // only supported by SHT3x

  // // GPS setup
  // while (!GNSS.begin(Wire1)) // Connect to the u-blox module using Wire port
  // {
  //   Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
  // }
  // GNSS.setI2COutput(COM_TYPE_UBX); // Set the I2C port to output UBX only (turn off NMEA noise)
  // GNSS.setNavigationFrequency(5);  // Set output to 5 times a second

  // uint8_t rate = GNSS.getNavigationFrequency(); // Get the update rate of this module
  // Serial.print("Current update rate: ");
  // Serial.println(rate);
  // Serial.println("intialization done");
  // int startTime = millis();
  // digitalWrite(LED_BLUE, LOW);
  // digitalWrite(LED_RED, LOW);
  Serial.begin(9600);
  Serial.println("SparkFun u-blox Example");

  Serial1.begin(38400);

  do
  {
    Serial.println("GNSS: trying 38400 baud");
    Serial1.begin(38400);
    if (GNSS.begin(Serial1) == true)
      break;

    delay(100);
    Serial.println("GNSS: trying 9600 baud");
    Serial1.begin(9600);
    if (GNSS.begin(Serial1) == true)
    {
      Serial.println("GNSS: connected at 9600 baud, switching to 38400");
      GNSS.setSerialRate(38400);
      delay(100);
    }
    else
    {
      // myGNSS.factoryReset();
      delay(2000); // Wait a bit before trying again to limit the Serial output
    }
  } while (1);
  Serial.println("GNSS serial connected");

  GNSS.setUART1Output(COM_TYPE_UBX); // Set the UART port to output UBX only
  GNSS.setI2COutput(COM_TYPE_UBX);   // Set the I2C port to output UBX only (turn off NMEA noise)
  GNSS.setNavigationFrequency(8);    // Set output to 5 times a second
  GNSS.saveConfiguration();          // Save the current settings to flash and BBR

  uint8_t rate = GNSS.getNavigationFrequency(); // Get the update rate of this module
  Serial.print("Current update rate: ");
  Serial.println(rate);

  startTime = millis();
}

void setup1()
{
  // LoRa.setPins(csPin, resetPin, irqPin);
  // if (!LoRa.begin(915E6))
  // {
  //   iluminateErrorLed();
  //   while (1)
  //   {
  //     Serial.println("LoRa init failed. Check your connections.");
  //   }
  // }
  // if (!sd.begin(SD_CONFIG))
  // {
  //   Serial.println("initialization failed. Things to check:");
  //   Serial.println("* is a card inserted?");
  //   Serial.println("* is your wiring correct?");
  //   Serial.println("* did you change the chipSelect pin to match your shield or module?");
  //   iluminateErrorLed();
  //   while (1)
  //   {
  //     Serial.println("SD failed");
  //   }
  // }
  // fileCreated = createDataLoggingFile();
  // if (!fileCreated)
  // {
  //   iluminateErrorLed();
  //   while (1)
  //   {
  //     Serial.println("datalogging file failed");
  //   }
  // }
  // bootSelectButton.attachDoubleClick(buttonDoubleClick);
  // powerButton.attachDuringLongPress(handleLongPress);
  // powerButton.attachLongPressStop(handleLongPressStop);
}

void loop1()
{
  // if (mode == 1)
  // {
  //   int pulsePinGPS = digitalRead(ppsPin);
  //   if (pulsePinGPS == HIGH)
  //   {
  //     digitalWrite(LED_BLUE, HIGH);
  //   }
  //   else
  //   {
  //     digitalWrite(LED_BLUE, LOW);
  //   }
  // }
  // bootSelectButton.tick();
  // powerButton.tick();
}

void loop()
{
  // switch (mode)
  // {
  // // charge mode
  // case 0:
  //   if (!powerButtonPressed)
  //   {
  //     batterySOC = lipo.getSOC();
  //     if (batterySOC < 100)
  //     {
  //       blinkLED();
  //     }
  //     else
  //     {
  //       digitalWrite(LED_BLUE, HIGH);
  //     }
  //   }
  //   break;

  // // data log mode
  // case 1:
  //   // // Get the current time
  //   // currentLogMillis = millis();

  //   // // Check if it's time to run the function
  //   // if (currentLogMillis - previousLogMillis > DATA_LOG_INTERVAL)
  //   // {
  //   //   // Run the function
  //   //   logGPSData();
  //   //   // Update the previous time to be the current time
  //   //   previousLogMillis = currentLogMillis;
  //   // }
  //   logGPSData();
  //   break;
  //   // if the mode value is not covered by the case statements, do something else:
  // default:
  //   // insert your code here to handle other mode values
  //   break;
  // }
  // Query module every 25 ms. Doing it more often will just cause I2C traffic.
  // The module only responds when a new position is available. This is defined
  // by the update freq.
  if (millis() - lastTime > 25)
  {
    lastTime = millis(); // Update the timer

    long latitude = GNSS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    long longitude = GNSS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);

    updateCount++;

    // Calculate the actual update rate based on the sketch start time and the
    // number of updates we've received.
    Serial.print(F(" Rate: "));
    Serial.print(updateCount / ((millis() - startTime) / 1000.0), 2);
    Serial.print(F("Hz"));

    Serial.println();
  }
}
