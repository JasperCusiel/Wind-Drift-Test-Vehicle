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

// for led blink
uint8_t ppsLedCount = 0;
int previousPPSState = LOW; // store the previous state of the pin

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
int logCount = 0;                    // counter to keep track of number of logs made
const int TIME_BETWEEN_SENDS = 2000; // delay between lora sends
volatile bool loraBufferAvalible;

// Ublox Neo M9N module
SFE_UBLOX_GNSS GNSS;
const int ppsPin = 17;

// USB Mass Storage object
Adafruit_USBD_MSC usb_msc;
// File system on SD Card
#define SPI_CLOCK SD_SCK_MHZ(20)
#define SD_CONFIG SdSpiConfig(PIN_SPI1_SS, SHARED_SPI, SPI_CLOCK, &SPI1)
SdFat sd;
SdFile dataFile;
// string to buffer output
bool fileCreated = false;
char loraBuffer[100];

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
    if (dataFile.open(newFileName, O_CREAT | O_WRITE))
    {
      Serial.print("Created new file: ");
      Serial.println(newFileName);
      dataFile.print("Time UTC (H:M:S),Time Valid (0 = Invalid 1 = Valid),Longitude (DD°),Latitude (DD°),GPS Altitude (m),GPS Ground Speed (m/s),GPS Track Over Ground (deg°),Satellites In View, Fix Type (0 = No Fix 3 = 3D 4 = GNSS 5 = Time Fix), Primary Temperature (C°), Humidity (RH%), Altimeter Temperature (C°), Altitude Relative To Sea Level (1013.25 mBar) (m), Battery Percentage, Battery Discharge Rate (%/h)");
      dataFile.println();
      dataFile.sync();
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
  int time2 = millis();
  char dataBuffer[100];
  uint8_t hour = GNSS.getHour();
  uint8_t min = GNSS.getMinute();
  uint8_t sec = GNSS.getSecond();
  uint16_t millisecs = GNSS.getMillisecond();
  double gpsLongitude = ((GNSS.getLongitude()) * 1E-7);
  double gpsLatitude = ((GNSS.getLatitude()) * 1E-7);
  float gpsAltitude = ((GNSS.getAltitudeMSL()) * 1E-3);
  float gpsGroundSpeed = ((GNSS.getGroundSpeed()) * 1E-3); // Ground Speed (2-D): m/s
  float gpsHeading = ((GNSS.getHeading()) * 1E-5);         // Heading of motion (2-D): deg
  uint8_t satelitesInView = GNSS.getSIV();                 // Number of satellites used in Nav Solution
  uint8_t fixType = GNSS.getFixType();
  bool timeValid = GNSS.getTimeValid();
  float externalTemp = SHT30.getTemperature();
  float externalHumidity = SHT30.getHumidity();
  float altimeterTemp = altimeter.getTemperature();
  float altimeterAltitude = getAltitude();
  float lipoStateOfCharge = lipo.getSOC();
  float lipoDischargeRate = lipo.getChangeRate();

  sprintf(dataBuffer, "%d:%d:%d.%d,%d,%.6f,%.6f,%.2f,%.2f,%.2f,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", hour, min, sec, millisecs, timeValid, gpsLatitude, gpsLongitude, gpsAltitude, gpsGroundSpeed, gpsHeading, satelitesInView, fixType, externalTemp, externalHumidity, altimeterTemp, altimeterAltitude, lipoStateOfCharge, lipoDischargeRate);
  noInterrupts();
  dataFile.println(dataBuffer);
  if (!dataFile.sync())
  {
    iluminateErrorLed();
    return;
  }
  digitalWrite(LED_GREEN, HIGH);
  interrupts();
  logCount++;
  if (logCount == 10)
  {
    loraBufferAvalible = false;
    sprintf(loraBuffer, "%d:%d:%d.%d,%.6f,%.6f,%d,%.1f,%.1f,%.1f,%.1f,%.1f", hour, min, sec, gpsLatitude, gpsLongitude, altimeterAltitude, gpsGroundSpeed, gpsHeading, externalTemp, externalHumidity, lipoStateOfCharge);
    loraBufferAvalible = true;
    logCount = 0;
  }
}
void buttonDoubleClick()
{
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
}

// This function will be called when the button is released after a long press
void handleLongPressStop()
{
  // Reset the long press start time
  longPressStartTime = 0;
  powerButtonPressed = false;
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

//====================================================================================
//                                    Setup
//====================================================================================

void setup()
{
  rp2040.idleOtherCore();
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(bootSelectButtonPin, INPUT_PULLUP);
  int bootButtonReading = digitalRead(bootSelectButtonPin);
  if (bootButtonReading == LOW)
  {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, HIGH);
    start_usb_mass_storage();
    while (1)
    {
    }
  }
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_RED, HIGH);
  rp2040.resumeOtherCore();

  Serial.begin(9600);
  Wire1.begin();
  Wire1.setClock(400000);

  // Altimeter Initialization
  if (!altimeter.begin(Wire1))
  {
    iluminateErrorLed();
    Serial.println("MS5637 sensor did not respond. Please check wiring.");
    return;
  }

  altimeter.setResolution(ms5637_resolution_osr_2048); // 5ms per reading, 0.028mbar resolution

  // MAX17048 Battery Fuel Gauge start
  if (!lipo.begin(Wire1)) // Connect to the MAX17043 using non-standard wire port
  {
    iluminateErrorLed();
    Serial.println(F("MAX17048 not detected."));
    return;
  }

  lipo.setThreshold(20);

  // SHT30 Temperature and Humidity Sensor Initalization
  if (!SHT30.init(Wire1))
  {
    iluminateErrorLed();
    Serial.print("SHT30 error");
    return;
  }

  SHT30.setAccuracy(SHTSensor::SHT_ACCURACY_HIGH); // only supported by SHT3x

  Serial1.begin(38400);
  if (!GNSS.begin(Serial1))
  {
    Serial.println("gps not started");
    return;
  }

  // Create storage for the time pulse parameters
  UBX_CFG_TP5_data_t timePulseParameters;

  // Get the time pulse parameters
  if (!GNSS.getTimePulseParameters(&timePulseParameters))
  {
    Serial.println(F("getTimePulseParameters failed!"));
    return;
  }

  timePulseParameters.tpIdx = 0; // Select the TIMEPULSE pin

  // We can configure the time pulse pin to produce a defined frequency or period
  // Here is how to set the frequency:

  // When the module is _locked_ to GNSS time, make it generate 10Hz
  timePulseParameters.freqPeriod = 1;            // Set the frequency/period to 1Hz
  timePulseParameters.pulseLenRatio = 50000;     // Set the period to 50,000 us
  timePulseParameters.freqPeriodLock = 10;       // Set the frequency/period to 10Hz
  timePulseParameters.pulseLenRatioLock = 50000; // Set the period to 50,000 us

  timePulseParameters.flags.bits.active = 1;         // Make sure the active flag is set to enable the time pulse. (Set to 0 to disable.)
  timePulseParameters.flags.bits.lockedOtherSet = 1; // Tell the module to use freqPeriod while locking and freqPeriodLock when locked to GNSS time
  timePulseParameters.flags.bits.isFreq = 1;         // Tell the module that we want to set the frequency (not the period)
  timePulseParameters.flags.bits.isLength = 1;       // Tell the module that pulseLenRatio is a length (in us) - not a duty cycle
  timePulseParameters.flags.bits.polarity = 1;       // Tell the module that we want the rising edge at the top of second. (Set to 0 for falling edge.)

  // Now set the time pulse parameters
  if (GNSS.setTimePulseParameters(&timePulseParameters) == false)
  {
    Serial.println("setTimePulseParameters failed!");
    return;
  }

  GNSS.setUART1Output(COM_TYPE_UBX); // Set the UART port to output UBX only
  GNSS.setI2COutput(COM_TYPE_UBX);   // Set the I2C port to output UBX only (turn off NMEA noise)
  GNSS.setNavigationFrequency(10);   // Set output to 5 times a second
  GNSS.saveConfiguration();          // Save the current settings to flash and BBR
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_RED, LOW);
}

void setup1()
{
  LoRa.setPins(csPin, resetPin, irqPin);

  if (!LoRa.begin(915E6))
  {
    iluminateErrorLed();
    Serial.println("LoRa init failed. Check your connections.");
    return;
  }

  if (!sd.begin(SD_CONFIG))
  {
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    iluminateErrorLed();
    Serial.println("SD failed");
    return;
  }

  fileCreated = createDataLoggingFile();

  if (!fileCreated)
  {
    iluminateErrorLed();
    Serial.println("datalogging file failed");
    return;
  }

  bootSelectButton.attachDoubleClick(buttonDoubleClick);
  powerButton.attachDuringLongPress(handleLongPress);
  powerButton.attachLongPressStop(handleLongPressStop);
}

void loop1()
{
  if (mode == 1)
  {
    if (runEvery(TIME_BETWEEN_SENDS))
    { // repeat every 2000 millis
      if (!loraBufferAvalible)
      {
        return;
      }
      // Serial.print("Sending packet non-blocking: ");
      // Serial.println(loraBuffer);
      //  send in async / non-blocking mode
      LoRa.beginPacket();
      LoRa.write((const uint8_t *)loraBuffer, strlen(loraBuffer));
      LoRa.endPacket(true); // true = async / non-blocking mode
    }
    int currentPPSState = digitalRead(ppsPin); // read the current state of the pin
    if (currentPPSState != previousPPSState)
    { // if the state has changed
      if (currentPPSState == HIGH)
      {
        if ((ppsLedCount++) == 10)
        {
          digitalWrite(LED_BLUE, HIGH);
          ppsLedCount = 0;
        }
        else
        {
          digitalWrite(LED_BLUE, LOW);
        }
      }
    }
    previousPPSState = currentPPSState; // update the previous state with the current state
  }
  powerButton.tick();
  bootSelectButton.tick();
}

void loop()
{
  switch (mode)
  {
  case 0:
    if (!powerButtonPressed)
    {
      batterySOC = lipo.getSOC();
      if (batterySOC < 100)
      {
        blinkLED();
      }
      else
      {
        digitalWrite(LED_BLUE, HIGH);
      }
    }
    break;

  // data log mode
  case 1:

    if ((digitalRead(ppsPin) == HIGH))
    {
      if (GNSS.getGnssFixOk())
      {
        logGPSData();
        break;
      }
      else if (GNSS.getMillisecond() == 0)
      {
        logGPSData();
        Serial.println(millis());
      }
    }
    break;
    // if the mode value is not covered by the case statements, do something else:
  default:
    mode = 0;
    break;
  }
}
