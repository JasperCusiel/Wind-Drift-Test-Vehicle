/**
 * @file main.cpp
 * @author Jasper Cusiel (jaspercusiel@icloud.com)
 * @brief This code runs on the vehicle. It samples data from the sensors at 10Hz when it has a 2D or 3D GPS fix
 * and at 1Hz when it is trying to obtain a fix. Data is sent over LoRa to the ground station at 0.5Hz. Core 0 handles sampling
 * the sensors and core 1 handles sending the LoRa data.
 */

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_TinyUSB.h>
#include <OneButton.h>
#include <SdFat.h>
#include <SdFatConfig.h>
// Sensor libraries
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h> // Onboard lipo fuel gauge - measures charge/discharge rates, state of charge (SOC) and voltage
#include <SparkFun_MS5637_Arduino_Library.h>              // MS5637 altimeter
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>         // Mikroe GNSS 7 GPS - Ublox Neo M9N module
#include <arduino-sht.h>                                  // Adafruit SHT-30 Mesh-protected Weather-proof Temperature/Humidity Sensor
#include <LoRa.h>                                         // Adafruit LoRa Radio FeatherWing - RFM95W 900 MHz

// LED indicator pins
const int LED_GREEN = 26;
const int LED_RED = 28;
const int LED_BLUE = 27;

// Blue charging blink on and off times in milliseconds
const int ON_TIME = 250;
const int OFF_TIME = 2500;

// GPS pulse per second blink variables
uint8_t ppsLedCount = 0;
int previousPPSState = LOW;
unsigned long lastUpdateTime = 0;

unsigned long previousMillis = 0; // Stores the last time the LED was updated
int ledState = LOW;               // Stores the current state of the LED

// Black power button
const int powerButtonPin = 21;
OneButton powerButton(powerButtonPin);
const int POWER_DOWN_TIME = 4000; // Time (in milliseconds) to hold the button to initiate power down (add about 2000 millisec for shut down function)

// White function button
const int bootSelectButtonPin = 22;
OneButton bootSelectButton(bootSelectButtonPin);
unsigned long longPressStartTime = 0;

// MS5637 Altimeter
MS5637 altimeter;
const float SEA_LEVEL_PRESSURE = 101325; // Standard sea level pressure in pascals

// MAX17048 Battery Fuel Gauge
SFE_MAX1704X lipo(MAX1704X_MAX17048);

// SHT30 Temperature and Humidity Sensor
SHTSensor SHT30;
float temp, humidity;

// LoRa RFM95W featherwing
const int csPin = 19;
const int resetPin = 18;
const int irqPin = 20;
const int TIME_BETWEEN_SENDS = 2000; // Delay between lora sends in milliseconds
volatile bool loraBufferAvalible;
char loraBuffer[100]; // Char array to store the data to send via LoRa radio

// Mikroe GNSS 7 - Ublox Neo M9N GPS module
SFE_UBLOX_GNSS GNSS;
const int ppsPin = 17;

// USB mass storage - adafruit tiny usb
Adafruit_USBD_MSC usb_msc;

// SD card SPI settings
#define SPI_CLOCK SD_SCK_MHZ(20)
#define SD_CONFIG SdSpiConfig(PIN_SPI1_SS, SHARED_SPI, SPI_CLOCK, &SPI1)

// SD card
SdFat sd;
SdFile dataFile;
bool fileCreated = false;
char newFileName[13]; // For storing new file name created on boot

// Program Variables
int mode = 0; // Current vehicle mode: 0 = charging, 1 = data logging
float batterySOC = 0.0;

/**
 * @brief This function turns off all of the other led indicator led's and only illuminates the red led.
 * Message is printed to the serial monitor indicating the function triggered
 */
void illuminateErrorLed()
{
  Serial.println("Error Triggered");
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);
}

//====================================================================================
//                                    SD card callbacks
//====================================================================================

/**
 * @brief Callback invoked when received READ10 command.
 * Copy disk's data to buffer (up to bufsize) and return number of copied bytes (must be multiple of block size)
 */
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

/**
 * @brief Callback invoked when received WRITE10 command.
 * Process data in buffer to disk's storage and return number of written bytes (must be multiple of block size)
 */
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

/**
 * @brief Callback invoked when WRITE10 command is completed (status received and accepted by host).
 * Used to flush any pending cache.
 */
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

/**
 * @brief This function starts the vehicle as a USB thumb drive so that the connected computer can access the contents on the SD card.
 * The vehicle must be rebooted via power cycle to get stop emulating the thumb drive.
 */
void start_usb_mass_storage()
{
  usb_msc.setID("Adafruit", "SD Card", "1.0");
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
  usb_msc.setUnitReady(false); // Still initialize MSC but tell usb stack that MSC is not ready to read/write
  usb_msc.begin();             // If we don't initialize, board will be enumerated as CDC only

  Serial.begin(9600); // Start serial for debug messages
  Serial.print("Initializing SD card ... ");

  if (!sd.begin(SD_CONFIG))
  {
    Serial.println("SD failed to begin");
    illuminateErrorLed();
    return; // Illuminate error led then re-try starting SD
  }
  Serial.println("SD started");
  // Size in blocks (512 bytes)
#if SD_FAT_VERSION >= 20000
  uint32_t block_count = sd.card()->sectorCount();
#else
  uint32_t block_count = sd.card()->cardSize();
#endif

  Serial.print("Volume size (MB):  ");
  Serial.println((block_count / 2) / 1024);

  usb_msc.setCapacity(block_count, 512); // Set disk size, SD block size is always 512
  usb_msc.setUnitReady(true);            // MSC is ready for read/write
}

//====================================================================================
//                                  Data Logging
//====================================================================================
/**
 * @brief This function creates a newfile with the named one digit higher than the highest numbered file
 * @return true if creating new file was succesfull
 * @return false if creating new file failed
 */
bool createDataLoggingFile()
{
  int highestFileNumber = 0;

  SdFile root;
  root.openRoot(sd.vol());

  SdFile file;

  while (file.openNext(&root, O_READ))
  {
    char fileName[13];
    file.getName(fileName, 13); // Extract the file number from the file's name
    int fileNumber = atoi(fileName);
    if (fileNumber > highestFileNumber)
    {
      highestFileNumber = fileNumber; // Update the highest file number if necessary
    }
    file.close();
  }

  sprintf(newFileName, "%d.csv", highestFileNumber + 1); // Add one to the highest numbered file to create the name for the new file
  if (!sd.exists(newFileName))
  {
    if (dataFile.open(newFileName, O_CREAT | O_WRITE)) // Open new file and add data column headers
    {
      Serial.print("Created new file: ");
      Serial.println("Data log file created");
      Serial.println(newFileName);
      dataFile.print("Time UTC (H:M:S),Time Valid (0 = Invalid 1 = Valid),Latitude (DD°),Longitude (DD°),GPS Altitude (m),GPS Ground Speed (m/s),GPS Track Over Ground (deg°),Satellites In View, Fix Type (0 = No 2 = 2D Fix 3 = 3D 4 = GNSS 5 = Time Fix), Primary Temperature (C°), Humidity (RH%), Altimeter Temperature (C°), Altitude Relative To Sea Level (1013.25 mBar) (m), Battery Percentage, Battery Discharge Rate (%/h)");
      dataFile.println();
      dataFile.sync(); // sync file but don't close file as it is much slower to open/close file each time new LoRa data needs to be written to SD
      return true;
    }
    else
    {
      Serial.println("Error creating new file");
      return false;
    }
  }
  else
  {
    return false;
  }
}

/**
 * @brief Samples the altimeter and calculates the altitude above sea level using the barometric formula and the standard SEA_LEVEL_PRESSURE
 * @return altitude above sea level (float)
 */
float getAltitude()
{
  float pressure = (altimeter.getPressure() * 100);
  float altitude = (1 - pow(pressure / SEA_LEVEL_PRESSURE, 0.1903)) * 44330.8;
  return altitude;
}

/**
 * @brief This function samples the GPS and sensors and then logs that data to the SD card
 */
void logGPSData()
{
  char dataBuffer[100]; // Buffer to hold the sampled data
  // Get time stamp
  uint8_t hour = GNSS.getHour();
  uint8_t min = GNSS.getMinute();
  uint8_t sec = GNSS.getSecond();
  uint16_t millisecs = GNSS.getMillisecond();
  // Get GPS data
  float gpsLongitude = ((GNSS.getLongitude()) * 1E-7);
  float gpsLatitude = ((GNSS.getLatitude()) * 1E-7);
  float gpsAltitude = ((GNSS.getAltitudeMSL()) * 1E-3);    // Get the current altitude in m according to mean sea level
  float gpsGroundSpeed = ((GNSS.getGroundSpeed()) * 1E-3); // Ground Speed (2-D): m/s
  float gpsHeading = ((GNSS.getHeading()) * 1E-5);         // Heading of motion (2-D): deg
  uint8_t satelitesInView = GNSS.getSIV();                 // Number of satellites used in Nav Solution
  uint8_t fixType = GNSS.getFixType();                     // Get current fix type 0=no fix, 1=dead reckoning, 2=2D, 3=3D, 4=GNSS, 5=Time fix
  bool timeValid = GNSS.getTimeValid();
  // Get temp and humidity sensor data
  SHT30.readSample();
  float externalTemp = SHT30.getTemperature();
  float externalHumidity = SHT30.getHumidity();
  float altimeterTemp = altimeter.getTemperature();
  // Get altimeter data
  float altimeterAltitude = getAltitude();
  // Get battery data
  float lipoStateOfCharge = lipo.getSOC();        // get lipo state of charge (SOC)
  float lipoDischargeRate = lipo.getChangeRate(); // get battery percentage change per hour

  sprintf(dataBuffer, "%d:%d:%d.%d,%d,%f,%f,%.2f,%.2f,%.2f,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", hour, min, sec, millisecs, timeValid, gpsLatitude, gpsLongitude, gpsAltitude, gpsGroundSpeed, gpsHeading, satelitesInView, fixType, externalTemp, externalHumidity, altimeterTemp, altimeterAltitude, lipoStateOfCharge, lipoDischargeRate);
  noInterrupts(); // Make sure interupts dont block SD save
  dataFile.println(dataBuffer);
  if (!dataFile.sync())
  {
    Serial.println("Data Failed to save to SD");
    illuminateErrorLed();
    return;
  }
  digitalWrite(LED_GREEN, HIGH); // Indicate SD write was succesfull by illuminating green led
  interrupts();                  // Turn interupts back on to allow power button and function button interupts to work

  loraBufferAvalible = false; // Tell other core the lora buffer is not avalible to be read from while we update the LoRa buffer with fresh data
  sprintf(loraBuffer, "%d:%d:%d.%d,%d,%f,%f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f", hour, min, sec, millisecs, fixType, gpsLatitude, gpsLongitude, altimeterAltitude, gpsGroundSpeed, gpsHeading, externalTemp, externalHumidity, lipoStateOfCharge);
  loraBufferAvalible = true;
}

/**
 * @brief Function is called when white function button is double clicked
 */
void buttonDoubleClick()
{
  mode = 1; // Change to data logging mode
}

/**
 * @brief Function to blink the blue led in the charging mode. Called if the battery is not fully charged
 */
void blinkLED()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > (ledState ? ON_TIME : OFF_TIME))
  {
    ledState = !ledState;
    digitalWrite(LED_BLUE, ledState);
    previousMillis = currentMillis;
  }
}

void slowPowerDown()
{
  /**
   * @brief This function closes the data logging file, turns the led indicator white
   * and then pulls the power button pin LOW which after ~2s cuts power to the board
   */
  dataFile.close(); // close the data log file for good measure
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_RED, HIGH);
  noInterrupts();
  pinMode(powerButtonPin, OUTPUT);
  digitalWrite(powerButtonPin, LOW);
  while (1) // do nothing while power button pin is low (waiting for soft power switch circuity to kill the power)
  {
  }
}

void handleLongPress()
{
  /**
   * @brief This function is called while the power button is being pressed and checks if the button has been pressed for longer than the POWER_DOWN_TIME.
   * Calls the slowPowerDown() function if the power button has been held for long enough.
   */
  if (longPressStartTime == 0) // If this is the first time the function is called, record the start time
  {
    longPressStartTime = millis();
  }

  if (millis() - longPressStartTime >= POWER_DOWN_TIME)
  {
    slowPowerDown();
  }
}

void handleLongPressStop()
{
  /**
   * @brief This function is called when the power button is released and resets the longPressStart time to zero.
   */
  longPressStartTime = 0;
}

/**
 * @brief This function returns true if it has been longer than the interval and false if it hasn't.
 * Used in the main loop to check if its time to send fresh LoRa data
 * @param interval (milliseconds)
 * @return true
 * @return false
 */
bool runEvery(unsigned long interval)
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
  rp2040.idleOtherCore(); // Idle other core such that the setup dont interfere with checking if the white function button is pressed on boot
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(bootSelectButtonPin, INPUT_PULLUP);
  int bootButtonReading = digitalRead(bootSelectButtonPin);
  if (bootButtonReading == LOW)
  {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, HIGH);
    start_usb_mass_storage(); // Boot as USB thumb drive
    while (1)                 // Do nothing while in thumb drive mode, the adafruit tiny USB MSC(mass storage controller) handles the callbacks in the "sd card callback" section.
    {                         // We can't do anything else here as the computer assumes it has exclusive control over the flash (SD card in this case)
    }
  }
  // Boot into normal mode
  digitalWrite(LED_BLUE, HIGH); // Indicate device is initalizing (purple led)
  digitalWrite(LED_RED, HIGH);
  rp2040.resumeOtherCore(); // Start core 1 setup
  Wire1.begin();
  Wire1.setClock(400000);

  // Altimeter Initialization
  if (!altimeter.begin(Wire1))
  {
    illuminateErrorLed();
    Serial.println("Altimeter did not respond");
    return;
  }

  altimeter.setResolution(ms5637_resolution_osr_2048); // Set resolution (5ms per reading, 0.028mbar resolution)

  if (!lipo.begin(Wire1)) // MAX17048 Battery Fuel Gauge start
  {
    illuminateErrorLed();
    Serial.println(F("Lipo fuel gauge not detected"));
    return;
  }
  lipo.setThreshold(10);        // Set low battery alert percentage, this can be used to run a callback when the alert is triggered (not implemented in this code)
  lipo.setVALRTMax((float)4.2); // Set high voltage threshold (Volts)
  lipo.setVALRTMin((float)3.2); // Set low voltage threshold (Volts)

  if (!SHT30.init(Wire1)) // SHT30 Temperature and Humidity Sensor Initalization
  {
    illuminateErrorLed();
    Serial.print("SHT30 failed to start");
    return;
  }

  SHT30.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM); // Set accuracy to medium (the higher the accuracy the longer the sample takes)

  Serial1.begin(38400); // Start serial1 UART connection to GPS module

  if (!GNSS.begin(Serial1)) // Start GPS module on UART connection
  {
    Serial.println("GPS failed to start");
    return;
  }

  UBX_CFG_TP5_data_t timePulseParameters; // For storing the PPS pin parameters

  // Get the time pulse parameters
  if (!GNSS.getTimePulseParameters(&timePulseParameters))
  {
    Serial.println(F("getTimePulseParameters failed!"));
    return;
  }

  timePulseParameters.tpIdx = 0; // Select the TIMEPULSE pin

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

  if (GNSS.setTimePulseParameters(&timePulseParameters) == false) // Check if setting the time pulse parameters where succesfully set
  {
    Serial.println("Setting Time Pulse Parameters failed");
    return;
  }

  GNSS.setUART1Output(COM_TYPE_UBX); // Set the UART port to output UBX only
  GNSS.setI2COutput(COM_TYPE_UBX);   // Set the I2C port to output UBX only (turn off NMEA noise)
  GNSS.setNavigationFrequency(10);   // Set output to 10 times a second
  GNSS.saveConfiguration();          // Save the current settings to flash and BBR
  digitalWrite(LED_BLUE, LOW);       // Turn off purple led as setup was succesfull
  digitalWrite(LED_RED, LOW);
}

void setup1()
{
  // LoRa setup
  LoRa.setPins(csPin, resetPin, irqPin);
  if (!LoRa.begin(915E6)) // Setup to run at 915Mhz
  {
    illuminateErrorLed();
    Serial.println("LoRa failed to start");
    return;
  }

  // SD card setup
  if (!sd.begin(SD_CONFIG))
  {
    illuminateErrorLed();
    Serial.println("SD failed to start");
    return;
  }

  // Call function to create new data logging file
  fileCreated = createDataLoggingFile();

  if (!fileCreated) // Check if file was created, retry if failed
  {
    illuminateErrorLed();
    Serial.println("Creating datalogging file failed");
    return;
  }

  // Set callbacks for power and funciton button
  bootSelectButton.attachDoubleClick(buttonDoubleClick);
  powerButton.attachDuringLongPress(handleLongPress);
  powerButton.attachLongPressStop(handleLongPressStop);
}
/**
 * @brief Loop 1 handles blinking the blue led when the PPS (pulse per second) pin pulses,
 * sending data via LoRa and checking the power and function button states.
 */
void loop1()
{
  if (mode == 1) // If in data logging mode
  {
    if (runEvery(TIME_BETWEEN_SENDS))
    {
      if (!loraBufferAvalible) // Check if core 0 is not writing fresh data to the buffer
      {
        return; // Wait until the buffer is available
      }
      LoRa.beginPacket();
      LoRa.write((const uint8_t *)loraBuffer, strlen(loraBuffer));
      LoRa.endPacket(true); // true = async / non-blocking mode
    }
    int currentPPSState = digitalRead(ppsPin);
    if (currentPPSState != previousPPSState)
    { // if the state has changed
      if (currentPPSState == HIGH)
      {
        if ((ppsLedCount++) == 10)
        {
          digitalWrite(LED_BLUE, HIGH); // Blink every 10 PPS pulses
          ppsLedCount = 0;
        }
        else
        {
          digitalWrite(LED_BLUE, LOW);
        }
      }
    }
    previousPPSState = currentPPSState;
  }
  // Check powerbutton and function button for their state
  powerButton.tick();
  bootSelectButton.tick();
}

void loop()
{
  switch (mode)
  {
  case 0:
    batterySOC = lipo.getSOC();
    if (batterySOC < 98) // Consider 98% or higher a fully charged battery
    {
      blinkLED(); // Blink blue led if battery isn't fully charged
    }
    else
    {
      digitalWrite(LED_BLUE, HIGH); // Solid blue led to indicate fully charged battery
    }
    break;

  // Data logging mode
  case 1:
    if ((digitalRead(ppsPin) == HIGH)) // PPS pulses at 10Hz with a GPS fix and aids with timing when to log data as it has a much higher precision than the inbuilt timer
    {                                  // The PPS pin pulses at 1Hz wiht no GPS fix.
      logGPSData();
    }
    break;
  default:
    mode = 0; // Defult to charging mode
    break;
  }
}