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
// RP2040 Board
const int board_SPI_SCK = 2;
const int board_SPI_TX = 3;
const int board_SPI_RX = 4;

const int board_SPI1_SCK = 14;
const int board_SPI1_TX = 15;
const int board_SPI1_RX = 12;

const int board_SDA = 6;
const int board_SCL = 7;

const int bootSelectButtonPin = 22;
int vehicleState = 0; // 0 = charging mode, 1 = dataLogging, 2 = Error
bool loggingData = false;
volatile bool bufferAvalible = false;

// Status LED
const int LED_GREEN = 26;
const int LED_RED = 28;
const int LED_BLUE = 27;
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

// LoRa Setup
const int csPin = 19;
const int resetPin = 18;
const int irqPin = 20;
const int MAX_MESSAGE_LENGTH = 100;
char message[MAX_MESSAGE_LENGTH];

// Ublox Neo M9N module
SFE_UBLOX_GNSS GNSS;
long lastTime = 0; // Simple local timer. Limits amount if I2C traffic to u-blox module.
const int ppsPin = 17;

// USB Mass Storage object
Adafruit_USBD_MSC usb_msc;
// File system on SD Card
#define SPI_CLOCK SD_SCK_MHZ(20)
#define SD_CONFIG SdSpiConfig(PIN_SPI1_SS, SHARED_SPI, SPI_CLOCK, &SPI1)
SdFat sd;
// string to buffer output
String dataBuffer;
bool fileCreated = false;

// Create a new file with a number one higher than the highest numbered file
char newFileName[13];

int mode = 1;

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
  while (!Serial)
  {
    delay(10);
  }
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
    while (1)
    {
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED, LOW);
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

void createDataLoggingFile()
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
      newFile.print("Time UTC (H:M:S),Time Valid (0 = Invalid 1 = Valid),Longitude (DD°),Latitude (DD°),GPS Altitude (m),GPS Ground Speed (m/s),GPS Track Over Ground (deg°),Satellites In View, Fix Type (0 = No Fix 3 = 3D 4 = GNSS 5 = Time Fix), Primary Temperature (C°), Humidity (RH%), Altimeter Temperature (C°), Altitude Change (m), Battery Percentage, Battery Discharge Rate (%/h), timestamp");
      newFile.println();
      newFile.close();
    }
    else
    {
      Serial.println("Error creating new file!");
    }
  }
}

void logGPSData()
{
  SdFile dataFile;
  if (dataFile.open(newFileName, FILE_WRITE))
  {
    bufferAvalible = false;
    dataBuffer = "";
    // add a new line to the dataBuffer
    dataBuffer += GNSS.getHour();
    dataBuffer += ":";
    dataBuffer += GNSS.getMinute();
    dataBuffer += ":";
    dataBuffer += GNSS.getSecond();
    dataBuffer += ",";
    dataBuffer += GNSS.getTimeValid();
    dataBuffer += ",";
    dataBuffer += (GNSS.getLongitude() * 1E-7);
    dataBuffer += ",";
    dataBuffer += (GNSS.getLatitude() * 1E-7);
    dataBuffer += ",";
    dataBuffer += (GNSS.getAltitude() * 1E-3);
    dataBuffer += ",";
    dataBuffer += (GNSS.getGroundSpeed() * 1E-3);
    dataBuffer += ",";
    dataBuffer += (GNSS.getHeading() * 1E-5);
    dataBuffer += ",";
    dataBuffer += GNSS.getSIV();
    dataBuffer += ",";
    dataBuffer += GNSS.getFixType();
    dataBuffer += ",";
    dataBuffer += SHT30.getTemperature();
    dataBuffer += ",";
    dataBuffer += SHT30.getHumidity();
    dataBuffer += ",";
    dataBuffer += altimeter.getTemperature();
    dataBuffer += ",";
    dataBuffer += altimeter.altitudeChange(altimeter.getPressure(), startingPressure);
    dataBuffer += ",";
    dataBuffer += lipo.getSOC();
    dataBuffer += ",";
    dataBuffer += lipo.getChangeRate();
    dataFile.println(dataBuffer.c_str());
    dataFile.close();
    bufferAvalible = true;
  }
}
void iluminateErrorLed()
{
  Serial.println("error");
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_BLUE, LOW);
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
    digitalWrite(LED_RED, HIGH);
    start_usb_mass_storage();
    while (1)
    {
    }
  }
  rp2040.resumeOtherCore();
  digitalWrite(LED_BLUE, HIGH);
  Serial.begin(9600);
  while (!Serial)
  {
    delay(10);
  }
  Wire1.begin();
  // Wire1.setClock(400000); // Increase I2C clock speed to 400kHz

  // Altimeter Initialization
  if (altimeter.begin(Wire1) == false)
  {
    Serial.println("MS5637 sensor did not respond. Please check wiring.");
  }
  // Set the resolution of the sensor to the highest level of resolution: 0.016 mbar
  altimeter.setResolution(ms5637_resolution_osr_8192);

  // Take 16 readings and average them
  startingPressure = 0.0;
  for (int x = 0; x < 16; x++)
    startingPressure += altimeter.getPressure();
  startingPressure /= (float)16;
  // MAX17048 Battery Fuel Gauge start
  while (lipo.begin(Wire1) == false) // Connect to the MAX17043 using non-standard wire port
  {
    Serial.println(F("MAX17048 not detected."));
    iluminateErrorLed();
  }
  lipo.setThreshold(20);

  // SHT30 Temperature and Humidity Sensor Initalization
  while (!SHT30.init(Wire1))
  {
    Serial.print("SHT30 error");
    iluminateErrorLed();
  }

  // GPS setup
  if (GNSS.begin(Wire1) == false) // Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
    {
      iluminateErrorLed();
    }
  }
  GNSS.setI2COutput(COM_TYPE_UBX);                 // Set the I2C port to output UBX only (turn off NMEA noise)
  GNSS.setNavigationFrequency(5);                  // Set output to 10 times a second
  GNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); // Save (only) the communications port settings to flash and BBR
}

void setup1()
{
  LoRa.setPins(csPin, resetPin, irqPin);
  if (!LoRa.begin(915E6))
  {
    Serial.println("LoRa init failed. Check your connections.");
    iluminateErrorLed();
    while (1)
    {
    }
  }
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
  createDataLoggingFile();
}

void loop1()
{
  if (vehicleState == 1)
  {
    if (digitalRead(ppsPin) == HIGH)
    {
      digitalWrite(LED_BLUE, HIGH);
    }
    else
    {
      digitalWrite(LED_BLUE, LOW);
    }
  }
  // Serial.print("Sending packet: ");
  // Serial.println(count);

  // // send packet
  // LoRa.beginPacket();
  // LoRa.print("hello ");
  // LoRa.print(count);
  // LoRa.endPacket();

  // count++;

  // delay(5000);
}

void loop()
{
  // if (!fileCreated)
  // {
  //   createDataLoggingFile();
  // }
  // if (millis() - lastTime > 1000)
  // {
  //   int time = millis();
  //   if (logGPSData())
  //   {
  //     digitalWrite(LED_GREEN, HIGH);
  //     digitalWrite(LED_RED, LOW);
  //     digitalWrite(LED_BLUE, LOW);
  //   }
  //   else
  //   {
  //     digitalWrite(LED_RED, HIGH);
  //     digitalWrite(LED_GREEN, LOW);
  //     digitalWrite(LED_BLUE, LOW);
  //   }
  //   lastTime = millis(); // Update the timer
  // }
}