#include <Arduino.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include <SparkFun_MS5637_Arduino_Library.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <SPI.h>
#include <arduino-sht.h>
#include <Adafruit_TinyUSB.h>
#include <RP2040_SD.h>
#include <EEPROM.h>

#define DEBUGGING
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

// LoRa
// const int RFM_CS = 19;          // RFM95 chip select pin
// const int RFM_RST = 18;         // RFM95 reset pin
// const int RFM_IQR = 20;         // RFM95 IQR pin
const int RFM_CS = 13;          // RFM95 chip select pin
const int RFM_RST = 9;          // RFM95 reset pin
const int RFM_IQR = 8;          // RFM95 IQR pin
byte localAddress = 0xAA;       // address of this device
byte destinationAddress = 0xBB; // destination to send to
byte msgCount = 0;              // count of out going messages
int count = 0;
long lastSendTime = 0; // last send time

// Ublox Neo M9N module
SFE_UBLOX_GNSS GNSS;
long lastTime = 0; // Simple local timer. Limits amount if I2C traffic to u-blox module.
const int ppsPin = 17;

#define LOG_RATE 5000
unsigned long lastLog = 0;
// SD card setup
#define SDCARD_SPI SPI1
#define PIN_SD_MOSI 15
#define PIN_SD_MISO 12
#define PIN_SD_SCK 14
#define PIN_SD_SS 9
// SD card setup
const int chipSelect = 7;
Adafruit_USBD_MSC usb_msc;
Sd2Card card;
RP2040_SdVolume volume;
File logFile;
bool fileCreated = false;
bool fileOpen = false;
// Log file format
char filename[] = "LOG000.CSV";

//====================================================================================
//                                    SD card callbacks
//====================================================================================

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and
// return number of copied bytes (must be multiple of block size)
int32_t msc_read_cb(uint32_t lba, void *buffer, uint32_t bufsize)
{
  (void)bufsize;
  return card.readBlock(lba, (uint8_t *)buffer) ? 512 : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and
// return number of written bytes (must be multiple of block size)
int32_t msc_write_cb(uint32_t lba, uint8_t *buffer, uint32_t bufsize)
{
  (void)bufsize;
  return card.writeBlock(lba, buffer) ? 512 : -1;
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
void msc_flush_cb(void)
{
  // nothing to do
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

  if (!card.init(SPI_FULL_SPEED, PIN_SD_SS))
  {
  }

  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card))
  {
    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
    while (1)
      delay(1);
  }

  uint32_t block_count = volume.blocksPerCluster() * volume.clusterCount();

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
  EEPROM.begin(256);
  // for (int i = 0; i < 512; i++)
  // {
  //   EEPROM.write(i, 0);
  // }
  int fileNum = EEPROM.read(0);
  Serial.println(fileNum);
  filename[3] = fileNum / 100 + '0';
  filename[4] = (fileNum % 100) / 10 + '0';
  filename[5] = fileNum % 10 + '0';
  if (!SD.exists(filename))
  {
    Serial.println("test");
    // only open a new file if it doesn't exist
    // generate a new file name
    filename[3] = fileNum / 100 + '0';
    filename[4] = (fileNum % 100) / 10 + '0';
    filename[5] = fileNum % 10 + '0';
    // create the new file
    RP2040_SDLib::File logFile = SD.open(filename, FILE_WRITE);
    logFile.print("Time UTC (H:M:S),Time Valid (0 = Invalid 1 = Valid),Longitude (DD°),Latitude (DD°),GPS Altitude (m),GPS Ground Speed (m/s),GPS Track Over Ground (deg°),Satellites In View, Fix Type (0 = No Fix 3 = 3D 4 = GNSS 5 = Time Fix), Primary Temperature (C°), Humidity (RH%), Altimeter Temperature (C°), Altitude Change (m), Battery Percentage, Battery Discharge Rate (%/h)");
    logFile.println();
    logFile.close();
    fileNum++;                 // increment the file number
    EEPROM.update(0, fileNum); // store the new file number in eeprom
    digitalWrite(LED_GREEN, HIGH);
    fileCreated = true;
  }
  else
  {
    vehicleState = 2;
    fileCreated = false;
  }
  EEPROM.end();
}

void logGPSData()
{
  // String dataString = "";
  // dataString += ((String(GNSS.getHour()) + ":" + String(GNSS.getMinute()) + ":" + String(GNSS.getSecond())));
  // dataString += ',';
  // dataString += String(GNSS.getTimeValid());
  // dataString += ',';
  // dataString += String((GNSS.getLongitude() * 1E-7), 6);
  // dataString += ',';
  // dataString += String((GNSS.getLatitude() * 1E-7), 6);
  // dataString += ',';
  // dataString += String((GNSS.getAltitude() * 1E-3), 1);
  // dataString += ',';
  // dataString += String((GNSS.getGroundSpeed() * 1E-3), 1);
  // dataString += ',';
  // dataString += String((GNSS.getHeading() * 1E-5), 1);
  // dataString += ',';
  // dataString += String(GNSS.getSIV());
  // dataString += ',';
  // dataString += String(GNSS.getFixType());
  // dataString += ',';
  // dataString += String(SHT30.getTemperature(), 2);
  // dataString += ',';
  // dataString += String(SHT30.getHumidity(), 2);
  // dataString += ',';
  // dataString += String(altimeter.getTemperature(), 2);
  // dataString += ',';
  // dataString += String(altimeter.altitudeChange(altimeter.getPressure(), startingPressure), 1);
  // dataString += ',';
  // dataString += String(lipo.getSOC(), 1);
  // dataString += ',';
  // dataString += String(lipo.getChangeRate(), 1);
  logFile.print(GNSS.getHour() + ":" + GNSS.getMinute() + ":" + GNSS.getSecond());
  logFile.print(',');
  logFile.print(GNSS.getTimeValid());
  logFile.print(',');
  logFile.print((GNSS.getLongitude() * 1E-7), 6);
  logFile.print(',');
  logFile.print(GNSS.getLatitude() * 1E-7), 6);
  logFile.print(',');
  dataString += String((GNSS.getAltitude() * 1E-3), 1);
  logFile.print(',');
  dataString += String((GNSS.getGroundSpeed() * 1E-3), 1);
  logFile.print(',');
  dataString += String((GNSS.getHeading() * 1E-5), 1);
  logFile.print(',');
  dataString += String(GNSS.getSIV());
  logFile.print(',');
  dataString += String(GNSS.getFixType());
  logFile.print(',');
  dataString += String(SHT30.getTemperature(), 2);
  logFile.print(',');
  dataString += String(SHT30.getHumidity(), 2);
  logFile.print(',');
  dataString += String(altimeter.getTemperature(), 2);
  logFile.print(',');
  dataString += String(altimeter.altitudeChange(altimeter.getPressure(), startingPressure), 1);
  logFile.print(',');
  dataString += String(lipo.getSOC(), 1);
  logFile.print(',');
  dataString += String(lipo.getChangeRate(), 1);
  logFile.print(dataString);
  logFile.println();
  logFile.flush();
}

void PowerDown()
{
  pinMode(powerBtnSense, OUTPUT);
  digitalWrite(powerBtnSense, LOW);
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
    start_usb_mass_storage();
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_RED, HIGH);
    while (1)
    {
    }
  }
  pinMode(powerBtnSense, INPUT_PULLUP);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  Serial.begin(9600);
  // while (!Serial)
  // {
  //   delay(10); // wait for native usb
  // }
  // I2C Initialization
  Wire1.setSDA(board_SDA);
  Wire1.setSCL(board_SCL);
  Wire1.begin();
  Wire1.setClock(400000); // Increase I2C clock speed to 400kHz

  // SPI Initialization
  SPI.setRX(board_SPI_RX);
  SPI.setTX(board_SPI_TX);
  SPI.setSCK(board_SPI_SCK);
  SPI.begin();
  SPI1.setRX(board_SPI1_RX);
  SPI1.setTX(board_SPI1_TX);
  SPI1.setSCK(board_SPI1_SCK);
  SPI1.begin();

  // LoRa Initialization
  // LoRa.setPins(RFM_CS, RFM_RST, RFM_IQR);
  // LoRa.setSPI(SPI);
  // while (!LoRa.begin(915E6))
  // {
  //   Serial.println("LoRa init falied !");
  //   delay(1000);
  // }

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
  }
  lipo.quickStart();
  lipo.setThreshold(20);

  // SHT30 Temperature and Humidity Sensor Initalization

  while (!SHT30.init(Wire1))
  {
    Serial.print("SHT30 error");
  }

  // GPS setup
  // myGNSS.enableDebugging(); // Uncomment this line to enable debug messages

  if (GNSS.begin(Wire1) == false) // Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  GNSS.setI2COutput(COM_TYPE_UBX);                 // Set the I2C port to output UBX only (turn off NMEA noise)
  GNSS.setNavigationFrequency(1);                  // Set output to 10 times a second
  GNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); // Save (only) the communications port settings to flash and BBR

  // Start SD card

  if (!SD.begin(PIN_SD_SS))
  {
    Serial.println("Initialization failed!");
    return;
  }
  Serial.println("Initialization done.");
  digitalWrite(LED_RED, LOW);
  rp2040.resumeOtherCore();
}

void setup1()
{
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
}

void loop()
{
  if (vehicleState == 0)
  {
    // Charging check for both button press
    if (digitalRead(powerBtnSense) == LOW && digitalRead(bootSelectButtonPin) == LOW)
    {
      delay(50); // debounce buttons
      if (digitalRead(powerBtnSense) == LOW && digitalRead(bootSelectButtonPin) == LOW)
      {
        vehicleState = 1;
      }
    }
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= 2000)
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
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED, LOW);
    }
  }
  else if (vehicleState == 1)
  {
    if (!fileCreated)
    {
      digitalWrite(LED_BLUE, LOW);
      createDataLoggingFile();
    }
    if (!fileOpen)
    {
      logFile = SD.open(filename, FILE_WRITE); // Open the log file
    }

    if (millis() - lastTime > 1000)
    {
      unsigned long timestamp = millis();
      logGPSData();
      lastTime = millis(); // Update the timer
      unsigned long Currenttimestamp = millis();
      Serial.println(Currenttimestamp - timestamp);
    }
  }
  else
  {
    // error light red led
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_BLUE, LOW);
    digitalWrite(LED_GREEN, LOW);
  }
}
