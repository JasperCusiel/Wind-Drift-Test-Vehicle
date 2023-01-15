/**
 * @file main.cpp
 * @author Jasper Cusiel (jaspercusiel@icloud.com)
 * @brief This code runs on the ground station. Fresh data is checked for on the LoRa radio, the data is saved to the SD card and then drawn on the tft display.
 * The program only runs on one core due to the SD and tft display sharing the same SPI bus. This was causing writes to the SD card to block updating the display
 * causing the buttons to change pages to appear to be frozen. It also caused artifacting on the display due to SD writes interupting display SPI transactions.
 */

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Adafruit_TinyUSB.h>
#include <LoRa.h>
#include <OneButton.h>
#include <SdFat.h>
#include <SdFatConfig.h>

// Include bitmap images for the display UI
#include "Hundredths Needle.h"
#include "Thousandths Needle.h"
#include "Altimeter Background.h"
#include "Data Transfer Icon.h"
#include "Data Receive Icon.h"
#include "Direction Arrow Icon.h"
#include "Data Receive Grey Icon.h"
#include "Data Transfer Grey Icon.h"
#include "GPS Fix Icon.h"
#include "GPS Fix Icon Grey.h"
// Splash Screen and USB data transfer page
#include "splash screen.h"
#include "Data Transfer Screen.h"
/*Include Font Files (these could be trimmed down to only include the characters used on the screen.
However, the preformance doesn't appear to be too severely effected even when loading and unloading the fonts with every character included)*/
#include "Roboto mono 8pt.h"
#include "Roboto mono 10pt.h"
#include "Roboto mono 17pt.h"
#include "Roboto mono 14pt.h"
#define FONT_8PT robotoMono8
#define FONT_10PT robotoMono10
#define FONT_17PT robotoMono17
#define FONT_14PT robotoMono14

// USB Mass Storage
Adafruit_USBD_MSC usb_msc;
// SD card SPI settings
#define SPI_CLOCK SD_SCK_MHZ(20)
#define SD_CONFIG SdSpiConfig(chipSelect, SHARED_SPI, SPI_CLOCK, &SPI1)
// SD card setup
const int chipSelect = 13;
SdFat sd;
SdFile dataFile;
bool fileCreated = false;

// LoRa Setup
const int csPin = 1;
const int resetPin = 0;
const int irqPin = 6;
char message[100];

// Buttons
const int upButtonPin = 24;
const int downButtonPin = 25;
const int leftButtonPin = 28;
const int rightButtonPin = 29;
OneButton upButton(upButtonPin);
OneButton downButton(downButtonPin);
OneButton leftButton(leftButtonPin);
OneButton rightButton(rightButtonPin);

// Variables (excuse the poor naming here with the test-varibale convention)
int rssi = 0;
float batteryPercentage = 0.0;
float vehicleHeading = 0.0;
float testLat = 00.000000;
float testLong = 00.000000;
float passLat = 00.000000;  // To store the last valid latitude to show on the display
float passLong = 00.000000; // To store the last valid longitude
bool testSend = false;
bool testReceive = false;
int testHeading = 0;
int testAltitude = 0;
int testTemp = 0;
int testHumdity = 0;
int testGroundSpeed = 0;
int testGpsFixType = 0;
bool currentlyConnected = false;
unsigned long lastRssiCheck = 0;
const int TIMEOUT = 10000; // Assume LoRa is dissconnected if a packet hasn't been received 10 seconds after the last packet
char newFileName[13];
int mode = 1; // Used to keep track of which page to draw, 0 = altimeter page, 1 = GPS coordinate page

// TFT e_SPI Instance
TFT_eSPI tft = TFT_eSPI(128, 160);
TFT_eSprite page = TFT_eSprite(&tft);             // Create altimeter background sprite
TFT_eSprite needleThousandth = TFT_eSprite(&tft); // Create hundredth needle sprite
TFT_eSprite needleHundredth = TFT_eSprite(&tft);  // Create thousandth needle sprite
TFT_eSprite statusBar = TFT_eSprite(&tft);        // Create status bar sprite
TFT_eSprite dataPanel = TFT_eSprite(&tft);        // Create sprite for temp, humidity and direction data
TFT_eSprite arrow = TFT_eSprite(&tft);            // Create direction arrow sprite
TFT_eSprite messages = TFT_eSprite(&tft);         // Create sprite to display system messages

//====================================================================================
//                                 Create Sprites
//====================================================================================

void createPageSprite() // Create sprite the size of a page where either the altimeter of GPS coordinate UI can be pushed
{
    page.setColorDepth(8);
    page.createSprite(108, 107);
    page.setPivot(52, 54);
    page.fillSprite(TFT_TRANSPARENT);
}

void createNeedleHundredth() // Create a needlehundredth's sprite and push the hundredths needle bitmap image into the needleHundredth's sprite
{
    needleHundredth.setColorDepth(8);
    needleHundredth.createSprite(8, 64);
    needleHundredth.fillSprite(TFT_TRANSPARENT);
    needleHundredth.setPivot(4, 42);
    needleHundredth.pushImage(0, 0, 8, 64, altHandHundredths);
}

void createNeedleThousandth() // Create a needleThousandth's sprite and push the thousandths needle bitmap image into the needleThousandth's sprite
{
    needleThousandth.setColorDepth(8);
    needleThousandth.createSprite(12, 44);
    needleThousandth.fillSprite(TFT_TRANSPARENT);
    needleThousandth.setPivot(6, 30);
    needleThousandth.pushImage(0, 0, 12, 44, altHandThousandths);
}

void createDirectionArrow() // Create a sprite for the track over ground direction arrow and push bitmap image into sprite
{
    arrow.setColorDepth(8);
    arrow.createSprite(30, 30);
    arrow.fillSprite(TFT_TRANSPARENT);
    arrow.pushImage(0, 0, 24, 24, directionArrow);
    arrow.setPivot(12, 12);
}

void createMessages() // Create a sprite to hold the system messages (LoRa connection status)
{
    messages.setColorDepth(8);
    messages.createSprite(90, 20);
    messages.fillSprite(TFT_BLACK);
}

void createStatusBar() // Create a sprite to hold all of the icons
{
    statusBar.setColorDepth(8);
    statusBar.createSprite(66, 20, TFT_TRANSPARENT);
    statusBar.fillSprite(TFT_BLACK);
}

void createDataPanel() // Create sprite to hold the data that is always displayed on the right side (track over ground, speed, temp, humidty)
{
    dataPanel.setColorDepth(8);
    dataPanel.createSprite(50, 108, TFT_TRANSPARENT);
    dataPanel.fillSprite(TFT_BLACK);
    dataPanel.setPivot(25, 24);
}

//====================================================================================
//                             Draw Sprite Graphics
//====================================================================================

void drawAltimeter(int thousandthsAngle, int hundredthsAngle)
{
    /**
     * @brief Takes needle angles and draws on altimeter background
     * @param thousandthsAngle angle of the thousandths indicator needle
     * @param hundredthsAngle angle of the hundredths indicator needle
     */
    page.pushImage(0, 0, 108, 108, altimeterBackgroundImg);
    needleThousandth.pushRotated(&page, thousandthsAngle, TFT_TRANSPARENT);
    needleHundredth.pushRotated(&page, hundredthsAngle, TFT_TRANSPARENT);
    page.pushSprite(0, 21, TFT_TRANSPARENT);
}

void drawStatusBar(float percentage, int rssi, bool transmitting, bool receiving, bool isConnected, int fixType)
{
    /**
     * @brief Draws the signal strength, battery, transmitting/receiving indicaors aswell as the satellite icon to indicate a valid fix or not
     * @param percentage battery percentage as int value
     * @param rssi LoRa radio signal strength as RSSI value (0 -> -120 range)
     * @param transmitting true if LoRa radio is transmittingd ata from vehicle
     * @param receiving true if LoRa radio is receiving data from vehicle
     * @param isConnected true if the last packet was received within 10 seconds of the previous packet, i.e. hasn't timed out
     * @param fixType the current GPS fix type
     */

    // Draw RSSI
    const int sigX = 0;
    const int sigY = 5;
    int strength = map(rssi, -120, -75, 0, 4); // Map RSSI value to number of bars to show
    unsigned short barZero = TFT_DARKGREY;
    unsigned short barOne = TFT_DARKGREY;
    unsigned short barTwo = TFT_DARKGREY;
    unsigned short barThree = TFT_DARKGREY;
    if (strength >= 3)
    {
        barThree = TFT_WHITE;
    }
    if (strength >= 2)
    {
        barTwo = TFT_WHITE;
    }
    if (strength >= 1)
    {
        barOne = TFT_WHITE;
    }
    if (strength >= 0)
    {
        barZero = TFT_WHITE;
    }
    if (!isConnected) // If timed out, dont colour any bars indicating no signal
    {
        barZero = TFT_DARKGREY;
        barOne = TFT_DARKGREY;
        barTwo = TFT_DARKGREY;
        barThree = TFT_DARKGREY;
    }
    // Draw signal strength into statusBar sprite
    statusBar.fillSprite(TFT_BLACK);
    statusBar.fillSmoothRoundRect((sigX + 12), sigY, 2, 10, 1, barThree);
    statusBar.fillSmoothRoundRect((sigX + 8), (sigY + 2), 2, 8, 1, barTwo);
    statusBar.fillSmoothRoundRect((sigX + 4), (sigY + 4), 2, 6, 1, barOne);
    statusBar.fillSmoothRoundRect(sigX, (sigY + 6), 2, 4, 1, barZero);

    // Draw data transfer symbol
    const int datX = 16;
    const int datY = 5;
    statusBar.pushImage(datX, datY, 5, 10, dataTransmitGrey);
    statusBar.pushImage((datX + 5), datY, 5, 10, dataReceiveGrey);
    if (transmitting == true)
    {
        statusBar.pushImage(datX, datY, 5, 10, dataTransmit);
    }
    if (receiving == true)
    {
        statusBar.pushImage((datX + 5), datY, 5, 10, dataReceive);
    }

    // Draw battery icon
    const int batX = 42;
    const int batY = 5;
    unsigned short batColour = tft.color565(53, 191, 25);
    int recWidth = map(round(percentage), 0, 100, 1, 14);

    if (percentage <= 15) // Change battery colour to red if battery bellow 10%
    {
        batColour = tft.color565(219, 37, 24);
    }
    else if (percentage <= 25) // Change battery colour to yellow if battery bellow 25%
    {
        batColour = tft.color565(227, 217, 30);
    }
    statusBar.fillSmoothRoundRect((batX + 2), (batY + 2), recWidth, 6, 1, batColour);
    statusBar.drawRoundRect(batX, batY, 18, 10, 1, TFT_WHITE);
    statusBar.fillSmoothRoundRect((batX + 18), (batY + 2), 2, 6, 2, TFT_WHITE);

    // Draw gps fix symbol
    if ((fixType == 2) || (fixType == 3)) // 2 = 2D fix, 3 = 3D fix
    {
        statusBar.pushImage(29, 5, 10, 10, gpsFix);
    }
    else
    {
        statusBar.pushImage(29, 5, 10, 10, gpsNoFix);
    }
    // Draw status bar on tft display
    statusBar.pushSprite(94, 0, TFT_TRANSPARENT);
}

void drawDataPanel(float heading, float temp, float humidity, float speed)
{
    /**
     * @brief This function draws the data down on the right hand side of the screen
     * @param heading the vehicles track over ground heading in degrees
     * @param temp the temperature from the external temperature sensor
     * @param humidity the humidity from the external humidity sensor
     * @param speed the vehicle ground speed
     */

    dataPanel.fillSprite(TFT_BLACK);
    const int panelHeight = 108;
    const int panelWidth = 50;
    char direction[3];
    // Changes heading angle to N, NE, E, SE, S, SW, W, NW
    if (((heading > 337.5) && (heading <= 360)) || ((heading >= 0) && (heading <= 22.5)))
    {
        strcpy(direction, "N ");
    }
    else if (((heading > 22.5) && (heading <= 67.5)))
    {
        strcpy(direction, "NE");
    }
    else if (((heading > 67.5) && (heading <= 112.5)))
    {
        strcpy(direction, "E ");
    }
    else if (((heading > 112.5) && (heading <= 157.5)))
    {
        strcpy(direction, "SE");
    }
    else if (((heading > 157.5) && (heading <= 202.5)))
    {
        strcpy(direction, "S ");
    }
    else if (((heading > 202.5) && (heading <= 247.5)))
    {
        strcpy(direction, "SW");
    }
    else if (((heading > 247.5) && (heading <= 292.5)))
    {
        strcpy(direction, "W ");
    }
    else if (((heading > 292.5) && (heading <= 337.5)))
    {
        strcpy(direction, "NW");
    }

    // Change the temperature text colour to reflect
    unsigned char r, g, b;
    if ((temp <= 20) && (temp >= 0))
    {
        r = map(temp, 0, 20, 0, 255);
        g = map(temp, 0, 20, 255, 0);
        b = map(temp, 0, 20, 255, 0);
    }
    else if (temp > 20)
    {
        r = 255;
        g = 0;
        b = 0;
    }
    else if (temp < 0)
    {
        r = 0;
        g = 255;
        b = 255;
    }

    // Draw panel lines
    dataPanel.drawLine(0, 0, 0, panelHeight, TFT_WHITE);
    dataPanel.drawLine(0, 54, panelWidth, 54, TFT_WHITE);
    dataPanel.drawLine(0, 82, panelWidth, 82, TFT_WHITE);
    dataPanel.drawLine(0, 54, panelWidth, 54, TFT_WHITE);
    dataPanel.drawLine(0, 0, panelWidth, 0, TFT_WHITE);
    // Draw data on panel
    dataPanel.setTextDatum(MC_DATUM);
    dataPanel.loadFont(FONT_10PT);
    dataPanel.setTextColor(TFT_WHITE, TFT_BLACK);
    dataPanel.drawFloat(speed, 1, 14, 48);
    dataPanel.drawString(direction, 8, 7);
    dataPanel.unloadFont();
    dataPanel.loadFont(FONT_8PT);
    dataPanel.drawString("KNTS", 39, 49);
    dataPanel.unloadFont();
    dataPanel.loadFont(FONT_17PT);
    dataPanel.setTextColor(TFT_CYAN, TFT_BLACK);
    dataPanel.drawString((String(humidity, 0) + "%"), 25, 98);
    dataPanel.setTextColor(tft.color565(r, g, b), TFT_BLACK);
    dataPanel.drawString((String(temp, 0) + "C°"), 25, 70);
    dataPanel.unloadFont();

    arrow.pushRotated(&dataPanel, heading, TFT_TRANSPARENT); // Draw arrow icon at heading angle
    dataPanel.pushSprite(108, 20);                           // Push sprite to display
}

void drawMessages(bool connected)
{
    /**
     * @brief This function draws text to indicate if the LoRa radio is connected
     * @param connected the current connection state of the vehicle, true = connected, false = LoRa radio disconnected
     */
    messages.fillSprite(TFT_BLACK);
    messages.loadFont(FONT_10PT);
    if (connected)
    {
        messages.setTextColor(TFT_GREEN);
        messages.drawString("LoRa Connected", 4, 6, 1);
    }
    else
    {
        messages.setTextColor(TFT_RED);
        messages.drawString("Disconnected", 4, 6, 1);
    }
    messages.unloadFont();
    messages.pushSprite(0, 0);
}

//====================================================================================
//                                    Pages
//====================================================================================

void drawLayout(int batteryPercentage, int rssi, bool transmitting, bool receiving, int heading, float velocity, int temp, int humidity, bool isConnected, int gnssFix)
{
    /**
     * @brief Draws the base UI --> status bar, data panel and message bar
     * @param batteryPercentage vehicle battery percentage to be displayed
     * @param rssi LoRa signal strength as RSSI value
     * @param transmitting True if LoRa radio is transmittingd ata from vehicle
     * @param receiving True if LoRa radio is receiving data from vehicle
     * @param heading Vehicle heading in degrees
     * @param velocity Vehicle velocity
     * @param temp SHT30 external sensor temp in degrees
     * @param humidity SHT30 humidity in RH (relative humidity)
     * @param isConnected if the vehicle is currently connected
     * @param gnssFix the current GPS fix type
     */
    drawStatusBar(batteryPercentage, rssi, transmitting, receiving, isConnected, gnssFix);
    drawDataPanel(heading, temp, humidity, velocity);
    drawMessages(isConnected);
    tft.drawLine(0, 20, 108, 20, TFT_WHITE);
}

void drawAltimeterPage(int alt)
{
    /**
     * @brief Converts altitude to needle positions and calls drawAltimeter function
     * @param alt current vehicle altitude (MS5637 altimeter)
     */
    int thousandths = (alt / 1000U) % 10;
    int altitudeHundredths = alt - (thousandths * 1000);
    int angleMapHundredths = map(altitudeHundredths, 0, 1000, 0, 360);
    int angleMapThousandths = map(alt, 0, 10000, 0, 360);
    page.fillSprite(TFT_BLACK);
    drawAltimeter(angleMapThousandths, angleMapHundredths);
}

void drawGpsPage(float latitude, float longitude)
{
    /**
     * @brief This function draws the GPS coordinates on the screen
     * @param latitude the vehicles latitude
     * @param longitude the vehicles longitude
     */

    // Converting the lat and long to degree minute second format
    int latDegrees = latitude;
    int latMin = (latitude - latDegrees) * 60;
    float latSec = (((latitude - latDegrees) * 60) - latMin) * 60;
    char latitudeDMS[16];
    char latSec1DP[5];
    dtostrf(abs(latSec), 4, 1, latSec1DP);
    sprintf(latitudeDMS, "%d° %d' %s\"", abs(latDegrees), abs(latMin), latSec1DP);

    int longDegrees = longitude;
    int longMin = (longitude - longDegrees) * 60;
    float longSec = (((longitude - longDegrees) * 60) - longMin) * 60;
    char longitudeDMS[16];
    char longSec1DP[5];
    dtostrf(abs(longSec), 4, 1, longSec1DP);
    sprintf(longitudeDMS, "%d° %d' %s\"", abs(longDegrees), abs(longMin), longSec1DP);

    page.fillSprite(TFT_BLACK);
    page.setTextDatum(MC_DATUM);
    page.setTextColor(TFT_WHITE);
    page.loadFont(FONT_10PT);
    page.drawString("DECIMAL DEGREES", 54, 5);
    page.drawString("DEGREES MIN SEC", 54, 60);

    page.unloadFont();
    page.drawLine(0, 9, 108, 9, TFT_WHITE);
    page.drawLine(0, 54, 108, 54, TFT_WHITE);
    page.drawLine(0, 64, 108, 64, TFT_WHITE);
    page.loadFont(FONT_17PT);
    page.drawFloat(latitude, 5, 54, 23);
    page.drawFloat(longitude, 5, 54, 41);
    page.unloadFont();
    page.loadFont(FONT_14PT);
    page.drawString(latitudeDMS, 54, 78);
    page.drawString(longitudeDMS, 54, 94);
    page.unloadFont();
    page.pushSprite(0, 21);
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

void start_usb_mass_storage()
{
    /**
     * @brief This function starts the ground station as a USB thumb drive so that the connected computer can access the contents on the SD card.
     * The up button is held on powerup to get into this mode. The ground station must be power cycled to boot back into the normal mode
     */

    usb_msc.setID("Adafruit", "SD Card", "1.0");
    usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
    usb_msc.setUnitReady(false); // Still initialize MSC but tell usb stack that MSC is not ready to read/write
    usb_msc.begin();             // If we don't initialize, board will be enumerated as CDC only

    Serial.begin(9600); // Start serial for debug messages
    tft.begin();        // Start tft to display transfer screen
    tft.setRotation(1);
    tft.setSwapBytes(true);
    tft.pushImage(0, 0, 160, 128, dataTransferScreen);
    tft.endWrite(); // Stop tft to avoid conflict between SD and tft, image remains on screen until power is removed or new image pushed to screen

    Serial.print("\nInitializing SD card ... ");
    Serial.print("CS = ");
    Serial.println(chipSelect);

    if (!sd.begin(SD_CONFIG))
    {
        Serial.println("Sd initialization failed.");
        return;
    }

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
            dataFile.print("Time UTC (H:M:S), Fix Type (0 = No 2 = 2D Fix 3 = 3D 4 = GNSS 5 = Time Fix), Latitude (DD°),Longitude (DD°), Altitude (m), GPS Ground Speed (m/s), GPS Track Over Ground (deg°), Primary Temperature (C°), Humidity (RH%), Battery Percentage (%)");
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
// this function is called when the up button is clicked:
void upButtonClicked()
{
    // change the mode to 2:
    if (mode == 1)
    {
        mode = 0;
    }
    else
    {
        mode = 1;
    }
}

// this function is called when the down button is clicked:
void downButtonClicked()
{
    // change the mode to 2:
    if (mode == 0)
    {
        mode = 1;
    }
    else
    {
        mode = 0;
    }
}

//====================================================================================
//                                    Setup
//====================================================================================
void setup()
{
    rp2040.idleOtherCore();
    pinMode(upButtonPin, INPUT_PULLUP);
    int upButtonReading = digitalRead(upButtonPin);

    if (upButtonReading == LOW)
    {
        start_usb_mass_storage();
        while (1)
        {
        }
    }
    rp2040.resumeOtherCore();
    // start sd and tft on core 0 as they share the SPI1 bus
    if (!sd.begin(SD_CONFIG))
    {
        Serial.println("initialization failed. Things to check:");
        Serial.println("* is a card inserted?");
        Serial.println("* is your wiring correct?");
        Serial.println("* did you change the chipSelect pin to match your shield or module?");
        while (1)
        {
        }
    }
    tft.begin();
    tft.setRotation(1);
    tft.setSwapBytes(true);
    tft.pushImage(0, 0, 160, 128, splashImg);
    // Create Sprites
    createPageSprite();
    createNeedleHundredth();
    createNeedleThousandth();
    createStatusBar();
    createMessages();
    createDirectionArrow();
    createDataPanel();
    // Create data logging file
    fileCreated = createDataLoggingFile();

    if (!fileCreated)
    {
        Serial.println("datalogging file failed");
        return;
    }
    // draw home page
    tft.fillScreen(TFT_BLACK);
}
void setup1()
{
    LoRa.setPins(csPin, resetPin, irqPin);
    if (!LoRa.begin(915E6))
    {
        Serial.println("LoRa init failed. Check your connections.");
        while (true)
        {
            delay(10);
        }
    }

    // Setup Physical buttons
    // attach the up button:
    upButton.attachClick(upButtonClicked);
    // attach the down button:
    downButton.attachClick(downButtonClicked);
}

//====================================================================================
//                                    Loop
//====================================================================================
void loop()
{
    int packetSize = LoRa.parsePacket();
    if (packetSize)
    {
        lastRssiCheck = millis();
        // Read and parse the message
        // Read the message and store it in the character array
        for (int i = 0; i < packetSize; i++)
        {
            message[i] = (char)LoRa.read();
        }
        // Add a null terminator to the end of the message
        message[packetSize] = '\0';
        // Log sensor data to SD card
        rssi = LoRa.packetRssi();
        currentlyConnected = true;
        lastRssiCheck = millis();
        noInterrupts();
        dataFile.println(message);
        if (!dataFile.sync())
        {
            Serial.println("error writing to sd");
            return;
        }
        interrupts();
        // Serial.println(message);
        float groundSpeed, trackOverGround, temp, humidity, bat, latitude, longitude, altitude;
        ;
        int hour, min, sec, millisec, gpsFixVal;

        sscanf(message, "%d:%d:%d.%d,%d,%f,%f,%f,%f,%f,%f,%f,%f", &hour, &min, &sec, &millisec, &gpsFixVal, &latitude, &longitude, &altitude, &groundSpeed, &trackOverGround, &temp, &humidity, &bat);

        testLat = latitude;
        testLong = longitude;
        testAltitude = int((altitude * 3.281)); // convert to feet
        testTemp = temp;
        testHumdity = humidity;
        vehicleHeading = trackOverGround;
        batteryPercentage = bat;
        testGroundSpeed = (groundSpeed * 1.943844); // convert to knots
        testGpsFixType = gpsFixVal;
        testReceive = true;
        Serial.println(message);
    }
    if (millis() - lastRssiCheck > TIMEOUT)
    {
        // if no new RSSI value has been received for TIMEOUT seconds
        Serial.println("No signal received");
        lastRssiCheck = millis();
        currentlyConnected = false;
    }
    // update the state of the up button:
    upButton.tick();
    // update the state of the down button:
    downButton.tick();
    tft.startWrite();
    drawLayout(batteryPercentage, rssi, testSend, testReceive, vehicleHeading, testGroundSpeed, testTemp, testHumdity, currentlyConnected, testGpsFixType);
    // use a switch statement to check the value of the mode variable:
    switch (mode)
    {
    // if the mode value is 1, do something:
    case 0:
        drawAltimeterPage(testAltitude);
        break;

    // if the mode value is 2, do something else:
    case 1:
        if ((testGpsFixType == 2) || (testGpsFixType == 3))
        {
            passLat = testLat;
            passLong = testLong;
        }
        drawGpsPage(passLat, passLong);
        break;

    // if the mode value is not covered by the case statements, do something else:
    default:
        // insert your code here to handle other mode values
        break;
    }
    tft.endWrite();
    testReceive = false;
}