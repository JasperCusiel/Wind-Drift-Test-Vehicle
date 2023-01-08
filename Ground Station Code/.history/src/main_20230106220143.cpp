#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <RP2040_SD.h>
#include <Adafruit_TinyUSB.h>
#include <EEPROM.h>
#include <LoRa.h>
#include <OneButton.h>

// Include Bitmap Images
#include "Hundredths Needle.h"
#include "Thousandths Needle.h"
#include "Altimeter Background.h"
#include "Data Transfer Icon.h"
#include "Data Receive Icon.h"
#include "Direction Arrow Icon.h"
#include "Data Receive Grey Icon.h"
#include "Data Transfer Grey Icon.h"
#include "Export Data Icon.h"
#include "Usb to GX12 Icon.h"
#include "Caution Icon.h"
#include "File Transfer Icon.h"
#include "Data Transfer Arrow.h"
// Splash Screen
#include "splash screen.h"
// Include Font Files
#include "Roboto mono 8pt.h"
#include "Roboto mono 10pt.h"
#include "Roboto mono 17pt.h"
#include "Roboto mono 14pt.h"
#define FONT_8PT robotoMono8
#define FONT_10PT robotoMono10
#define FONT_17PT robotoMono17
#define FONT_14PT robotoMono14

// SD card setup
const int chipSelect = 13;
Adafruit_USBD_MSC usb_msc;
Sd2Card card;
RP2040_SdVolume volume;
// Log file format
char filename[] = "LOG000.CSV";

bool okayToWrite = true;

// LoRa Setup
const int csPin = 1;
const int resetPin = 0;
const int irqPin = 6;

// Buttons
const int upButtonPin = 24;
const int downButtonPin = 25;
const int leftButtonPin = 28;
const int rightButtonPin = 29;
OneButton upButton(upButtonPin);       // create an up button object
OneButton downButton(downButtonPin);   // create a down button object
OneButton leftButton(leftButtonPin);   // create an up button object
OneButton rightButton(rightButtonPin); // create a down button object

// Variables
int rssi = 0;
float batteryPercentage = 100.0;
float vehicleHeading = 0.0;
int count = 0;
int tick = 0;
int pageNum = 0;
float testLat = -90.0000;
float testLong = -180.0000;
bool testSend = false;
bool testReceive = true;
int testHeading = 0;
int testAltitude = 0;

// data transfer page variables
int transferDataState;
int transferArrowXVal = 0;

// Button Variables
int upButtonState;
int upButtonLastState = HIGH;
unsigned long lastDebounceTimeUpButton = 0;

int downButtonState;
int downButtonLastState = HIGH;
unsigned long lastDebounceTimeDownBtn = 0;

int leftButtonState;
int leftButtonLastState = HIGH;
unsigned long lastDebounceTimeLeftBtn = 0;
bool leftButtonPressed;

int rightButtonState;
int rightButtonLastState = HIGH;
unsigned long lastDebounceTimeRightBtn = 0;
bool rightButtonPressed;

unsigned long debounceDelay = 50;

int mode = 0;

// TFT e_SPI Instance
TFT_eSPI tft = TFT_eSPI(128, 160);
TFT_eSprite page = TFT_eSprite(&tft);             // create altimeter background sprite
TFT_eSprite needleThousandth = TFT_eSprite(&tft); // create hundredth needle sprite
TFT_eSprite needleHundredth = TFT_eSprite(&tft);  // create thousandth needle sprite
TFT_eSprite statusBar = TFT_eSprite(&tft);        // create status bar sprite
TFT_eSprite dataPanel = TFT_eSprite(&tft);        // create sprite for temp, humidity and direction data
TFT_eSprite arrow = TFT_eSprite(&tft);            // create direction arrow sprite
TFT_eSprite messages = TFT_eSprite(&tft);         // create sprite to display system messages

//====================================================================================
//                                 Create Sprites
//====================================================================================
void createPageSprite()
{
    page.setColorDepth(8);
    page.createSprite(108, 107);
    page.setPivot(52, 54);
    page.fillSprite(TFT_TRANSPARENT);
}
void createNeedleHundredth()
{
    needleHundredth.setColorDepth(8);
    needleHundredth.createSprite(8, 64);
    needleHundredth.fillSprite(TFT_TRANSPARENT);
    needleHundredth.setPivot(4, 42);
    needleHundredth.pushImage(0, 0, 8, 64, altHandHundredths);
}
void createNeedleThousandth()
{
    needleThousandth.setColorDepth(8);
    needleThousandth.createSprite(12, 44);
    needleThousandth.fillSprite(TFT_TRANSPARENT);
    needleThousandth.setPivot(6, 30);
    needleThousandth.pushImage(0, 0, 12, 44, altHandThousandths);
}
void createDirectionArrow()
{
    arrow.setColorDepth(8);
    arrow.createSprite(30, 30);
    arrow.fillSprite(TFT_TRANSPARENT);
    arrow.pushImage(0, 0, 24, 24, directionArrow);
    arrow.setPivot(12, 12);
}
void createMessages()
{
    messages.setColorDepth(8);
    messages.createSprite(100, 37);
    messages.fillSprite(TFT_BLACK);
    messages.setScrollRect(0, 0, 100, 37, TFT_BLACK);
}
void createStatusBar()
{
    statusBar.setColorDepth(8);
    statusBar.createSprite(56, 20, TFT_TRANSPARENT);
    statusBar.fillSprite(TFT_BLACK);
}
void createDataPanel()
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

void drawStatusBar(float percentage, int rssi, bool transmitting, bool receiving)
{
    /**
     * @brief Draws the signal strength, battery and transmitting/receiving indicaors
     * @param percentage battery percentage as int value
     * @param rssi LoRa radio signal strength as RSSI value (0 -> -120 range)
     * @param transmitting True if LoRa radio is transmittingd ata from vehicle
     * @param receiving True if LoRa radio is receiving data from vehicle
     *
     */

    // draw RSSI
    const int sigX = 14;
    const int sigY = 4;
    int strength = map(rssi, -120, 0, 0, 4);
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
    statusBar.fillSprite(TFT_BLACK);
    statusBar.fillSmoothRoundRect((sigX + 12), sigY, 2, 10, 1, barThree);
    statusBar.fillSmoothRoundRect((sigX + 8), (sigY + 2), 2, 8, 1, barTwo);
    statusBar.fillSmoothRoundRect((sigX + 4), (sigY + 4), 2, 6, 1, barOne);
    statusBar.fillSmoothRoundRect(sigX, (sigY + 6), 2, 4, 1, barZero);

    // draw data transfer symbol
    const int datX = 0;
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
    const int batX = 32; // battery icon x, y coordinate (top left)
    const int batY = 4;
    unsigned short batColour = tft.color565(53, 191, 25);
    int recWidth = map(round(percentage), 0, 100, 1, 14);

    if (percentage <= 15) // change battery colour to red if battery bellow 10%
    {
        batColour = tft.color565(219, 37, 24);
    }
    else if (percentage <= 25) // change battery colour to yellow if battery bellow 25%
    {
        batColour = tft.color565(227, 217, 30);
    }
    statusBar.fillSmoothRoundRect((batX + 2), (batY + 2), recWidth, 6, 1, batColour);
    statusBar.drawRoundRect(batX, batY, 18, 10, 1, TFT_WHITE);
    statusBar.fillSmoothRoundRect((batX + 18), (batY + 2), 2, 6, 2, TFT_WHITE);
    // Draw status bar on screen
    statusBar.pushSprite(104, 0, TFT_TRANSPARENT);
}

void drawDataPanel(float heading, float temp, float humidity, float speed)
{
    /**
     * @brief
     *
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

    // temp to colour
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

    // draw panel lines
    dataPanel.drawLine(0, 0, 0, panelHeight, TFT_WHITE);
    dataPanel.drawLine(0, 54, panelWidth, 54, TFT_WHITE);
    dataPanel.drawLine(0, 82, panelWidth, 82, TFT_WHITE);
    dataPanel.drawLine(0, 54, panelWidth, 54, TFT_WHITE);
    dataPanel.drawLine(0, 0, panelWidth, 0, TFT_WHITE);
    // draw data
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
    // dataPanel.drawCircle(43, 60, 2, tft.color565(r, g, b));
    dataPanel.unloadFont();
    // Draw arrow icon at heading angle
    arrow.pushRotated(&dataPanel, heading, TFT_TRANSPARENT);

    dataPanel.pushSprite(108, 20);
}

void drawMessages()
{ // max message length = 17 characters
    messages.scroll(0, 1);
    messages.loadFont(FONT_10PT);
    count++;
    if (count == 14)
    {
        count = 0;
        if (tick == 1)
        {
            tick = 0;
            messages.setTextColor(TFT_GREEN);
            messages.drawString("Connected", 0, 0, 1);
        }
        else
        {
            tick = 1;
            messages.setTextColor(TFT_RED);
            messages.drawString("Disconnected", 0, 0, 1);
        }
    }
    messages.pushSprite(0, -20);
    messages.unloadFont();
}

//====================================================================================
//                                    Pages
//====================================================================================

void drawLayout(int batteryPercentage, int rssi, bool transmitting, bool receiving, int heading, float velocity, int temp, int humidity)
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
     * @param humidity SHT30 humidity in RH
     */
    drawStatusBar(batteryPercentage, rssi, transmitting, receiving);
    drawDataPanel(heading, temp, humidity, velocity);
    drawMessages();
    tft.drawLine(0, 20, 108, 20, TFT_WHITE);
}

void drawAltimeterPage(int alt)
{
    /**
     * @brief converst altitude to needle positions and calls drawAltimeter function
     * @param alt vehicle current altitude
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

// void drawDataTransferPage(int transferState)
// {
//     int moveButtonPositionX = 0;
//     page.fillSprite(TFT_BLACK);

//     page.drawLine(0, 9, 108, 9, TFT_WHITE); // divider lines
//     page.drawLine(0, 70, 108, 70, TFT_WHITE);

//     page.loadFont(FONT_10PT); // text
//     page.setTextDatum(MC_DATUM);
//     page.setTextColor(TFT_WHITE);
//     page.drawString("USB DATA TRANSFER", 54, 5);
//     page.setTextColor(TFT_LIGHTGREY);

//     if (transferState == 0)
//     {
//         page.drawString("Start Transfer?", 54, 78);
//         page.pushImage(4, 18, 100, 24, downloadIcon); // file to computer icon
//         page.pushImage(4, 50, 100, 14, usbGx12Icon);  // gx12 to usb cable icon
//         page.setTextColor(TFT_GREEN);
//         page.drawString("YES", 90, 95);
//     }
//     else if (transferState == 1)
//     {
//         page.pushImage(37, 15, 40, 50, cautionIcon);
//         page.drawString("Are You Sure?", 54, 78);
//         page.setTextColor(TFT_RED);
//         page.drawString("NO", 20, 95);
//         page.setTextColor(TFT_GREEN);
//         page.drawString("YES", 90, 95);
//     }
//     else if (transferState == 2)
//     {
//         page.drawString("Transfer Mode", 54, 78);
//         page.pushImage(4, 30, 100, 24, fileTransfer);
//         page.setTextColor(TFT_RED);
//         page.drawString("CANCEL", 30, 95);
//         moveButtonPositionX += 20;
//         if (transferArrowXVal < 20)
//         {
//             transferArrowXVal++;
//         }
//         else
//         {
//             transferArrowXVal = 0;
//         }
//         page.pushImage((28 + transferArrowXVal), 36, 13, 10, transferArrow);
//     }
//     page.unloadFont();
//     page.fillSmoothRoundRect((31 + moveButtonPositionX), 88, 46, 13, 6, TFT_LIGHTGREY); // button icons
//     page.fillRect((50 + moveButtonPositionX), 88, 8, 13, TFT_BLACK);
//     page.fillSmoothCircle((37 + moveButtonPositionX), 94, 4, TFT_BLACK);
//     page.fillSmoothCircle((37 + moveButtonPositionX), 94, 2, TFT_LIGHTGREY);
//     page.fillSmoothRoundRect((68 + moveButtonPositionX), 90, 2, 9, 2, TFT_BLACK);

//     page.pushSprite(0, 21);
// }
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
    SPI1.setTX(11);
    SPI1.setRX(12);
    SPI1.setSCK(10);
    SPI1.begin();
    // while (!Serial)
    // {
    //     delay(10); // wait for native usb
    // }

    Serial.println("Adafruit TinyUSB Mass Storage SD Card example");

    Serial.println("\nInitializing SD card...");

    if (!card.init(SPI_HALF_SPEED, chipSelect))
    {
        Serial.println("initialization failed. Things to check:");
        Serial.println("* is a card inserted?");
        Serial.println("* is your wiring correct?");
        Serial.println("* did you change the chipSelect pin to match your shield or module?");
        while (1)
            delay(1);
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
    usb_msc.setStartStopCallback(startNorm);
}

//====================================================================================
//                                  Data Logging
//====================================================================================

void createDataLoggingFile()
{
    EEPROM.begin(512);
    // for (int i = 0; i < 512; i++)
    // {
    //     EEPROM.write(i, 0);
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
        RP2040_SDLib::File logfile = SD.open(filename, FILE_WRITE);
        fileNum++;                 // increment the file number
        EEPROM.update(0, fileNum); // store the new file number in eeprom
    }
    EEPROM.end();
}
// this function is called when the up button is clicked:
void upButtonClicked()
{
    // change the mode to 2:
    mode = 1;
}

// this function is called when the down button is clicked:
void downButtonClicked()
{
    // change the mode to 1:
    mode = 0;
}
void startNorm()
{
    okayToWrite = false;
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
        while (okayToWrite)
        {
        }
    }
    Serial.begin(9600);
    // while (!Serial)
    // {
    //     delay(10);
    // }
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
    // Setup Physical buttons
    // attach the up button:
    upButton.attachClick(upButtonClicked);
    // attach the down button:
    downButton.attachClick(downButtonClicked);
    // Start SD card
    SPI1.setTX(11);
    SPI1.setRX(12);
    SPI1.setSCK(10);
    SPI1.begin();

    if (!SD.begin(chipSelect))
    {
        Serial.println("Initialization failed!");
        return;
    }
    Serial.println("Initialization done.");
    createDataLoggingFile();

    LoRa.setPins(csPin, resetPin, irqPin);

    if (!LoRa.begin(915E6))
    {
        Serial.println("LoRa init failed. Check your connections.");
        while (true)
        {
        }
    }
    SPI.beginTransaction(SPISettings(27000000, MSBFIRST, SPI_MODE0));
    rp2040.resumeOtherCore();
    // draw home page
    tft.fillScreen(TFT_BLACK);
}
void setup1()
{
}

//====================================================================================
//                                    Loop
//====================================================================================
void loop()
{
    // update the state of the up button:
    upButton.tick();
    // update the state of the down button:
    downButton.tick();
    drawLayout(100, -60, testSend, testReceive, testHeading, 13.5, 20, 100);
    // use a switch statement to check the value of the mode variable:
    switch (mode)
    {
    // if the mode value is 1, do something:
    case 0:
        drawAltimeterPage(testAltitude);
        break;

    // if the mode value is 2, do something else:
    case 1:
        drawGpsPage(testLat, testLong);
        break;

    // if the mode value is not covered by the case statements, do something else:
    default:
        // insert your code here to handle other mode values
        break;
    }
}