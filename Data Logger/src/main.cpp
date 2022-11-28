#include <Arduino.h>
#include <TFT_eSPI.h>

#include "Hundredths Needle.h" //include bitmap images
#include "Thousandths Needle.h"
#include "Altimeter Background.h"
#include "Data Transfer Icon.h"
#include "Data Receive Icon.h"
#include "Direction Arrow Icon.h"
#include "Data Receive Grey Icon.h"
#include "Data Transfer Grey Icon.h"

const int upButtonPin = 2;
const int downButtonPin = 3;
const int leftButtonPin = 4;
const int rightButtonPin = 5;
int rssi = 0;
float batteryPercentage = 100.0;
float vehicleHeading = 0.0;
int count = 0;
int tick = 0;

TFT_eSPI tft = TFT_eSPI(128, 160);                   /* TFT instance */
TFT_eSprite altimeterBackground = TFT_eSprite(&tft); // create altimeter background sprite
TFT_eSprite needleThousandth = TFT_eSprite(&tft);    // create hundredth needle sprite
TFT_eSprite needleHundredth = TFT_eSprite(&tft);     // create thousandth needle sprite
TFT_eSprite statusBar = TFT_eSprite(&tft);           // create status bar sprite
TFT_eSprite dataPanel = TFT_eSprite(&tft);           // create sprite for temp, humidity and direction data
TFT_eSprite arrow = TFT_eSprite(&tft);               // create direction arrow sprite
TFT_eSprite messages = TFT_eSprite(&tft);

void createAltimeterBackground()
{
    altimeterBackground.setColorDepth(8);
    altimeterBackground.createSprite(108, 108);
    altimeterBackground.setPivot(52, 52);
    altimeterBackground.fillSprite(TFT_TRANSPARENT);
    altimeterBackground.pushImage(0, 4, 104, 104, altimeterBackgroundImg);
    altimeterBackground.drawLine(0, 0, 108, 0, TFT_WHITE);
}

void createNeedleHundredth()
{
    needleHundredth.setColorDepth(8);
    needleHundredth.createSprite(8, 64);
    needleHundredth.fillSprite(TFT_TRANSPARENT);
    needleHundredth.pushImage(0, 0, 8, 64, altHandHundredths);
    needleHundredth.setPivot(4, 42);
}

void createNeedleThousandth()
{
    needleThousandth.setColorDepth(8);
    needleThousandth.createSprite(12, 44);
    needleThousandth.fillSprite(TFT_TRANSPARENT);
    needleThousandth.pushImage(0, 0, 12, 44, altHandThousandths);
    needleThousandth.setPivot(6, 30);
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

void drawAltimeter(int thousandthsAngle, int hundredthsAngle)
{
    createAltimeterBackground();
    needleThousandth.pushRotated(&altimeterBackground, thousandthsAngle, TFT_TRANSPARENT);
    needleHundredth.pushRotated(&altimeterBackground, hundredthsAngle, TFT_TRANSPARENT);
    altimeterBackground.pushSprite(0, 20, TFT_TRANSPARENT);
}

void drawStatusBar(float percentage, int rssi, bool transmitting, bool receiving)
{
    statusBar.setColorDepth(8);
    statusBar.createSprite(56, 20, TFT_TRANSPARENT);
    statusBar.fillSprite(TFT_BLACK);

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
    statusBar.deleteSprite();
}

void drawDataPanel(float heading, float temp, float humidity, float speed)
{
    const int panelHeight = 108;
    const int panelWidth = 50;
    char direction[3];

    // heading angle to N, NE, E, SE, S, SW, W, NW
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
    // setup sprite
    dataPanel.setColorDepth(8);
    dataPanel.createSprite(panelWidth, panelHeight, TFT_TRANSPARENT);
    dataPanel.fillSprite(TFT_BLACK);
    dataPanel.setPivot(25, 24);

    dataPanel.setTextColor(TFT_WHITE);
    dataPanel.setTextDatum(MC_DATUM);
    dataPanel.setTextSize(2);
    // draw panel lines
    dataPanel.drawLine(0, 0, 0, panelHeight, TFT_WHITE);
    dataPanel.drawLine(0, 54, panelWidth, 54, TFT_WHITE);
    dataPanel.drawLine(0, 82, panelWidth, 82, TFT_WHITE);
    dataPanel.drawLine(0, 54, panelWidth, 54, TFT_WHITE);
    dataPanel.drawLine(0, 0, panelWidth, 0, TFT_WHITE);
    // draw data
    dataPanel.setTextSize(1);
    dataPanel.drawString((String(speed, 1) + "KNTS"), 25, 48);
    dataPanel.drawString(direction, 10, 8);
    dataPanel.setTextSize(1);
    dataPanel.setTextColor(TFT_CYAN);
    dataPanel.drawString((String(humidity, 1) + "%"), 25, 97);
    dataPanel.setTextColor(TFT_ORANGE);
    dataPanel.drawString((String(temp, 1) + "C"), 25, 70);
    dataPanel.drawCircle(43, 60, 2, TFT_ORANGE);

    // Draw arrow icon at heading angle
    createDirectionArrow();
    arrow.pushRotated(&dataPanel, heading, TFT_TRANSPARENT);

    dataPanel.pushSprite(108, 20);
}
void drawMessages()
{ // max message length = 17 characters
    messages.scroll(0, 1);
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
}
void setup()
{
    Serial.begin(9600);
    tft.begin();        /* TFT init */
    tft.setRotation(1); /* set rotation */
    tft.fillScreen(TFT_BLACK);
    createNeedleHundredth();
    createNeedleThousandth();
    createMessages();
    pinMode(upButtonPin, INPUT_PULLUP);
    pinMode(downButtonPin, INPUT_PULLUP);
    pinMode(leftButtonPin, INPUT_PULLUP);
    pinMode(rightButtonPin, INPUT_PULLUP);
}

void loop()
{
    for (int i = 0; i < 10000; i += 10)
    {
        int altitude = i;
        // Find the power of ten for the digit you want, divide your number by that value, and then modulus the quotient with ten.
        int thousandths = (altitude / 1000U) % 10;
        int altitudeHundredths = altitude - (thousandths * 1000);
        int angleMapHundredths = map(altitudeHundredths, 0, 1000, 0, 360);
        int angleMapThousandths = map(altitude, 0, 10000, 0, 360);
        bool dataUp = false;
        bool dataDown = false;
        if (vehicleHeading < 360)
        {
            vehicleHeading += 10.0;
        }
        else
        {
            vehicleHeading = 0;
        }

        delay(300);
        bool status = true;
        if (digitalRead(upButtonPin) == LOW && rssi < 0)
        {
            rssi += 10;
            dataUp = true;
        }
        else if (digitalRead(downButtonPin) == LOW && rssi > -120)
        {
            rssi -= 10;
            dataDown = true;
        }
        if (digitalRead(leftButtonPin) == LOW && batteryPercentage < 100)
        {
            batteryPercentage += 10.0;
        }
        else if (digitalRead(rightButtonPin) == LOW && batteryPercentage > 0)
        {
            batteryPercentage -= 10.0;
        }
        drawAltimeter(angleMapThousandths, angleMapHundredths);
        drawStatusBar(batteryPercentage, rssi, dataDown, dataUp);
        drawDataPanel(vehicleHeading, 22.222, 51.3, 13.5);
        drawMessages();
    }
}