#include <Arduino.h>
#include <TFT_eSPI.h>

#include "Hundredths Needle.h" //include bitmap images
#include "Thousandths Needle.h"
#include "Altimeter Background.h"

const int upButtonPin = 2;
const int downButtonPin = 3;
const int leftButtonPin = 4;
const int rightButtonPin = 5;
int rssi = 0;
float batteryPercentage = 100.0;

TFT_eSPI tft = TFT_eSPI(128, 160);                   /* TFT instance */
TFT_eSprite altimeterBackground = TFT_eSprite(&tft); // create altimeter background sprite
TFT_eSprite needleThousandth = TFT_eSprite(&tft);    // create hundredth needle sprite
TFT_eSprite needleHundredth = TFT_eSprite(&tft);     // create thousandth needle sprite
TFT_eSprite statusBar = TFT_eSprite(&tft);           // create status bar sprite

void createAltimeterBackground()
{
    altimeterBackground.setColorDepth(8);
    altimeterBackground.createSprite(104, 104);
    altimeterBackground.setPivot(52, 52);
    altimeterBackground.fillSprite(TFT_TRANSPARENT);
    altimeterBackground.pushImage(0, 0, 104, 104, altimeterBackgroundImg);
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

void drawAltimeter(int thousandthsAngle, int hundredthsAngle)
{
    createAltimeterBackground();
    needleThousandth.pushRotated(&altimeterBackground, thousandthsAngle, TFT_TRANSPARENT);
    needleHundredth.pushRotated(&altimeterBackground, hundredthsAngle, TFT_TRANSPARENT);
    altimeterBackground.pushSprite(0, 24, TFT_TRANSPARENT);
}

void drawStatusBar(bool connected, float percentage, int rssi)
{
    statusBar.setColorDepth(8);
    statusBar.createSprite(160, 20, TFT_TRANSPARENT);
    statusBar.fillSprite(TFT_BLACK);

    // draw RSSI
    const int sigX = 140;
    const int sigY = 3;
    int strength = map(rssi, -120, 0, 0, 4);
    if (strength >= 3)
    {
        statusBar.fillSmoothRoundRect(sigX, sigY, 2, 12, 1, TFT_WHITE);
    }
    if (strength >= 2)
    {
        statusBar.fillSmoothRoundRect((sigX + 4), (sigY + 2), 2, 10, 1, TFT_WHITE);
    }
    if (strength >= 1)
    {
        statusBar.fillSmoothRoundRect((sigX + 8), (sigY + 4), 2, 8, 1, TFT_WHITE);
    }
    if (strength >= 0)
    {
        statusBar.fillSmoothRoundRect((sigX + 12), (sigY + 6), 2, 6, 1, TFT_WHITE);
    }

    statusBar.setTextColor(TFT_ORANGE);
    statusBar.setTextSize(1);
    statusBar.drawString("Status:", 0, 6);

    statusBar.drawLine(0, 19, 160, 19, TFT_WHITE);
    tft.drawLine(108, 18, 108, 128, TFT_WHITE);

    // Draw battery icon
    const int batX = 116; // battery icon x, y coordinate (top left)
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
    statusBar.fillSmoothRoundRect(((batX + 16) - recWidth), (batY + 2), recWidth, 6, 1, batColour);
    statusBar.drawRoundRect(batX, batY, 18, 10, 1, TFT_WHITE);
    statusBar.fillSmoothRoundRect((batX - 2), (batY + 2), 2, 6, 1, TFT_WHITE);

    // Draw status bar on screen
    statusBar.pushSprite(0, 0, TFT_TRANSPARENT);
    statusBar.deleteSprite();
}

void setup()
{
    Serial.begin(9600);
    tft.begin();        /* TFT init */
    tft.setRotation(1); /* set rotation */
    tft.fillScreen(TFT_BLACK);
    createNeedleHundredth();
    createNeedleThousandth();
    pinMode(upButtonPin, INPUT_PULLUP);
    pinMode(downButtonPin, INPUT_PULLUP);
    pinMode(leftButtonPin, INPUT_PULLUP);
    pinMode(rightButtonPin, INPUT_PULLUP);
}

void loop()
{
    bool status = true;
    if (digitalRead(upButtonPin) == LOW && rssi < 0)
    {
        rssi += 10;
    }
    else if (digitalRead(downButtonPin) == LOW && rssi > -120)
    {
        rssi -= 10;
    }
    if (digitalRead(leftButtonPin) == LOW && batteryPercentage < 100)
    {
        batteryPercentage += 10.0;
    }
    else if (digitalRead(rightButtonPin) == LOW && batteryPercentage > 0)
    {
        batteryPercentage -= 10.0;
    }
    delay(100);
    drawAltimeter(1, 1);
    Serial.println(String(batteryPercentage));
    drawStatusBar(status, batteryPercentage, rssi);

    /*for (int i = 0; i < 10000; i += 10)
    {
        int altitude = i;
        // Find the power of ten for the digit you want, divide your number by that value, and then modulus the quotient with ten.
        int thousandths = (altitude / 1000U) % 10;
        int altitudeHundredths = altitude - (thousandths * 1000);
        int angleMapHundredths = map(altitudeHundredths, 0, 1000, 0, 360);
        int angleMapThousandths = map(altitude, 0, 10000, 0, 360);
        drawAltimeter(angleMapThousandths, angleMapHundredths);
        drawStatusBar(status, 25);
        delay(25);
    }*/
}