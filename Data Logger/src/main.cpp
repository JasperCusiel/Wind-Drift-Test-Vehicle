#include <Arduino.h>
#include <TFT_eSPI.h>

#include "Hundredths Needle.h" //include bitmap images
#include "Thousandths Needle.h"
#include "Altimeter Background.h"

const int upButtonPin = 2;
const int downButtonPin = 3;
const int leftButtonPin = 4;
const int rightButtonPin = 5;

TFT_eSPI tft = TFT_eSPI(128, 160);                   /* TFT instance */
TFT_eSprite altimeterBackground = TFT_eSprite(&tft); // create altimeter background sprite
TFT_eSprite needleThousandth = TFT_eSprite(&tft);    // create hundredth needle sprite
TFT_eSprite needleHundredth = TFT_eSprite(&tft);     // create thousandth needle sprite

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

void drawAltitude(int altitudeVal)
{
    tft.drawString(String(altitudeVal), 104, 0, 14);
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
    for (int i = 0; i < 10000; i += 10)
    {
        int altitude = i;
        // Find the power of ten for the digit you want, divide your number by that value, and then modulus the quotient with ten.
        int thousandths = (altitude / 1000U) % 10;
        int altitudeHundredths = altitude - (thousandths * 1000);
        int angleMapHundredths = map(altitudeHundredths, 0, 1000, 0, 360);
        int angleMapThousandths = map(altitude, 0, 10000, 0, 360);
        drawAltimeter(angleMapThousandths, angleMapHundredths);
        delay(25);
    }
}