#include <SparkFun_MS5637_Arduino_Library.h>
#include <Arduino.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include <LoRa.h>
#include <SPI.h>

// Uncomment to turn on debugging
#define DEBUGGING

// RP2040 Board
int Board_SDA = 6;
int Board_SCL = 7;

// MAX17048 Battery Fuel Gauge
SFE_MAX1704X lipo(MAX1704X_MAX17048);
double voltage = 0;
double soc = 0;
bool alert;

// Soft Power Switch
int POWER_BUTTON = 17;
int FAST_OFF = 16;
long powerPressedStartTime = 0;
int debounceDelay = 20;

// Status LED
int LED_GREEN = 26;
int LED_RED = 28;
int LED_BLUE = 27;

// MS5637 Altimeter
MS5637 barometricSensor;

// GPS
int SPI_CLK = 2;
int GPS_MOSI = 3;
int GPS_MISO = 4;
int GPS_CS = 5;
int TX = 0;
int RX = 1;
int RST = 13;
int PWM = 16;
int INT = 17;
int AN = 26;

// LoRa
const int RFM_CS = 19;          // RFM95 chip select pin
const int RFM_RST = 18;         // RFM95 reset pin
const int RFM_IQR = 20;         // RFM95 IQR pin
byte localAddress = 0xAA;       // address of this device
byte destinationAddress = 0xBB; // destination to send to
byte msgCount = 0;              // count of out going messages
int count = 0;
long lastSendTime = 0; // last send time
int interval = 500;    // interval between sends

// Micro SD Card
int SDO = 12;
int SDI = 15;
int CLK = 14;
int DATA_1 = 11;
int DATA_2 = 10;
int DATA_3 = 9;

// Soft power switch functions
void fastPowerDown()
{
  pinMode(POWER_BUTTON, OUTPUT);
  digitalWrite(POWER_BUTTON, LOW);
  pinMode(FAST_OFF, OUTPUT);
  digitalWrite(FAST_OFF, LOW);
  powerPressedStartTime = millis();
  while (1)
  {
    delay(1);
  }
}

void slowPowerDown()
{
  pinMode(POWER_BUTTON, OUTPUT);
  digitalWrite(POWER_BUTTON, LOW);
  powerPressedStartTime = millis();
#ifdef DEBUGGING
  Serial.print("Pulling power line low");
#endif
  while (1)
  {
    delay(1);
  }
}

// LoRa functions

void sendMessage(String outgoing)
{
  LoRa.beginPacket();             // start packet
  LoRa.write(destinationAddress); // add sender address
  LoRa.write(localAddress);       // add sender address
  LoRa.write(msgCount);           // add message ID
  LoRa.write(outgoing.length());  // add payload length
  LoRa.print(outgoing);           // add payload
  LoRa.endPacket();
  msgCount++; // increment message ID
}

void receiveMessage(int packetSize)
{
  if (packetSize == 0)
    return; // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();       // recipient address
  byte sender = LoRa.read();         // sender address
  byte incomingMsgId = LoRa.read();  // incoming msg ID
  byte incomingLength = LoRa.read(); // incoming msg length

  String incoming = "";

  while (LoRa.available())
  {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) // check length for error
  {
#ifdef DEBUGGING
    Serial.println("Error: Message length does not match length");
#endif

    return;
  }

  if (recipient != localAddress) // check message is for this device
  {
#ifdef DEBUGGING
    Serial.println("Error: Recipient adress does not match local address");
#endif

    return;
  }

// if message is for this device, or broadcast, print details:
#ifdef DEBUGGING
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
#endif
}

void setup(void)
{
  // Power Switch Input Setup
  pinMode(POWER_BUTTON, INPUT_PULLUP);
  pinMode(FAST_OFF, INPUT_PULLUP);
  powerPressedStartTime = millis();
  while (digitalRead(POWER_BUTTON) == LOW)
  {
    // Wait for user to stop pressing button before
    delay(100);
    if (millis() - powerPressedStartTime > 500)
      break;
  }

  if (millis() - powerPressedStartTime < 500)
  {
    fastPowerDown();
  }

  // Display Red LED to indiacate system is on and not data logging
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  powerPressedStartTime = 0; // Reset var to return to normal 'on' state

  // Turn on
  // Serial Initialization

  Serial.begin(9600);
  Serial.println("Begin");

  // I2C Initialization
  Wire1.setSDA(Board_SDA);
  Wire1.setSCL(Board_SCL);
  Wire1.begin();

  // SPI Inititialization
  SPI.begin();

  barometricSensor.begin(Wire1);

  // LoRa Initialization
  LoRa.setSPI(SPI);
  LoRa.setPins(RFM_CS, RFM_RST, RFM_IQR);
  if (!LoRa.begin(915E6))
  {
    delay(2000);
    Serial.println("LoRa init failed, check connections.");
    while (1)
    {
    }
  }

  // MAX17048 Battery Fuel Gauge start
  if (lipo.begin() == false) // Connect to the MAX17043 using non-standard wire port
  {
#ifdef DEBUGGING
    Serial.println(F("MAX17048 not detected. Please check wiring. Freezing."));
#endif

    while (1)
      ;
  }
  lipo.quickStart();
  lipo.setThreshold(20);
}

void loop(void)
{
  while (digitalRead(POWER_BUTTON) == LOW)
  {
    if (digitalRead(POWER_BUTTON) == LOW && powerPressedStartTime == 0)
    {
      // Debounce check
      delay(debounceDelay);
      if (digitalRead(POWER_BUTTON) == LOW)
      {
#ifdef DEBUGGING
        Serial.println("User is pressing power button. Start timer.");
#endif

        powerPressedStartTime = millis();
      }
    }
    else if (digitalRead(POWER_BUTTON) == LOW && powerPressedStartTime > 0)
    {
      // Debounce check
      delay(debounceDelay);
      if (digitalRead(POWER_BUTTON) == LOW)
      {
        if ((millis() - powerPressedStartTime) > 2000)
        {
#ifdef DEBUGGING
          Serial.println("Time to power down!");
#endif
          fastPowerDown();
        }
      }
    }
    else if (digitalRead(POWER_BUTTON) == HIGH && powerPressedStartTime > 0)
    {
      // Debounce check
      delay(debounceDelay);
      if (digitalRead(POWER_BUTTON) == HIGH)
      {
#ifdef DEBUGGING
        Serial.print("Power button released after ms: ");
        Serial.println(millis() - powerPressedStartTime);
#endif
      }
      powerPressedStartTime = 0; // Reset var to return to normal 'on' state
    }
  }

  // Check sensor data
  if (!barometricSensor.isConnected())
  {
#ifdef DEBUGGING
    Serial.println("Barometric Pressure Sensor Not connected");
#endif
  }
  float temperature = barometricSensor.getTemperature();
  float pressure = barometricSensor.getPressure();

#ifdef DEBUGGING
  Serial.print(" Temp=");
  Serial.print(temperature, 1);
  Serial.print("(C)");
  Serial.print(" Press=");
  Serial.print(pressure, 3);
  Serial.print("(hPa)");
  Serial.println();
#endif

  // Send data via LoRa
  if (millis() - lastSendTime > interval)
  {
    String sensorData = String(count++);
    sendMessage(sensorData);
#ifdef DEBUGGING
    Serial.print("Sending data " + sensorData);
    Serial.print(" from 0x" + String(localAddress, HEX));
    Serial.print(" to 0x" + String(destinationAddress, HEX));
#endif

    lastSendTime = millis();
    interval = random(2000) + 1000;
  }

  receiveMessage(LoRa.parsePacket());
}