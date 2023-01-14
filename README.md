# Wind Drift Test Vehicle

This project is based around building data collection device that can be dropped out of a helicopter and GPS position, altitude, temperature and humidity while relaying this information back to a ground station. The vehicle samples GPS, altitude, temperature and humidity at 10Hz while the GPS has a 3D fix and 1Hz while trying to obtain a fix. This data is logged to the internal SD card and relayed over the LoRa radio to the ground station a 0.5Hz. The ground station has an inbuilt SD card where it logs all of the incoming data while displaying this reviced information to the user. These systems operate independantly so the vehicle always logs data regardless of its connection status to the ground station.

Test Vehicle | Ground Station
------------ | -------------
![vehicle_gif](https://github.com/JasperCusiel/Wind-Drift-Test-Vehicle/blob/f5f4aa340e76b96d537dc1a6af973c7dd2ffaf03/Test_Vehicle.gif) | ![ground station](https://github.com/JasperCusiel/Wind-Drift-Test-Vehicle/blob/e8371d43ec23b7cb48514da858f554845b8bd735/Ground%20Station.gif)

## CAD Model
The vehicle and ground station where both designed in Fusion 360 and the .F3D fusion project file and .STEP file are included in the "CAD" folder. These models contains all of the 3D printable parts but exclude things such as bolts and associated hardware due to the file getting quite sluggish when designing my computer.

## Platform IO Project Code
The code that runs on the [Sparkfun RP2040 mikroBUS](https://www.sparkfun.com/products/18721) board in the vehicle and [Adafruit Feather RP2040](https://www.adafruit.com/product/4884) are linked in the "Vehicle code" and "Ground Station Code" respectively. These folders include the whole Platform IO project so can be imported directly into Platform IO.

The code is running the [arduino-pico](https://arduino-pico.readthedocs.io/en/latest/) framework since it supports pretty much all of the Arduino functionality unlike the MbedOS core developed by Arduino. I devloped with Arduino since all of the sensor I chose had many suppoorting arduino libarays pre written so made it much easier than developing in something else like Python which I am more familiar with. Therefore the code could definity be further optimized to run more effiecintly (This was my first time devling into Arduino/C++ coding).

### Dual Core Program
Since the RP2040 chip in the vehicles electronics is a dual core proccessor, core 0 runs the sensor sampling and logging to the SD card while core 1 runs code to send data via LoRa radio to the ground station. 

### Importing The Project
To install the project, first download the code

## Wiring Diagrams
Below is the wiring diamgram for the ground station for referance as it was first protoyped on a solderless breadboard then transfered to a proto-board. I would have loved to then get this design made into a pcb to make the whole cirucit abit more compact and pottentially move to using the indiviual components rather than off the self breakout boards.

(add diagram here)

### Connector Pinouts
Below is the pinouts of all of the differnt connectors in the vehicle. All of the connectors used in the vehicle are JST-XH connectors as these where easy to crimp on all of the diffent sensor and usb cables in the vehicle.

(add pinouts here)

