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
Since the RP2040 chip in the vehicles electronics is a dual core proccessor, core 0 runs the sensor sampling and logging to the SD card while core 1 runs code to send data via LoRa radio to the ground station. This allows core 0 to sample data at 10hz while core 1 sends the data via LoRa such that sending the data is non-blocking and doesn't interupt sampling the sensors. 

Unfortunalty I couldn't figure out away to implement a similar strategy with the ground stations code whereby core 0 could handle reciving the LoRa data while core 1 would log that data and update the display. However I ran into an issue where the tft code to run the screen could block writing data to the SD card as the SD card is on the back of the tft display so shares the same SPI bus as the tft. Therefore if the program needed to access the SD, the screen would become frozen and cause articfacting on the screen as the whole SPI transactions wheren't being completed. This also meant that changing screens between the GPS coordinates and altimeter page is interupted so it appears as if the system is frozen. A solution to get around this would be to add the SD card onto a seperate SPI bus. However, this would require a chnage in microcontroller to one that supported more than two SPI buses such that the LoRa, SD and tft display could all have a dedicated SPI bus such that there is only one device per SPI bus. Instead the program only runs on core 0 and simply checks for new LoRa data, writes any new data to the SD then updates the tft display. This way changing pages with the buttons remains responsive and the tft code doesn't block SD writes avoiding conflicts and display artifacting. I'm sure there is probably software method around this issue and could involve a operating system like [FreeRTOS](https://www.freertos.org) (Had a look into this as a solution but it is abit above my current coding paygrade and would involve alot of time to learn)

### Importing The Project
To install the project, first download the code

## Wiring Diagrams
Below is the wiring diamgram for the ground station for referance as it was first protoyped on a solderless breadboard then transfered to a proto-board. I would have loved to then get this design made into a pcb to make the whole cirucit abit more compact and pottentially move to using the indiviual components rather than off the self breakout boards.
![ground station wiring](https://github.com/JasperCusiel/Wind-Drift-Test-Vehicle/blob/7d3b5c572471ea2a260b697b29c4c50e110d7b82/Images%20&%20Gifs/Ground%20Station%20Wiring.png)

### Connector Pinouts
Below is the pinouts of all of the differnt connectors in the vehicle. All of the connectors used in the vehicle are JST-XH connectors as these where easy to crimp on all of the diffent sensor and usb cables in the vehicle.

(add pinouts here)

