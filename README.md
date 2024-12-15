# UltimateAltimeter
This altimeter uses the TS-ESP32-S3 board without any additional components

The board is based on the Adafruit ESP32 S3 TFT Feather

https://cdn-learn.adafruit.com/downloads/pdf/adafruit-esp32-s3-tft-feather.pdf


# Building the code

You will need to download the Arduino ide from the [Arduino web site](https://www.arduino.cc/). 
Make sure that you install ESP32 support
The project depend on the following libraries
  - TFT
  - Button2
  - TFT_eWidget
  - Preferences
  - BME280I2C
  - SensorLib (used for the QMI8658 sensor)
    
You will need my fork of the TFT_eSPI
https://github.com/bdureau/TFT_eSPI

Compile with the following options:
<img src="/photos/TS-ESP32-S3-compile.png" width="35%">

Prior to compiling go to the TFT_eSPI and open up the file User_Setup_Select.h

Comment out the following line

//#include <User_Setup.h> 

and uncomment the following line

#include <User_Setups/Setup400_Adafruit_Feather.h>

