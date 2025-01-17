# UltimateAltimeter
This altimeter uses the TS-ESP32-S3 board without any additional components
You can find it on Aliexpress
https://vi.aliexpress.com/w/wholesale-TS%2525252dESP32%2525252dS3.html

That board has a BME280 sensor as well as a QMI8658 sensor.
You can measure the pressure (and derive the altitude from it). It also has a 3 axes accelerometer and well as gyroscope

The board is based on the Adafruit ESP32 S3 TFT Feather

https://cdn-learn.adafruit.com/downloads/pdf/adafruit-esp32-s3-tft-feather.pdf

https://github.com/adafruit/Adafruit-ESP32-S3-TFT-Feather-PCB/blob/main/Adafruit%20ESP32-S3%20TFT%20Feather%20Pinout.pdf

The board looks like this

<img src="/photos/UltimateAltimeter1.jpg" width="15%"><img src="/photos/UltimateAltimeter2.jpg" width="35%"><img src="/photos/ultimateAlti_flight.jpg" width="35%">

# Building the code

You will need to download the Arduino ide from the [Arduino web site](https://www.arduino.cc/). 
Make sure that you install ESP32 support. 

2 versions are provided one that uses the preference library to store the flight.
https://github.com/bdureau/UltimateAltimeter/tree/main/UltimateAltimeter

and another one that uses the LittleFS library
https://github.com/bdureau/UltimateAltimeter/tree/main/UltimateAltimeterLittleFS

The project depend on the following libraries
  - TFT
  - Button2
  - TFT_eWidget
  - Preferences or LittleFS
  - BME280
  - SensorLib (used for the QMI8658 sensor)
    
You will need my fork of the TFT_eSPI
https://github.com/bdureau/TFT_eSPI

Compile with the following options:

<img src="/photos/TS-ESP32-S3-compile.png" width="35%">

Prior to compiling go to the TFT_eSPI and open up the file User_Setup_Select.h

Comment out the following line
```
//#include <User_Setup.h> 
```
and uncomment the following line
```
#include <User_Setups/Setup400_Adafruit_Feather.h>
```

You will need to have the ESP32 board support version 2.0.14, anything higher than that may not work !!!

<img src="/photos/Esp32 board.png" width="55%">

# Contributing

If you want to contribute to the project just fork my project or send me some code. 

Report any issue or bug that you have

Suggestions and enhancement are welcome

The code is free for you to download and you do not need to buy anything from me. However it cost money to try out new boards, you need to buy them and fly them so if you want to financially help me you can donate via paypal

| Paypal | 
| ------ |
| [![](https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif)](https://www.paypal.com/paypalme/bearaltimeter) | 

# Disclaimer

I am not responsible for any damage that could happen. The code is provided as it is


