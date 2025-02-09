This version uses the LittleFS library to store data. It might be slower but it can store a lot more data.
The following is recorded:
altitude, pressure, temperature, humidity and acceration on the 3 axis

Specific instruction to compile it
Make sure SPIFFS otherwise it will not store the data

As of version 0.5 I am switching to the BMP280 library from dvarrel (ver 1.0.3)
https://github.com/dvarrel/BMP280

use the following compilation directive #define USE_SLEEP_MODE to enable or disable the sleep mode
