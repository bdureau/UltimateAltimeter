/*
    UltimateAltimeter ver0.5
    Copyright Boris du Reau 2012-2025
    This is using a BME280 presure sensor
    for the accelerometer

    !!!IMPORTANT!!!!
    to compile it use the board "Adafruit Feather ESP32-S3 TFT"
    !!!IMPORTANT!!!!
    The flight is recorded on the board using LITTLE FS rather than the preferences library so that we can store longer flight

    Major changes on version 0.1
    Initial version of the code, this is re-using code from the TTGO board
    can display altitude, acceleration, temperature and pressure
    Major changes on version 0.2
    Code cleanup
    Added communication with the BearConsole application
    Major changes on version 0.3
    use the reset button to power on or of the board
    Major changes on version 0.4
    Added the status to the console app
    Removed humidity
    Fix to prevent stopping recording due to the ejection charge over pressure
    Major changes on version 0.5
    changes son that it uses the normal BMP280 library

*/
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Button2.h>
#include <TFT_eWidget.h>
#include <driver/rtc_io.h>
#include <Wire.h>
#include <EnvironmentCalculations.h>
#include "SensorQMI8658.hpp"

#include <BMP280.h>
#include "kalman.h"
#include "logger.h"
#include "images/bear_altimeters128x128.h"
#define MAJOR_VERSION 0
#define MINOR_VERSION 5
#define BOARD_FIRMWARE "UltimateAltimeter"
#include <Preferences.h>
#include <WiFi.h>

//#define USE_SLEEP_MODE
//#define DEBUG
struct bmpValues {
  float pressure;
  float temperature;
  float altitude;
};
BMP280 bmp280;

#ifndef SENSOR_SDA
#define SENSOR_SDA  42
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  41
#endif

logger flightLogger;

//////////////////////////////////////////////////////////////////////
// Global variables
//////////////////////////////////////////////////////////////////////

#define STATUS_HEIGHT_BAR 10

// Built in button GPIO - adjust for your board
#define BUTTON_GPIO GPIO_NUM_0

Button2 btn(BUTTON_GPIO); // Initialize the down button

TFT_eSPI tft = TFT_eSPI();
GraphWidget gr = GraphWidget(&tft);    // Graph widget
// Flight curves are drawn on tft using graph instance
TraceWidget trAltitude = TraceWidget(&gr);    // Altitude
TraceWidget trTemperature = TraceWidget(&gr);
TraceWidget trPressure = TraceWidget(&gr);
TraceWidget trAccelX = TraceWidget(&gr);    // Accel X
TraceWidget trAccelY = TraceWidget(&gr);    // Accel Y
TraceWidget trAccelZ = TraceWidget(&gr);    // Accel Z
TraceWidget trHumidity = TraceWidget(&gr);

// Assumed environmental values:
bool inGraph = false;

//ground level altitude
long initialAltitude;

//stop recording a maximum of 20 seconds after main has fired
long recordingTimeOut = 120 * 1000;
boolean canRecord = true;
boolean exitRecording = false;

long liftoffAltitude = 20;
long lastAltitude;
//current altitude
long currAltitude;
long diplayedFlightNbr = 0;
long currentCurveType = 0;
long apogeeAltitude;
//nbr of measures to do so that we are sure that apogee has been reached
unsigned long measures = 5;
boolean liftOff = false;
unsigned long initialTime = 0;
//telemetry
boolean telemetryEnable = false;
long lastTelemetry = 0;

SensorQMI8658 qmi;

IMUdata acc;
IMUdata gyr;

Kalman KalmanAltitude;

int onoroff = 2;

/*
   tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
*/
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
{
  if ( y >= tft.height() ) return 0;
  tft.pushImage(x, y, w, h, bitmap);
  return 1;
}

/*
   button_init()

*/
void button_init()
{
  btn.setClickHandler([](Button2 & b) {
    // Down
    Serial.println(F("Button Down fast")); // It's called upCmd because it increases the index of an array. Visually that would mean the selector goes downwards.
    if (inGraph) {
      long lastFlightNbr = flightLogger.getLastFlightNbr();
      currentCurveType++;
      //Make sure we have not reach the last flight
      if ((lastFlightNbr > diplayedFlightNbr) & (currentCurveType > 4) ) {
        diplayedFlightNbr ++;
        currentCurveType = 0;
      } else {
        // if not lets go back to the first one if it exists
        if (!((lastFlightNbr < 1))) {
          if (currentCurveType > 4) {
            diplayedFlightNbr = 1;
            currentCurveType = 0;
          }
        }
      }
#ifdef DEBUG
      Serial.print("Flight:");
      Serial.println(diplayedFlightNbr);
      Serial.print("type:");
      Serial.println(currentCurveType);
#endif

      drawFlightNbr(diplayedFlightNbr, currentCurveType);
    }
  });

  btn.setLongClickHandler([](Button2 & b) {

    Serial.println("Button Down slow");
    unsigned int time = b.wasPressedFor();

    if (time >= 1000 & time < 3000) {
      if (!inGraph) {
        long lastFlightNbr = flightLogger.getLastFlightNbr();
#ifdef DEBUG
        Serial.print("lastFlightNbr:");
        Serial.println(lastFlightNbr);
#endif
        if (!(lastFlightNbr < 1)) {
          inGraph = true;
          diplayedFlightNbr = 1;
          drawFlightNbr(diplayedFlightNbr, currentCurveType);
        }
      } else {
        inGraph = false;
        tft.init();
        tft.fillScreen(TFT_BLACK);

        // Graph area is 200 pixels wide, 150 high, dark grey background
        gr.createGraph(200, 100, tft.color565(5, 5, 5));
        // x scale units is from 0 to 100, y scale units is 0 to 50
        gr.setGraphScale(0.0, 100.0, 0, 50.0);
      }
    }
    if (time >= 10000) {
      inGraph = false;
      tft.init();

      tft.fillScreen(TFT_BLACK);

      // Graph area is 200 pixels wide, 150 high, dark grey background
      gr.createGraph(200, 100, tft.color565(5, 5, 5));
      // x scale units is from 0 to 100, y scale units is 0 to 50
      gr.setGraphScale(0.0, 100.0, 0, 50.0);
      Serial.println("Erasing flights!!!");

      // erasing flights
      flightLogger.clearFlightList();
    }
  });
}

/*
   button_loop()

*/
void button_loop()
{
  // Check for button presses
  btn.loop();
}


void setup() {
  Wire.begin();
  Serial.begin(38400);
  Serial.println(F("Starting"));
  if (!flightLogger.initFileSystem()) {
    Serial.println(F("Failed to initialize file system"));
  } else {
    Serial.println(F("initFileSystem Ok"));
  }

  WiFi.mode(WIFI_OFF);
  flightLogger.initFlight();
  // turn on backlite
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  // turn on the TFT / I2C power supply
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);

#ifdef USE_SLEEP_MODE
  Preferences preferences;
  preferences.begin("alti", false);
  onoroff = preferences.getUInt("onoroff", 0);
  if (onoroff == 0) {
    // system should turn on, but save onoroff = 1
    // Take no action
    preferences.putUInt("onoroff", 1);
    preferences.end();
  }
  else {
    // system should go to deep sleep, but save onoroff = 0
    // Turn off all power options and enter deep sleep forever.
    preferences.putUInt("onoroff", 0);
    preferences.end();
    // Disable accelerometer and gyro
    qmi.disableGyroscope();
    qmi.disableAccelerometer();

    pinMode(TFT_BACKLITE, OUTPUT);
    pinMode(TFT_I2C_POWER, OUTPUT);
    pinMode(4, OUTPUT);
    digitalWrite(TFT_BACKLITE, LOW);
    digitalWrite(TFT_I2C_POWER, LOW);
    pinMode(NEOPIXEL_POWER, OUTPUT);
    pinMode(41, OUTPUT);
    pinMode(42, OUTPUT);
    pinMode(26, OUTPUT);
    pinMode(7, OUTPUT);
    delay(100);
    digitalWrite(4, LOW);
    // Set pin states
    digitalWrite(TFT_BACKLITE, LOW);
    digitalWrite(TFT_I2C_POWER, LOW);
    digitalWrite(NEOPIXEL_POWER, LOW);
    // SDA SDL pins to the Stemma QT need to be set high as they have physical pull up resistors (2x10K)
    // PS Ram CS and TFT CS should be high.
    digitalWrite(41, HIGH);
    digitalWrite(42, HIGH);
    digitalWrite(26, HIGH);
    digitalWrite(7, HIGH);
    delay(10);
    gpio_deep_sleep_hold_en();
    gpio_hold_en((gpio_num_t) TFT_BACKLITE);
    gpio_hold_en((gpio_num_t) TFT_I2C_POWER);
    gpio_hold_en((gpio_num_t)NEOPIXEL_POWER);
    gpio_hold_en((gpio_num_t)41);
    gpio_hold_en((gpio_num_t)42);
    gpio_hold_en((gpio_num_t)26);
    gpio_hold_en((gpio_num_t)7);
    delay(2000);
    esp_deep_sleep_start();
  }
#endif

  tft.init();
  tft.fillScreen(TFT_BLACK);
  tft.setSwapBytes(true);

  tft.pushImage(6, 0, 128, 128, bear_altimeters128x128);
  tft.drawString("Ultimate Altimeter", 6, 135);
  tft.drawString("ver 1.0", 6, 145);
  tft.drawString("Copyright", 6, 155);
  tft.drawString("Boris du Reau", 6, 165);
  tft.drawString("2012-2025", 6, 175);
  tft.drawString("Initializing....", 6, 185);

  bmp280.begin();

  // let's do some dummy altitude reading
  // to initialise the Kalman filter
  for (int i = 0; i < 50; i++) {
    ReadAltitude();
  }

  //let's read the launch site altitude
  float sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += ReadAltitude().altitude;
    delay(50);
  }
  initialAltitude = (long)(sum / 10.0);
  button_init();

  // Initialize QMI8658C sensor with provided configuration
  while (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, SENSOR_SDA, SENSOR_SCL)) {
    tft.drawString("qmi error", 6, 195);
    delay(500);
  }
  delay(1000); // Delay for sensor initialization


  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.drawString("altitude", 6, 0);
  // Graph area is 200 pixels wide, 150 high, dark grey background
  gr.createGraph(200, 100, tft.color565(5, 5, 5));
  // x scale units is from 0 to 100, y scale units is 0 to 50
  gr.setGraphScale(0.0, 100.0, 0, 50.0);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.fillScreen(TFT_BLACK);
  tft.setSwapBytes(true);
  tft.setTextFont(2);

  qmi.configAccelerometer(
    /*
       ACC_RANGE_2G
       ACC_RANGE_4G
       ACC_RANGE_8G
       ACC_RANGE_16G
    */
    SensorQMI8658::ACC_RANGE_4G,
    /*
       ACC_ODR_1000H
       ACC_ODR_500Hz
       ACC_ODR_250Hz
       ACC_ODR_125Hz
       ACC_ODR_62_5Hz
       ACC_ODR_31_25Hz
       ACC_ODR_LOWPOWER_128Hz
       ACC_ODR_LOWPOWER_21Hz
       ACC_ODR_LOWPOWER_11Hz
       ACC_ODR_LOWPOWER_3H
    */
    SensorQMI8658::ACC_ODR_1000Hz,
    /*
       LPF_MODE_0     //2.66% of ODR
       LPF_MODE_1     //3.63% of ODR
       LPF_MODE_2     //5.39% of ODR
       LPF_MODE_3     //13.37% of ODR
       LPF_OFF        // OFF Low-Pass Fitter
    */
    SensorQMI8658::LPF_MODE_0);

  qmi.configGyroscope(
    /*
      GYR_RANGE_16DPS
      GYR_RANGE_32DPS
      GYR_RANGE_64DPS
      GYR_RANGE_128DPS
      GYR_RANGE_256DPS
      GYR_RANGE_512DPS
      GYR_RANGE_1024DPS
    */
    SensorQMI8658::GYR_RANGE_64DPS,
    /*
       GYR_ODR_7174_4Hz
       GYR_ODR_3587_2Hz
       GYR_ODR_1793_6Hz
       GYR_ODR_896_8Hz
       GYR_ODR_448_4Hz
       GYR_ODR_224_2Hz
       GYR_ODR_112_1Hz
       GYR_ODR_56_05Hz
       GYR_ODR_28_025H
    */
    SensorQMI8658::GYR_ODR_896_8Hz,
    /*
       LPF_MODE_0     //2.66% of ODR
       LPF_MODE_1     //3.63% of ODR
       LPF_MODE_2     //5.39% of ODR
       LPF_MODE_3     //13.37% of ODR
       LPF_OFF        // OFF Low-Pass Fitter
    */
    SensorQMI8658::LPF_MODE_3);


  /*
    If both the accelerometer and gyroscope sensors are turned on at the same time,
    the output frequency will be based on the gyroscope output frequency.
    The example configuration is 896.8HZ output frequency,
    so the acceleration output frequency is also limited to 896.8HZ
  */
  qmi.enableGyroscope();
  qmi.enableAccelerometer();

}

/*
   loop()


*/
void loop() {

  char readVal = ' ';
  int i = 0;

  char commandbuffer[100];

  while ( readVal != ';')
  {
    button_loop();

    currAltitude = (long)ReadAltitude().altitude - initialAltitude;
    if (liftOff)
      SendTelemetry(millis() - initialTime, 200);
    if (!( currAltitude > liftoffAltitude) )
    {
      if (!inGraph) {
        SendTelemetry(0, 500);
        tft.setCursor (0, STATUS_HEIGHT_BAR);
        char Altitude [40];

        sprintf(Altitude, "Altitude = %i meters    ", currAltitude );
        tft.setCursor (0, STATUS_HEIGHT_BAR);
        tft.println("                                     ");
        tft.println(Altitude);

        char temp [15];
        if (qmi.getDataReady()) {
          if (qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
            sprintf(temp, "x=%3.2f m/s", (float)acc.x );
            tft.println("");
            tft.println(temp);
            sprintf(temp, "y=%3.2f m/s", (float) acc.y );
            tft.println(temp);
            sprintf(temp, "z=%3.2f m/s", (float) acc.z );
            tft.println(temp);
          }
        }
      }

      while (Serial.available())
      {
        readVal = Serial.read();
        if (readVal != ';' )
        {
          if (readVal != '\n')
            commandbuffer[i++] = readVal;
        }
        else
        {
          commandbuffer[i++] = '\0';
          break;
        }
      }
    }
    else {
      //Serial.println("Recording!!!!");
      recordAltitude();
    }
  }
  interpretCommandBuffer(commandbuffer);
}



/*

   Read Altitude function for a BMP280 Bosch sensor


*/
bmpValues ReadAltitude() {
  bmpValues values;
  //Get pressure value
  uint32_t pressure = bmp280.getPressure();
  float temperature = bmp280.getTemperature();
  values.pressure = pressure;
  values.temperature = temperature;
  values.altitude = KalmanAltitude.KalmanCalc(bmp280.calAltitude(pressure, 1013.0));
  return values;
}


/*
   drawAxesXY(float minX, float maxX, float minY, float maxY )

*/
void drawAxesXY(float minX, float maxX, float minY, float maxY, int flightNbr, char *curveName ) {
  tft.fillScreen(TFT_BLACK);

  // x scale units is from 0 to 100, y scale units is 0 to 50
  gr.setGraphScale(minX, maxX, minY, maxY);
  // X grid starts at 0 with lines every 10 x-scale units
  // Y grid starts at 0 with lines every 10 y-scale units
  // blue grid
  gr.setGraphGrid(0.0, maxX / 5, 0.0, maxY / 5, TFT_BLUE);

  // Draw empty graph, top left corner at 20,10 on TFT
  gr.drawGraph(30, 10);

  // Draw the x axis scale
  tft.setTextDatum(TC_DATUM); // Top centre text datum
  tft.drawNumber(minX, gr.getPointX(minX), gr.getPointY(0) + 3);
  if (maxX < 1000) {
    tft.drawNumber(maxX / 2, gr.getPointX(maxX / 2), gr.getPointY(0) + 3);
    tft.drawNumber(maxX, gr.getPointX(maxX), gr.getPointY(0) + 3);
    tft.drawString("time(ms)", gr.getPointX(maxX / 4),  gr.getPointY(0) + 3);
  } else {
    char temp[10];
    sprintf(temp, "%3.1f",  maxX / 1000 / 2);
    tft.drawString(temp, gr.getPointX(maxX / 2),  gr.getPointY(0) + 3);
    sprintf(temp, "%3.1f",  maxX / 1000);
    tft.drawString(temp, gr.getPointX(maxX) - 10,  gr.getPointY(0) + 3);
    tft.drawString("time (s)", gr.getPointX(maxX / 4),  gr.getPointY(0) + 3);
  }
  char flight[15];
  sprintf(flight, "Flight %i  ",  flightNbr);
  tft.drawString(flight, gr.getPointX(maxX) - 10,  gr.getPointY(maxY));
  tft.drawString(curveName, gr.getPointX(maxX / 3), gr.getPointY(maxY));

  // Draw the y axis scale
  tft.setTextDatum(MR_DATUM); // Middle right text datum
  if (maxY < 1000)
    tft.drawNumber(maxY, gr.getPointX(0.0), gr.getPointY(maxY));
  else
    tft.drawNumber(round2dec(maxY / 1000), gr.getPointX(0.0), gr.getPointY(maxY));
}

long roundUp(float val) {
  long ret = (long)val;
  if (val > ret) {
    return ret + 1;
  }
  return ret;
}
float round2dec(float var)
{
  // 37.66666 * 100 =3766.66
  // 3766.66 + .5 =3767.16    for rounding off value
  // then type cast to int so value is 3767
  // then divided by 100 so the value converted into 37.67
  float value = (int)(var * 100 + .5);
  return (float)value / 100;
}
/*
   drawFlightNbr(int flightNbr, int curveType)

*/
void drawFlightNbr(int flightNbr, int curveType) {

  if (flightLogger.readFlight(flightNbr)) {
    FlightDataStruct* currentFlight;
    currentFlight = flightLogger.getFlightData();
    flightLogger.getFlightMinAndMax(flightNbr);

    //altitude
    if ( curveType == 0) {
      // Start altitude trace
      trAltitude.startTrace(TFT_GREEN);
      if ((float) flightLogger.getMaxAltitude() < 1000)
        drawAxesXY(0.0, flightLogger.getFlightDuration(), 0, (float) flightLogger.getMaxAltitude(), flightNbr, "Altitude (meters)" );
      else {
        drawAxesXY(0.0, flightLogger.getFlightDuration(), 0, (float) flightLogger.getMaxAltitude(), flightNbr, "Altitude (km)" );
      }
    }
    //accel
    if (curveType == 1) {
      trAccelX.startTrace(TFT_RED);
      trAccelY.startTrace(TFT_PURPLE);
      trAccelY.startTrace(TFT_YELLOW);
      float maxAccel = 0.0f;
      if (flightLogger.getMaxAccelX() > maxAccel)
        maxAccel = (float)flightLogger.getMaxAccelX();
      if (flightLogger.getMaxAccelY() > maxAccel)
        maxAccel = (float)flightLogger.getMaxAccelY();
      if (flightLogger.getMaxAccelZ() > maxAccel)
        maxAccel = (float)flightLogger.getMaxAccelZ();

      //Serial.println(maxAccel);
      drawAxesXY(0.0, flightLogger.getFlightDuration(), 0, (float) roundUp(maxAccel) , flightNbr, "Accel X,Y,Z (m/s)" );
    }
    //pressure
    if (curveType == 2) {
      trPressure.startTrace(TFT_GREY);
      //Serial.println(flightLogger.getMaxPressure());
      drawAxesXY(0.0, flightLogger.getFlightDuration(), 0, (float) roundUp(flightLogger.getMaxPressure()), flightNbr, "Pressure (mBar)" );
    }
    //temperature
    if (curveType == 3) {
      trTemperature.startTrace(TFT_BROWN);
      //Serial.println(flightLogger.getMaxTemperature());
      drawAxesXY(0.0, flightLogger.getFlightDuration(), 0, (float) roundUp(flightLogger.getMaxTemperature()), flightNbr, "Temp (°C)" );
    }
    //humidity
    /*if (curveType == 4) {
      trHumidity.startTrace(TFT_YELLOW);
      Serial.println(flightLogger.getMaxHumidity());
      drawAxesXY(0.0, flightLogger.getFlightDuration(), 0, (float) roundUp(flightLogger.getMaxHumidity() + 1), flightNbr, "Hum %" );
      }*/

    unsigned long currentTime = 0;

    for (long i = 0; i < flightLogger.getFlightSize(); i++)
    {
      currentTime = currentTime + currentFlight[i].diffTime; //logger.getFlightTimeData();

      //altitude
      if ( curveType == 0) {
        trAltitude.addPoint(currentTime, currentFlight[i].altitude);
      }

      if ( curveType == 1) {
        trAccelX.addPoint(currentTime, (float)currentFlight[i].accelX );
        trAccelY.addPoint(currentTime, (float)currentFlight[i].accelY );
        trAccelZ.addPoint(currentTime, (float)currentFlight[i].accelZ );
      }
      if ( curveType == 2) {
        trPressure.addPoint(currentTime, currentFlight[i].pressure);
      }
      if ( curveType == 3) {
        trTemperature.addPoint(currentTime, currentFlight[i].temperature);
      }
      /*if ( curveType == 4) {
        trHumidity.addPoint(currentTime, currentFlight[i].humidity);
        }*/
    }
  }
}

/*
   recordAltitude()

*/
void recordAltitude()
{
  long recordingTimeOut = 120 * 1000;
  unsigned long timeToApogee = 0;

  exitRecording = false;

  //long initialTime = 0;
  lastAltitude = 0;

  while (!exitRecording)
  {
    //read current altitude
    currAltitude = (ReadAltitude().altitude - initialAltitude);

    if ((currAltitude > liftoffAltitude) && !liftOff)
    {
      liftOff = true;
      SendTelemetry(0, 200);
      // save the time
      initialTime = millis();

      if (canRecord)
      {
        //init flight
        flightLogger.initFlight();
      }
    }
    unsigned long prevTime = 0;
    long prevAltitude = 0;
    // loop until we have reach an altitude of 3 meter
    while (liftOff)
    {
      unsigned long currentTime;
      unsigned long diffTime;

      currAltitude = (ReadAltitude().altitude - initialAltitude);

      currentTime = millis() - initialTime;

      prevAltitude = currAltitude;
      SendTelemetry(currentTime, 200);
      //display
      char Altitude [15];
      currAltitude = (long)ReadAltitude().altitude - initialAltitude;
      sprintf(Altitude, "Altitude = %i meters    ", currAltitude );
      tft.setCursor (0, STATUS_HEIGHT_BAR);

      tft.println("Recording in progress .....");
      tft.println(Altitude);

      // detect apogee
      if (currAltitude < lastAltitude) {
        measures = measures - 1;
        if (measures == 0)
        {
          apogeeAltitude = lastAltitude;
          timeToApogee = currentTime;
        }
      }
      else
      {
        lastAltitude = currAltitude;
        measures = 5;
      }

      char temp [15];

      if (qmi.getDataReady()) {
        if (qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
          sprintf(temp, "x=%3.2f m/s", (float)acc.x );
          tft.println("");
          tft.println(temp);
          sprintf(temp, "y=%3.2f m/s", (float) acc.y );
          tft.println(temp);
          sprintf(temp, "z=%3.2f m/s", (float) acc.z );
          tft.println(temp);
        }
      }

      //record
      if (canRecord)
      {
        if ((currentTime - prevTime) > 150) {
          diffTime = currentTime - prevTime;
          prevTime = currentTime;

          flightLogger.setFlightTimeData(diffTime);
          flightLogger.setFlightAltitudeData(currAltitude);
          double temperature, pressure;
          //bmp.getTemperatureAndPressure(temperature, pressure);
          bmpValues val = ReadAltitude();
          flightLogger.setFlightTemperatureData((long) val.temperature);
          flightLogger.setFlightPressureData((long) val.pressure);

          if (qmi.getDataReady()) {
            if (qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
              flightLogger.setAccelX(acc.x);
              flightLogger.setAccelY(acc.y);
              flightLogger.setAccelZ(acc.z);
            }
          }
          flightLogger.addToCurrentFlight();
        }
      }


      if ((canRecord && (timeToApogee > 0) && (currAltitude < 10) && (millis() - timeToApogee) > 2000) || (canRecord  && (millis() - initialTime) > recordingTimeOut) )
      {
        //end loging
        //save flight
        liftOff = false;
        flightLogger.writeFastFlight();

        exitRecording = true;
      }
    } // end while (liftoff)
  } //end while(recording)
}

/*

   This interprets menu commands. This can be used in the commend line or
   this is used by the Android console

   Commands are as folow:
   a  get all flight data
   b  get altimeter config
   c  toggle continuity on and off
   d  reset alti config
   e  erase all saved flights
   f  FastReading on
   g  FastReading off
   h  hello. Does not do much
   i  unused
   k  folowed by a number turn on or off the selected output
   l  list all flights
   m  followed by a number turn main loop on/off. if number is 1 then
      main loop in on else turn it off
   n  Return the number of recorded flights in the EEprom
   o  requesting test trame
   r  followed by a number which is the flight number.
      This will retrieve all data for the specified flight
   s  write altimeter config
   t  reset alti config (why?)
   w  Start or stop recording
   x  delete last curve
   y  followed by a number turn telemetry on/off. if number is 1 then
      telemetry in on else turn it off
   z  send gps raw data

*/
void interpretCommandBuffer(char *commandbuffer) {
  //get all flight data
  if (commandbuffer[0] == 'a')
  {
    Serial.print(F("$start;\n"));
    int i;
    ///todo
    for (i = 0; i < flightLogger.getLastFlightNbr() + 1; i++)
    {
      flightLogger.printFlightData(i);
    }
    Serial.print(F("$end;\n"));
  }
  //get altimeter config
  else if (commandbuffer[0] == 'b')
  {
    Serial.print(F("$start;\n"));
    printAltiConfig((char *)"UltimateAltimeter");
    Serial.print(F("$end;\n"));
  }
  //toggle continuity on and off
  else if (commandbuffer[0] == 'c')
  {
    //not implemeted
  }
  //reset alti config this is equal to t why do I have 2 !!!!
  else if (commandbuffer[0] == 'd')
  {

  }
  //this will erase all flight
  else if (commandbuffer[0] == 'e')
  {
    Serial.println(F("Erase\n"));
    flightLogger.clearFlightList();
  }
  //FastReading
  else if (commandbuffer[0] == 'f')
  {
    //not implemeted
  }
  //FastReading off
  else if (commandbuffer[0] == 'g')
  {
    //not implemeted
  }
  //hello
  else if (commandbuffer[0] == 'h')
  {
    Serial.print(F("$OK;\n"));
  }
  // unused
  else if (commandbuffer[0] == 'i')
  {
    //not implemeted
  }
  //turn on or off the selected output
  else if (commandbuffer[0] == 'k')
  {
    //not implemeted
  }
  //list all flights
  else if (commandbuffer[0] == 'l')
  {
    //logger.printFlightList();
  }

  //mainloop on/off
  else if (commandbuffer[0] == 'm')
  {
    if (commandbuffer[1] == '1') {
      //mainLoopEnable = true;
    }
    else {
      //mainLoopEnable = false;
    }

    Serial.print(F("$OK;\n"));

  }
  //Number of flight
  else if (commandbuffer[0] == 'n')
  {
    char flightData[30] = "";
    char temp[9] = "";

    Serial.print(F("$start;\n"));

    strcat(flightData, "nbrOfFlight,");
    sprintf(temp, "%i,", (int)flightLogger.getLastFlightNbr() + 1 );
    strcat(flightData, temp);
    unsigned int chk = msgChk(flightData, sizeof(flightData));
    sprintf(temp, "%i", chk);
    strcat(flightData, temp);
    strcat(flightData, ";\n");

    Serial.print("$");
    Serial.print(flightData);
    Serial.print(F("$end;\n"));

  }
  // send test tram
  else if (commandbuffer[0] == 'o')
  {

  }
  //altimeter config param
  //write  config
  else if (commandbuffer[0] == 'p')
  {

  }
  else if (commandbuffer[0] == 'q')
  {
    Serial.print(F("$OK;\n"));
  }
  //this will read one flight
  else if (commandbuffer[0] == 'r')
  {
    char temp[3];

    temp[0] = commandbuffer[1];
    if (commandbuffer[2] != '\0')
    {
      temp[1] = commandbuffer[2];
      temp[2] = '\0';
    }
    else
      temp[1] = '\0';

    if (atol(temp) > -1)
    {
      Serial.print(F("$start;\n"));

      flightLogger.printFlightData(atoi(temp));

      Serial.print(F("$end;\n"));
    }
    else
      Serial.println(F("not a valid flight"));
  }
  //write altimeter config
  else if (commandbuffer[0] == 's')
  {

  }
  //reset config and set it to default
  else if (commandbuffer[0] == 't')
  {
    //reset config

  }
  else if (commandbuffer[0] == 'v')
  {

  }
  // Recording
  else if (commandbuffer[0] == 'w')
  {
    recordAltitude();
  }
  //delete last curve
  else if (commandbuffer[0] == 'x')
  {
    flightLogger.deleteLastFlight();
    /*logger.eraseLastFlight();
      logger.readFlightList();
      long lastFlightNbr = logger.getLastFlightNbr();
      if (lastFlightNbr < 0)
      {
      currentFileNbr = 0;
      currentMemaddress = 201;
      }
      else
      {
      currentMemaddress = logger.getFlightStop(lastFlightNbr) + 1;
      currentFileNbr = lastFlightNbr + 1;
      }
      canRecord = logger.CanRecord();*/
  }

  //telemetry on/off
  else if (commandbuffer[0] == 'y')
  {
    if (commandbuffer[1] == '1') {
      Serial.print(F("Telemetry enabled\n"));
      telemetryEnable = true;
    }
    else {
      Serial.print(F("Telemetry disabled\n"));
      telemetryEnable = false;
    }
    Serial.print(F("$OK;\n"));
  }

  //alti Name for ESP32
  else if (commandbuffer[0] == 'z')
  {
    //updateAltiName(commandbuffer);
    Serial.print(F("$OK;\n"));
  }
  // empty command
  else if (commandbuffer[0] == ' ')
  {
    Serial.print(F("$K0;\n"));
  }
  else
  {
    Serial.print(F("$UNKNOWN;"));
    Serial.println(commandbuffer[0]);
  }
}

unsigned int msgChk( char * buffer, long length ) {

  long index;
  unsigned int checksum;

  for ( index = 0L, checksum = 0; index < length; checksum += (unsigned int) buffer[index++] );
  return (unsigned int) ( checksum % 256 );
}

/*

   Print altimeter config to the Serial line

*/
void printAltiConfig(char *altiName)
{
  char altiConfig[160] = "";
  char temp[25] = "";

  strcat(altiConfig, "alticonfig,");

  //Unit
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //beepingMode
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //output1
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //output2
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //output3
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //supersonicYesNo
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //mainAltitude
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //AltimeterName
  strcat(altiConfig, BOARD_FIRMWARE);
  strcat(altiConfig, ",");
  //alti major version
  sprintf(temp, "%i,", MAJOR_VERSION);
  strcat(altiConfig, temp);
  //alti minor version
  sprintf(temp, "%i,", MINOR_VERSION);
  strcat(altiConfig, temp);
  //output1 delay
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //output2 delay
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //output3 delay
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //Beeping frequency
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  sprintf(temp, "%lu,", 0);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //output4
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //output4 delay
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //Lift off altitude
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //Battery type
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  // recording timeout
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //altiID
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  //useTelemetryPort
  sprintf(temp, "%i,", 0);
  strcat(altiConfig, temp);
  sprintf(temp, "%s,", altiName);
  strcat(altiConfig, temp);
  unsigned int chk = 0;
  chk = msgChk( altiConfig, sizeof(altiConfig) );
  sprintf(temp, "%i;\n", chk);
  strcat(altiConfig, temp);
  Serial.print("$");
  Serial.print(altiConfig);
}

/*
   SendTelemetry(long sampleTime, int freq)
   Send telemety so that we can plot the flight

*/
void SendTelemetry(long sampleTime, int freq) {
  char altiTelem[150] = "";
  char temp[10] = "";

  if (telemetryEnable && (millis() - lastTelemetry) > freq) {
    lastTelemetry = millis();
    int val = 0;
    //check liftoff
    int li = 0;
    if (liftOff)
      li = 1;


    int landed = 0;
    if ( liftOff && currAltitude < 10)
      landed = 1;

    strcat(altiTelem, "telemetry," );
    sprintf(temp, "%i,", currAltitude);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", li);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", -1);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", apogeeAltitude);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", -1);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", -1);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", landed);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", sampleTime);
    strcat(altiTelem, temp);
    strcat(altiTelem, "-1,");
    strcat(altiTelem, "-1,");
    strcat(altiTelem, "-1,");
    strcat(altiTelem, "-1,");

    //dtostrf(BL.getBatteryVolts(), 4, 2, temp);
    //strcat(altiTelem, temp);
    strcat(altiTelem, "-1,");

    // temperature
    bmpValues val1 = ReadAltitude();
    /*#ifdef BMP085_180
      float temperature;
      temperature = bmp.readTemperature();
      sprintf(temp, "%i,", (int)temperature );
      #endif
      #ifdef BMP280_sensor
      double temperature, pressure;
      bmp.getTemperatureAndPressure(temperature, pressure);
      sprintf(temp, "%i,", (int)temperature );
      #endif*/
    sprintf(temp, "%i,", (int) val1.temperature);
    strcat(altiTelem, temp);
    //sprintf(temp, "%i,", (int) val1.pressure);
    //strcat(altiTelem, temp);

    sprintf(temp, "%i,", -1 );
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", flightLogger.getLastFlightNbr() + 1 );
    strcat(altiTelem, temp);

    //drogueFiredAltitude
    sprintf(temp, "%i,", -1);
    strcat(altiTelem, temp);

    if (qmi.getDataReady()) {
      if (qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
        sprintf(temp, "%i,", (int)(1000 * acc.x));
        strcat(altiTelem, temp);
        sprintf(temp, "%i,", (int)(1000 * acc.y));
        strcat(altiTelem, temp);
        sprintf(temp, "%i,", (int)(1000 * acc.z));
      } else {
        sprintf(temp, "%i,", -1);
        strcat(altiTelem, temp);
        sprintf(temp, "%i,", -1);
        strcat(altiTelem, temp);
        sprintf(temp, "%i,", -1);
      }
    } else {
      sprintf(temp, "%i,", -1);
      strcat(altiTelem, temp);
      sprintf(temp, "%i,", -1);
      strcat(altiTelem, temp);
      sprintf(temp, "%i,", -1);
    }
    strcat(altiTelem, temp);

    unsigned int chk;
    chk = msgChk(altiTelem, sizeof(altiTelem));
    sprintf(temp, "%i", chk);
    strcat(altiTelem, temp);
    strcat(altiTelem, ";\n");

    Serial.print("$");
    Serial.print(altiTelem);
  }
}
