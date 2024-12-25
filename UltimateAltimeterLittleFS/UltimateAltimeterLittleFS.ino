/*
    UltimateAltimeter ver0.1
    Copyright Boris du Reau 2012-2024
    This is using a BME280 presure sensor
    for the accelerometer

    !!!IMPORTANT!!!!
    to compile it use the board "Adafruit Feather ESP32-S3 TFT"
    !!!IMPORTANT!!!!
    The flight is recorded on the board using LITTLE FS rather than the preferences library so that we can store longer flight

    Major changes on version 0.1
    Initial version of the code, this is re-using code from the TTGO board
    can display altitude, acceleration, temperature and pressure

*/
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Button2.h>
#include <TFT_eWidget.h>
#include <driver/rtc_io.h>
#include <Wire.h>
#include <EnvironmentCalculations.h>
#include "SensorQMI8658.hpp"
#include <BME280I2C.h>
#include "kalman.h"
#include "logger.h"
#include "images/bear_altimeters128x128.h"
#include "images/battery_01.h"
#include "images/battery_02.h"
#include "images/battery_03.h"
#include "images/battery_04.h"
#include "images/battery_05.h"

//#define DEBUG
struct bmeValues {
  float pressure; 
  float temperature;
  float humidity;
  float altitude;
};

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
#define ICON_WIDTH 70
#define ICON_HEIGHT 36
#define STATUS_HEIGHT_BAR ICON_HEIGHT
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))
#define ICON_POS_X (tft.width() - ICON_WIDTH)

#define MIN_USB_VOL 4.7
#define ADC_PIN 34
#define CONV_FACTOR 1.8
#define READS 20

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
float referencePressure = 1018.6;  // hPa local QFF (official meteor-station reading)
float outdoorTemp = 4.7;           // °C  measured local outdoor temp.
float barometerAltitude = 1650.3;  // meters ... map readings + barometer position

BME280I2C::Settings settings(
  BME280::OSR_X1,
  BME280::OSR_X1,
  BME280::OSR_X1,
  BME280::Mode_Forced,
  BME280::StandbyTime_1000ms,
  BME280::Filter_Off,
  BME280::SpiEnable_False,
  BME280I2C::I2CAddr_0x77 // I2C address. I2C specific.
);

BME280I2C bme(settings);

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
unsigned long timeToApogee = 0;

SensorQMI8658 qmi;

IMUdata acc;
IMUdata gyr;

Kalman KalmanAltitude;

/*
   drawingText(String text)

*/
void drawingText(String text) {
  tft.fillRect(0, 0, ICON_POS_X, ICON_HEIGHT, TFT_BLACK);
  tft.setTextDatum(5);
  tft.drawString(text, ICON_POS_X - 2, STATUS_HEIGHT_BAR / 2, 4);
}

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
    // Up
    /*Serial.println("Changing curve type");// It's called downCmd because it decreases the index of an array. Visually that would mean the selector goes upwards.
      if (inGraph) {
      long lastFlightNbr = flightLogger.getLastFlightNbr();
      //Make sure we have not reach the last flight
      if (lastFlightNbr >= diplayedFlightNbr) {
        if (currentCurveType < 3) {
          currentCurveType++;
          drawFlightNbr(diplayedFlightNbr, currentCurveType);
        } else {
          currentCurveType = 0;
          drawFlightNbr(diplayedFlightNbr, currentCurveType);
        }
      }
      }*/
    // Down
    Serial.println("Button Down fast"); // It's called upCmd because it increases the index of an array. Visually that would mean the selector goes downwards.
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
          if(currentCurveType > 4) {
          diplayedFlightNbr = 1;
          currentCurveType = 0;
          }
        }
      }
      Serial.print("Flight:");
      Serial.println(diplayedFlightNbr);
      Serial.print("type:");
      Serial.println(currentCurveType);
      
      drawFlightNbr(diplayedFlightNbr, currentCurveType);
    }
  });

  btn.setLongClickHandler([](Button2 & b) {

    Serial.println("Button Down slow");
    unsigned int time = b.wasPressedFor();
    // Exit
    if (time >= 3000 & time < 10000 & !inGraph) {
      //Serial.println("Turning off");
      //inGraph = false;
      //SerialCom.end();//carefull it might crash
      enter_sleep();
    }

    if (time >= 1000 & time < 3000) {
      if (!inGraph) {
        long lastFlightNbr = flightLogger.getLastFlightNbr();
        Serial.print("lastFlightNbr:");
        Serial.println(lastFlightNbr);
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
        //Serial.println(gr.getTextPadding());
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
      //logger.writeFlightList();
      //currentFileNbr = 0;
      //currentMemaddress = 201;
    }

  });

  /* btnDwn.setClickHandler([](Button2 & b) {
    // Down
    Serial.println("Button Down fast"); // It's called upCmd because it increases the index of an array. Visually that would mean the selector goes downwards.
    if (inGraph) {
      long lastFlightNbr = logger.getLastFlightNbr();
      //Make sure we have no reach the last flight
      if (lastFlightNbr > diplayedFlightNbr) {

        diplayedFlightNbr ++;
        Serial.print("Flight:");
        Serial.println(diplayedFlightNbr);
        drawFlightNbr(diplayedFlightNbr, currentCurveType);
      } else {
        // if not lets go back to the first one if it exists
        if (!(lastFlightNbr < 0)) {
          diplayedFlightNbr = 0;
          drawFlightNbr(diplayedFlightNbr, currentCurveType);
        }
      }
    }
    });*/
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
  Serial.begin(115200);

  if (!flightLogger.initFileSystem()) {
    //delay(5000);
    Serial.println(F("Failed to initialize file system"));
  } else {
    Serial.println(F("initFileSystem Ok"));
  }

  flightLogger.initFlight();

  tft.init();
  tft.fillScreen(TFT_BLACK);
  tft.setSwapBytes(true);

  tft.pushImage(6, 0, 128, 128, bear_altimeters128x128);
  tft.drawString("Ultimate Altimeter", 6, 135);
  tft.drawString("ver 1.0", 6, 145);
  tft.drawString("Copyright", 6, 155);
  tft.drawString("Boris du Reau", 6, 165);
  tft.drawString("2012-2024", 6, 175);
  tft.drawString("Initializing....", 6, 185);

  while (!bme.begin())
  {
    tft.drawString("BMe280 error", 6, 195);
    delay(500);
  }

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
/*void loop2() {

  button_loop();
  currAltitude = (long)ReadAltitude() - initialAltitude;
  if (!( currAltitude > liftoffAltitude) )
  {
    if (!inGraph) {
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
  } else {
    Serial.println("Recording!!!!");
    recordAltitude();
  }
  }*/
void loop() {



  char readVal = ' ';
  int i = 0;

  char commandbuffer[100];

  while ( readVal != ';')
  {
    button_loop();

    currAltitude = (long)ReadAltitude().altitude - initialAltitude;

    if (!( currAltitude > liftoffAltitude) )
    {

      if (!inGraph) {
        //SendTelemetry(0, 500);
        tft.setCursor (0, STATUS_HEIGHT_BAR);


        /* if (lc.cellVoltage() >= MIN_USB_VOL) {
           drawingText("Chrg");
           tft_output(ICON_POS_X, 0, ICON_WIDTH, 36, (uint16_t*) battery_01);
           delay(500);
           tft_output(ICON_POS_X, 0, ICON_WIDTH, 36, (uint16_t*) battery_02);
           delay(500);
           tft_output(ICON_POS_X, 0, ICON_WIDTH, 36, (uint16_t*) battery_03);
           delay(500);
           tft_output(ICON_POS_X, 0, ICON_WIDTH, 36, (uint16_t*) battery_04);
           delay(500);
           tft_output(ICON_POS_X, 0, ICON_WIDTH, 36, (uint16_t*) battery_05);
           delay(500);
          } else {

           //int batteryLevel = maxlipo.cellPercent();
           int batteryLevel = lc.cellPercent();
           if (batteryLevel >= 80) {
             tft_output(ICON_POS_X, 0, 70, 36, (uint16_t*) battery_04);
           } else if (batteryLevel < 80 && batteryLevel >= 50 ) {
             tft_output(ICON_POS_X, 0, 70, 36, (uint16_t*) battery_03);
           } else if (batteryLevel < 50 && batteryLevel >= 20 ) {
             tft_output(ICON_POS_X, 0, 70, 36, (uint16_t*) battery_02);
           } else if (batteryLevel < 20 ) {
             tft_output(ICON_POS_X, 0, 70, 36, (uint16_t*) battery_01);
           }
           drawingText(String(batteryLevel) + "%");
          }*/
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
  //interpretCommandBuffer(commandbuffer);
}



/*

   Read Altitude function for a BMP280 Bosch sensor


*/
bmeValues ReadAltitude()
{
  bmeValues values;
  float temp(NAN), hum(NAN), pres(NAN);

  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);

  bme.read(pres, temp, hum, tempUnit, presUnit);
  EnvironmentCalculations::AltitudeUnit envAltUnit  =  EnvironmentCalculations::AltitudeUnit_Meters;
  EnvironmentCalculations::TempUnit     envTempUnit =  EnvironmentCalculations::TempUnit_Celsius;
  float altitude = EnvironmentCalculations::Altitude(pres, envAltUnit, referencePressure, outdoorTemp, envTempUnit);
  values.pressure = pres;
  values.temperature = temp;
  values.humidity = hum;
  values.altitude = KalmanAltitude.KalmanCalc(altitude);
  return values;
}

/*

   enter_sleep()

*/
void enter_sleep()
{
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(0);
  tft.drawString("turning off...", 6, 185);
  digitalWrite(4, LOW);
  delay(2000);
  pinMode(BUTTON_GPIO, INPUT_PULLUP);
  rtc_gpio_hold_en(BUTTON_GPIO);
  esp_sleep_enable_ext0_wakeup(BUTTON_GPIO, LOW);
  esp_deep_sleep_start();
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
  tft.drawNumber(maxY, gr.getPointX(0.0), gr.getPointY(maxY));
}

long  roundUp(float val) {
  long ret = (long)val; 
  if (val > ret) {   
    return ret + 1;     
  }
  return ret;          
}
/*
   drawFlightNbr(int flightNbr)

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
      drawAxesXY(0.0, flightLogger.getFlightDuration(), 0, (float) flightLogger.getMaxAltitude(), flightNbr, "Altitude (meters)" );
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
    if (curveType == 4) {
      trHumidity.startTrace(TFT_YELLOW);
      Serial.println(flightLogger.getMaxHumidity());
      drawAxesXY(0.0, flightLogger.getFlightDuration(), 0, (float) roundUp(flightLogger.getMaxHumidity()+1), flightNbr, "Hum %" );
    }
    
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
      if ( curveType == 4) {
        trHumidity.addPoint(currentTime, currentFlight[i].humidity);
      }
    }
  }
}

/*
   recordAltitude()

*/
void recordAltitude()
{
  long recordingTimeOut = 120 * 1000;

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
        if ((currentTime - prevTime) > 10) {
          diffTime = currentTime - prevTime;
          prevTime = currentTime;

          flightLogger.setFlightTimeData(diffTime);
          flightLogger.setFlightAltitudeData(currAltitude);
          double temperature, pressure;
          //bmp.getTemperatureAndPressure(temperature, pressure);
          bmeValues val = ReadAltitude();
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


      if ((canRecord  && (currAltitude < 10) && (millis() - timeToApogee) > 2000) || (canRecord  && (millis() - initialTime) > recordingTimeOut) )
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
