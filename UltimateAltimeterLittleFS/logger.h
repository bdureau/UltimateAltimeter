#ifndef _LOGGER_H
#define _LOGGER_H

#include <ArduinoJson.h>
#include <LittleFS.h>

#define FORMAT_LITTLEFS_IF_FAILED true

struct FlightDataStruct {
  long diffTime;
  long altitude;
  long temperature;
  long humidity;
  long pressure;
  float accelX;
  float accelY;
  float accelZ;
};

struct FlightMinAndMaxStruct {
  long minAltitude;
  long maxAltitude;
  long minTemperature;
  long maxTemperature;
  long minPressure;
  long maxPressure;
  long minHumidity;
  long maxHumidity;
  float minAccelX;
  float maxAccelX;
  float minAccelY;
  float maxAccelY;
  float minAccelZ;
  float maxAccelZ;
  long flightDuration;
};

class logger {
  public:
    logger();
    bool initFileSystem();
    bool clearFlightList();
    long getLastFlightNbr();
    bool writeFlight(long flightNbr);
    bool writeFastFlight();
    bool readFlight(long flightNbr);
    bool addToCurrentFlight();
    bool initFlight();
    FlightDataStruct* getFlightData();
    void setFlightTimeData(long diffTime);
    void setFlightAltitudeData(long altitude);
    void setFlightPressureData( long pressure);
    void setFlightHumidityData( long humidity);
    void setFlightTemperatureData(long temperature);
    void setAccelX(float accelX);
    void setAccelY(float accelY);
    void setAccelZ(float accelZ);
    void getFlightMinAndMax(long flightNbr);
    long getMaxTemperature();
    long getMaxPressure();
    long getMinAltitude();
    long getMaxAltitude();
    long getMinHumidity();
    long getMaxHumidity();
    float getMinAccelX();
    float getMaxAccelX();
    float getMinAccelY();
    float getMaxAccelY();
    float getMinAccelZ();
    float getMaxAccelZ();
    long getFlightDuration();
    long getFlightSize();
    void printFlightData(int flightNbr);

  private:
    FlightDataStruct* flightData;
    FlightDataStruct currentRecord;
    long dataPos;
    unsigned int msgChk( char * buffer, long length );
    FlightMinAndMaxStruct _FlightMinAndMax;
};

#endif
