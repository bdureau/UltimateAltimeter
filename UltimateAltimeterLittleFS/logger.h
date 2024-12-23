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
  long accelX;
  long accelY;
  long accelZ;
};

struct FlightMinAndMaxStruct {
  long minAltitude;
  long maxAltitude;
  long minTemperature;
  long maxTemperature;
  long minPressure;
  long maxPressure;
  long minAccelX;
  long maxAccelX;
  long minAccelY;
  long maxAccelY;
  long minAccelZ;
  long maxAccelZ;
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
    void setAccelX(long accelX);
    void setAccelY(long accelY);
    void setAccelZ(long accelZ);
    void getFlightMinAndMax(long flightNbr);
    long getMaxTemperature();
    long getMaxPressure();
    long getMinAltitude();
    long getMaxAltitude();
    long getMinAccelX();
    long getMaxAccelX();
    long getMinAccelY();
    long getMaxAccelY();
    long getMinAccelZ();
    long getMaxAccelZ();
    long getFlightDuration();
    long getFlightSize();

  private:
    FlightDataStruct* flightData;
    FlightDataStruct currentRecord;
    long dataPos;
    FlightMinAndMaxStruct _FlightMinAndMax;
};

#endif
