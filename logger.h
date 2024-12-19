#ifndef _LOGGER_H
#define _LOGGER_H

#include <Preferences.h>

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

class logger
{
  public:
    logger();
    bool clearFlightList();
    long readFlightList();
    long getLastFlightNbr();
    bool writeFlight(long);
    bool eraseLastFlight();
    bool readFlight(long);
    bool writeFastFlight();
    bool addToCurrentFlight();
    bool initFlight();
    FlightDataStruct* getFlightData();


    void setFlightTimeData( long difftime);
    void setFlightAltitudeData( long altitude);
    void setFlightPressureData( long pressure);
    void setFlightHumidityData( long humidity);
    void setFlightTemperatureData(long temperature);
    void setAccelX(long accelX);
    void setAccelY(long accelY);
    void setAccelZ(long accelZ);
    void getFlightMinAndMax(long flightNbr);

    long getMinAltitude();
    long getMaxAltitude();
    long getMaxTemperature();
    long getMaxPressure();
    long getMinAccelX();
    long getMaxAccelX();
    long getMinAccelY();
    long getMaxAccelY();
    long getMinAccelZ();
    long getMaxAccelZ();
    long getFlightDuration();
    long getFlightSize();

  private:
    Preferences prefs;
    FlightDataStruct* flightData;
    FlightDataStruct currentRecord;
    long dataPos;
    FlightMinAndMaxStruct _FlightMinAndMax;
};

#endif
