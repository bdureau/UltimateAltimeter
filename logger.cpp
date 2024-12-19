#include "logger.h"

logger::logger() {
  // Initialize flightData as nullptr and dataPos to 0
  flightData = nullptr;
  dataPos = 0;
}

/*
   clearFlightList()
   Clear the flight list.

*/
bool logger::clearFlightList() {
  bool success;
  prefs.begin(WORKSPACE);
  success = prefs.clear();
  prefs.end();
  #ifdef DEBUG
  if(success)
    Serial.println(F("success"));
  #endif
  return success;
}

/*
   readFlightList()
*/
long logger::readFlightList() {
  long numberOfFlights = 0;
  prefs.begin(WORKSPACE);

  prefs.end();
  return numberOfFlights;
}

long logger::getLastFlightNbr() {
  #ifdef DEBUG
  Serial.println(F("Begin getLastFlightNbr"));
  #endif
  long numberOfFlights = 0;
  prefs.begin(WORKSPACE);
  char flightName [15];
  bool exit =false;
  
  while(!exit) {
    numberOfFlights++;
    sprintf(flightName, "flight%i", numberOfFlights );
    if(!prefs.isKey(flightName)){
      exit = true;
      numberOfFlights--;
    }
  }
  
  prefs.end();
  #ifdef DEBUG
  Serial.println(F("End getLastFlightNbr"));
  #endif

  return numberOfFlights;
}

FlightDataStruct* logger::getFlightData() {
  return flightData;
}

bool logger::writeFlight(long flightNbr) {
  bool success = false;
  prefs.begin(WORKSPACE);
  char flightName [15];
  sprintf(flightName, "flight%i", flightNbr );
  // Write current flight data
  if (flightData != nullptr) {
    prefs.putBytes(flightName, flightData, sizeof(FlightDataStruct) * dataPos);
    success = true;
  }
  prefs.end();
  return success;
}

bool logger::readFlight(long flightNbr) {
  bool success = false;
  prefs.begin(WORKSPACE);
  char flightName [15];
  sprintf(flightName, "flight%i", flightNbr );
  Serial.println(F("Reading flight:"));
  Serial.println(flightName);
  //retrieve flight
  // Retrieve the flight data from preferences
  size_t schLen = prefs.getBytesLength(flightName);  // Get the total length of the stored data for the flight
  if (schLen == 0) {
    Serial.println(F("No data found."));
    prefs.end();
    return false;
  }
  // Calculate how many flight_t structures we have stored (this is safe because the size of flight_t is known)
  size_t numFlights = schLen / sizeof(FlightDataStruct);

  Serial.print(F("Number of flights found: "));
  Serial.println(numFlights);
  //check if the current flight is empty or not
  //if not empty free it first to avoid memory leak
  if (flightData != nullptr) {
    free(flightData);
  }
  // Dynamically allocate memory for the retrieved flight data array
  flightData = (FlightDataStruct*)malloc(schLen);

  if (flightData == nullptr) {
    #ifdef DEBUG
    Serial.println(F("Memory allocation failed."));
    #endif
    prefs.end();
    return false;
  }

  // Retrieve the stored bytes into the dynamically allocated array
  prefs.getBytes(flightName, flightData, schLen);
  dataPos = numFlights; // Update data position

  prefs.end();
  return true;
}

bool logger::writeFastFlight() {
  bool success = false;
  long lastFlightNbr = getLastFlightNbr();
  Serial.print(F("lastFlightNbr"));
  Serial.println(lastFlightNbr);
  lastFlightNbr++;

  char flightName [15];
  sprintf(flightName, "flight%i", lastFlightNbr );

  prefs.begin(WORKSPACE);
  size_t whatsLeft = prefs.freeEntries();    // this method works regardless of the mode in which the namespace is opened.
  Serial.printf("There are: %u entries available in the namespace table.\n", whatsLeft);
  // Check if flightData is valid before writing
  if (flightData != nullptr && dataPos > 0) {
    Serial.println(F("Writting flight:"));
    Serial.println(flightName);
    success = prefs.putBytes(flightName, flightData, sizeof(FlightDataStruct) * dataPos);
    //success = true;
    if(success)
      Serial.println(F("flight write success"));
  }
  prefs.end();
  return success;
}

bool logger::addToCurrentFlight() {
  bool success = false;
  
  // Allocate memory if necessary
  FlightDataStruct* newFlightData = (FlightDataStruct*)realloc(flightData, (dataPos + 1) * sizeof(FlightDataStruct));

  if (newFlightData != nullptr) {
    flightData = newFlightData;
    flightData[dataPos] = currentRecord;  // Add new record to flight data
    dataPos++;
    success = true;
  } else {
    Serial.println(F("Reallocation failed."));
  }
  return success;
}
bool logger::initFlight() {
  // Free previously allocated memory for flightData
  if (flightData != nullptr)
    free(flightData);
  flightData = nullptr;
  dataPos = 0;
  return true;
}

bool logger::eraseLastFlight() {
  long success = false;
  long lastFlightNbr = getLastFlightNbr();
  if (lastFlightNbr > 0) {
    prefs.begin(WORKSPACE);
    char flightName [15];
    sprintf(flightName, "flight%i", lastFlightNbr );
    success = prefs.remove(flightName);
    /*if(success)
    Serial.println("Erase success");*/
    prefs.end();
  }
  return success;
}

void logger::setFlightTimeData( long diffTime) {
  currentRecord.diffTime = diffTime;
}

void logger::setFlightAltitudeData( long altitude) {
  currentRecord.altitude = altitude;
}

/*void logger::setFlightPressureData( long pressure) {
  currentRecord.pressure = pressure;
}

void logger::setFlightTemperatureData(long temperature) {
  currentRecord.temperature = temperature;
}

void logger::setFlightHumidityData( long humidity) {
  currentRecord.humidity = humidity;
}*/

void logger::setAccelX(long accelX) {
  currentRecord.accelX = accelX;
}

void logger::setAccelY(long accelY) {
  currentRecord.accelY = accelY;
}

void logger::setAccelZ(long accelZ) {
  currentRecord.accelZ = accelZ;
}

void logger::getFlightMinAndMax(long flightNbr)
{

  _FlightMinAndMax.minAltitude = 0;
  _FlightMinAndMax.maxAltitude = 0;
  /*_FlightMinAndMax.minTemperature = 0;
  _FlightMinAndMax.maxTemperature = 0;
  _FlightMinAndMax.minPressure = 0;
  _FlightMinAndMax.maxPressure = 0;*/
  _FlightMinAndMax.minAccelX = 0;
  _FlightMinAndMax.maxAccelX = 0;
  _FlightMinAndMax.minAccelY = 0;
  _FlightMinAndMax.maxAccelY = 0;
  _FlightMinAndMax.minAccelZ = 0;
  _FlightMinAndMax.maxAccelZ = 0;
  _FlightMinAndMax.flightDuration = 0;

  if (readFlight(flightNbr)) {
    for (long i = 0; i < dataPos ; i++)
    {
      _FlightMinAndMax.flightDuration = _FlightMinAndMax.flightDuration + flightData[i].diffTime;

      if (flightData[i].altitude < _FlightMinAndMax.minAltitude)
        _FlightMinAndMax.minAltitude = flightData[i].altitude;
      if (flightData[i].altitude > _FlightMinAndMax.maxAltitude)
        _FlightMinAndMax.maxAltitude = flightData[i].altitude;

      /*if (flightData[i].temperature < _FlightMinAndMax.minTemperature)
        _FlightMinAndMax.minTemperature = flightData[i].temperature;
      if (flightData[i].temperature > _FlightMinAndMax.maxTemperature)
        _FlightMinAndMax.maxTemperature = flightData[i].temperature;

      if (flightData[i].pressure < _FlightMinAndMax.minPressure)
        _FlightMinAndMax.minPressure = flightData[i].pressure;
      if (flightData[i].pressure > _FlightMinAndMax.maxTemperature)
        _FlightMinAndMax.maxPressure = flightData[i].pressure;*/

      //long accelX = getADXL345accelX();
      if (flightData[i].accelX < _FlightMinAndMax.minAccelX)
        _FlightMinAndMax.minAccelX = flightData[i].accelX;
      if (flightData[i].accelX > _FlightMinAndMax.maxAccelX)
        _FlightMinAndMax.maxAccelX = flightData[i].accelX;

      //long accelY = getADXL345accelY();
      if (flightData[i].accelY < _FlightMinAndMax.minAccelY)
        _FlightMinAndMax.minAccelX = flightData[i].accelY;
      if (flightData[i].accelY > _FlightMinAndMax.maxAccelY)
        _FlightMinAndMax.maxAccelX = flightData[i].accelY;

      //long accelZ = getADXL345accelZ();
      if (flightData[i].accelZ < _FlightMinAndMax.minAccelZ)
        _FlightMinAndMax.minAccelZ = flightData[i].accelZ;
      if (flightData[i].accelZ > _FlightMinAndMax.maxAccelZ)
        _FlightMinAndMax.maxAccelZ = flightData[i].accelZ;
    }
  }
}

long logger::getFlightDuration()
{
  return _FlightMinAndMax.flightDuration;
}

long logger::getMinAltitude()
{
  return _FlightMinAndMax.minAltitude;
}

long logger::getMaxAltitude()
{
  return _FlightMinAndMax.maxAltitude;
}

/*long logger::getMaxTemperature()
{
  return _FlightMinAndMax.maxTemperature;
}

long logger::getMaxPressure()
{
  return _FlightMinAndMax.maxPressure;
}*/

long logger::getMaxAccelX()
{
  return _FlightMinAndMax.maxAccelX;
}

long logger::getMinAccelX()
{
  return _FlightMinAndMax.minAccelX;
}

long logger::getMinAccelY()
{
  return _FlightMinAndMax.minAccelY;
}
long logger::getMaxAccelY()
{
  return _FlightMinAndMax.maxAccelY;
}

long logger::getMinAccelZ()
{
  return _FlightMinAndMax.minAccelZ;
}

long logger::getMaxAccelZ()
{
  return _FlightMinAndMax.maxAccelZ;
}

long logger::getFlightSize() {
  return dataPos;
}
