#include "logger.h"

logger::logger() {
  flightData = nullptr;
  dataPos = 0;
}

bool logger::initFileSystem() {
  if (!LittleFS.begin(false)) {
#ifdef DEBUG
    Serial.println("LITTLEFS Mount failed");
    Serial.println("Did not find filesystem; starting format");
#endif
    // format if begin fails
    if (!LittleFS.begin(true)) {
#ifdef DEBUG
      Serial.println("LITTLEFS mount failed");
      Serial.println("Formatting not possible");
#endif
      return false;
    } else {
      Serial.println("Formatting");
    }
  }
  return true;
}

bool logger::clearFlightList() {
#ifdef DEBUG
  Serial.println(F("Clearing all flights..."));
#endif
  // Open the root directory
  File root = LittleFS.open("/");
  if (!root || !root.isDirectory()) {
#ifdef DEBUG
    Serial.println(F("Failed to open the root directory"));
#endif
    return false;
  }

  // Iterate over all files in the root directory
  File file = root.openNextFile();
  while (file) {
#ifdef DEBUG
    Serial.print(F("Deleting file: "));
    Serial.println(file.name());
#endif
    // Delete the current file
    char fileName [15];
    sprintf(fileName, "/%s", file.name() );
    //make sure that you close the file so that it can be deleted
    file.close();
    //if (LittleFS.remove(file.name())) {
    if (LittleFS.remove(fileName)) {
#ifdef DEBUG
      Serial.println(F("Flight deleted successfully"));
#endif
    } else {
#ifdef DEBUG
      Serial.println(F("Failed to delete flight"));
#endif
    }

    // Move to the next file
    file = root.openNextFile();
  }
#ifdef DEBUG
  Serial.println(F("All files cleared."));
#endif
  return true;
}

long logger::getLastFlightNbr() {
  long maxFlightNumber = 0; // Default to 0 if no valid flight file is found

  // Open the root directory
  File root = LittleFS.open("/");
  if (!root || !root.isDirectory()) {
#ifdef DEBUG
    Serial.println(F("Failed to open the root directory"));
#endif
    return maxFlightNumber;
  }

  // Iterate over all files in the root directory
  File file = root.openNextFile();
  while (file) {
    String fileName = file.name();
    #ifdef DEBUG
    Serial.print("Found file: ");
    Serial.println(fileName);
    #endif

    // Check if the filename matches the pattern "flight<number>.json"
    if (fileName.startsWith("flight") && fileName.endsWith(".json")) {
      // Extract the flight number
      String numberPart = fileName.substring(6, fileName.length() - 5); // Extract between "flight" and ".json"
      int flightNumber = numberPart.toInt(); // Convert to integer

      // Update the maxFlightNumber if this one is larger
      if (flightNumber > maxFlightNumber) {
        maxFlightNumber = flightNumber;
      }
    }

    // Move to the next file
    file = root.openNextFile();
  }

  return maxFlightNumber;
}

bool logger::writeFastFlight() {
  return writeFlight(getLastFlightNbr() + 1);
}

bool logger::writeFlight(long flightNbr) {
  char flightName [15];
  sprintf(flightName, "/flight%i.json", flightNbr);

  DynamicJsonDocument doc(4096);
  JsonObject flight = doc.createNestedObject(flightName);

  for (long i = 0; i < dataPos; i++) {
    JsonObject record = flight.createNestedObject(String(i));
    record["diffTime"] = flightData[i].diffTime;
    record["altitude"] = flightData[i].altitude;
    record["accelX"] = flightData[i].accelX;
    record["accelY"] = flightData[i].accelY;
    record["accelZ"] = flightData[i].accelZ;
    record["pressure"] = flightData[i].pressure;
    record["humidity"] = flightData[i].humidity;
    record["temperature"] = flightData[i].temperature;
  }

  //Serial.println(String(doc)));
  File file = LittleFS.open(flightName, "w");
  serializeJson(doc, file);
  file.close();

  return true;
}

bool logger::readFlight(long flightNbr) {
  char flightName [15];
  sprintf(flightName, "/flight%i.json", flightNbr);
  #ifdef DEBUG
  Serial.println(flightName);
  #endif
  File file = LittleFS.open(flightName, "r");
  if (!file) return false;

  DynamicJsonDocument doc(4096);
  deserializeJson(doc, file);
  file.close();

  JsonObject flight = doc[flightName];
  if (flight.isNull()) return false;

  dataPos = flight.size();
  if (flightData != nullptr) free(flightData);

  flightData = (FlightDataStruct*)malloc(sizeof(FlightDataStruct) * dataPos);
  if (!flightData) return false;

  long index = 0;
  for (JsonPair pair : flight) {
    JsonObject record = pair.value().as<JsonObject>();
    flightData[index].diffTime = record["diffTime"];
    flightData[index].altitude = record["altitude"];
    flightData[index].accelX = record["accelX"];
    flightData[index].accelY = record["accelY"];
    flightData[index].accelZ = record["accelZ"];
    flightData[index].pressure = record["pressure"];
    flightData[index].humidity = record["humidity"];
    flightData[index].temperature = record["temperature"];
    index++;
  }

  return true;
}

bool logger::addToCurrentFlight() {
  FlightDataStruct* newFlightData = (FlightDataStruct*)realloc(flightData, (dataPos + 1) * sizeof(FlightDataStruct));
  if (!newFlightData) return false;

  flightData = newFlightData;
  flightData[dataPos] = currentRecord;
  dataPos++;
  return true;
}

bool logger::initFlight() {
  if (flightData != nullptr) free(flightData);
  flightData = nullptr;
  dataPos = 0;
  return true;
}

FlightDataStruct* logger::getFlightData() {
  return flightData;
}

void logger::setFlightTimeData(long diffTime) {
  currentRecord.diffTime = diffTime;
}

void logger::setFlightAltitudeData(long altitude) {
  currentRecord.altitude = altitude;
}
void logger::setFlightPressureData( long pressure) {
  currentRecord.pressure = pressure;
}

void logger::setFlightTemperatureData(long temperature) {
  currentRecord.temperature = temperature;
}

void logger::setFlightHumidityData( long humidity) {
  currentRecord.humidity = humidity;
}

void logger::setAccelX(float accelX) {
  currentRecord.accelX = accelX;
}

void logger::setAccelY(float accelY) {
  currentRecord.accelY = accelY;
}

void logger::setAccelZ(float accelZ) {
  currentRecord.accelZ = accelZ;
}

long logger::getFlightSize() {
  return dataPos;
}

void logger::getFlightMinAndMax(long flightNbr)
{

  _FlightMinAndMax.minAltitude = 0;
  _FlightMinAndMax.maxAltitude = 0;
  _FlightMinAndMax.minTemperature = 0;
  _FlightMinAndMax.maxTemperature = 0;
  _FlightMinAndMax.minPressure = 0;
  _FlightMinAndMax.maxPressure = 0;
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

      if (flightData[i].temperature < _FlightMinAndMax.minTemperature)
        _FlightMinAndMax.minTemperature = flightData[i].temperature;
      if (flightData[i].temperature > _FlightMinAndMax.maxTemperature)
        _FlightMinAndMax.maxTemperature = flightData[i].temperature;

      if (flightData[i].pressure < _FlightMinAndMax.minPressure)
        _FlightMinAndMax.minPressure = flightData[i].pressure;
      if (flightData[i].pressure > _FlightMinAndMax.maxTemperature)
        _FlightMinAndMax.maxPressure = flightData[i].pressure;


      if (flightData[i].accelX < _FlightMinAndMax.minAccelX)
        _FlightMinAndMax.minAccelX = flightData[i].accelX;
      if (flightData[i].accelX > _FlightMinAndMax.maxAccelX)
        _FlightMinAndMax.maxAccelX = flightData[i].accelX;


      if (flightData[i].accelY < _FlightMinAndMax.minAccelY)
        _FlightMinAndMax.minAccelX = flightData[i].accelY;
      if (flightData[i].accelY > _FlightMinAndMax.maxAccelY)
        _FlightMinAndMax.maxAccelX = flightData[i].accelY;


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

long logger::getMaxTemperature()
{
  return _FlightMinAndMax.maxTemperature;
}

long logger::getMaxPressure()
{
  return _FlightMinAndMax.maxPressure;
}
long logger::getMaxHumidity()
{
  return _FlightMinAndMax.maxHumidity;
}
long logger::getMinHumidity()
{
  return _FlightMinAndMax.minHumidity;
}

float logger::getMaxAccelX()
{
  return _FlightMinAndMax.maxAccelX;
}

float logger::getMinAccelX()
{
  return _FlightMinAndMax.minAccelX;
}

float logger::getMinAccelY()
{
  return _FlightMinAndMax.minAccelY;
}
float logger::getMaxAccelY()
{
  return _FlightMinAndMax.maxAccelY;
}

float logger::getMinAccelZ()
{
  return _FlightMinAndMax.minAccelZ;
}

float logger::getMaxAccelZ()
{
  return _FlightMinAndMax.maxAccelZ;
}

void logger::printFlightData(int flightNbr)
{

  unsigned long currentTime = 0;

  if (readFlight(flightNbr)) {
    for (long i = 0; i < dataPos ; i++)
    {
      char flightDt[120] = "";
      char temp[20] = "";
      currentTime = currentTime + flightData[i].diffTime;
      strcat(flightDt, "data,");
      sprintf(temp, "%i,", flightNbr );
      strcat(flightDt, temp);
      sprintf(temp, "%i,", currentTime );
      strcat(flightDt, temp);
      sprintf(temp, "%i,", flightData[i].altitude );
      strcat(flightDt, temp);
      sprintf(temp, "%i,", flightData[i].temperature );
      strcat(flightDt, temp);
      sprintf(temp, "%i,", flightData[i].pressure );
      strcat(flightDt, temp);
      sprintf(temp, "%i,", 0 ); //dummy voltage
      strcat(flightDt, temp);
      sprintf(temp, "%i,", flightData[i].accelX * 1000 );
      strcat(flightDt, temp);
      sprintf(temp, "%i,", flightData[i].accelY * 1000);
      strcat(flightDt, temp);
      sprintf(temp, "%i,", flightData[i].accelZ * 1000 );
      strcat(flightDt, temp);

      unsigned int chk = msgChk(flightDt, sizeof(flightDt));
      sprintf(temp, "%i", chk);
      strcat(flightDt, temp);
      strcat(flightDt, ";\n");

      Serial.print("$");
      Serial.print(flightDt);
    }
  }
}
unsigned int logger::msgChk( char * buffer, long length ) {

  long index;
  unsigned int checksum;

  for ( index = 0L, checksum = 0; index < length; checksum += (unsigned int) buffer[index++] );
  return (unsigned int) ( checksum % 256 );

}
