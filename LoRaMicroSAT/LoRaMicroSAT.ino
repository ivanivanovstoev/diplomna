
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

#include "SerialLoRa.h"
#include "protocol_parser.h"

/* Assign a unique ID to the sensors */
Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
HardwareSerial GPS(2);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

namespace  sp = sat_protocol;

void displaySensorDetails(void)
{
  sensor_t sensor;

  accel.getSensor(&sensor);
  String sensorInfo = "";
  sensorInfo = "----------- ACCELEROMETER ----------";
  sensorInfo += "Sensor:       ";sensorInfo += sensor.name;
  sensorInfo += "Driver Ver:   ";sensorInfo += sensor.version;
  sensorInfo += "Unique ID:    ";sensorInfo += sensor.sensor_id;
  sensorInfo += "Max Value:    ";sensorInfo += sensor.max_value;sensorInfo += " m/s^2";
  sensorInfo += "Min Value:    ";sensorInfo += sensor.min_value;sensorInfo += " m/s^2";
  sensorInfo += "Resolution:   ";sensorInfo += sensor.resolution;sensorInfo += " m/s^2";
  sensorInfo += "------------------------------------";

  gyro.getSensor(&sensor);
  sensorInfo += "------------- GYROSCOPE -----------";
  sensorInfo += "Sensor:       ";sensorInfo += sensor.name;
  sensorInfo += "Driver Ver:   ";sensorInfo += sensor.version;
  sensorInfo += "Unique ID:    ";sensorInfo += sensor.sensor_id;
  sensorInfo += "Max Value:    ";sensorInfo += sensor.max_value;sensorInfo += " rad/s";
  sensorInfo += "Min Value:    ";sensorInfo += sensor.min_value;sensorInfo += " rad/s";
  sensorInfo += "Resolution:   ";sensorInfo += sensor.resolution;sensorInfo += " rad/s";
  sensorInfo += "------------------------------------";

  mag.getSensor(&sensor);
  sensorInfo += "----------- MAGNETOMETER -----------";
  sensorInfo += "Sensor:       ";sensorInfo += sensor.name;
  sensorInfo += "Driver Ver:   ";sensorInfo += sensor.version;
  sensorInfo += "Unique ID:    ";sensorInfo += sensor.sensor_id;
  sensorInfo += "Max Value:    ";sensorInfo += sensor.max_value;sensorInfo += " uT";
  sensorInfo += "Min Value:    ";sensorInfo += sensor.min_value;sensorInfo += " uT";
  sensorInfo += "Resolution:   ";sensorInfo += sensor.resolution;sensorInfo += " uT";
  sensorInfo += "------------------------------------";

  pp.SendMsg(sp::CMD_INFO, sp::INS_SENSOR_INF, (const unsigned char *)sensorInfo.c_str(), sensorInfo.length() );

  delay(500);
}

/**************************************************************************/
/*!
    @brief  Initialises all the sensors used by this example
*/
/**************************************************************************/
void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println(F("Ooops, no ADXL345 detected ... Check your wiring!"));
    while(1);
  }
  Serial.println(F("ADXL345 detected ... "));
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  Serial.println(F("LSM303 detected ... "));
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  Serial.println(F("L3GD20 detected ... "));
  //Display some basic information on this sensor
  displaySensorDetails();
}

/**************************************************************************/
/*!

*/
/**************************************************************************/

void setup(void)
{
  Serial.begin(115200);
  while (!Serial);
  SLoRa.Init();
  Serial.println("Adafruit 9 DOF Pitch/Roll/Heading Example");

  /* Initialise the sensors */
  initSensors();

  //pp.Init();


  GPS.begin(9600);

}

/**************************************************************************/
/*!
    @brief  Constantly check the roll/pitch/heading/altitude/temperature
*/
/**************************************************************************/
void loop(void)
{
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;
  String balisticData = "";
  /* Calculate pitch and roll from the raw accelerometer data */
  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    balisticData += "Roll: ";
    balisticData += orientation.roll;
    balisticData += "; ";
    balisticData += "Pitch: ";
    balisticData += orientation.pitch;
    balisticData += "; ";
  }

  /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .heading data now */
    balisticData += "Heading: ";
    balisticData += orientation.heading;
    balisticData += "; ";
  }


  /* Get a new sensor event */
  sensors_event_t event;

  /* Display the results (gyrocope values in rad/s) */
  gyro.getEvent(&event);
  balisticData += "\nGYRO  ";
  balisticData += "X: ";balisticData += event.gyro.x;balisticData += "  ";
  balisticData += "Y: ";balisticData += event.gyro.y;balisticData += "  ";
  balisticData += "Z: ";balisticData += event.gyro.z;balisticData += "  ";balisticData += "rad/s ";

  pp.SendMsg(sp::CMD_DATA, sp::INS_SENSOR_DATA, (const unsigned char *)balisticData.c_str(), balisticData.length() );
  String storedData = "";
  while (GPS.available() > 0) {
    // read the incoming byte:
    int incomingByte = GPS.read();
    if (incomingByte < 0)
      break;
    storedData += (char)incomingByte;
    if ((char)incomingByte == '\n'){
      Serial.print(storedData);
      break;
    }
  }
  if (storedData.length() > 0)
    pp.SendMsg(sp::CMD_DATA, sp::INS_GPS_DATA, (const unsigned char *)storedData.c_str(), storedData.length() );
  //String test = "test za boga ";
 // SLoRa.BlockingWrite((unsigned char *)test.c_str(), test.length());
  SLoRa.Main();
  delay(10000);
}
