#include <Wire.h> // I2C Library
#include <SPI.h> // SPI Library
#include <SD.h> // SD Card Library


#include <Adafruit_Sensor.h> // Sensor Library
#include <Adafruit_LSM303_U.h> // Accelerometer & Magnetometer
#include <Adafruit_L3GD20_U.h> // Gyroscope
#include <Adafruit_BMP085_U.h> // Barometer
#include <Adafruit_10DOF.h> // IMU API
#include <Adafruit_GPS.h>   // GPS receiver firmware


#define mySerial Serial1
// This works when the Arduino Due Serial1 RX/TX ports (pins 19 and 18) are used.

const int chipSelect = 10;
// SD Card Setup END
 
// Assign a unique ID to the sensors
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_GPS                  GPS(&mySerial);

// Set filename (8 characters or less before the ".txt")
String my_filename = "pX_MMDD.txt";
String dataString  = "";

File dataFile;
// =========================================================================================================
 
void displaySensorDetails(void)
{
  sensor_t sensor;
 
  accel.getSensor(&sensor);
  Serial.println(F("----------- ACCELEROMETER ----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" m/s^2"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
 
  gyro.getSensor(&sensor);
  Serial.println(F("------------- GYROSCOPE -----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" rad/s"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
 
  mag.getSensor(&sensor);
  Serial.println(F("----------- MAGNETOMETER -----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" uT"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
 
  delay(500);
}
 
// ==========================================================================================================
 
void setup() {
 
  Serial.begin(115200);
  Serial.println(F("AHSL")); Serial.println("");

  // GPS receiver setup
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's
  GPS.begin(9600);
  mySerial.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
 
  // SD Card Setup
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
 
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  // SD Card Setup END
 
 
  // Initialise the sensors
 
  // ACCEL
  // Enable auto-ranging
  accel.enableAutoRange(true);
 
  if (!accel.begin())
  {
    // There was a problem detecting the ADXL345 ... check your connections
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while (1);
  }
  // ACCEL END
 
  // MAG
  // Enable auto-ranging
  mag.enableAutoRange(true);
 
  if (!mag.begin())
  {
    // There was a problem detecting the LSM303 ... check your connections
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1);
  }
  // MAG END
 
  // GYRO
  // Enable auto-ranging
  gyro.enableAutoRange(true);
 
  if (!gyro.begin())
  {
    // There was a problem detecting the L3GD20 ... check your connections
    Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  // GYRO END
 
  // Display some basic information on this sensor
  displaySensorDetails();
 
  // String header row
  dataString += "Time (s), ";
  dataString += "Acceleration x (m/s^2), ";
  dataString += "Acceleration y (m/s^2), ";
  dataString += "Acceleration z (m/s^2), ";
  
  dataString += "Angular velocity p (rad/s), ";
  dataString += "Angular velocity q (rad/s), ";
  dataString += "Angular velocity r (rad/s), ";

  dataString += "Magnetic field x (muT), ";
  dataString += "Magnetic field y (muT), ";
  dataString += "Magnetic field z (muT), ";

  dataString += "Roll angle phi (deg), ";
  dataString += "Pitch angle theta (deg), ";
  dataString += "Yaw angle psi (deg), ";

  dataString += "GPS Latitude (DDMM.MMMM), ";
  dataString += "GPS Longiude (DDMM.MMMM), ";
  dataString += "GPS Altitude (cm), ";
  dataString += "GPS Speed (knots), ";
  dataString += "GPS Heading (deg), ";
  dataString += "GPS Number of Satellites";

  dataFile = SD.open(my_filename, FILE_WRITE);
 
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    // dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }  
  else {
    Serial.println("error opening data file.");
  }
  
}
 
// ==========================================================================================================
uint32_t timer_gps = millis();
uint32_t timer_imu = millis();

double  gps_latitude   = 0;
double  gps_longitude  = 0;
uint8_t gps_n_sat      = 0;
double  gps_speed      = 0;
double  gps_heading    = 0;
double  gps_altitude   = 0;

void loop() {


  
  // Read GPS NMEA sentence
  char gps_data = GPS.read();
  //if (gps_data) Serial.print(gps_data);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
 
  if (timer_gps > millis())  timer_gps = millis();
  if (millis() - timer_gps > 2000) { 
    timer_gps = millis(); // reset the timer

    // Variables to store GPS data
    gps_latitude   = 0;
    gps_longitude  = 0;
    gps_n_sat      = 0;
    gps_speed      = 0;
    gps_heading    = 0;
    gps_altitude   = 0;
    
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);


      gps_latitude   = GPS.latitude;
      gps_longitude  = GPS.longitude;
      gps_n_sat      = (int)GPS.satellites;
      gps_speed      = GPS.speed;
      gps_heading    = GPS.angle;
      gps_altitude   = GPS.altitude;
    }   
    
    // Append GPS data
    dataString += String(gps_latitude);
    dataString += ", ";
    dataString += String(gps_longitude);
    dataString += ", ";
    dataString += String(gps_altitude);
    dataString += ", ";
    dataString += String(gps_speed);
    dataString += ", ";
    dataString += String(gps_heading);
    dataString += ", ";
    dataString += String(gps_n_sat);

    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    // File dataFile = SD.open(my_filename, FILE_WRITE);
   
   
    // // if the file is available, write to it:
    // if (dataFile) {
    //   dataFile.println(dataString);
    //   dataFile.close();
    //   // print to the serial port too:
    //   Serial.println(dataString);
    // }  
    // else {
    //   Serial.println("error opening data file");
    // }
  }


  if (timer_imu > millis())  timer_imu = millis();
  if(millis()- timer_imu > 500){
    timer_imu = millis();
    
    // Get a new sensor event
    sensors_event_t event;
   
    // make an empty string for assembling the data to log:
    dataString = "";
   
    // Setup Timestamp
    double currentMillis = millis();
    double timestamp = currentMillis / 1000;
  
   
    // read sensors and append data to the string:
    accel.getEvent(&event);
    // create acceleration (x, y, z) variables
    double accelx = event.acceleration.x ;
    double accely = event.acceleration.y ;
    double accelz = event.acceleration.z ;
   
  
   // append measurement second timestand to datastring
    dataString += String(timestamp);
    dataString += ", ";
    
   // append accelerations to datastring
  
    
    dataString += String(accelx);
    dataString += ", ";
    dataString += String(accely); 
    dataString += ", ";
    dataString += String(accelz);
    dataString += ", ";
   
    // read sensors and append data to the string:
    gyro.getEvent(&event);
    // create angular rate (x, y, z) variables
    double gyrox = event.gyro.x ;
    double gyroy = event.gyro.y ;
    double gyroz = event.gyro.z ;
   
    // append angular rates to datastring
    dataString += String(gyrox);
    dataString += ", ";
    dataString += String(gyroy);
    dataString += ", ";
    dataString += String(gyroz);
    dataString += ", ";
   
    // read sensors and append data to the string:
    mag.getEvent(&event);
    // create magnetic field (x, y, z) variables
    double magx = event.magnetic.x ;
    double magy = event.magnetic.y ;
    double magz = event.magnetic.z ;
   
    // append magnetic field values to datastring
    dataString += String(magx) ;
    dataString += ", ";
    dataString += String(magy) ;
    dataString += ", ";
    dataString += String(magz) ;
    dataString += ", ";
    
   
    // read mag and accel
    sensors_event_t accel_event;
    sensors_event_t mag_event;
    sensors_vec_t   orientation;

    // MAG HEADING WITH ACCEL (INCLINO) PITCH COMPENSATION
    // read mag and accel data
    accel.getEvent(&accel_event);
    mag.getEvent(&mag_event);
   
    //// create tilt compensated heading
    //double truehead;
   
    // tilt compensation
    if (dof.magTiltCompensation(SENSOR_AXIS_Z, &mag_event, &accel_event))
    {
      // Do something with the compensated data in mag_event!
      if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
      {
        dataString += String(orientation.roll) ;
        dataString += ", ";
        dataString += String(orientation.pitch) ;
        dataString += ", ";
        dataString += String(orientation.heading) ;
        dataString += ", ";
      }
    }
    
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    // File dataFile = SD.open(my_filename, FILE_WRITE);
   
   
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.flush();
      // dataFile.close();
      // print to the serial port too:
      Serial.println(dataString);
    }  
    else {
      Serial.println("error opening data file");
    }
  }
 
}
