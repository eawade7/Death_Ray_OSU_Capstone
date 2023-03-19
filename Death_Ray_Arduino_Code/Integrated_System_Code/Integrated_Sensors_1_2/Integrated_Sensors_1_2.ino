// Code for Death Ray Capstone

// Current Libraries
#include <Robojax_AllegroACS_Current_Sensor.h> //Library to interpret current sensor readings
const int PIN_CURRENT = A0; // Pin receiving analog signal from current sensor
const int currentSensorModel = 2;
Robojax_AllegroACS_Current_Sensor robojax(currentSensorModel,PIN_CURRENT);

// ACCEL libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

// GPS includes
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 9);
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  true

// Encoder libraries
#include <SPI.h>
#define timoutLimit 100

//SPI commands used by the AMT20
#define nop 0x00            //no operation
#define rd_pos 0x10         //read position
#define set_zero_point 0x70 //set zero point
const int CS = 10;   

// Actuator control pins
const int ENA1 = 6; // ENA1 on motor controller. PWM to control speed
const int IN1 = 7; // IN1 on motor controller. Controls forward/backward
const int IN2 = 8; // IN2 on motor controller. Controls forward/backward
const int actuatorSpeed = 256*.99-1; // 30% speed
const int panSpeed = 256*.99-1; //65% speed
const int angleTolerance = 10; // +/- 5 degrees is tolerated

// globals
const float pi = 3.14159265;
uint8_t month;
uint8_t day;
uint8_t year;
uint8_t hour;
uint8_t minute;
uint8_t second;



void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_CURRENT,INPUT); // Analog signal from current sensor
  TCCR0B = TCCR0B & B11111000 | B00000010; // Sets PWM frequency of pins 5 and 6 (ENA1 and ENA2) to 7812.50 Hz
  //  pinMode(PIN_BUTTON,INPUT); // Digital signal from button
  Serial.begin(9600);
  // motor setup
  pinMode(ENA1,OUTPUT); // ENA1 on motor controller. PWM to control speed
  pinMode(IN1,OUTPUT); // IN1 on motor controller. Controls forward/backward
  pinMode(IN2,OUTPUT); // IN2 on motor controller. Controls forward/backward
  pinMode(5,OUTPUT); // ENA1 on motor controller. PWM to control speed
  pinMode(4,OUTPUT); // IN1 on motor controller. Controls forward/backward
  pinMode(3,OUTPUT); // IN2 on motor controller. Controls forward/backward
  
  //Set I/O mode of all SPI pins.
  pinMode(SCK, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(CS, OUTPUT);
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS, HIGH);
  
  //delay(1500);
  Serial.println("Adafruit GPS library basic parsing test!");
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
   // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);

  if (!mag.begin()) { Serial.println("w: mag"); }
  if (!accel.begin()) { Serial.println("w: accel"); }
  if (!accel.begin() || !mag.begin()) {
        Serial.println("din't work out, did it?");
        while(1);
  }
/*  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
        return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  while(!GPS.fix){
    char c = GPS.read();
    if (GPS.newNMEAreceived()) { 
      if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
          return;  // we can fail to parse a sentence in which case we should just wait for another
    }
    Serial.println("Not fixed");
  }
  month = GPS.month;
  day = GPS.day;
  year = GPS.year;
  hour = GPS.hour;
  minute = GPS.minute;
  second = GPS.second;*/
}

uint32_t timer = millis();
double elevation = 0;
double azimuth = 0;

void loop() {
  // put your main code here, to run repeatedly:
  float current1 = robojax.getCurrent();
  Serial.println(current1);
  
  uint8_t data;               //this will hold our returned data from the AMT20
  uint8_t timeoutCounter;     //our timeout incrementer
  uint16_t currentPosition;   //this 16 bit variable will hold our 12-bit position
  
  //reset the timoutCounter;
  timeoutCounter = 0;
  
  //send the rd_pos command to have the AMT20 begin obtaining the current position
  data = SPIWrite(rd_pos);
  while (data != rd_pos && timeoutCounter++ < timoutLimit){
    data = SPIWrite(nop);
  }
  
  if (timeoutCounter < timoutLimit) {  //rd_pos echo received 
    currentPosition = (SPIWrite(nop)& 0x0F) << 8;
    currentPosition |= SPIWrite(nop);
  }
  else {  //timeout reached
    Serial.write("Error obtaining position.\n");
    Serial.write("Reset Arduino to restart program.\n");
    
    while(true);
  }
  float Degrees = float(currentPosition)/4096.0*360.0;
  delay(10);
  Serial.println(Degrees);
/*
  if (Degrees > 150){
    //float current1 = robojax.getCurrent();
   // Serial.println(current1);
    timeoutCounter = 0;
    data = SPIWrite(rd_pos);
    while (data != rd_pos && timeoutCounter++ < timoutLimit){
      data = SPIWrite(nop);
    }
  
    if (timeoutCounter < timoutLimit) {  //rd_pos echo received 
      currentPosition = (SPIWrite(nop)& 0x0F) << 8;
      currentPosition |= SPIWrite(nop);
    }
    else {  //timeout reached
      Serial.write("Error obtaining position.\n");
      Serial.write("Reset Arduino to restart program.\n");
      while(true);
    }
    Degrees = float(currentPosition)/4096.0*360.0;
    turnCCW();
  }
  brakePanMotor();
 */  
/*  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  //if ((c) && (GPSECHO))
    //Serial.write(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }*/
/*
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    int gpsYear = int(year) + 2000;
    int dayOfYear = getDay(gpsYear, month, day);
    float gpshour = float(hour) - 8;
    if (gpshour < 0){
      gpshour = gpshour + 24;
    }
    //Serial.print("hour: ");
    float time1 = hour + minute/60;
    Serial.print((int)GPS.fix);
    Serial.print(',');
    Serial.print(GPS.day);
    Serial.print(',');
    Serial.print(GPS.hour); 
    Serial.print('\n');
    //int time1 = 14.5;
    //int day = 35;
    // Solar calculations
    double dec = -23.45*cos(((double)360 / 365)*(pi/180)*(dayOfYear+10));
    double hrAng = 15*(time1-12);
    elevation = asin(sin(dec*(pi/180))*sin(44.623032*(pi/180))+cos(dec*(pi/180))*cos(44.623032*(pi/180))*cos(hrAng*(pi/180)))*(180/pi);
    if (hrAng >= 0){
      azimuth = 360 - acos((sin(dec*(pi/180))*cos(44.623032*(pi/180))-cos(dec*(pi/180))*sin(44.623032*(pi/180))*cos(hrAng*(pi/180)))/cos(elevation*(pi/180)))*(180/pi);
    }
    else{
      azimuth = acos((sin(dec*(pi/180))*cos(44.623032*(pi/180))-cos(dec*(pi/180))*sin(44.623032*(pi/180))*cos(hrAng*(pi/180)))/cos(elevation*(pi/180)))*(180/pi);
    }
  }
*/
  //acctimer = millis(); // reset the timer
  sensors_event_t accevent;
  accel.getEvent(&accevent);
  float yAccel = accevent.acceleration.y;
  if (yAccel > 9.81) {yAccel = 9.81;} //Change by possibly sensor calibration
  if (yAccel < -9.81) {yAccel = -9.81;} //Change by possibly sensor calibration
  float angle1 = acos(yAccel/9.81)*180/pi;
  Serial.println(angle1);
 
}


int getDay(unsigned int y, unsigned int m, unsigned int d){
  int days[]={0,31,59,90,120,151,181,212,243,273,304,334};    // Number of days at the beginning of the month in a not leap year.
  int DN = 0;
  //Start to calculate the number of day
  if (m==1 || m==2){
    DN = days[(m-1)]+d;                     //for any type of year, it calculate the number of days for January or february
  }                        // Now, try to calculate for the other months
  else if (y % 4 == 0){  //those are the conditions to have a leap year
    DN = days[(m-1)]+d+1;     // if leap year, calculate in the same way but increasing one day
  }
  else {                                //if not a leap year, calculate in the normal way, such as January or February
    DN = days[(m-1)]+d;
  }
  return DN;
}

uint8_t SPIWrite(uint8_t sendByte)
{
  //holder for the received over SPI
  uint8_t data;
 
  //the AMT20 requires the release of the CS line after each byte
  digitalWrite(CS, LOW);
  data = SPI.transfer(sendByte);
  digitalWrite(CS, HIGH);
 
  //we will delay here to prevent the AMT20 from having to prioritize SPI over obtaining our position
  delayMicroseconds(10);
  
  return data;
}

void turnCW() { // direction dish is moving
  analogWrite(5,panSpeed);
  digitalWrite(4,LOW);
  digitalWrite(3,HIGH);
}

void turnCCW() {
  analogWrite(5,panSpeed);
  digitalWrite(4,HIGH);
  digitalWrite(3,LOW);
}

void brakePanMotor() {
  digitalWrite(5,LOW);
  digitalWrite(4,LOW);
  digitalWrite(3,LOW);
}

void extend() {
  analogWrite(ENA1,actuatorSpeed);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
}

void contract() {
  analogWrite(ENA1,actuatorSpeed);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
}

void brakeActuator() {
  digitalWrite(ENA1,LOW);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
}
