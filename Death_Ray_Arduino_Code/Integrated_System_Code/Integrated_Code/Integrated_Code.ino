// Code for Death Ray Capstone

// ACCEL includes
#include <Robojax_AllegroACS_Current_Sensor.h> //Library to interpret current sensor readings
//Libraries to interpret accelerometer readings
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <SPI.h>

// GPS includes
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 8->2
// Connect the GPS RX (receive) pin to Digital 7->9

// ACCEL globals
const float pi = 3.14159265;

const int ENA1 = 6; // ENA1 on motor controller. PWM to control speed
const int IN1 = 7; // IN1 on motor controller. Controls forward/backward
const int IN2 = 8; // IN2 on motor controller. Controls forward/backward

//Pan Motor control pins
const int ENA2 = 5; // ENA2 on motor controller. PWM to control speed - 
const int IN3 = 4; // IN1 on motor controller. Controls forward/backward
const int IN4 = 3; // IN2 on motor controller. Controls forward/backward

const int PIN_CURRENT = A0; // Pin receiving analog signal from current sensor
const int PIN_BUTTON = A3; // Pin receiving digital signal from button

const int actuatorSpeed = 256*.30-1; // 30% speed
const int panSpeed = 256*.65-1; //65% speed

//int desiredAngle = 45; // Drives actuator to make 45 degrees
const int angleTolerance = 10; // +/- 5 degrees is tolerated
const int panTolerance = 10;

//bool braking = true; //If true, actuator is braking
//bool wasExtending = false; //If true, the last direction was extending

const int currentSensorModel = 2;
Robojax_AllegroACS_Current_Sensor robojax(currentSensorModel,PIN_CURRENT);

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

// SOLAR globals
// you can change the pin numbers to match your wiring:
SoftwareSerial mySerial(2, 9);
Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true

double elevation = 0;
double azimuth = 0;
float encoder_val = 190;
bool night = false;
bool wasRun = false;

void setup() {
  // put your setup code here, to run once:

  //Linear Actuator
  pinMode(ENA1,OUTPUT); // ENA1 on motor controller. PWM to control speed
  pinMode(IN1,OUTPUT); // IN1 on motor controller. Controls forward/backward
  pinMode(IN2,OUTPUT); // IN2 on motor controller. Controls forward/backward

  //Pan Motor
  pinMode(ENA2,OUTPUT); // ENA2 on motor controller. PWM to control speed
  pinMode(IN3,OUTPUT); // IN3 on motor controller. Controls forward/backward
  pinMode(IN4,OUTPUT); // IN4 on motor controller. Controls forward/backward
  
  pinMode(PIN_CURRENT,INPUT); // Analog signal from current sensor
  pinMode(PIN_BUTTON,INPUT); // Digital signal from button
  
  
    // Write all control pins to low to make motor brake
  brake();
  brakePanMotor();
  
  Serial.begin(115200);
  if (!mag.begin()) { Serial.println("w: mag"); }
    if (!accel.begin()) { Serial.println("w: accel"); }
      if (!accel.begin() || !mag.begin()) {
        Serial.println("din't work out, did it?");
        while(1);
  }
  delay(5000);
  Serial.println("Adafruit GPS library basic parsing test!");
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);
}

// timer for checking how often to read GPS data
uint32_t timer = millis();
//uint32_t timerCur = millis();


void loop() {
  float current = robojax.getCurrent();
  Serial.print("Current: ");
  Serial.println(current);

  bool buttonState = digitalRead(PIN_BUTTON);

///////////// GPS Read in //////////////////////////////////////////////////////////
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if ((c) && (GPSECHO))
    Serial.write(c);

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  if (millis() - timer > 4000) {
    timer = millis(); // reset the timer
    int hour = GPS.hour;
    int minute = GPS.minute;
    int second = GPS.seconds;
    int gpsDay = GPS.day;
    int gpsMon = GPS.month;
    int gpsYear = GPS.year + 2000;
    int day = getDay(gpsYear, gpsMon, gpsDay);
    hour = hour-8;
    if (hour < 0){
      hour = hour + 24;
    }
    if (hour > 20 or hour < 7){
      night = true;
      if (wasRun == false){
        goHome();
      }
    }
    else{
      night = false;
      wasRun = false;
    }
    if (night == false) {
      Serial.print("hour: ");
      float time1 = hour + minute/60;
      Serial.println(time1);
      Serial.print("day: ");
      Serial.println(day);
      //int time1 = 14.5;
      //int day = 35;
      // Solar calculations
      double dec = -23.45*cos(((double)360 / 365)*(pi/180)*(day+10));
      double hrAng = 15*(time1-12);
      elevation = asin(sin(dec*(pi/180))*sin(44.623032*(pi/180))+cos(dec*(pi/180))*cos(44.623032*(pi/180))*cos(hrAng*(pi/180)))*(180/pi);
      if (hrAng >= 0){
        azimuth = 360 - acos((sin(dec*(pi/180))*cos(44.623032*(pi/180))-cos(dec*(pi/180))*sin(44.623032*(pi/180))*cos(hrAng*(pi/180)))/cos(elevation*(pi/180)))*(180/pi);
      }
      else{
        azimuth = acos((sin(dec*(pi/180))*cos(44.623032*(pi/180))-cos(dec*(pi/180))*sin(44.623032*(pi/180))*cos(hrAng*(pi/180)))/cos(elevation*(pi/180)))*(180/pi);
      }
      Serial.print("Elevation: ");
      Serial.println(elevation);
      Serial.print("Azimuth: ");
      Serial.println(azimuth);
    }
  }
//////////////////////////////////////////////////////////////////////////////////////

//////////// ACCELEROMETER READING ///////////////////////////////////////////////////
  sensors_event_t accevent;
  accel.getEvent(&accevent);
  float yAccel = accevent.acceleration.y;
  if (yAccel > 9.81) {yAccel = 9.81;} //Change by possibly sensor calibration
  if (yAccel < -9.81) {yAccel = -9.81;} //Change by possibly sensor calibration
  float angle = acos(yAccel/9.81)*180/pi;
  Serial.print(angle);
  Serial.print(",");
  Serial.println(yAccel);
//////////////////////////////////////////////////////////////////////////////////////
/*
  while ((angle-elevation) > angleTolerance) {
    contract();
    sensors_event_t accevent;
    accel.getEvent(&accevent);
    float yAccel = accevent.acceleration.y;
    if (yAccel > 9.81) {yAccel = 9.81;} //Change by possibly sensor calibration
    if (yAccel < -9.81) {yAccel = -9.81;} //Change by possibly sensor calibration
    angle = acos(yAccel/9.81)*180/pi;
    Serial.print(angle);
    Serial.print(",");
    Serial.println(yAccel);
  }
  while ((angle-elevation) < -angleTolerance) {
    extend();
    sensors_event_t accevent;
    accel.getEvent(&accevent);
    float yAccel = accevent.acceleration.y;
    if (yAccel > 9.81) {yAccel = 9.81;} //Change by possibly sensor calibration
    if (yAccel < -9.81) {yAccel = -9.81;} //Change by possibly sensor calibration
    angle = acos(yAccel/9.81)*180/pi;
    Serial.print(angle);
    Serial.print(",");
    Serial.println(yAccel);
  }*/
  brake();

  /*
Read encoder

 while ((encoder_val-azimuth) > panTolerance) {turnCW();}
 brake
   */

if(buttonState == HIGH){
    int atLoad = goLoad();
    while(atLoad == 0){
      atLoad = goLoad();
    }
  }
}
  
void extend() {
  analogWrite(ENA1,actuatorSpeed);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  Serial.println("extend");
}

void contract() {
  analogWrite(ENA1,actuatorSpeed);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  Serial.println("contract");
}

void brake() {
  digitalWrite(ENA1,LOW);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  Serial.println("brake");
}

void turnCW() {
  analogWrite(ENA2,panSpeed);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
}

void turnCCW() {
  analogWrite(ENA2,panSpeed);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
}

void brakePanMotor() {
  digitalWrite(ENA2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
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

int goLoad(){
  turnCW();
  if (encoder_val >= 180){
    return 1;
  }
  return 0;
}

void goHome(){
    turnCCW();
    delay(1000);
    float current = robojax.getCurrent();
  //Serial.println(current);
    while (current <= 4){
      current = robojax.getCurrent();
    }
    // set encoder val to 0
    wasRun = true;
}
