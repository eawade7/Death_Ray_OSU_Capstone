// Code for Death Ray Capstone

// Current Sensor Libraries
//#include <Robojax_AllegroACS_Current_Sensor.h> //Library to interpret current sensor readings
//const int PIN_CURRENT = A0; // Pin receiving analog signal from current sensor
//const int currentSensorModel = 2;
//Robojax_AllegroACS_Current_Sensor robojax(currentSensorModel,PIN_CURRENT);
const int PIN_LIMIT = A0; // Pin receiving digital input from limit switch

// Accelerometer libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

// GPS Libraries
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 9);
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  true

// Encoder libraries
#include <SPI.h>
#define timoutLimit 100

//SPI commands used by the AMT20 (encoder)
#define nop 0x00            //no operation
#define rd_pos 0x10         //read position
#define set_zero_point 0x70 //set zero point
const int CS = 10;   

// Motor controller pins
const int ENA1 = 6; // ENA1 on motor controller. PWM to control pan motor speed
const int IN1 = 7; // IN1 on motor controller. Controls forward/backward of pan motor
const int IN2 = 8; // IN2 on motor controller. Controls forward/backward of pan motor
const int actuatorSpeed = 256*.99-1; // 99% speed
const int panSpeed = 256*.99-1; //99% speed
const int angleTolerance = 5; // +/- 5 degrees is tolerated

//Pan Motor control pins
const int ENA2 = 5; // ENA2 on motor controller. PWM to control speed - 
const int IN3 = 4; // IN1 on motor controller. Controls forward/backward
const int IN4 = 3; // IN2 on motor controller. Controls forward/backward

// GPS globals
uint8_t month;
uint8_t day;
uint16_t year;
uint8_t hour;
uint8_t minute;
uint8_t second;
//uint16_t myElevation;
//float myLatitude;
//float myLongitude;
uint32_t clockTimer;
double elevation = 0;
double azimuth = 0;

const int PIN_BUTTON = A3; // Pin receiving digital signal from button
bool night = false;
bool wasRun = false;
float encoder_val = 0;
float encoder_off = 0; 
uint8_t rotationCount = 0;
float encDegrees = 0;

void setup() {
  
  // put your setup code here, to run once:
  // pinMode(PIN_CURRENT,INPUT); // Analog signal from current sensor
  // TCCR0B = TCCR0B & B11111000 | B00000010; // Sets PWM frequency of pins 5 and 6 (ENA1 and ENA2) to 7812.50 Hz
  // pinMode(PIN_BUTTON,INPUT); // Digital signal from button
  Serial.begin(9600);
  // motor setup
  pinMode(ENA1,OUTPUT); // ENA1 on motor controller. PWM to control speed
  pinMode(IN1,OUTPUT); // IN1 on motor controller. Controls forward/backward
  pinMode(IN2,OUTPUT); // IN2 on motor controller. Controls forward/backward
  pinMode(ENA2,OUTPUT); // ENA1 on motor controller. PWM to control speed
  pinMode(IN3,OUTPUT); // IN1 on motor controller. Controls forward/backward
  pinMode(IN4,OUTPUT); // IN2 on motor controller. Controls forward/backward

  pinMode(PIN_LIMIT,INPUT); // Analog signal from current sensor
  pinMode(PIN_BUTTON,INPUT); // Digital signal from button
  
  //Set I/O mode of all SPI pins.
  pinMode(SCK, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(CS, OUTPUT);
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS, HIGH);

  //Initialize Accelerometer
//  if (!mag.begin()) { Serial.println("w: mag"); }
//  if (!accel.begin()) { Serial.println("w: accel"); }
  if (!accel.begin() || !mag.begin()) {
        Serial.println("Accelerometer inititializaiton failed.\nReset arduino to try again.");
        while(1);
  }
  
//  Serial.println("Adafruit GPS library basic parsing test!");
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

  //read the GPS data a few times to clear out garbage
  unsigned long t = millis();
  while(millis() - t < 5000){
    readGPS();
  }
  turnCCW();
  bool lsState = digitalRead(PIN_LIMIT);
  //Once garbage is cleared, read until fix acquired
  while(!GPS.fix){
    readGPS();
    if (lsState == HIGH){
      brakePanMotor();
    }
    lsState = digitalRead(PIN_LIMIT);
  }
  clockTimer = millis();
  month = GPS.month;
  day = GPS.day;
  year = GPS.year+2000;
  if(GPS.hour >= 8){
    hour = GPS.hour-8;
  }
  else{
    hour = GPS.hour+16;
    day = day-1;
  }
  minute = GPS.minute;
  second = GPS.seconds;
  solarCalc();
  Serial.println(elevation);
  Serial.println(azimuth);
}

uint32_t timer = millis();


void loop() {
   bool buttonState = digitalRead(PIN_BUTTON);
   
/* 
  ///////////////////   Current Data Collection    ///////////////////
  
  // put your main code here, to run repeatedly:
  float current1 = robojax.getCurrent();
  delay(500);
//  Serial.println(current1);
*/
  ///////////////////   Encoder Data Collection    ///////////////////
  
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
    Serial.write("Error obtaining position.\nReset Arduino to restart program.\n");
//    while(true);
  }
  float oldDeg = encDegrees;
  encDegrees = float(currentPosition)/4096.0*360.0*(8/35);
  delay(10);
  if (encDegrees < (oldDeg - 10)){
    rotationCount++;
    encDegrees = encDegrees + rotationCount*(360.0*(8/35));
  }

  ///////////////////   Acceleration Data Collection    ///////////////////
  
  sensors_event_t accevent;
  accel.getEvent(&accevent);
  float yAccel = accevent.acceleration.y;
  if (yAccel > 9.81) {yAccel = 9.81;} //Change by possibly sensor calibration
  if (yAccel < -9.81) {yAccel = -9.81;} //Change by possibly sensor calibration
  float accelAngle = acos(yAccel/9.81)*180/PI;
  delay(10);
  //Serial.println(accelAngle);

  if (millis() - timer > 30000){
    Serial.println("sup bitch");
    timer = millis();
    // Solar calculations
    solarCalc();
    Serial.print("Azimuth: "); Serial.println(azimuth);
    Serial.print("Elevation: "); Serial.println(elevation);
  }

  if (elevation > 45){
    while((accelAngle-elevation) > angleTolerance) {
      contract();
      sensors_event_t accevent;
      accel.getEvent(&accevent);
      float yAccel = accevent.acceleration.y;
      if (yAccel > 9.81) {yAccel = 9.81;} //Change by possibly sensor calibration
      if (yAccel < -9.81) {yAccel = -9.81;} //Change by possibly sensor calibration
      float accelAngle = acos(yAccel/9.81)*180/PI;
      delay(10);
      //Serial.println(accelAngle);
    }
    while((accelAngle-elevation) < -angleTolerance) {
      extend();
      sensors_event_t accevent;
      accel.getEvent(&accevent);
      float yAccel = accevent.acceleration.y;
      if (yAccel > 9.81) {yAccel = 9.81;} //Change by possibly sensor calibration
      if (yAccel < -9.81) {yAccel = -9.81;} //Change by possibly sensor calibration
      float accelAngle = acos(yAccel/9.81)*180/PI;
      delay(10);
      //Serial.println(accelAngle);
    }
    brakeActuator();
  }

  if((encDegrees-azimuth) > angleTolerance || (encDegrees - azimuth) < -angleTolerance){
    turnCCW();
    while((encDegrees-azimuth) > angleTolerance || (encDegrees - azimuth) < -angleTolerance){
      checkDegree();
      Serial.print("Encoder: ");
      Serial.println(encDegrees);
    }
  }
  brakePanMotor();
  
  if(buttonState == HIGH){
    goLoad();
  }

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

void readGPS()
{
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if ((c) && (GPSECHO))
    Serial.write(c);

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
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

void solarCalc(){
    int dayOfYear = getDay(year, month, day);
    float currentTime = float(hour) + float(minute)/60.0 + float(millis()-clockTimer)/3600000.0;

    double dec = -23.45*cos(((double)360 / 365)*(PI/180)*(dayOfYear+10));
    double hrAng = 15*(currentTime-12);
    elevation = asin(sin(dec*(PI/180))*sin(44.623032*(PI/180))+cos(dec*(PI/180))*cos(44.623032*(PI/180))*cos(hrAng*(PI/180)))*(180/PI);
    if (hrAng >= 0){
      azimuth = 360 - acos((sin(dec*(PI/180))*cos(44.623032*(PI/180))-cos(dec*(PI/180))*sin(44.623032*(PI/180))*cos(hrAng*(PI/180)))/cos(elevation*(PI/180)))*(180/PI);
    }
    else{
      azimuth = acos((sin(dec*(PI/180))*cos(44.623032*(PI/180))-cos(dec*(PI/180))*sin(44.623032*(PI/180))*cos(hrAng*(PI/180)))/cos(elevation*(PI/180)))*(180/PI);
    }
}

void goLoad(){
  turnCW();
  while(encDegrees < 180){
    checkDegree();
  }
  brakePanMotor();
}

void goHome(){
    turnCCW();
    delay(1000);
    bool lsState = digitalRead(PIN_LIMIT);
    while(lsState != HIGH){
      lsState = digitalRead(PIN_LIMIT);
    }
    brakePanMotor();
}

void checkDegree(){
  uint8_t data;               //this will hold our returned data from the AMT20
  uint8_t timeoutCounter;     //our timeout incrementer
  uint16_t currentPosition;   //this 16 bit variable will hold our 12-bit position

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
    Serial.write("Error obtaining position.\nReset Arduino to restart program.\n");
//    while(true);
  }
  float oldDeg = encDegrees;
  float encDegrees = float(currentPosition)/4096.0*360.0*(8/35);
  delay(10);
  if (encDegrees < (oldDeg-10)){
    rotationCount++;
    encDegrees = encDegrees + rotationCount*(360.0*(8/35));
  }
}
