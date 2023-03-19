

// Motor Controller Test
// Driving Linear Actuator only off of Motor 1 side of controller

#include <Robojax_AllegroACS_Current_Sensor.h> //Library to interpret current sensor readings
//Libraries to interpret accelerometer readings
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

const float pi = 3.14159265;

const int ENA1 = 6; // ENA1 on motor controller. PWM to control speed
const int IN1 = 7; // IN1 on motor controller. Controls forward/backward
const int IN2 = 8; // IN2 on motor controller. Controls forward/backward
const int PIN_CURRENT = A0; // Pin receiving analog signal from current sensor
const int PIN_BUTTON = 10; // Pin receiving digital signal from button

const int actuatorSpeed = 256*.30-1; // 30% speed

int desiredAngle = 45; // Drives actuator to make 45 degrees
const int angleTolerance = 10; // +/- 5 degrees is tolerated

bool braking = true; //If true, actuator is braking
bool wasExtending = false; //If true, the last direction was extending

const int currentSensorModel = 2;
Robojax_AllegroACS_Current_Sensor robojax(currentSensorModel,PIN_CURRENT);

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

void setup() {
  // put your setup code here, to run once:

pinMode(ENA1,OUTPUT); // ENA1 on motor controller. PWM to control speed
pinMode(IN1,OUTPUT); // IN1 on motor controller. Controls forward/backward
pinMode(IN2,OUTPUT); // IN2 on motor controller. Controls forward/backward
pinMode(PIN_CURRENT,INPUT); // Analog signal from current sensor
pinMode(PIN_BUTTON,INPUT); // Digital signal from button


  // Write all control pins to low to make motor brake
brake();

Serial.begin(9600);
 if (!mag.begin()) { Serial.println("w: mag"); }
  if (!accel.begin()) { Serial.println("w: accel"); }
  if (!accel.begin() || !mag.begin()) {
    Serial.println("din't work out, did it?");
    while(1);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
float current = robojax.getCurrent();
//Serial.println(current);

sensors_event_t accevent;
accel.getEvent(&accevent);
float yAccel = accevent.acceleration.y;
if (yAccel > 9.81) {yAccel = 9.81;} //Change by possibly sensor calibration
if (yAccel < -9.81) {yAccel = -9.81;} //Change by possibly sensor calibration
float angle = acos(yAccel/9.81)*180/pi;
Serial.print(angle);
Serial.print(",");
Serial.println(yAccel);

if ((angle-desiredAngle) > angleTolerance) {contract();}
else if ((angle-desiredAngle) < -angleTolerance) {extend();}
else {brake();}
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

void brake() {
digitalWrite(ENA1,LOW);
digitalWrite(IN1,LOW);
digitalWrite(IN2,LOW);
}
