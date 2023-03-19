// Motor Controller Test - Field test using button

#include <Robojax_AllegroACS_Current_Sensor.h> //Library to interpret current sensor readings
//Libraries to interpret accelerometer readings
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

const float pi = 3.14159265;

//Actuator control pins
const int ENA1 = 6; // ENA1 on motor controller. PWM to control speed
const int IN1 = 7; // IN1 on motor controller. Controls forward/backward
const int IN2 = 8; // IN2 on motor controller. Controls forward/backward

//Pan Motor control pins
const int ENA2 = 5; // ENA2 on motor controller. PWM to control speed - 
const int IN3 = 4; // IN1 on motor controller. Controls forward/backward
const int IN4 = 3; // IN2 on motor controller. Controls forward/backward

const int PIN_CURRENT = A0; // Pin receiving analog signal from current sensor
const int PIN_BUTTON = A3; // Pin receiving digital signal from button

const int actuatorSpeed = 256*.99-1; // 30% speed
const int panSpeed = 256*.65-1; //65% speed

int desiredAngle = 45; // Drives actuator to make 45 degrees
const int angleTolerance = 10; // +/- 5 degrees is tolerated

bool braking = true; //If true, actuator is braking
bool wasCW = false; //If true, the last direction was extending

const int currentSensorModel = 2;
Robojax_AllegroACS_Current_Sensor robojax(currentSensorModel,PIN_CURRENT);

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

void setup() {
    // put your setup code here, to run once:
  TCCR0B = TCCR0B & B11111000 | B00000010; // Sets PWM frequency of pins 5 and 6 (ENA1 and ENA2) to 7812.50 Hz
  
  pinMode(ENA1,OUTPUT); // ENA1 on motor controller. PWM to control speed
  pinMode(IN1,OUTPUT); // IN1 on motor controller. Controls forward/backward
  pinMode(IN2,OUTPUT); // IN2 on motor controller. Controls forward/backward
  pinMode(ENA2,OUTPUT); // ENA1 on motor controller. PWM to control speed
  pinMode(IN3,OUTPUT); // IN1 on motor controller. Controls forward/backward
  pinMode(IN4,OUTPUT); // IN2 on motor controller. Controls forward/backward
  
  pinMode(PIN_CURRENT,INPUT); // Analog signal from current sensor
  pinMode(PIN_BUTTON,INPUT); // Digital signal from button
  
  
    // Write all control pins to low to make motor brake
  brakeActuator();
  //brakePanMotor();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  float current = robojax.getCurrent();
  Serial.println(current);

  bool buttonState = digitalRead(PIN_BUTTON);
  
  if ((buttonState == LOW) && not braking) {
    brakeActuator();
    braking = true;
    Serial.println("Braking");
  }
  else if ((buttonState == HIGH) && braking ) {
    if (wasCW) {
      extend();
    Serial.println("Extending");
    }
    else{
      contract();
    Serial.println("Contracting");
    }
    wasCW = not wasCW;
    braking = false;
  }
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
/*
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
}*/
