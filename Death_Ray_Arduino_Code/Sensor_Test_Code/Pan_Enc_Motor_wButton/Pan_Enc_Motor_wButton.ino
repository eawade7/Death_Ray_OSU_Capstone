// Motor Controller Test - Field test using button

#include <Robojax_AllegroACS_Current_Sensor.h> //Library to interpret current sensor readings
//Libraries to interpret accelerometer readings
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
//include SPI library
#include <SPI.h>
const float pi = 3.14159265;

#define timoutLimit 100
 
//SPI commands used by the AMT20
#define nop 0x00            //no operation
#define rd_pos 0x10         //read position
#define set_zero_point 0x70 //set zero point
 
//set the chip select pin for the AMT20
const int CS = 10;   

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

const int actuatorSpeed = 256*.30-1; // 30% speed
const int panSpeed = 256*.99-1; //65% speed

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
  Serial.begin(9600);
  pinMode(ENA1,OUTPUT); // ENA1 on motor controller. PWM to control speed
  pinMode(IN1,OUTPUT); // IN1 on motor controller. Controls forward/backward
  pinMode(IN2,OUTPUT); // IN2 on motor controller. Controls forward/backward
  pinMode(ENA2,OUTPUT); // ENA1 on motor controller. PWM to control speed
  pinMode(IN3,OUTPUT); // IN1 on motor controller. Controls forward/backward
  pinMode(IN4,OUTPUT); // IN2 on motor controller. Controls forward/backward
  
  pinMode(PIN_CURRENT,INPUT); // Analog signal from current sensor
  pinMode(PIN_BUTTON,INPUT); // Digital signal from button
  
    //Set I/O mode of all SPI pins.
  pinMode(SCK, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(CS, OUTPUT);
 
  //Initialize SPI using the SPISettings(speedMaxium, dataOrder, dataAMode) function
  //For our settings we will use a clock rate of 500kHz, and the standard SPI settings
  //of MSB First and SPI Mode 0
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  
  //Using SPI.beginTransaction seems to require explicitly setting the beginning state
  //of the CS pin as opposed to the SPI.begin() function that does this for us.
  digitalWrite(CS, HIGH);
    // Write all control pins to low to make motor brake
  //brakeActuator();
  brakePanMotor();

}

void loop() {
  // put your main code here, to run repeatedly:
  float current = robojax.getCurrent();
  Serial.println(current);

  uint8_t data;               //this will hold our returned data from the AMT20
  uint8_t timeoutCounter;     //our timeout incrementer
  uint16_t currentPosition;   //this 16 bit variable will hold our 12-bit position
  
  //reset the timoutCounter;
  timeoutCounter = 0;
  
  //send the rd_pos command to have the AMT20 begin obtaining the current position
  data = SPIWrite(rd_pos);
  
  //we need to send nop commands while the encoder processes the current position. We
  //will keep sending them until the AMT20 echos the rd_pos command, or our timeout is reached.
  while (data != rd_pos && timeoutCounter++ < timoutLimit)
  {
  data = SPIWrite(nop);
  }
  
  
  if (timeoutCounter < timoutLimit) {  //rd_pos echo received 
    //We received the rd_pos echo which means the next two bytes are the current encoder position.
    //Since the AMT20 is a 12 bit encoder we will throw away the upper 4 bits by masking.
    
    //Obtain the upper position byte. Mask it since we only need it's lower 4 bits, and then
    //shift it left 8 bits to make room for the lower byte.
    currentPosition = (SPIWrite(nop)& 0x0F) << 8;
    
    //OR the next byte with the current position
    currentPosition |= SPIWrite(nop);
  }
  else {  //timeout reached
    //This means we had a problem with the encoder, most likely a lost connection. For our
    //purposes we will alert the user via the serial connection, and then stay here forever.
    
    Serial.write("Error obtaining position.\n");
    Serial.write("Reset Arduino to restart program.\n");
    
    while(true);
  }
  float Degrees = float(currentPosition)/4096.0*360.0;
  Serial.println(Degrees);

  bool buttonState = digitalRead(PIN_BUTTON);
  Serial.println(buttonState);
  
  if ((buttonState == LOW) && not braking) {
    brakePanMotor();
    braking = true;
    Serial.println("Braking");
  }
  else if ((buttonState == HIGH) && braking ) {
    if (wasCW) {
      turnCCW();
    Serial.println("Turning CCW");
    }
    else{
      turnCW();
    Serial.println("Turning CW");
    }
    wasCW = not wasCW;
    braking = false;
  }
}

/*
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
*/
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
