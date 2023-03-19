/*
    ARDUINO LIMIT SWITCH TUTORIAL: NORMALLY CLOSED
    By: TheGeekPub.com
    More Arduino Tutorials: https://www.thegeekpub.com/arduino-tutorials/
*/
 
#define LIMIT_SWITCH_PIN 7
 
void setup() {
  Serial.begin(9600);
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
}
 
void loop() {
 
  if (digitalRead(LIMIT_SWITCH_PIN) == HIGH)
  {
    Serial.println("Activated!");
  }
 
  else
  {
    Serial.println("Not activated.");
  }

  delay(100);
}
