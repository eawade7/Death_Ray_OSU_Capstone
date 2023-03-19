const int LIMIT_SWITCH = A0;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
    pinMode(LIMIT_SWITCH,INPUT_PULLUP); // Analog signal from current sensor

}

void loop() {
  // put your main code here, to run repeatedly:
  bool limitSwitch = digitalRead(LIMIT_SWITCH);
  Serial.println(limitSwitch);
  while (limitSwitch != HIGH){
    limitSwitch = digitalRead(LIMIT_SWITCH);
  }
  Serial.println("went high");

}
