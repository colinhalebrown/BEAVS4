const int servoPin = 27;
const int servoPower = 28;

void setup() {
  Serial.begin(9600);
  pinMode(servoPower, OUTPUT);
  pinMode(servoPin, OUTPUT);

  digitalWrite(servoPower, HIGH);

}

void loop() {
  servo(1860);
  delay(1000);
}

void servo(int x) {
  for (int i = 0; i <= 2; i++) {
    // A pulse each 20ms
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(x); // Duration of the pusle in microseconds
    digitalWrite(servoPin, LOW);
    delayMicroseconds(18550); // 20ms - duration of the pusle  
  }
}