#define servoPin 28

// 500 load - 670 full extention - 1770 default - 1850 stowed
//int d = 0;      // Desired Value

void setup() {
  Serial.begin(115200);
  pinMode(servoPin, OUTPUT);
}

void loop() {
  while (Serial.available() == 0) {
  }

  int menuChoice = Serial.parseInt();

  Serial.println(menuChoice);

  servo(menuChoice);
  delay(600);
}

void servo(int x) {
  for (int i = 0; i <= 2; i++) {
    // A pulse each 20ms
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(x); // Duration of the pusle in microseconds
    digitalWrite(servoPin, LOW);
    delayMicroseconds(18550); // 20ms - duration of the pusle
    // Pulses duration: 500 - 0deg; 1500 - 90deg; 2500 - 180deg
  }
}