#define servoPin 28

int prevX1 = 0;
int prevX2 = 0;

void setup() {
  Serial.begin(115200);
  pinMode(servoPin, OUTPUT);
}

void loop() {
  while (Serial.available() == 0) {
  }

  int u = Serial.parseInt();

  Serial.print("Motor Signal: ");
  Serial.println(u);

  servo(u);
  delay(200);
}

void servo(int d){
  // 500 load - 670 full extention - 1770 default - 1850 stowed
  int max = 1770;
  int min = 670;
  int extendH = 5;
  int retractH = 5;
  int extention = 60;

  int x = -((max - min) / extention) * d + max;
  int dx = prevX1 - prevX2;

  if (x > prevX1 && dx < 0){
    x = x + extendH;
  } else if (x < prevX1 && dx > 0){
    x = x - retractH;
  }

  for (int i = 0; i <= 2; i++) {
    // A pulse each 20ms
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(x); // Duration of the pusle in microseconds
    digitalWrite(servoPin, LOW);
    delayMicroseconds(18550); // 20ms - duration of the pusle
    // Pulses duration: 500 - 0deg; 1500 - 90deg; 2500 - 180deg
  }
}