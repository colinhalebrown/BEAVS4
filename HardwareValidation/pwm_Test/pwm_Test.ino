#define servoPin 27
const int servoPower = 28;

int dly = 2000;

void setup() {
  Serial.begin(115200);
  pinMode(servoPin, OUTPUT);
  pinMode(servoPower, OUTPUT); // Servo power control pin

  digitalWrite(servoPower, HIGH);
}

void loop() {
  //while (Serial.available() == 0);

  //int u = Serial.parseInt();

  //Serial.print("Motor Signal: ");
  //Serial.println(u);
  motorController(0);
  delay(dly);
  motorController(33);
  delay(dly);
}

void motorController(int d){
  // 500 load - 670 full extention - 1770 default - 1850 stowed
  int max = 1860;
  int min = 770;
  int extendH = 5;
  int retractH = 5;
  int extention = 60;

  int x = -((max - min) / extention) * d + max;

  for (int i = 0; i <= 2; i++) {
    // A pulse each 20ms
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(x); // Duration of the pusle in microseconds
    digitalWrite(servoPin, LOW);
    delayMicroseconds(18550); // 20ms - duration of the pusle
    // Pulses duration: 500 - 0deg; 1500 - 90deg; 2500 - 180deg
  }
  //servo(x);
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