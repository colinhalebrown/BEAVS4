#define servoPin 28

int m = (1770-670)/2 + 670;
int d = 550;
int x;

void setup() {
  Serial.begin(9600);
  pinMode(servoPin, OUTPUT);
}

void loop() {
   d = d*-1;
   x = m + d;
   //x = 0;
   
   servo(x);
   Serial.println(x);
   delay(525);
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