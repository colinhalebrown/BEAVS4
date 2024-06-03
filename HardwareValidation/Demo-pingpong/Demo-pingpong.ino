#define servoPin 28

const int interupt_SW = 27;

int m = (1770-670)/2 + 670;
int d = 550;
int x;

void setup() {
  Serial.begin(9600);
  pinMode(servoPin, OUTPUT);
  servo(1850);
  delay(500);
  servo(1850);
  delay(500);

  pinMode(interupt_SW, INPUT); // Interupt switch input
  delay(8000);

  Serial.println("Ready");
  while(digitalRead(interupt_SW) == HIGH);
  Serial.println("Interupt Passed");

  // Setup Motor
  servo(1770);
  Serial.println("Flush");
  delay(1000);
  servo(670);
  Serial.println("Extend");
  delay(1000);
  servo(1770);
  Serial.println("Flush");
  delay(10000);

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