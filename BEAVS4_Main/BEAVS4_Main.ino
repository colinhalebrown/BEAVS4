/* 
BEAVS4 Code

This code is responsible for collecting and acting on live data to air brake durring the flight of a rocket. 

By: Colin Hale-Brown & Dexter Capenter
*/

/* --------------- CONFIG & INCLUDED LIBRARIES --------- */

// SD Card Config
const int _MISO = 8;
const int _MOSI = 11;
const int _CS = 9;
const int _SCK = 10;

// Libraries to include
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* -------------------- DEFINE HARDWARE ---------------- */

// Define Active Hardware
const int interrupt_SW = 26;
const int servoPin = 27;
const int servoPower = 28;

// Initialize I2C
#define WIRE Wire

// Define Sea level pressure
#define SEALEVELPRESSURE_HPA (1013.25)

// Defince Sensor addresses
Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

/* ------------------- DATA VARS ------------------ */

// Initialize SD card 
File file;
String cell = ",";
String dataString;

// BMP390 Variables
float pressure = 0;
float temperature = 0;
float altimeter = 0;

// BNO055 Variables
float xAccel = -100000;
float yAccel = -100000;
float zAccel = -100000;

// Calculated Variables
float velocity = 0;
float velTimeNow = 0;
float velTimePrev = 0;
float altPrev = 0;

/* ------------------- CONTROL VARS --------------- */
float groundLevel = 0;
float altAGL = 0; // Altitude Above Ground Level
int check = 120; //time between checks in ms

bool flightPhase = false; // latch for rocket flight
bool motorCut = false; // latch for motor cutoff
bool coastPhase = false; // latch for the coast phase
bool apogee = false; // latch for recover

float altTrigger = 150; // trigger altitude 150m ~ 500ft
long CoastStart = 0;
long RetractDelay = 40000;


/* ------------------- PID VARS ------------------- */
float H = 0; // height in meters // CORE 0 FUNCTION!!!!!!!!!
float V = 0; // velocity in m/s  // CORE 0 FUNCTION!!!!!!!!!
float Vtarg = 0; // target velocty, m/s
int dt = 0;
long timePrev = 0;
long timeNow = 0;
float u = 0;

// PID constants
float Kp = 0.200e-04;
float Ki = 1.250e-09;
float Kd = 7.500e-05;

// error
float err1 = 0;
float err2 = 0;
float err3 = 0;

/* -------------------- CORE 0 -------------------- */

void setup() {
  // Define Inputs and outputs
  pinMode(LED_BUILTIN, OUTPUT); // Pico LED

  // Data Startup
  digitalWrite(LED_BUILTIN, LOW); // Turn off LED at start

  // BMP390 check
  if (!bmp.begin_I2C()) {
    digitalWrite(LED_BUILTIN, HIGH); // Indicate System Check Failed
    while (1);

  } else {
    // Set up BMP390 oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }
  
  // BNO055 Check
  if (!bno.begin()){
    /* There was a problem detecting the BNO055 ... check your connections */
    digitalWrite(LED_BUILTIN, HIGH); // Indicate System Check Failed
    while (1);

  }

  // SD Card Check
  SPI1.setRX(_MISO);
  SPI1.setTX(_MOSI);
  SPI1.setSCK(_SCK);
  bool sdInitialized = false;
  sdInitialized = SD.begin(_CS, SPI1);

  if (!sdInitialized) {
    digitalWrite(LED_BUILTIN, HIGH); // Indicate System Check Failed
    while (1);

  } else {
    file = SD.open("BEAVS4_data.csv", FILE_WRITE);
    if (file) {
      file.println("Time (ms),Pressure (hPa),Temperature (*C),Altimeter (m),Velocity (m/s),Altimeter AGL (m),X Acceleration (m/s^2),Y Acceleration (m/s^2),Z Acceleration (m/s^2),Error,U");
      file.close();
    } else {
      digitalWrite(LED_BUILTIN, HIGH); // Indicate System Check Failed
    }
  }

  delay(180000);

  while(digitalRead(interrupt_SW) == HIGH); // can activate any time to scrub

}

// DATA LOOP
void loop() {

  measure();
  calculate();
  SDlog();

}

/* -------------------- CORE 1 -------------------- */

void setup1() {

  pinMode(servoPower, OUTPUT); // Servo power control pin
  pinMode(servoPin, OUTPUT); // Servo is a digital output
  pinMode(interrupt_SW, INPUT); // Interupt switch input

  delay(180000);

  while(digitalRead(interrupt_SW) == HIGH); // can activate any time to scrub

  digitalWrite(servoPower, HIGH); // Power servo

  servo(0);
  delay(500);
  servo(60);
  delay(500);
  servo(30);
  delay(500);
  servo(60);
  delay(500);
  servo(30);
  delay(500);
  servo(0);

  // wait for sensors to calibrate and initialize
  delay(30000);
  groundLevel = altimeter; // Set AGL
}

// CONTROL LOOP
void loop1() {

  // polling coast phase interrupt
  while(coastPhase == false){
    int i;

    // latch for rocket flight
    if (altAGL > altTrigger){
      flightPhase = true;
    }

    // latch for motor cutoff
    if (zAccel >= 0 && flightPhase == true){
      i++;
      delay(check);
    } else {
      i = 0;
    }
    if (i >= 5){
      motorCut = true;
    }

    // latch for coast phase
    if (flightPhase == true && motorCut == true){
      coastPhase = true;
      CoastStart = millis();
      digitalWrite(servoPower, HIGH); // Power servo
    }
  }

  // PID control
  if (coastPhase == true){
    PID();
    servo(u);

  }

  // Apogee latch trigger

  if ((millis() - CoastStart >= RetractDelay) && (coastPhase == true)){
    apogee = true;
  }

  // retract blades after apogee
  while(apogee == true){
    servo(0);
    delay(1000);
  }
}

/* -------------------- FUNCTIONS -------------------- */

void servo(int d) {
  int max = 1860;
  int min = 770;
  int maxExtention = 60;

  if (d > 60){
    d = 60;
  } else if (d < 0){
    d = 0;
  }
  int x = -((max - min) / maxExtention) * d + max;

  for (int i = 0; i <= 2; i++) {
    // A pulse each 20ms
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(x); // Duration of the pusle in microseconds
    digitalWrite(servoPin, LOW);
    delayMicroseconds(18550); // 20ms - duration of the pusle
    // Pulses duration: 500 - 0deg; 1500 - 90deg; 2500 - 180deg
  }
}

void measure() {
  // BMP390 Data Collection
  if (! bmp.performReading()) {
    digitalWrite(LED_BUILTIN, HIGH); // Indicate System Check Failed
    return;
  } else {
    temperature = bmp.temperature;
    pressure = bmp.pressure / 100.0;
    altimeter = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  }

  // Get BNO055 Data
  sensors_event_t accelerometerData;
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  sensors_event_t* event = &accelerometerData;
  event->type == SENSOR_TYPE_ACCELEROMETER;
  // Update BNO055 Data
  xAccel = event->acceleration.x;
  yAccel = event->acceleration.y;
  zAccel = event->acceleration.z;

}

void calculate() {
  // Calculate Velocity
  velTimeNow = millis();

  velocity = (altimeter - altPrev) / (velTimeNow - velTimePrev);

  velTimePrev = velTimeNow;
  altPrev = altimeter;

  // Calculate AGL
  altAGL = altimeter - groundLevel;
}

void SDlog(){
// Data Packing
  String timeData = (String)millis();
  String pressureData = (String)pressure;
  String temperatureData = (String)temperature;
  String altimeterData = (String)altimeter;
  String velocityData = (String)velocity;
  String altAGLData = (String)altAGL;
  String xAccelData = (String)xAccel;
  String yAccelData = (String)yAccel;
  String zAccelData = (String)zAccel;
  String errorData = (String)err1;
  String uData = (String)u;

  dataString = timeData + cell
            + pressureData + cell 
            + temperatureData + cell 
            + altimeterData + cell
            + velocityData + cell
            + altAGLData + cell
            + xAccelData + cell
            + yAccelData + cell
            + zAccelData + cell
            + errorData + cell
            + uData;
  //Serial.println(dataString);

  // Data Logging
  file = SD.open("BEAVS4_data.csv", FILE_WRITE);
  if (file) {
    file.println(dataString); //print data to file
    file.close();
  } else {
    digitalWrite(LED_BUILTIN, HIGH); // Indicate System Check Failed
  }
}

void PID() {
  // calculate change in time
  timeNow = millis();
  int dt = timeNow - timePrev; // in milliseconds

  // update target velocity
  VelLookup();

  // calculate error
  float err3 = err2; // prev prev error
  float err2 = err1; // prev error
  float err1 = Vtarg - V; // current error

  // update control function
  float u = u + (Kp+Ki*dt+Kd/dt)*err1 + abs((-Kp-2*Kd/dt)*err2) + (Kd/dt)*err3;

  // set prev time to curret time
  timePrev = timeNow;
}

void VelLookup() {
  // define polynomial
  // polynomial fit found via MATLAB
  float p1 = -2.197790209276072e-9;
  float p2 =  1.424454885385808e-6;
  float p3 = -2.425899535946396e-4;
  float p4 = -0.031992842779187;
  float p5 = -0.261849939656465;

  // set target velocity
  float Vtarg = p1*pow(H,4) + p2*pow(H,3) + p3*pow(H,2) + p4*(H) + p5;
}