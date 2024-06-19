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
const int servoPin = 28;
const int servoPower = 4;

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
float groundLevel = 1208;
float altAGL = 0; // Altitude Above Ground Level
int check = 120; //time between checks in ms

bool flightPhase = false; // latch for rocket flight
bool motorCut = false; // latch for motor cutoff
bool coastPhase = false; // latch for the coast phase
bool apogee = false; // latch for recover

float altTrigger = 150; // trigger altitude 
float dh = 0; // change in altitude
int c = 0; // counter for agogee trigger

// Hysteresis variables
int prevX1 = 0;
int prevX2 = 0;

/* ------------------- PID VARS ------------------- */
float H = 0; // height in meters // CORE 0 FUNCTION!!!!!!!!!
float V = 0; // velocity in m/s  // CORE 0 FUNCTION!!!!!!!!!
float Vtarg = 0; // target velocty, m/s
int dt = 0;
long timePrev = 0;
long timeNow = 0;
float u = 0;

// PID constants
float Kp = 1;
float Ki = 1;
float Kd = 1;

// error
float err1 = 0;
float err2 = 0;
float err3 = 0;

/* -------------------- CORE 0 -------------------- */

void setup() {
  // Define Inputs and outputs
  pinMode(servoPower, OUTPUT); // Servo power control pin
  pinMode(servoPin, OUTPUT); // Servo is a digital output
  pinMode(interrupt_SW, INPUT); // Interupt switch input
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
      file.println("Time (ms),Pressure (hPa),Temperature (*C),Altimeter (m),Velocity (m/s),X Acceleration (m/s^2),Y Acceleration (m/s^2),Z Acceleration (m/s^2),Error,U");
      file.close();
    } else {
      digitalWrite(LED_BUILTIN, HIGH); // Indicate System Check Failed
    }
  }
}

// DATA LOOP
void loop() {

  measure();
  calculate();
  SDlog();

}

/* -------------------- CORE 1 -------------------- */

void setup1() {

}

// CONTROL LOOP
void loop1() {
  // physical interrupt hold trigger
  while(interrupt_SW == HIGH); // can activate any time to scrub

  // polling coast phase interrupt
  while(coastPhase == false){
    altAGL = altimeter - groundLevel;
    int i = 0;

    // latch for rocket flight
    if (altAGL > altTrigger){
      flightPhase = true;
    }

    // latch for motor cutoff
    if (zAccel >= 0 && flightPhase == true){
      i = i + 1;
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
    }
  }

  // PID control
  if (coastPhase == true){
    PID();
  }

  // Apogee latch trigger
  dh = altimeter - altPrev;
  if (dh >= 0){
    c = c + 1;
    if (c >= 6){
      apogee = true;
    }
  } else {
    c = 0;
  }


  // retract blades after apogee
  while(apogee == true){
    servo(0);
    delay(1000);
  }
}

/* -------------------- FUNCTIONS -------------------- */

void servo(int d) {
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
  velTimeNow = millis();

  velocity = (altimeter - altPrev) / (velTimeNow - velTimePrev);

  velTimePrev = velTimeNow;
  altPrev = altimeter;
}

void SDlog(){
// Data Packing
  String timeData = (String)millis();
  String pressureData = (String)pressure;
  String temperatureData = (String)temperature;
  String altimeterData = (String)altimeter;
  String velocityData = (String)velocity;
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
            + xAccelData + cell
            + yAccelData + cell
            + zAccelData + cell
            + errorData + cell
            + uData;
  Serial.println(dataString);

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
  float H = H;
  
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
