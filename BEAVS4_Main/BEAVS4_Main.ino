/* 
BEAVS4 Code

This code is responsible for collecting and acting on live data to air brake durring the flight of a rocket. 

By: Colin Hale-Brown
*/

/* --------------- CONFIG & INCLUDED LIBRARIES --------- */

// SD Card Config
const int _MISO = 8;
const int _MOSI = 11;
const int _CS = 9;
const int _SCK = 10;

// Libraries to include
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* -------------------- DEFINE HARDWARE ---------------- */

// Define Active Hardware
const int interupt_SW = 26;
//const int interupt_LED = 26;
//const int IO1_LED = 21;
//const int IO2_LED = 20;
//const int IO3_LED = 19;
const int servoPin = 28;
const int servoPower = 4;

/* -------------------- GLOBAL VARS -------------------- */

// Define Active Variables
int intupSW = 0; // interupt state
uint16_t SAMPLERATE_DELAY = 100; // time in ms

// Initialize SD card 
File file;
String cell = ",";
String dataString;

// Initialize I2C
#define WIRE Wire

// Define Sea level pressure
#define SEALEVELPRESSURE_HPA (1013.25)

// Defince Sensor addresses
Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// Initialize Variables
float pressure;
float temperature;
float altimeter;

/* ------------------- PID VARS ------------------- */
float H = 0; // height in meters
float V = 0; // velocity in m/s
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
  pinMode(interupt_SW, INPUT); // Interupt switch input
  //pinMode(interupt_LED, OUTPUT); // Interupt LED output
  //pinMode(IO1_LED, OUTPUT); // Input-Output 1 Indicator output
  //pinMode(IO2_LED, OUTPUT); // Input-Output 2 Indicator output
  //pinMode(IO3_LED, OUTPUT); // Input-Output 3 Indicator output

  Serial.begin(115200);
  while (!Serial);

  delay(1000);
  Serial.println("    [ BEAVS Setup ]");

  // BMP390 check
  Serial.print("  BMP390 Status: ");
  if (!bmp.begin_I2C()) {
    Serial.println("Connection Failed");
    while (1);

  } else {
    Serial.println("Connected");
    // Set up BMP390 oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }
  
  // BNO055 Check
  Serial.print("  BNO055 Status: ");
  if (!bno.begin()){
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Connection Failed");
    while (1);

  } else {
    Serial.println("Connected");
  }

  // SD Card Check
  SPI1.setRX(_MISO);
  SPI1.setTX(_MOSI);
  SPI1.setSCK(_SCK);
  bool sdInitialized = false;
  sdInitialized = SD.begin(_CS, SPI1);

  Serial.print("      SD Status: ");
  if (!sdInitialized) {
    Serial.println("Connection Failed");
    while (1);

  } else {
    Serial.println("Connected");

    file = SD.open("BEAVS4_data.csv", FILE_WRITE);
    if (file) {
      file.println("Pressure (hPa),Temperature (*C),Altimeter (m)");
      file.close();
    } else {
      Serial.println("Writing Header Failed");
    }
  }

  // Physical Interupt Check
  intupSW = digitalRead(interupt_SW);

  Serial.print("Interupt Status: ");
  if (intupSW == HIGH) {
    Serial.println("System Interupted");
  } else {
    Serial.println("No Interupt ");
  }

  Serial.println("    ---------------");
  delay(1000);
}

// DATA LOOP
void loop() {

  while(intupSW == HIGH);
  
  // BMP390 Data Collection
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading whomp whomp");
    return;
  } else {
    temperature = bmp.temperature;
    pressure = bmp.pressure / 100.0;
    altimeter = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  }

  // Data Packing
  String temperatureData = (String)temperature;
  String pressureData = (String)pressure;
  String altimeterData = (String)altimeter;
  dataString = pressureData + cell + temperatureData + cell + altimeterData;
  Serial.println(dataString);

  // Data Logging
  file = SD.open("BEAVS4_data.csv", FILE_WRITE);
  if (file) {
    file.println(dataString); //print data to file
    file.close();
    Serial.println("Data Recorded");
  } else {
    Serial.println("Data Failed to record");
  }

  // Print Data to Serial Monitor


  delay(500);
}

/* -------------------- CORE 1 -------------------- */

void setup1() {


}

// CONTROL LOOP
void loop1() {


}

/* -------------------- FUNCTIONS -------------------- */

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

void measure() {
  // BMP390 Data Collection
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading whomp whomp");
    return;
  } else {
    temperature = bmp.temperature;
    pressure = bmp.pressure / 100.0;
    altimeter = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  }

}

void SDlog(){
// Data Packing
  String temperatureData = (String)temperature;
  String pressureData = (String)pressure;
  String altimeterData = (String)altimeter;
  dataString = pressureData + cell + temperatureData + cell + altimeterData;
  Serial.println(dataString);

  // Data Logging
  file = SD.open("BEAVS4_data.csv", FILE_WRITE);
  if (file) {
    file.println(dataString); //print data to file
    file.close();
    Serial.println("Data Recorded");
  } else {
    Serial.println("Data Failed to record");
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
  float u = u + (Kp+Ki*dt+Kd/dt)*err3 + abs((-Kp-2*Kd/dt)*err2) + (Kd/dt)*err1;

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