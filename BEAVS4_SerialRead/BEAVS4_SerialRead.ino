/* 
BEAVS4 Code

This code is responsible for collecting and acting on live data to air brake durring the flight of a rocket. 

By: Colin Hale-Brown & Dexter Carpenter

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
const int interupt_SW = 27;
const int interupt_LED = 26;
const int IO1_LED = 21;
const int IO2_LED = 20;
const int IO3_LED = 19;
const int servoPin = 28;

// Define Active Variables
int intupSW = 0;
//#define servoPin 28

/* -------------------- GLOBAL VARS -------------------- */

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

float SerialData[4];

/* -------------------- CORE 0 -------------------- */

void setup() {
  // Define Inputs and outputs
  pinMode(servoPin, OUTPUT); // Servo is a digital output
  pinMode(interupt_SW, INPUT); // Interupt switch input
  pinMode(interupt_LED, OUTPUT); // Interupt LED output
  pinMode(IO1_LED, OUTPUT); // Input-Output 1 Indicator output
  pinMode(IO2_LED, OUTPUT); // Input-Output 2 Indicator output
  pinMode(IO3_LED, OUTPUT); // Input-Output 3 Indicator output

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
      //file.println("Pressure (hPa),Temperature (*C),Altimeter (m)");
      file.println("Serial Data");
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

void loop() {

  while(intupSW == HIGH);
  
  if (Serial.available() > 0) {
    for (int i=0; i<3; i++) {
      SerialData[i] = Serial.parseFloat();
    }
  }
  
  /*
  // Data Packing
  String temperatureData = (String)temperature;
  String pressureData = (String)pressure;
  String altimeterData = (String)altimeter;
  dataString = pressureData + cell + temperatureData + cell + altimeterData;
  Serial.println(dataString);
  */

  for (int i=0; i<) {
    sprintf(dataString,);
    String dataString = (String)SerialData;
  }

  // Data Logging
  file = SD.open("BEAVS4_data.csv", FILE_WRITE);
  if (file) {
    file.println(dataString); //print data to file
    file.close();
    //Serial.println("Data Recorded");
  } else {
    //Serial.println("Data Failed to record");
  }

  // Print Data to Serial Monitor
  delay(10);
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
  /*
  // BMP390 Data Collection
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading whomp whomp");
    return;
  } else {
    temperature = bmp.temperature;
    pressure = bmp.pressure / 100.0;
    altimeter = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  }
  */
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

/*
double PID(time, Kp, Ki, Kd, u, V, Vtarg) {
  double time = time;
  double Kp = Kp;
  double Ki = Ki;
  double Kd = Kd;
  double u = u;
  double V = V;
  double Vtarg = Vtarg;

  // find time difference
  double dt = time[1] - time[0];

  // find error
  double err = Vtarg - V; // would be useful to save this value

  // update control function
  u = u + (Kp+Ki*dt+Kd/dt)*err[2] + abs((-Kp-2*Kd/dt)*err[1]) + (Kd/dt)*err[0];
  return u;
}

double VelLookUp(H) {
  double H = H;
  
  // define polynomial
  // polynomial fit found via MATLAB
  float p1 = -2.197790209276072e-9;
  float p2 =  1.424454885385808e-6;
  float p3 = -2.425899535946396e-4;
  float p4 = -0.031992842779187;
  float p5 = -0.261849939656465;

  double V = p1*(H^4) + p2*(H^3) + p3*(H^2) + p4*(H) + p5;

  return V;
}
*/