#include <Wire.h>
#include "MAX30105.h"    //MAX3010x library
#include <ArduinoBLE.h>  //Bluetooth library
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

MAX30105 particleSensor;       // Initialize MAX3010x sensor
BLEService wwdService("8c2296a3-971b-4285-98e8-222d51111ef2"); // create service

//For HeartRate
static double fbpmrate = 0.95; // low pass filter rate in bpm
static uint32_t crosstime = 0; //falling edge , zero crossing time in msec
static uint32_t crosstime_prev = 0;//previous falling edge , zero crossing time in msec
static double bpm = 40.0;
static double ebpm = 40.0;
static double eir = 0.0; //estimated lowpass filtered IR signal
static double firrate = 0.85; //IR filter coefficient
static double eir_prev = 0.0;

#define FINGER_ON_IR 50000
#define MAX_BPS 180
#define MIN_BPS 45

#define MINIMUM_SPO2 80.0

//vibration motor
int motorPin = 3; //motor transistor is connected to pin 3

//create characteristic and allow remote device to read and write, notify
BLECharacteristic sensorCharacteristic("369c0bef-e292-4e84-8696-75eb9a5c7a54", BLERead | BLEWrite | BLENotify, 20);
BLEByteCharacteristic vibrationCharacteristic("3de2545d-9de6-47ce-84a9-68c5ead85c67", BLERead | BLEWrite | BLENotify);
BLEDescriptor sensorDescriptor("2901", "Heartrate and Oxygen Saturation");
BLEDescriptor vibrationDescriptor("2901", "Vibration");

void setup() {
  Serial.begin(9600);
  pinMode(motorPin, OUTPUT);
  //while (!Serial); //waiting for the serial monitor to open
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30102 was not found. ");
  }
  
  //Initialize sensor
  //Setup MAX30102
  byte ledBrightness = 0x1F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 8; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  //Options: 1 = IR only, 2 = Red + IR
  int sampleRate = 400; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  // Set up with the parameters
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  BLE.setDeviceName("Croffle");
  // set the local name peripheral advertises
  BLE.setLocalName("Croffle");
  // set the UUID for the service this peripheral advertises
  BLE.setAdvertisedServiceUuid(wwdService.uuid());

  // add the characteristic to the service
  wwdService.addCharacteristic(sensorCharacteristic);
  wwdService.addCharacteristic(vibrationCharacteristic);

  // add the descriptor to the service and characteristic
  sensorCharacteristic.addDescriptor(sensorDescriptor);
  vibrationCharacteristic.addDescriptor(vibrationDescriptor);

  // add service
  BLE.addService(wwdService);

  // subscribe for notify
  sensorCharacteristic.subscribe();
  vibrationCharacteristic.subscribe();

  // assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // assign event handlers for characteristic
  sensorCharacteristic.setEventHandler(BLERead, sensorCharacteristicRead);
  vibrationCharacteristic.setEventHandler(BLEWritten, vibrationCharacteristicWritten);

  // int value bpm and SpO2 to string value
  // connecting bpm and SpO2 based on ":"
  // format "bpm:spo2"
  char bpm[20];
  char spo2[20];
  char split[] = ":";
  itoa(0, bpm, 10);
  itoa(0, spo2, 10);
  strcat(bpm, split);
  strcat(bpm, spo2);
  
  // set an initial value for the characteristic
  sensorCharacteristic.setValue(bpm);
  vibrationCharacteristic.setValue(0);

  // start advertising
  BLE.advertise();
  Serial.print("Croffle arduino : ");
  Serial.println(BLE.address());
  Serial.println(("Croffle : Bluetooth device active, waiting for connections..."));
}


int i = 0;
int Num = 100; //calculate SpO2 and average HeartRate by this sampling interval

//For filtering of IR and RED
double fred = 0; double fir = 0;
double frate = 0.95; //low pass filter rate

//For SpO2
double red_sum = 0; double ir_sum = 0;
double ESpO2 = 95.0;//initial value of estimated SpO2
double FSpO2 = 0.7; //filter factor for estimated SpO2

void loop() {
  double ir , red;
  double SpO2 = 0.0; //raw SpO2 value

  particleSensor.check();  //Check the sensor
  int Ebpm = 60;
  double Ebpm_sum = 60.0;
  
  while (1) {

    red = particleSensor.getRed(); //get RED data
    ir = particleSensor.getIR(); //get IR data

    i++;
    
    //filtering
    fred = fred * frate + red * (1.0 - frate);//filtered RED by low pass filter
    fir = fir * frate + ir * (1.0 - frate); //filtered IR by low pass filter

    //calcurate HeartRate and sum
    Ebpm_sum += HRM_estimator(ir, fir);

    ir_sum += (red - fred) * (red - fred); //sum of RED for R
    red_sum += (ir - fir) * (ir - fir);//sum of IR for R

    // poll for BLE events
    BLE.poll();

    if (ir < FINGER_ON_IR) { //No finger detected
      //Serial.println("NO finger");

      //reset measured value
      i = 0;
      ir_sum = 0.0;
      red_sum = 0.0;
      Ebpm_sum = 60.0;
      continue;
    }

    //calculate SpO2 and average HeartRate by 100 sampling interval
    if ((i % Num) == 0) {
      
      double R = (sqrt(ir_sum) / fred) / (sqrt(red_sum) / fir); // ratio of (redrms/fred) / (irrms/fir)
      SpO2 = 104 - 17 * R; // SpO2 formula based on MAX30102 datasheet
      ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2; // estimated SpO2 filtered
      
      Ebpm = Ebpm_sum / Num;  //Average bpm

      // from peripheral to react-native-app
      // heart rate and oxygen saturation
      // bpm and spo2 is a float value
      // so, multiplied by 100 to make it an integer
      // int value bpm and SpO2 to string value
      // connecting bpm and SpO2 based on ":"
      // format "bpm:spo2"
      char bpm[20];
      char spo2[20];
      char split[] = ":";
      itoa(Ebpm * 100, bpm, 10);
      itoa(ESpO2 * 100, spo2, 10);
      strcat(bpm, split);
      strcat(bpm, spo2);

      // change characteristic value
      sensorCharacteristic.writeValue(bpm);

      Serial.print("Ebpm : ");
      Serial.print(Ebpm);
      Serial.print(" SpO2 : ");
      Serial.print(SpO2);
      Serial.print(" ESpO2 : ");
      Serial.print(ESpO2);
      Serial.print(" R:");
      Serial.println(R);

      Serial.print("heart rate and oxygen saturation write : ");
      Serial.println(bpm);

      //reset
      ir_sum = 0.0;
      red_sum = 0.0;
      Ebpm_sum = 0.0;
      i = 0;
      break;
    }
  }

}

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}
void sensorCharacteristicRead(BLEDevice central, BLECharacteristic characteristic) {
  // characteristic wrote new value to central
  Serial.print("heartrateCharacteristic event, read: ");
  //  Serial.print(heartrateCharacteristic.value());
  Serial.println();
}
void vibrationCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic
  if (vibrationCharacteristic.value() == 48) { // 48 is zero
    Serial.print("Characteristic event, written 0: ");
    Serial.print(vibrationCharacteristic.value());
    Serial.println();
  }
  else if (vibrationCharacteristic.value() == 49) { // 49 is one
    // decoded value 49 means a vibration signal one
    Serial.print("Characteristic event, written 1: ");
    Serial.print(vibrationCharacteristic.value());
    Serial.println();

    // vibration start
    digitalWrite(motorPin, HIGH); //vibrate
    delay(100);  // delay one second
    digitalWrite(motorPin, LOW);  //stop vibrating
    
    //reset measured value
    i = 1;
    ir_sum = 0.0;
    red_sum = 0.0;

  }
  vibrationCharacteristic.writeValue(0); // reset vibratioin characteristics
}

/*****calculate HeartRate****
* measuring the time between consecutive systolic peaks
*/
double HRM_estimator( double ir , double fir) {
  int CTdiff;

  eir = eir * firrate + ir * (1.0 - firrate); //low pass filtered IR signal
  
  //measuring the time between consecutive systolic peaks
  if ( ((eir - fir) * (eir_prev - fir) < 0 ) && ((eir - fir) < 0.0)) { //find zero cross at falling edge
    crosstime = millis(); //system time in msec of falling edge
    CTdiff = crosstime - crosstime_prev;

    //if ( ((crosstime - crosstime_prev ) > (60 * 1000 / MAX_BPS)) &&((crosstime - crosstime_prev ) < (60 * 1000 / MIN_BPS)) )
    if ( ( CTdiff > 333 ) && ( CTdiff < 1333 ) ) {
      bpm = 60.0 * 1000.0 / (double)(crosstime - crosstime_prev); //get bpm
      ebpm = ebpm * fbpmrate + (1.0 - fbpmrate) * bpm; //estimated bpm by low pass filtered

    } else {
      //Serial.println("faild to find falling edge");
    }
    crosstime_prev = crosstime;
  }
  eir_prev = eir;
  return (ebpm);
}
