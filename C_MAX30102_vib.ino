
#include <Wire.h>
#include "MAX30105.h"           //MAX3010x library

MAX30105 particleSensor;


/****************** heart rate *********************/
static double fbpmrate = 0.95; // low pass filter coefficient for HRM in bpm
static uint32_t crosstime = 0; //falling edge , zero crossing time in msec
static uint32_t crosstime_prev = 0;//previous falling edge , zero crossing time in msec
static double bpm = 40.0;
static double ebpm = 40.0;
static double eir = 0.0; //estimated lowpass filtered IR signal to find falling edge without notch
static double firrate = 0.85; //IR filter coefficient to remove notch ,should be smaller than frate
static double eir_prev = 0.0;

#define MINIMUM_SPO2 80.0 //original code = 80.0

#define FINGER_ON_IR 50000 // if ir signal is lower than this , it indicates your finger is not on the sensor
#define MAX_BPM 180
#define MIN_BPM 45

//vibration motor
int motorPin = 3; //motor transistor is connected to pin 3

void setup() {
  Serial.begin(115200);
  pinMode(motorPin, OUTPUT);  
    // Initialize MAX3010x sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30102 was not found. Please check wiring/power/solder jumper at MH-ET LIVE MAX30102 board. ");
    while (1);
  }

  //Initialize sensor
  //Setup MAX30102
  byte ledBrightness = 0x1F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 8; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  //Options: 1 = IR only, 2 = Red + IR on MH-ET LIVE MAX30102 board
  int sampleRate = 400; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  // Set up the wanted parameters
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

}

double avered = 0; double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
int i = 0;
int Num = 100; //calculate SpO2 by this sampling interval

double ESpO2 = 95.0;//initial value of estimated SpO2
double FSpO2 = 0.7; //filter factor for estimated SpO2
double frate = 0.95; // .95 default low pass filter for IR/red LED value to eliminate AC component
double hr_check = 0.0;

void loop() {
  uint32_t ir, red;
  double fred, fir;
  double SpO2 = 0; //raw SpO2 before low pass filtered

  particleSensor.check();  //Check the sensor, read up to 3 samples
  double Ebpm_sum = 60;
  int Ebpm = 60;
  while (1) {//do we have newdata
    
    red = particleSensor.getRed(); //get RED value
    ir = particleSensor.getIR(); //get IR value
       
    i++;
    fred = (double)red;
    fir = (double)ir;
    avered = avered * frate + fred * (1.0 - frate);//average red level by low pass filter
    aveir = aveir * frate + fir * (1.0 - frate); //average IR level by low pass filter

    hr_check = HRM_estimator(fir, aveir);
    Ebpm_sum += HRM_estimator(fir, aveir); //Ebpm is estimated BPM
        
    sumredrms += (fred - avered) * (fred - avered); //square sum of alternate component of red level
    sumirrms += (fir - aveir) * (fir - aveir);//square sum of alternate component of IR level
    

    
    if (ir < FINGER_ON_IR){ //No finger detected
        Serial.println("NO finger");
        i = 0;
        sumredrms = 0.0;
        sumirrms = 0.0;
        Ebpm_sum = 0.0;
        continue;
    }

    //vibration signal 들어오면 이전 ir, red값 버리고, 진동
    //ble.poll();
    
//    if(SIGNAL){
//      //vibration
//      digitalWrite(motorPin, HIGH); //vibrate
//      delay(100);  // delay one second
//      digitalWrite(motorPin, LOW);  //stop vibrating
//
//      i=0;
//    }
//    
    //100개(Num)의 sample(4초) 모아서 최종 Spo2, bpm계산
    if ((i % Num) == 0) { //every Num samples
        double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir); // ratio of (redrms/avered) / (irrms/aveir)
        //SpO2 = -23.3 * (R - 0.4) + 100; // SpO2 based on Ratio by original code
        SpO2 = 104 - 17*R;  //SpO2 based on Ratio by MAX30102 datasheet
        ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2; // SpO2 filtered
        Ebpm = Ebpm_sum/100;

        Serial.print("Ebpm : ");
        Serial.print(Ebpm);        
        Serial.print(" SpO2 : ");
        Serial.print(SpO2);
        Serial.print(" ESpO2 : ");
        Serial.print(ESpO2);
        Serial.print(" R:"); 
        Serial.println(R);
  

        //reset sums and indices
        sumredrms = 0.0; 
        sumirrms = 0.0; 
        i = 0;

        break;
  }


  }
}



//HeartRate 계산
double HRM_estimator( double fir , double aveir) {
  int CTdiff;

  //Heart Rate Monitor by finding falling edge
  eir = eir * firrate + fir * (1.0 - firrate); //estimated IR : low pass filtered IR signal

  if ( ((eir - aveir) * (eir_prev - aveir) < 0 ) && ((eir - aveir) < 0.0)) { //find zero cross at falling edge
    crosstime = millis(); //system time in msec of falling edge
    CTdiff = crosstime-crosstime_prev;
    

    //if ( ((crosstime - crosstime_prev ) > (60 * 1000 / MAX_BPM)) &&((crosstime - crosstime_prev ) < (60 * 1000 / MIN_BPM)) ) {
    if ( ( CTdiff > 333 ) && ( CTdiff < 1333 ) ) { 
      bpm = 60.0 * 1000.0 / (double)(crosstime - crosstime_prev); //get bpm
      // Serial.println("crossed");
      ebpm = ebpm * fbpmrate + (1.0 - fbpmrate) * bpm; //estimated bpm by low pass filtered
      
    } else {
      Serial.println("faild to find falling edge");
    }
    crosstime_prev = crosstime;
  }
   eir_prev = eir;
   return (ebpm);
}
