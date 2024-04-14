// ---------- //
// Headers    //
// ---------- //

#include <QTRSensors.h>
#include <L298N.h>

// ---------- //
// Variables  //
// ---------- //

// Initialization

int init_sleep_time = 100;

// QTR Sensor

QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
int freq = 5;

// Motor control

int enA = 9;
int in1 = 5;
int in2 = 4;
L298N motorA(enA, in1, in2);

int enB = 6;
int in3 = 3;
int in4 = 2;
L298N motorB(enB, in3, in4);

int motorSpeed = 160;

// PD Control
double error = 0;

// ---------- //
// Functions  //
// ---------- //

// QTR Sensor

void init_qtr() {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4}, SensorCount);
  
  delay(100);
}

void loop_qtr() {
  qtr.read(sensorValues);
}


// PD Controller

double error_function(int value, int idx) {
  double offset[5] = {-2, -1, 0, 1, 2};
  return value * offset[idx] / 10;
}

// Main

void setup() {

  Serial.begin(9600);
  
  // Set up pins
  
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialization
  init_qtr();

  // Signal initialization

  for(int i = 0; i < 10; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(init_sleep_time / 6); 
    digitalWrite(LED_BUILTIN, LOW);
    delay(init_sleep_time / 6);
  }

}

void loop() {
  
  // QTR loop

  loop_qtr();

  // Controller

  double error = 0;

  if(sensorValues[0] >= 300 || sensorValues[1] >= 300) {
    motorA.setSpeed(motorSpeed);
    motorB.setSpeed(0);
  } else if(sensorValues[3] >= 300 || sensorValues[4] >= 300) {
    motorB.setSpeed(motorSpeed);
    motorA.setSpeed(0);
  } else {
    motorA.setSpeed(motorSpeed);
    motorB.setSpeed(motorSpeed);
  }

  motorA.forward();
  motorB.forward();
  
}