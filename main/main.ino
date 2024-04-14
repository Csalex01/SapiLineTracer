// ---------- //
// Headers    //
// ---------- //

#include <QTRSensors.h>
#include <L298N.h>
#include <Servo.h>

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

// Servo control

Servo servo;

int servo_pin = 11;

// Color sensor

#define S0 0
#define S1 1
#define S2 7
#define S3 8
#define OUT 10

int RED = 0;
int GREEN = 0;
int BLUE = 0;

int COLOR_SAMPLES = 10;
int COLOR_SAMPLE_COUNT = 0;

int RED_SAMPLES = 0;
int GREEN_SAMPLES = 0;
int BLUE_SAMPLES = 0;

bool DETECTED_RED = false;
bool DETECTED_GREEN = false;

int COLOR_COOLDOWN_MS = 20;

// Ultrasonic distance sensor

int TRIGGER_PIN = 13;
int ECHO_PIN = 12;

long duration;
int distance;

// ---------- //
// Functions  //
// ---------- //

// QTR Sensor

void init_qtr() {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4}, SensorCount);
  
  delay(100);
}

// Servo

void init_servo() {
  servo.attach(servo_pin);
  servo.write(0);
}

void servo_test() {
  for(int pos = 0; pos <= 180; pos += 1) {
    servo.write(pos);
    delay(15);
  }

  for(int pos = 180; pos >= 0; pos -= 1) {
    servo.write(pos);
    delay(15);
  }
}

// Color sensor

void init_color_sensor() {
  digitalWrite(S0, HIGH);
  digitalWrite(S1, HIGH);
}

void get_colors() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);

  // RED = pulseIn(OUT, digitalRead(OUT) == HIGH ? LOW : HIGH);
  RED = pulseIn(OUT, LOW);
  delay(COLOR_COOLDOWN_MS);

  digitalWrite(S3, HIGH);

  // BLUE = pulseIn(OUT, digitalRead(OUT) == HIGH ? LOW : HIGH);
  BLUE = pulseIn(OUT, LOW);
  delay(COLOR_COOLDOWN_MS);

  digitalWrite(S2, HIGH);
  
  // GREEN = pulseIn(OUT, digitalRead(OUT) == HIGH ? LOW : HIGH);
  GREEN = pulseIn(OUT, LOW);
  delay(COLOR_COOLDOWN_MS);

}

void color_sensor_loop() {
  if(COLOR_SAMPLE_COUNT >= COLOR_SAMPLES) {

    if(RED_SAMPLES > GREEN_SAMPLES) {

      Serial.println("RED");
      DETECTED_RED = true;

    } else if(RED_SAMPLES < GREEN_SAMPLES) {

      Serial.println("GREEN");
      DETECTED_GREEN = true;

    } else {
      Serial.println("EQUAL");
    }

    RED_SAMPLES = 0;
    GREEN_SAMPLES = 0;
    BLUE_SAMPLES = 0;
    COLOR_SAMPLE_COUNT = 0;
  }

  get_colors();

  if(RED < BLUE && RED <= GREEN && RED < 23) {
    // Serial.println("RED");
    RED_SAMPLES++;
    COLOR_SAMPLE_COUNT++;
  }

  else if(GREEN < RED && GREEN - BLUE <= 8) {
    // Serial.println("GREEN");
    GREEN_SAMPLES++;
    COLOR_SAMPLE_COUNT++;
  }
}

// Ultrasonic distance sensor

void distance_loop() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.println(distance);
}

void flash_led() {
  if(distance >= 10 && distance < 30) {

    for(int i = 0; i < 3; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(2000);
    }

  } else if(distance >= 30 && distance < 50) {

    for(int i = 0; i < 3; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(2000);
    }

  } else if(distance > 50 && distance <= 70) {

    for(int i = 0; i < 3; i++) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(2000);
    }

  }
}

// Main

void setup() {

  // Serial communication

  Serial.begin(9600);
  
  // Set up pins

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize QTR

  init_qtr();

  // Initialize servo

  init_servo();

  // Initialize color sensor

  init_color_sensor();

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

  qtr.read(sensorValues);

  // Color sensor

  color_sensor_loop();

  if(DETECTED_GREEN) {
    
    servo.write(0);
    distance_loop();
    flash_led();
    DETECTED_GREEN = false;

    return;
    
  } else if(DETECTED_RED) {

    servo.write(180);
    distance_loop();
    flash_led();
    DETECTED_RED = false;

    return;

  }

  // Motor control

  if(sensorValues[0] >= 500 || sensorValues[1] >= 500) {
    motorA.setSpeed(motorSpeed);
    motorB.setSpeed(0);
  } else if(sensorValues[3] >= 500 || sensorValues[4] >= 500) {
    motorB.setSpeed(motorSpeed);
    motorA.setSpeed(0);
  } else {
    motorA.setSpeed(motorSpeed);
    motorB.setSpeed(motorSpeed);
  }

  motorA.forward();
  motorB.forward();
  
}