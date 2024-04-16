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

int enA = 11; // 9
int in1 = 5;
int in2 = 4;
L298N motorA(enA, in1, in2);

int enB = 6;
int in3 = 3;
int in4 = 2;
L298N motorB(enB, in3, in4);

int motorSpeed = 200;

// Servo control

Servo servo;

int servo_pin = A5; // 11
int servo_pos = 0;
int dir = 1;

// Color sensor

#define S0 0
#define S1 1
#define S2 7
#define S3 8
#define OUT 10 // 10

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

// PD Controller


int error_prev = 0;
int current_time = millis ();
float kp = 5;


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

void loop_old() {

  qtr.read(sensorValues);

  /*
  int err_scale[4] = {-1, -0.5, 0.5, 1};
  int normalizedSensorValues[4] = {0};
  int hist = 500;

  uint16_t scaled_values[4] = {
    err_scale[0] * sensorValues[0],
    err_scale[1] * sensorValues[1],
    err_scale[3] * sensorValues[3],
    err_scale[4] * sensorValues[4],
  };

  uint16_t cum_error = 0;

  for(int i = 0; i < 4; i++) {
    cum_error += scaled_values[i];
  }
  */

  Serial.print(sensorValues[0]);
  Serial.print(", ");
  Serial.print(sensorValues[1]);
  Serial.print(", ");
  Serial.print(sensorValues[2]);
  Serial.print(", ");
  Serial.print(sensorValues[3]);
  Serial.print(", ");
  Serial.print(sensorValues[4]);
  Serial.println("");


  // if(sensorValues[2] >= 500) {

    int errors[5] = {
      -2 * (sensorValues[0] >= 500),
      -1 * (sensorValues[1] >= 500),
      0 * (sensorValues[2] >= 500),
      1 * (sensorValues[3] >= 500),
      2 * (sensorValues[4] >= 500),
    };

    int error = 0;
    
    for(int i = 0; i < 5; i++)
      error += errors[i];
    
    // float u = (float)error * kp + 0.01 + kd * ((error - error_prev) / (millis() - current_time));
    float u = (float)error * kp + 0.01;

    error_prev = error;
    current_time = millis();
    
    if(u > 0.01) {
      motorB.setSpeed(motorSpeed);
      motorA.setSpeed(motorSpeed / (abs(u) * 0.5));
    } else if(u < 0.01) {
      motorB.setSpeed(motorSpeed / (abs(u) * 0.5));
      motorA.setSpeed(motorSpeed);
    }

  // } else {

    // motorA.setSpeed(motorSpeed);
    // motorB.setSpeed(motorSpeed);

  // }

  motorA.forward();
  motorB.forward();

  /*
  if(error == 1) {
    motorB.setSpeed(motorSpeed / 2);
    motorA.setSpeed(0);
  } else if(error == 2) {
    motorB.setSpeed(motorSpeed);
    motorA.setSpeed(0);
  } else if(error == -1) {
    motorA.setSpeed(motorSpeed / 2);
    motorB.setSpeed(0);
  } else if(error == -2) {
    motorA.setSpeed(motorSpeed);
    motorB.setSpeed(0);
  } else {
    motorA.setSpeed(motorSpeed);
    motorB.setSpeed(motorSpeed);
  }
  */

  /*
  if(error > 0) {

    motorB.setSpeed(motorSpeed);
    motorA.setSpeed(0);

  } else if(error < 0) {
    
    motorB.setSpeed(0);
    motorA.setSpeed(motorSpeed);

  } else {

    motorA.setSpeed(motorSpeed);
    motorB.setSpeed(motorSpeed);

  }
  */

  /*
  if(sensorValues[0] >= 500 || sensorValues[1] >= 500) {

    motorA.setSpeed(motorSpeed);
    motorB.setSpeed(0);

  } else if(sensorValues[3] >= 500 || sensorValues[4] >= 500) {

    motorA.setSpeed(0);
    motorB.setSpeed(motorSpeed);
  } 
  */

}


void loop() {
  
  // QTR loop

  qtr.read(sensorValues);

  /*
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

  servo.write(servo_pos);
  delay(15);

  if(servo_pos == 180) {
    dir = -1;
  } else if(servo_pos == 0) {
    dir = 1;
  }

  servo_pos += dir;
  */

  Serial.print(sensorValues[0]);
  Serial.print(", ");
  Serial.print(sensorValues[1]);
  Serial.print(", ");
  Serial.print(sensorValues[2]);
  Serial.print(", ");
  Serial.print(sensorValues[3]);
  Serial.print(", ");
  Serial.print(sensorValues[4]);
  Serial.println("");  

  /*
  if(sensorValues[0] >= 500 || sensorValues[1] >= 500) {

    motorA.setSpeed(motorSpeed * 0.75);
    motorB.setSpeed(0);

  } else if(sensorValues[3] >= 500 || sensorValues[4] >= 500) {

    motorB.setSpeed(motorSpeed * 0.75);
    motorA.setSpeed(0);

  // } else if(sensorValues[1] >= 500 && sensorValues[3] >= 500) {
  } else if(sensorValues[2] >= 500 || (sensorValues[1] >= 500 && sensorValues[3] >= 500)) {

    motorA.setSpeed(motorSpeed);
    motorB.setSpeed(motorSpeed);

  }
  */

  if(sensorValues[0] >= 500) {

    motorA.setSpeed(motorSpeed);
    motorB.setSpeed(motorSpeed * 0.6);

  } else if(sensorValues[1] >= 500) {

    motorA.setSpeed(motorSpeed);
    motorB.setSpeed(motorSpeed * 0.7);

  } else if(
    sensorValues[2] >= 500 &&
    (sensorValues[2] >= 500 && sensorValues[1] >= 500) ||
    (sensorValues[2] >= 500 && sensorValues[3] >= 500)
    ) {

    motorA.setSpeed(motorSpeed);
    motorB.setSpeed(motorSpeed);

  } else if(sensorValues[3] >= 500) {

    motorB.setSpeed(motorSpeed);
    motorA.setSpeed(motorSpeed * 0.7);

  } else if(sensorValues[4] >= 500) {

    motorB.setSpeed(motorSpeed);
    motorA.setSpeed(motorSpeed * 0.6);

  }

  motorA.forward();
  motorB.forward();

}