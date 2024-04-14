#include <Servo.h>

Servo servo;

int servo_pin = 11;

void setup() {
  servo.attach(servo_pin);
}

void loop() {
  
  for(int pos = 0; pos <= 180; pos += 1) {
    servo.write(pos);
    delay(15);
  }

  for(int pos = 180; pos >= 0; pos -= 1) {
    servo.write(pos);
    delay(15);
  }
  
  // servo.write(0);
}
