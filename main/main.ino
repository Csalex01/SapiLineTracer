// ---------- //
// Headers    //
// ---------- //

#include "ir_array.h"

// ---------- //
// Variables  //
// ---------- //

int init_sleep_time = 100;


void setup() {

  Serial.begin(9600);
  
  // Set up pins
  
  pinMode(LED_BUILTIN, OUTPUT);

  // Signal initialization

  for(int i = 0; i < 10; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(init_sleep_time / 2); 
    digitalWrite(LED_BUILTIN, LOW);
    delay(init_sleep_time / 2);
  }


}

void loop() {

}
