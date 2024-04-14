/*
// ---------- //
// Headers    //
// ---------- //

#include <QTRSensors.h>
#include <Arduino.h>

#ifndef H_IR_ARRAY
#define H_IR_ARRAY

#define NUM_SENSORS             5                   // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4                   // average 4 analog samples per sensor reading
#define EMITTER_PIN             0  // emitter is controlled by digital pin 2

QTRSensors qtr;

const uint8_t SensorCount = NUM_SENSORS;
uint16_t sensorValues[SensorCount];

// ---------- //
// Functions  //
// ---------- //

void init_qtr();
void loop_qtr();

#endif
/*