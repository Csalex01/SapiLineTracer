#include <L298N.h>

int viteza=200;  //De aici se regleaza viteza motorului (0-255)

/* motorul A */
int enA = 9;                           // motor A; initializeaza "enA" ca fiind PIN-ul 9;
int in1 = 5;                           // initializeaza "in1" ca fiind PIN-ul 5;
int in2 = 4;                           // initializeaza "in2" ca fiind PIN-ul 4;
L298N motorA(enA, in1, in2); // 
/* motorul B */
int enB = 6;                           // motor B; initializeaza "enB" ca fiind PIN-ul 6;
int in3 = 3;                           // initializeaza "in3" ca fiind PIN-ul 3;
int in4 = 2;                           // initializeaza "in4" ca fiind PIN-ul 2;
L298N motorB(enB, in3, in4);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
   Serial.println("Seteaza viteza");
   motorA.setSpeed(viteza);
   motorB.setSpeed(viteza);
   Serial.println("Misca in fata");
   motorA.forward();
   motorB.forward();
   delay(1000);
   Serial.println("Opreste");
   motorA.stop();
   motorB.stop();
   delay(1000);
   Serial.println("Misca in spate");
   motorA.backward();
   motorB.backward();
   delay(1000);
   Serial.println("Opreste");
   motorA.stop();
   motorB.stop();
   delay(1000);
}
