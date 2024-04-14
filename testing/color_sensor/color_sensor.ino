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

int COLOR_COOLDOWN_MS = 20;

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

  /*
  Serial.print("RED: ");
  Serial.print(RED);
  Serial.println("");
  Serial.print("GREEN: ");
  Serial.print(GREEN);
  Serial.println("");
  Serial.print("BLUE: ");
  Serial.print(BLUE);
  Serial.println("");

  delay(100);
  */
  
}

void setup() {

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);

  Serial.begin(9600);

  digitalWrite(S0, HIGH);
  digitalWrite(S1, HIGH);

}

void loop() {

  if(COLOR_SAMPLE_COUNT >= COLOR_SAMPLES) {

    if(RED_SAMPLES > GREEN_SAMPLES) {
      Serial.println("RED");
    } else if(RED_SAMPLES < GREEN_SAMPLES) {
      Serial.println("GREEN");
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
