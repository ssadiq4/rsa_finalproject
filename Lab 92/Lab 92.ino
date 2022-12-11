#include <math.h>
// have to declare variables as volatile for ISRs (false = low and true = high)
volatile bool encoderA = false;
volatile bool encoderB = false;
volatile int counter;
const int I1 = 10;
const int I2 = 9;
const int E1 = 11;
int pmCounter;
unsigned long previousTime = millis();


void setup() {
  // starting Serial Monitor
  Serial.begin(9600);
  // attach ISR for when encoder event occurs
  attachInterrupt(0, encoderChangeA, CHANGE);
  attachInterrupt(1, encoderChangeB, CHANGE);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(A3, INPUT);
}

void loop() {
  pmCounter = (analogRead(A3) + 1) * 15.0 / 32.0;
  float error = (pmCounter - counter) * 2 * M_PI / 480;
  int k = 3;
  float control = k * error;
  analogWrite(E1, control);
  if (control > 0) {
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
  }
    if (control < 0) {
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
  }
  control = abs(control / 5 * 255);
  control = constrain(control, 0, 255);
  control = int(control);
  
  const int deltaT = 10;
  while (millis() - previousTime < deltaT) {};
  previousTime = millis();

  //constantly print counter variable and then wait 10 ms
  Serial.println(counter);
  delay(10);
  Serial.println(pmCounter);
  Serial.println(control);
  delay(500);
}

void encoderChangeA () {
  // increase counter variable if channel A is on different level than channel B
  if (encoderA != encoderB) {
    counter++;
  }
  // decrease counter variable if channel A is on same level than channel B
  else {
    counter--;

  }
  // change encoder boolean
  encoderA = !encoderA;
}

void encoderChangeB () {
  // increase counter variable if channel B is on same level as channel A
  if (encoderA == encoderB) {
    counter++;
  }
  // decrease counter variable if channel B is on different level than channel A
  else {
    counter--;
  }
  encoderB = !encoderB;
}


