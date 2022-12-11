//this code is going to calculate the angular velocity from the left motor encoders and display it on the LED
volatile bool encoderA = false; //these two booleans will determine whether the two encoder signals are HIGH or LOW. 
volatile bool encoderB = false;
volatile long counter = 0; //will track the relative position of encoder A
volatile float radius = 3.5; //radius of wheel in cm
volatile float deltaT; //time elapsed between every half cycle
volatile float startTime = 0; //time in which cycle begins
volatile float omega; //angular velocity in rad/s
volatile float velocity; //cm/s
volatile float distance = 0; //m

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  
  attachInterrupt(0, encoderChangeA, CHANGE); //this interrupt is triggered when encoder A changes to HIGH or LOW.
  attachInterrupt(1, encoderChangeB, CHANGE); //This interrupt is triggered when encoder B changes to HIGH or LOW. 
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(counter); //prints the encoder position repeatedly
  Serial.print("     ");
  Serial.print(velocity);
  Serial.print("     ");
  Serial.println(distance);
  delay(10);
  if (abs(micros() - startTime) > 250000) {
    velocity = 0;
  }
}

void encoderChangeA () {
  deltaT = micros() - startTime;
  startTime = micros();
  omega = 3.14159/120/(deltaT/1000000);
  velocity = omega*radius;
  
  if (encoderA != encoderB) { //checks whether both encoders are HIGH or LOW.
    counter++; //if the encoder signals are not equal, then the motor is spinning in a certain direction. counter increases.
    
  }
  else { //if the encoder signals are equal, the motor is spinning in the opposite direction. counter decreases. 
    counter--;
    
  }
  encoderA = !encoderA; //changes the boolean to match the measured signal change
  distance = counter*3.14*(radius/100)/240; //calculates distance the car has travelled
}

void encoderChangeB () {
  if (encoderA == encoderB) {
    counter++;
  }
  else {
    counter--;
  }
  encoderB = !encoderB; //changes the boolean to match the measured signal change
  distance = counter*3.14*(radius/100)/240; //calculates distance the car has travelled
}
