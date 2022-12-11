//This code uses an H-bridge to regulate the speed and direction of two DC motors. 

const int input1 = 10;
const int input2 = 9;
const int EN1 = 11;
const int potentiometer1 = A0;

const int input3 = 8;
const int input4 = 7;
const int EN2 = 6;
const int potentiometer2 = A1;

int inputSignalOne;
int enableSignalOne;

int inputSignalTwo;
int enableSignalTwo;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(input1, OUTPUT); //inputs 1 and 2 control motor direction. 
  pinMode(input2, OUTPUT);
  pinMode(EN1, OUTPUT); //pwm signal that controls motor speed
  pinMode(potentiometer1, INPUT); //reads potentiometer input

}

void loop() {
  // put your main code here, to run repeatedly:
  inputSignalOne = analogRead(potentiometer1); //takes in potentiometer reading
//  Serial.print(inputSignal);
//  Serial.print("     ");
  //determine which way motor will spin.
  if (inputSignalOne > 511) { //spins motor one way
    digitalWrite(input1, HIGH);
    digitalWrite(input2, LOW);   
    enableSignalOne = (inputSignalOne - 512)/2;
  }
  else { //spins motor the other way
    digitalWrite(input1, LOW);
    digitalWrite(input2, HIGH); 
    enableSignalOne = (511 - inputSignalOne)/2;
  }
  //Serial.println(enableSignalOne);
  analogWrite(EN1, enableSignalOne);

  //now for the other motor
  inputSignalTwo = analogRead(potentiometer2); //takes in potentiometer reading
  //Serial.print(inputSignalTwo);
  //Serial.print("     ");
  //determine which way motor will spin.
  if (inputSignalTwo > 511) { //spins motor one way
    digitalWrite(input3, HIGH);
    digitalWrite(input4, LOW);   
    enableSignalTwo = (inputSignalTwo - 512)/2;
  }
  else { //spins motor the other way
    digitalWrite(input3, LOW);
    digitalWrite(input4, HIGH); 
    enableSignalTwo = (511 - inputSignalTwo)/2;
  }
  Serial.println(enableSignalTwo);
  analogWrite(EN2, enableSignalTwo);
  
  
}
