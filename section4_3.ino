/*
 * section4_3 Objective: Spin motor using Arduino. 
 */

const int PWM1 = 9;
const int PWM2 = 10;
const int ENABLE = 4;
const int BUTTON = 12;
const int SIGN1 = 7;
const int SIGN2 = 8;

int dutyCycle1 = 127;
int dutyCycle2 = 40;
bool dir1 = 1;
bool dir2 = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(ENABLE, OUTPUT);
  pinMode(SIGN1, OUTPUT);
  pinMode(SIGN2, OUTPUT);
  pinMode(BUTTON, INPUT);

  digitalWrite(BUTTON, HIGH);   // internal pull-up resistor
  digitalWrite(ENABLE, HIGH);   // ENABLE pin normal high
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  analogWrite(PWM1,dutyCycle1);
  digitalWrite(SIGN1,dir1);

}
