
/* 4.3 */
const int PERIOD = 1000;    // 1000 ms
unsigned long timeNow = 0;

const int PWM1 = 9;
const int PWM2 = 10;
const int ENABLE = 4;
const int BUTTON = 12;
const int SIGN1 = 7;
const int SIGN2 = 8;

int dutyCycle = 127;
bool dir = 1;
/* 4.3 */

/* 4.4 */
#include <Encoder.h>  // Downloaded from https://www.pjrc.com/teensy/td_libs_Encoder.html 

/************************************* VARIABLE DECLARATIONS *************************************/ 
const int N = 3195;                             // # of turns per one revolution, experimental
const double fullrotation = 6.22;               // approx angular position when full rotation, experimental

const int pinA = 2; const int pinB = 13;        // ChannelA = pin2(ISR) & ChannelB = pin13

Encoder encoder(pinA,pinB);                     // initializing the encoder library 

long countPrev = -999;                          // used to compare current vs. old position
static double angularPos_prev = 0;              // angular position radians
double angPos_now;
static double angularVel_prev = 0; 
const int sampling = 15;   //[ms]
/* 4.4 */

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(PWM1, OUTPUT);

  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(ENABLE, OUTPUT);
  pinMode(SIGN1, OUTPUT);
  pinMode(SIGN2, OUTPUT);
  pinMode(BUTTON, INPUT);

  digitalWrite(BUTTON, HIGH);   // internal pull-up resistor
  digitalWrite(ENABLE, HIGH);   // ENABLE pin normal high
  
  // wait 1 second to apply step function
  timeNow = millis();
  while(millis() < (timeNow + PERIOD));
  analogWrite(PWM1, dutyCycle);
  digitalWrite(SIGN1, dir);
}


void loop() {
  /* 4.3 */
  analogWrite(PWM1,dutyCycle);
  digitalWrite(SIGN1,dir);
  /* 4.3 end */

  /* 4.4 */
  long countNew = encoder.read();                         // read in the cumulative count of the encoder 
  double angularVel; 
  
  if (countNew != countPrev) {                            // if there is a change to the count 
    double angularPos_now = 2.0*PI*(double)countNew/(double)N;       // calculate angular position 
    if (abs(angularPos_now) > fullrotation) {
      encoder.write(0);                                   // reset encoder count to zero, if full rotation
      countNew = 0;                                       // reset position to zero
      angularPos_now = 0;                                     // reset angular position to zero
      angularVel = angularVel_prev;
    } else angularVel = ((double)angularPos_now - (double)angularPos_prev)/((double)sampling/(double)1000);    

    // print position and angular position
    //Serial.print("Position: "); Serial.print(countNew);
    //Serial.print(", Angular Position: "); Serial.print(angularPos_now);
    //Serial.print(", Angular Velocity: "); Serial.println(angularVel);  
    if (millis() < 2800){
      Serial.print(millis()); Serial.print("\t"); Serial.println(angularVel);    
    }
    countPrev = countNew;                                 // set current count to previous count
    angularPos_prev = angularPos_now; 
    angularVel_prev = angularVel;  
  }
  /* 4.4 end */
  
  timeNow = millis();
  while(millis() < (timeNow + sampling)) ;
  
}
