
/* Name: Madison Heeg and Andrew Rouze
 * Date: 30 Sept 2020
 * 
 * Title: Mini Project - Section 4.6 Simulation of Motor - Step Response Experiment
 * Setup - 
 *  - Encoder:  Channel A (yellow) = pin 2 (ISR)
 *              Channel B (white) = pin 13
 *              Vcc (blue) = 5 V on Arduino
 *  - Motor1:   PMW1 =  pin 9; SIGN1 = pin 7
 *  - Motor2:   PMW2 = pin 10; SIGN2 = pin 8
 *   
 * Functionality
 *  - Builds off of Sketch 4.4 and 4.3
 *  - Program a step response sketch to dtermine angular velocity
 *  
 *  - Display for each interval -> WATCH UNITS
 *        - current time 
 *        - motor votlage command 
 *        - angular velocity
 *  - Fixed cycle time (sampling time) - millis() - between 5-10ms
 *    https://www.norwegiancreations.com/2017/09/arduino-tutorial-using-millis-instead-of-delay/
 */

#include <Encoder.h>  // Downloaded from https://www.pjrc.com/teensy/td_libs_Encoder.html 

/************************************* VARIABLE DECLARATIONS *************************************/ 
const int PERIOD = 1000;                        // [ms], time to wait for step response (1 sec)
const int N = 3195;                             // # of turns per one revolution, experimental
const double fullrotation = 6.22;               // approx angular position when full rotation, experimental

const int ENABLE = 4; const int BUTTON = 12;    // Buttons pins? 
const int PWM1 = 9; const int PWM2 = 10;        // pins for PMW for each motor 
const int SIGN1 = 7; const int SIGN2 = 8;       // pins direction of rotation for each motor
const int pinA = 2; const int pinB = 13;        // ChannelA = pin2(ISR) & ChannelB = pin13

unsigned long timeNow = 0;                      // tracks time when program starts so can implement step after 1 sec

long countPrev = -999;                          // used to compare current vs. old encoder position / counts
static double angularPos_prev = 0;              // [rad] - used to compare previous angular position 
static double angularVel_prev = 0;              // [rad/s] - used to compare previous angular velocity
double angPos_now;                              // [rad] - used to calculate the current angular position

Encoder encoder(pinA,pinB);                     // initializing the encoder library 

/************************************* VARIABLE TO MANIPULATE *************************************/ 
const int sampling = 15;   //[ms]
int dutyCycle = 127;                            // controls the speed of the motor 0 - 255 
bool dir = 1;                                   // controls the direction of rotation

/********************************************* SETUP *********************************************/ 
void setup() {
 
  Serial.begin(115200);
  pinMode(PWM1, OUTPUT); pinMode(PWM2, OUTPUT);       // setting PMWs as outputs
  pinMode(SIGN1, OUTPUT); pinMode(SIGN2, OUTPUT);     // setting signs for direction as outputs
  
  pinMode(ENABLE, OUTPUT);
  pinMode(BUTTON, INPUT);

  digitalWrite(BUTTON, HIGH);                         // internal pull-up resistor
  digitalWrite(ENABLE, HIGH);                         // ENABLE pin normal high
  
  /*********** Controls Arduino to wait 1 second to apply step function ***********/
      timeNow = millis();
      while(millis() < (timeNow + PERIOD));
      analogWrite(PWM1, dutyCycle);
      digitalWrite(SIGN1, dir);
}

/********************************************* LOOP *********************************************/ 
void loop() {
  /* 4.3 controls the motor to move at desired speed and direction */
  analogWrite(PWM1,dutyCycle);
  digitalWrite(SIGN1,dir);
  /* 4.3 end */

  /* 4.4 reads in */
  long countNew = encoder.read();                         // read in the cumulative count of the encoder 
  double angularVel; 
  
  if (countNew != countPrev) {                                       // if there is a change to the count 
    double angularPos_now = 2.0*PI*(double)countNew/(double)N;       // calculate angular position 
    if (abs(angularPos_now) > fullrotation) {             // if full rotation,
      encoder.write(0);                                   // reset encoder count to zero
      countNew = 0;                                       // reset position to zero
      angularPos_now = 0;                                 // reset angular position to zero
      angularVel = angularVel_prev;
    } else angularVel = ((double)angularPos_now - (double)angularPos_prev)/((double)sampling/(double)1000);    

    //print position and angular position
    //Serial.print("Position: "); Serial.print(countNew);
    //Serial.print(", Angular Position: "); Serial.print(angularPos_now);
    //Serial.print(", Angular Velocity: "); Serial.println(angularVel);  
    
    if (millis() < 2800){               
      Serial.print(millis()); Serial.print("\t"); Serial.println(angularVel);    
    }
    countPrev = countNew;                                 // 
    angularPos_prev = angularPos_now;                     // set current variables to their 
    angularVel_prev = angularVel;                         //
  }
  /* 4.4 end */
  
  timeNow = millis();
  while(millis() < (timeNow + sampling)) ;
  
}
