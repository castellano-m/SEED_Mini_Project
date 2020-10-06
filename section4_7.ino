
/* Name: Madison Heeg and Andrew Rouze
 * Date: 30 Sept 2020
 * 
 * Title: Mini Project - Section 4.7 Arduino Motor Controller
 * Setup - 
 *  - Encoder:  Channel A (yellow) = pin 2 (ISR)
 *              Channel B (white) = pin 13
 *              Vcc (blue) = 5 V on Arduino
 *  - Motor1:   PMW1 =  pin 9; SIGN1 = pin 7
 *  - Motor2:   PMW2 = pin 10; SIGN2 = pin 8
 *   
 * Functionality
 *  - Builds off of Sketch 4.4 and 4.6
 *  - Spin wheel to desired position
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
const float batteryVoltage = 8;                 // [V] - voltage available from battery
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

double error        = 0;
float errorInteg    = 0;                        // [rad*s] - integral of error
float kProp         = 1.25; /*1.25*/                    // [V/rad] - proportional controller gain
float kInteg        = 0.8; /*0.53*/                     // [V/rad*s] - integral controller gain
float kDeriv        = 0.25; /*0.3055*/                   // [V/rad/s] - derivative controller gain
float errorRange    = 0.01;

Encoder encoder(pinA,pinB);                     // initializing the encoder library 

/************************************* VARIABLE TO MANIPULATE *************************************/ 
const int sampling = 50;                        //[ms]
bool dir = 1;                                   // controls the direction of rotation

float motorVoltage  = 0.0;                      // [V] - analog voltage signal to motor
char  desireAngChar = '3';                        // designates which position the PI is telling the motor to go to
double desireAngPos = 0;                        // [rad] - goal angular position
//double desireAngPos = PI;

/********************************************* SETUP *********************************************/ 
void setup() {
 
  Serial.begin(115200);
  pinMode(PWM1, OUTPUT); pinMode(PWM2, OUTPUT);       // setting PWMs as outputs
  pinMode(SIGN1, OUTPUT); pinMode(SIGN2, OUTPUT);     // setting signs for direction as outputs
  
  pinMode(ENABLE, OUTPUT);
  pinMode(BUTTON, INPUT);

  digitalWrite(BUTTON, HIGH);                         // internal pull-up resistor
  digitalWrite(ENABLE, HIGH);                         // ENABLE pin normal high

}

/********************************************* LOOP *********************************************/ 
void loop() {
  timeNow = millis();                                 // monitor the length of time passed in loop

  if(Serial.available() > 0){
    desireAngChar = Serial.read();
  }

  if(desireAngChar == '1')      desireAngPos = 0;
  else if(desireAngChar == '2') desireAngPos = PI/2;
  else if(desireAngChar == '3') desireAngPos = PI;
  else if(desireAngChar == '4') desireAngPos = 3*PI/2;
  
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
    } else {
      angularVel = (((double)angularPos_now - (double)angularPos_prev)/((double)sampling/(double)1000));
      /* 4.7 start */
      // manipulate error to find PWM input to motor
      double signedError = (double)desireAngPos - (double)angularPos_now;
      /*if (signedError < 0){
        signedError = 2*PI - signedError;
      }
      if (signedError < 0){                                         // if the difference between desired position and current position is negative
        dir = 0;                                                    // then drive motor counterclockwise
      } else {
        dir = 1;                                                    // else drive motor clockwise
      }*/
      
      /*if (angularPos_now < 0){
        error = (double)desireAngPos - (2*PI - (double)angularPos_now);            // find difference between current position and desired position
      } else {
        error = (double)desireAngPos - (double)angularPos_now;            // find difference between current position and desired position
      }*/

      error = (double)desireAngPos - (double)angularPos_now;            // find difference between current position and desired position

      if(error > fullrotation)    error = error - 2*PI;
      //Serial.println(error);
      errorInteg += error*((double)sampling/(double)1000);                          // calculate total error integral
      motorVoltage = ((float)error*(kProp) + (kDeriv*(float)angularVel) + (kInteg*errorInteg));     // calculate voltage supplied to motor
      if(abs(error) < errorRange)     errorInteg = 0;
      
      //if(motorVoltage > batteryVoltage)     errorInteg = 0;
      
      if(motorVoltage < 0){
        digitalWrite(SIGN1, 0);
      } else {
        digitalWrite(SIGN1, 1);
      }
      //Serial.print(error); Serial.print("\t"); Serial.print(motorVoltage); Serial.print("\t"); Serial.println(angularPos_now);
      if(abs(motorVoltage) > (float)8)    motorVoltage = (float)8;
      //Serial.println(motorVoltage);
      float dutyCycle = ((abs((float)motorVoltage)/(float)batteryVoltage)) * (float)255;   // convert motor voltage to PWM
      
      //digitalWrite(SIGN1, dir);                                                     // set motor direction
      analogWrite(PWM1, (int)dutyCycle);                                            // drive motor with rounded duty cycle value
      /* 4.7 end */    
    }
    
    countPrev = countNew;                                 // 
    angularPos_prev = angularPos_now;                     // set current variables to their 
    angularVel_prev = angularVel;                         //
  }
  /* 4.4 end */

  while(millis() < (timeNow + sampling)) ;                // wait for sampling period to pass
  
}
