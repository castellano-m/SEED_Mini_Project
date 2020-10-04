
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
const float batteryVoltage = 12.0;              // [V] - voltage available from battery
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

float errorInteg   = 0;                         // [rad*s] - integral of error
float kProp         = 1.26059529783407;         // [V/rad] - proportional controller gain
float kInteg        = 0.539867799412839;        // [V/rad*s] - integral controller gain
float kDeriv        = 0.0473385613441023;       // [V/rad/s] - derivative controller gain

Encoder encoder(pinA,pinB);                     // initializing the encoder library 

/************************************* VARIABLE TO MANIPULATE *************************************/ 
const int sampling = 50;   //[ms]
bool dir = 1;                                   // controls the direction of rotation

float motorVoltage  = 0.0;                      // [V] - analog voltage signal to motor
double desireAngPos = 3.14;                     // [rad] - desired angular position

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
      angularVel = ((double)angularPos_now - (double)angularPos_prev/((double)sampling/(double)1000));
      /* 4.7 start */
      // manipulate error to find PWM input to motor
      if (PI > ((double)desireAngPos - (double)angularPos_now)){    // if the difference between desired position and current position is less than pi rads
        dir = 1;                                                    // then drive motor counterclockwise
      } else {
        dir = 0;                                                    // else drive motor clockwise
      }
      double error = abs((double)desireAngPos - (double)angularPos_now);            // find difference between current position and desired position
      errorInteg += error*((double)sampling/(double)1000);                          // calculate total error integral
      motorVoltage = (kProp*error) + (kDeriv*angularVel) + (kInteg*errorInteg);     // calculate voltage supplied to motor
      float dutyCycle = ((float)motorVoltage/(float)batteryVoltage) * (float)255;   // convert motor voltage to PWM
      digitalWrite(SIGN1, dir);                                                     // set motor direction
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
