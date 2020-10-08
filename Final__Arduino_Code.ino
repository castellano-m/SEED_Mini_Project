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
const float batteryVoltage = 8;               // [V] - voltage available from battery
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
static double angularPos_now = 0;


double error        = 0;
float errorInteg    = 0;                         // [rad*s] - integral of error
float kProp         = 1.25;         // [V/rad] - proportional controller gain
float kInteg        = 0.53;                      // [V/rad*s] - integral controller gain
float kDeriv        = 0.3055;       // [V/rad/s] - derivative controller gain
float errorRange    = 0.2;

Encoder encoder(pinA,pinB);                     // initializing the encoder library 

/************************************* VARIABLE TO MANIPULATE *************************************/ 
const int sampling = 50;   //[ms]
bool dir = 1;                                   // controls the direction of rotation

float motorVoltage  = 0.0;                      // [V] - analog voltage signal to motor
char desireAngChar;                             // read-in from Raspberry Pi
double desireAngPos;                            // [rad] - desired angular position

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
  timeNow = millis();   // monitor the length of time passed in loop
  
  if (Serial.available() > 0) {     // If the Pi is sending data
    desireAngChar = Serial.read();  // Read in the quadrant as a char
    Serial.println(angularPos_now); // Send the angular position to the Pi
      }
  
  switch (desireAngChar) {          // Determine desired angle based on quadrant
    case '1': 
      desireAngPos = 0;
      break;
    case '2': 
      desireAngPos = PI/2;
      break;
    case '3': 
      desireAngPos = PI;
      break;
    case '4': 
      desireAngPos = 3*PI/2;
      break;
    default:
      desireAngPos = 0;
  }

  
  /* 4.4 reads in */
  long countNew = encoder.read();                         // read in the cumulative count of the encoder 
  double angularVel; 
  
    if (countNew != countPrev) {                                       // if there is a change to the count 
      angularPos_now = 2.0*PI*(double)countNew/(double)N;       // calculate angular position 
      if (abs(angularPos_now) > fullrotation) {             // if full rotation,
        encoder.write(0);                                   // reset encoder count to zero
        countNew = 0;                                       // reset position to zero
        angularPos_now = 0;                                 // reset angular position to zero
        angularVel = angularVel_prev;
      } else angularVel = (((double)angularPos_now - (double)angularPos_prev)/((double)sampling/(double)1000));
    }
    
     
     //Serial.print("Before Control:\t");  Serial.println(desireAngChar);
    
      /* 4.7 start */
      // manipulate error to find PWM input to motor
      double signedError = (double)desireAngPos - (double)angularPos_now;

      error = (double)desireAngPos - (double)angularPos_now;            // find difference between current position and desired position

      if(error > fullrotation)    error = error - 2*PI;
      //Serial.println(error);
      errorInteg += error*((double)sampling/(double)1000);                          // calculate total error integral
      motorVoltage = ((float)error*(kProp) + (kDeriv*(float)angularVel) + (kInteg*errorInteg));     // calculate voltage supplied to motor
      if(abs(error) < errorRange) {
        errorInteg = 0;
      }
      //if(motorVoltage > batteryVoltage)     errorInteg = 0;
      
      if(motorVoltage < 0){
        digitalWrite(SIGN1, 0);
      } else {
        digitalWrite(SIGN1, 1);
      }
      
      if(abs(motorVoltage) > (float)8)    motorVoltage = (float)8;
      //Serial.println(motorVoltage);
      float dutyCycle = ((abs((float)motorVoltage)/(float)batteryVoltage)) * (float)255;   // convert motor voltage to PWM
      
      //digitalWrite(SIGN1, dir);                                                     // set motor direction
      analogWrite(PWM1, (int)dutyCycle);                                            // drive motor with rounded duty cycle value
      /* 4.7 end */    
  
    //Serial.print("After Control:\t");  Serial.println(desireAngChar);
    
    countPrev = countNew;                                 // 
    angularPos_prev = angularPos_now;                     // set current variables to their 
    angularVel_prev = angularVel;                         //
 
  /* 4.4 end */

  while(millis() < (timeNow + sampling)) ;                // wait for sampling period to pass
  
}
