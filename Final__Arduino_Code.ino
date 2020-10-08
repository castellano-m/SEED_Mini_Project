/* Name: Madison Heeg and Andrew Rouze
 * Date: 30 Sept 2020
 * 
 * Title: Mini Project - Final Arduino Code
 * Setup - 
 *  - Encoder:  Channel A (yellow) = pin 2 (ISR)
 *              Channel B (white) = pin 13
 *              Vcc (blue) = 5 V on Arduino
 *  - Motor1:   PMW1 =  pin 9; SIGN1 = pin 7
 *  - Motor2:   PMW2 = pin 10; SIGN2 = pin 8
 *   
 * Functionality
 *  - Read data sent from Raspberry Pi 
 *  - Depending on the data received, set a desired anglular position
 *  - Implement PID controller to set required voltage to go to position 
 *  - Send current angular position back to the Raspberry Pi 
 */

#include <Encoder.h>  // Downloaded from https://www.pjrc.com/teensy/td_libs_Encoder.html 

/************************************* VARIABLE DECLARATIONS *************************************/ 
const float batteryVoltage = 8;               // [V] - voltage available from battery
const int N = 3195;                             // # of turns per one revolution, experimental
const double fullrotation = 6.22;               // approx angular position when full rotation, experimental

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
float kProp         = 1.25;                      // [V/rad] - proportional controller gain
float kInteg        = 0.53;                      // [V/rad*s] - integral controller gain
float kDeriv        = 0.3055;                    // [V/rad/s] - derivative controller gain
float errorRange    = 0.2;                       // [rads] - acceptable error between desireAngChar and angularPos_now

Encoder encoder(pinA,pinB);                     // initializing the encoder library 

/************************************* VARIABLE TO MANIPULATE *************************************/ 
const int sampling = 50;                         //[ms] - sampling time
bool dir = 1;                                   // boolean, controls the direction of rotation

float motorVoltage  = 0.0;                      // [V] - analog voltage signal to motor
char desireAngChar;                             // read in from Raspberry Pi
double desireAngPos;                            // [rad] - desired angular position

/********************************************* SETUP *********************************************/ 
void setup() {
 
  Serial.begin(115200);
  pinMode(PWM1, OUTPUT); pinMode(PWM2, OUTPUT);       // setting PWMs as outputs
  pinMode(SIGN1, OUTPUT); pinMode(SIGN2, OUTPUT);     // setting signs for direction as outputs

  digitalWrite(BUTTON, HIGH);                         // internal pull-up resistor
  digitalWrite(ENABLE, HIGH);                         // ENABLE pin normal high

}

/********************************************* LOOP *********************************************/ 
void loop() {
  timeNow = millis();   // monitor the length of time passed in loop
  
  if (Serial.available() > 0) {     // If the Pi is sending data
    desireAngChar = Serial.read();  // Read in the quadrant as a char from the Pi
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

  
  long countNew = encoder.read();                           // read in the cumulative count of the encoder 
  double angularVel; 
  
    if (countNew != countPrev) {                            // if there is a change to the count 
      angularPos_now = 2.0*PI*(double)countNew/(double)N;         // calculate angular position 
      if (abs(angularPos_now) > fullrotation) {                   // if full rotation,
        encoder.write(0);                                             // reset encoder count to zero
        countNew = 0;                                                 // reset position to zero
        angularPos_now = 0;                                           // reset angular position to zero
        angularVel = angularVel_prev;
      } else angularVel = (((double)angularPos_now - (double)angularPos_prev)/((double)sampling/(double)1000));
    }
    
      // manipulate error to find PWM input to motor
      double signedError = (double)desireAngPos - (double)angularPos_now;

      error = (double)desireAngPos - (double)angularPos_now;            // find difference between current position and desired position

      if(error > fullrotation) error = error - 2*PI;                    // reset error so always less than one revolution, i.e. 0 < error < 2*PI
      errorInteg += error*((double)sampling/(double)1000;               // calculate total error integral, accumultion of all previous errors
      
      motorVoltage = ((float)error*(kProp) + (kDeriv*(float)angularVel) + (kInteg*errorInteg));     // MOTOR CONTROL - calculate voltage supplied to motor
      
      if(abs(error) < errorRange) errorInteg = 0;                       // reset errorIntegral once angular position reaches desired position
      
      if(motorVoltage < 0){                                             // set spin direction of motor by sign of calculated motor voltage
        digitalWrite(SIGN1, 0);
      } else {
        digitalWrite(SIGN1, 1);
      }
      
      if(abs(motorVoltage) > (float)batteryVoltage) motorVoltage = (float)batteryVoltage;   // if calculated motor voltage above available voltage from battery, cap it at motorVoltage
      float dutyCycle = ((abs((float)motorVoltage)/(float)batteryVoltage)) * (float)255;    // convert motor voltage to PWM
      analogWrite(PWM1, (int)dutyCycle);                                                    // drive motor with rounded duty cycle value

    countPrev = countNew;                                 // 
    angularPos_prev = angularPos_now;                     // set current variables to their 
    angularVel_prev = angularVel;                         //

  while(millis() < (timeNow + sampling)) ;                // wait for sampling period to pass
  
}
