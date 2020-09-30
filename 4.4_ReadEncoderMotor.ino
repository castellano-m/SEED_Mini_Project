/* Name: Madison Heeg
 * Date: 25 Sept 2020
 * 
 * Title: Mini Project - Section 4.4 Read Motor Encoder 
 * Setup - One Pololu Motor
 *       - Channel A (yellow) = pin 2 (ISR); Channel B (white) = pin 13
 *       - Vcc (blue) = 5 V on Arduino
 *   
 * Functionality
 *  - Use high perforamnce encoder library to read the encoder
 *  - Display angular position 
 *  - Be able to reset angular position to zero
 *  
 * Encoder.h library
 *  - Declaration               Encoder myEnc(pinISR, pin2) 
 *  - myEnc.read()              returns accumulated position (signed)     
 *  - myEnc.write(newPosition)  set the accumlated position to a new number
 *  
 * TO DO: 
 *  - TEST code such that angular position can be re-set to zero  
 *  
 */

#include <Encoder.h>  // Downloaded from https://www.pjrc.com/teensy/td_libs_Encoder.html 

/************************************* VARIABLE DECLARATIONS *************************************/ 
const int N = 3200;                             // # of turns per one revolution, motor data sheet
const double fullrotation = 6.22;               // approx angular position when full rotation, experimental

const int pinA = 2; const int pinB = 13;        // ChannelA = pin2(ISR) & ChannelB = pin13

Encoder encoder(pinA,pinB);                     // initializing the encoder library 

long countPrev = -999;                          // used to compare current vs. old position
static double angularPos = 0;                   // angular position radians

/********************************************* SETUP *********************************************/ 
void setup() {
  Serial.begin(115200); 
}

/********************************************* LOOP *********************************************/ 
void loop() {
  
  long countNew = encoder.read();                         // read in the cumulative count of the encoder
  
  if (countNew != countPrev) {                            // if there is a change to the count 
    angularPos = 2.0*PI*(double)countNew/(double)N;       // calculate angular position 
    if (abs(angularPos) > fullrotation) {
      encoder.write(0);                                   // reset encoder count to zero, if full rotation
      countNew = 0;                                       // reset position to zero
      angularPos = 0;                                     // reset angular position to zero
    }

    // print position and angular position
    Serial.print("Position: "); Serial.print(countNew);
    Serial.print(", Angular Position: "); Serial.println(angularPos);   
    
    countPrev = countNew;                                 // set current count to previous count 
  }

}
