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
 *  - add functionality to this code such that angular position can be re-set to zero  
 *  
 */

#include <Encoder.h>  // Downloaded from https://www.pjrc.com/teensy/td_libs_Encoder.html 

/************************************* VARIABLE DECLARATIONS *************************************/ 
const int pinA = 2; const int pinB = 13;        // ChannelA = pin2(ISR) & ChannelB = pin13
const int N = 3195;                             // # of turns per one revolution, experimentally determined

Encoder encoder(pinA,pinB);                     // initializing the encoder library 

long countPrev = -999;                          // used to compare current vs. old position
static double angularPos = 0;                   // angular position radians

/********************************************* SETUP *********************************************/ 
void setup() {
  Serial.begin(9600); 
}


/********************************************* LOOP *********************************************/ 
void loop() {
  
  long countNew = encoder.read(); 
  
  if (countNew != countPrev) {
    angularPos = 2.0*PI*(double)countNew/(double)N;
    Serial.print("Position: "); Serial.print(countNew);
    Serial.print(", Angular Position: "); Serial.println(angularPos);   
    countPrev = countNew; 
  }

}
