/* Name: Madison Heeg
 * Date: 25 Sept 2020
 * 
 * Title: Mini Project - Section 4.4,
 * Setup - One Pololu Motor
 *       - Channel A = pin 2 (ISR); Channel B = pin 13
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
 */

#include <Encoder.h>  // Downloaded from https://www.pjrc.com/teensy/td_libs_Encoder.html 

/************************************* VARIABLE DECLARATIONS *************************************/ 
const int pinA = 2; const int pinB = 13;        // ChannelA = pin2(ISR) & ChannelB = pin13
// const int N = # of turns per one revolution

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
    // angularPos = 2*PI*(double)countNew/N
    Serial.print("Position: "); Serial.print(countNew);
    Serial.print(", Angular Position: "); Serial.println(angularPos);   
    countPrev = countNew; 
  }

}
