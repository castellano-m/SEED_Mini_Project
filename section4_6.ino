const int PERIOD = 1000;    // 1000 ms
const int PWM1 = 9;
unsigned long timeNow = 0;

int dutyCycle = 127;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(PWM1, OUTPUT);
  
  // wait 1 second to apply step function
  timeNow = millis();

  Serial.println("in main");

  while(millis() < (timeNow + PERIOD)){
    
  }
  Serial.println("STEP");

  analogWrite(PWM1, dutyCycle);

}


void loop() {
  // put your main code here, to run repeatedly:
  
  
}
