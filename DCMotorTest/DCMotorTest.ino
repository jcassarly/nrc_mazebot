/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *frontL = AFMS.getMotor(1);
Adafruit_DCMotor *frontR = AFMS.getMotor(4);
Adafruit_DCMotor *rearL  = AFMS.getMotor(2);
Adafruit_DCMotor *rearR  = AFMS.getMotor(3);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

void setup() {
  //Serial.begin(9600);           // set up Serial library at 9600 bps

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  frontL->setSpeed(150);
  frontR->setSpeed(150);
  rearL ->setSpeed(150);
  rearR ->setSpeed(150);
  
  frontL->run(FORWARD);
  frontR->run(FORWARD);
  rearL ->run(FORWARD);
  rearR ->run(FORWARD);
  // turn on motor
  frontL->run(RELEASE);
  frontR->run(RELEASE);
  rearL ->run(RELEASE);
  rearR ->run(RELEASE);
}

void loop() {
  uint8_t i;
  
  frontL->run(BACKWARD);
  frontR->run(FORWARD);
  rearL ->run(BACKWARD);
  rearR ->run(FORWARD);
  
  for (i=0; i<255; i++) {
    frontL->setSpeed(i);
    frontR->setSpeed(i);
    rearL ->setSpeed(i);
    rearR ->setSpeed(i);  
    delay(10);
  }
  for (i=255; i!=0; i--) {
    frontL->setSpeed(i);
    frontR->setSpeed(i);
    rearL ->setSpeed(i);
    rearR ->setSpeed(i);  
    delay(10);
  }
  

  frontL->run(FORWARD);
  frontR->run(BACKWARD);
  rearL ->run(FORWARD);
  rearR ->run(BACKWARD);
  for (i=0; i<255; i++) {
    frontL->setSpeed(i);
    frontR->setSpeed(i);
    rearL ->setSpeed(i);
    rearR ->setSpeed(i);    
    delay(10);
  }
  for (i=255; i!=0; i--) {
    frontL->setSpeed(i);
    frontR->setSpeed(i);
    rearL ->setSpeed(i);
    rearR ->setSpeed(i);    
    delay(10);
  }

  frontL->run(RELEASE);
  frontR->run(RELEASE);
  rearL ->run(RELEASE);
  rearR ->run(RELEASE);
  delay(1000);
}
