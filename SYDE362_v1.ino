/* 
Must install the Adafruit software first:
https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/install-software
*/

// PID library: playground.arduino.cc/Code/PIDLibrary

const int pingPin = 7;

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *Motor1 = AFMS.getMotor(1);
Adafruit_DCMotor *Motor2 = AFMS.getMotor(2);

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  Motor1->setSpeed(250);
  Motor1->run(FORWARD);
  // turn on motor
  Motor1->run(RELEASE);

  Motor2->setSpeed(250);
  Motor2->run(BACKWARD);
  // turn on motor
  Motor2->run(RELEASE);
}


void loop() {

/* Ultrasonic sensor code:
// establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  long duration, inches, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);


  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);



  // convert the time into a distance
  inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);

 // Serial.print(inches);
 // Serial.print("in, ");
  Serial.print(cm);
 // Serial.print("cm");
  Serial.println();

  delay(100);
*/

/*
int speed = 150;

  if (cm > 50){
    Motor1->setSpeed(speed);
    Motor2->setSpeed(speed);
    Motor1->run(FORWARD);
    Motor2->run(FORWARD);
    // turn on motor
  //  Motor1->run(RELEASE);
  }
  else{
      Motor1->setSpeed(0);
      Motor2->setSpeed(0);
}
*/

int speed = 250;

    Motor1->setSpeed(speed);
    Motor2->setSpeed(speed);
    Motor1->run(FORWARD);
    Motor2->run(BACKWARD);


  delay(5000);
//    Motor1->setSpeed(0);
//    Motor2->setSpeed(0);

//  delay();

}



// Release kills the motors (kills power but allows them to run)
//    Motor1->run(RELEASE);
//    Motor2->run(RELEASE);






long microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}
