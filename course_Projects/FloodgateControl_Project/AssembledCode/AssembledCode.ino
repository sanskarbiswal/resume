/*
 Stepper Motor Controller
 language: Wiring/Arduino

 This program drives a unipolar or bipolar stepper motor.
 The motor is attached to digital pins 8 and 9 of the Arduino.

 The motor moves 100 steps in one direction, then 100 in the other.

 Created 11 Mar. 2007
 Modified 7 Apr. 2007
 by Tom Igoe

 */

// define the pins that the motor is attached to. You can use
// any digital I/O pins.

#include <Stepper.h>

#define motorSteps 200     // change this depending on the number of steps
                           // per revolution of your motor
#define motorPin1 8
#define motorPin2 9
#define motorPin3 10
#define motorPin4 11
#define ledPin 13

const int trigPin = 7;
const int echoPin = 6;

long duration;
int distance;
int d;

// initialize of the Stepper library:
Stepper myStepper(motorSteps, motorPin1,motorPin2,motorPin3, motorPin4); 

void setup() {
  // set the motor speed at 60 RPMS:
  myStepper.setSpeed(60);
  
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  
  // Initialize the Serial port:
  Serial.begin(9600);

  //Set initial distance as 0
  d=0;
  // set up the LED pin:
  pinMode(ledPin, OUTPUT);
  // blink the LED:
  myStepper.step(100);
  blink(3);
}

void loop() {
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance= duration*0.034/2;
  if(distance!=d){
    d=distance;
    Serial.println(d);
    if(d>20){
      Serial.println("Water Level are within limits");
      delay(1000);
    }else if(d<=20 && d>10){
      Serial.println("Water Levels on the rise. Precaution Necessary");
      blink(20);
      delay(1000);
    }else if(d<=10 && d>5){
      Serial.println("Level critical. Releasing water");
      blink(50);
      myStepper.step(-30);
      delay(1000);
    }else{
      Serial.println("Overflow Imminent. Opening All floodgates");
      myStepper.step(100);
      delay(1000);
      myStepper.step(-100);
      delay(1000);
    }
  }else{
    Serial.println("No change in water level");
    delay(1000);
  }
}

// Blink the reset LED:
void blink(int howManyTimes) {
  int i;
  for (i=0; i< howManyTimes; i++) {
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
  }
}
