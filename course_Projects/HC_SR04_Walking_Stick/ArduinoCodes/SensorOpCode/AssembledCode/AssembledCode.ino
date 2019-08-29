#include <TTS.h>

#define ledPin 13

const int trigPin = 7;
const int echoPin = 6;
TTS text2speech;  // speech output is digital pin 10

long duration;
int distance;
//TTS Variables
char text [50];
boolean state=0;

//IPS variables
int range_inch = 0; // Variable to store range in Inch
int range_cm = 0;   // Variable to store range in Centi meter
int time_ms = 0;    // Varibale to store echo time in ms


void setup() {
  
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication 
}


void loop()
{ // Clears the trigPin
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
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);
  Text_Speech(distance);                // calls function to send output
  delay(3000);                         // waiting for a while
}

void Text_Speech(int r) {
 text2speech.setPitch(6); //higher values = lower voice pitch
 if(r<20){
    strcpy(text,"Turn Right");
    text2speech.say(text);
  }
 else if(r>=20 && r <=100){
    strcpy(text,"One step forward");
    text2speech.say(text);
  }
  else if(r>100 && r<=200 ){
    strcpy(text, "Three steps forward");
    text2speech.say(text);
    }
  else{
    strcpy(text, "Five steps forward");
    text2speech.say(text);
    }
  delay(500);
}
