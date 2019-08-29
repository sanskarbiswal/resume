#include <Ultrasonic.h>
Ultrasonic ultrasonic(6,7); // (Trig PIN,Echo PIN)

int range_inch = 0; // Variable to store range in Inch
int range_cm = 0;   // Variable to store range in Centi meter
int time_ms = 0;    // Varibale to store echo time in ms
void setup() {
  Serial.begin(9600); // Initializing serial communication.  
}

void loop()
{
  range_inch = ultrasonic.Ranging(INC); // function returns range in Inch.
  range_cm = ultrasonic.Ranging(CM);    // function returns range in cm.
  time_ms = ultrasonic.Timing();
  Serial.print(range_cm);              // Printing range in CM   
  Serial.println(" cm" ); 
  Guide(range_cm);
  delay(3000);                         // waiting for a while
}

void Guide(int r){
  if(r<10){
    Serial.println("There is no path forward. You might want to turn to your right");  
  }  
  else if(r>=10 && r<=30){
    Serial.println("Move forwards slowly. There is a small opening ahead");
  }
  else if(r>30 and r<49){
    Serial.println("Please move forward by two steps.");
  }
  else{
    Serial.println("Move forward by 3 steps please");
  }
}
