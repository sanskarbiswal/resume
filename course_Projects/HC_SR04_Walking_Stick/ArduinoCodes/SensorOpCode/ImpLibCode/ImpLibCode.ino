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
  time_ms = ultrasonic.Timing();        // function returns echo time in ms
  Serial.print(range_inch);
  Serial.println(" Inch" );            // Printing range in Inch
  Serial.print(range_cm);              // Printing range in CM   
  Serial.println(" cm" );             
  Serial.print(time_ms);              // Printing echo time in ms  
  Serial.println(" ms" );
  delay(3000);                         // waiting for a while
}
