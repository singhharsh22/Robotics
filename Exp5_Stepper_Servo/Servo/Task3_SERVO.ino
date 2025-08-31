
#include <Servo.h>

Servo myservo;  // create Servo object to control a servo
// twelve Servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
int potPin=A0;  // pin attached to potentiometer

void setup() {
  myservo.attach(9);  // attaches the servo on pin 4 to the Servo object
  pinMode(potPin,INPUT); // declear pin as INPUT
}

void loop() {
    int val=analogRead(potPin);       // read the analog pin
    int angle= map(val,0,1023,0,180); // map the value of analog input to angle

    myservo.write(angle);
    delay(15); // waits 15ms for the servo to reach the position
    
    
  
}
