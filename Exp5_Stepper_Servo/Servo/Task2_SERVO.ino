#include <Servo.h>

Servo myservo;  // create Servo object to control a servo
// twelve Servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(9);  // attaches the servo on pin 4 to the Servo object
}

void loop() {
    pos=0;
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    //add delay here
    //use another rotaion value here  
}
