//www.kuongshun.com

#include <Servo.h>  //servo library
Servo myservo;      // create servo object to control servo

int Echo = 12;  
int Trig = 13; 

#define servoPIN 2
#define ENA 3
#define IN1 5
#define IN2 6
#define IN3 7
#define IN4 8
#define ENB 11
#define carSpeed 150
int rightDistance = 0, leftDistance = 0, middleDistance = 0;

void forward(){ 
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Forward");
}

void back(){ 
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("back");
}




void left() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH); 
  Serial.println("Left");
}

void right() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Right");
}

void stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  Serial.println("Stop!");
} 

//Ultrasonic distance measurement Sub function
int Distance_test() {
  digitalWrite(Trig, LOW);   
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  
  delayMicroseconds(20);
  digitalWrite(Trig, LOW);   
  float Fdistance = pulseIn(Echo, HIGH);  
  Fdistance= Fdistance / 58;       
  return (int)Fdistance;
}  

void setup() { 
  myservo.attach(servoPIN);  // attach servo on pin 3 to servo object
  Serial.begin(9600);     
  pinMode(Echo, INPUT);    
  pinMode(Trig, OUTPUT);  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  stop();
} 

void loop() {
    forward(); 
    myservo.write(90);  //setservo position according to scaled value
    delay(500); 
    middleDistance = Distance_test();

    if(middleDistance <= 25) {     
      stop();
      delay(500);                         
      myservo.write(10);          
      delay(500);      
      rightDistance = Distance_test();
      
      delay(500);
      myservo.write(90);              
      delay(500);                                                  
      myservo.write(170);              
      delay(500); 
      leftDistance = Distance_test();
      
      delay(500);
      myservo.write(90);              
      delay(500);
      if(rightDistance > leftDistance) {
        stop();
        delay(100);
        back();
        delay(200);
        right();
        delay(300);
      }
      else if(rightDistance < leftDistance) {
        stop();
        delay(100);
        back();
        delay(200);
        left();
        delay(300);
      }
      else if((rightDistance <= 25) || (leftDistance <= 25)) {
        back();
        delay(200);
      }
      else {
        forward();
      }
    }  
    else {
        forward();
    }                     
}
