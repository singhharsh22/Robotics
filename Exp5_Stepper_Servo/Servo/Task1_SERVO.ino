9
const int servo_pin= 9;
//change the servo pin here
void setup()
{
  pinMode(servo_pin, OUTPUT);
}

void loop()
{
  // total cycle 20 ms (50 hz)
  // pwm with 0.5 ms dutycycle represent zero degree
  // creating custom pwm signal with 0.5 ms dutycycle
  
    
  // digitalWrite(servo_pin, HIGH);
  // delayMicroseconds(500);   //tup for .5 ms
  // digitalWrite(servo_pin, LOW);
  // delayMicroseconds(19500); //tdown for 19.5 ms  
  
   //total 20ms PWM period
   
  // pwm signal with 2.3 ms dutycycle represent one-eighty degree 
  
  digitalWrite(servo_pin, HIGH);
  delayMicroseconds(2300);   //tup for 2.3 ms
  digitalWrite(servo_pin, LOW);
  delayMicroseconds(17700); //tdown for 17.7 ms  
  
  //total 20ms PWM period
  
  //add delay of 200ms for 1(c)

  
}
