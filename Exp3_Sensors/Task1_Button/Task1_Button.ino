// ME381 ROBOTICS: Experiment 3
// Task 1: Button
// Observe the LED when button is pressed

int Led=13;           //define LED pin
int buttonpin=3;      //define the to which button's output is connected
int val;              //variable to store value read from button

void  setup()
{
pinMode(Led,OUTPUT);      //define LED pin 13 as a output
pinMode(buttonpin,INPUT); //define button pin as input
}

void  loop()
{ 
  val=digitalRead(buttonpin);   //read the value at pin 3
  if(val==HIGH)            
  {
    digitalWrite(Led,HIGH);
  }
  else
  {
    digitalWrite(Led,LOW);
  }
}
