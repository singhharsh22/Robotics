// ME381 ROBOTICS: Experiment 3
// Task 3: Infrared Transmitter Receiver
// Connect as follows:
        // Module GND to Arduino GND
        // Module VCC to Arduino 5V
        // Module OUT to Arduino pin 3

int buttonpin=3; 
int Led=13;
int val;

void setup()
{

pinMode(Led,OUTPUT); 
pinMode(buttonpin,INPUT); 
}

void loop()
{
  val=digitalRead(buttonpin);
  if(val==HIGH)
    {
    digitalWrite(Led,HIGH);
    }
  else
    {
    digitalWrite(Led,LOW);
    }
}
