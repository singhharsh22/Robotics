// ME381 ROBOTICS: Experiment 1
// Task6: Analog Write 
// Connect pin 9 to multimeter +ve and GND to multimeter -ve. Try changing the pwmValue and observer multimeter voltage.

const int pwmPin = 9;  // PWM output pin

void setup() {
  pinMode(pwmPin, OUTPUT);  // Set the PWM pin as an output
}

void loop() {
  int pwmValue=255;
  analogWrite(pwmPin, pwmValue);
  delay(10);
  
}
