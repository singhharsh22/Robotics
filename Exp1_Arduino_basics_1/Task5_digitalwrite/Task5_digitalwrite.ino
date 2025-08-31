// ME381 ROBOTICS: Experiment 1
// Task5: Digital Write 
// Observe the LED at pin 13. Try changing the delay value to 100, 500, 5000 etc.

const int pin = 13;  // Digital pin to be controlled

void setup() {
  pinMode(pin, OUTPUT);  // Set pin 13 as an output
}

void loop() {
  digitalWrite(pin, HIGH);  // Set pin 13 HIGH
  delay(1000);  // Wait for 1 second
  digitalWrite(pin, LOW);  // Set pin 13 LOW
  delay(1000);  // Wait for 1 second
}

