// ME381 ROBOTICS: Experiment 1
// Task3: Digital Read 
// Connect pin 8 to GND. Disconnect and connect it again. Keep on repeating this. Observe the LED at pin 13.

const int ledPin = LED_BUILTIN; // Built-in LED pin (usually pin 13 on most boards)
const int inputPin = 8;         // Input pin (pin 8)

void setup() {
  pinMode(ledPin, OUTPUT);             // Set the LED pin as an output
  pinMode(inputPin, INPUT_PULLUP);    // the inputPin is set to HIGH when left unconnected
}

void loop() {
  int pinState = digitalRead(inputPin); // Read the state of the input pin
  
  if (pinState == LOW) {
    digitalWrite(ledPin, HIGH); // Turn on the LED
  } else {
    digitalWrite(ledPin, LOW);  // Turn off the LED
  }
  
}