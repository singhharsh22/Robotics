// ME381 ROBOTICS: Experiment 2
// Task1: LED blink without delay
// No jumper wires to be connected to board. Try changing the interval between blinks and duration for which LED stays on

const int ledPin = LED_BUILTIN; // Built-in LED pin (usually pin 13 on most boards)
const unsigned long interval = 1000; // Interval between blinks (1000 ms = 1 second)
const unsigned long duration = 100;  // Duration the LED stays on (100 ms)

unsigned long previousMillis = 0;  // Stores the last time the LED was updated
bool ledState = LOW;               // Current state of the LED

void setup() {
  pinMode(ledPin, OUTPUT); // Set the LED pin as an output
}

void loop() {
  unsigned long currentMillis = millis(); // Get the current time

  if (ledState == LOW) {
    if (currentMillis - previousMillis >= interval) {
      digitalWrite(ledPin, HIGH);     // Turn on the LED
      ledState = HIGH;                // Update the LED state
      previousMillis = currentMillis; // Save the current time
    }
  } else {
    if (currentMillis - previousMillis >= duration) {
      digitalWrite(ledPin, LOW);      // Turn off the LED
      ledState = LOW;                 // Update the LED state
    }
  }
}