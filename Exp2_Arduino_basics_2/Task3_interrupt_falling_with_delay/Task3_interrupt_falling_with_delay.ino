// ME381 ROBOTICS: Experiment 2
// Task3: interrupt with falling edge with delay in the main loop
// Connect pin 2 to GND once and then disconnect it

const int ledPin = 13;       // Internal LED pin
const int interruptPin = 2;  // Interrupt pin
bool latch = 0;

// ISR
void lightUpLED() {
  latch = 1;
  digitalWrite(ledPin,HIGH);
}

void setup() {
  pinMode(ledPin, OUTPUT);  // Set the LED pin as output
  digitalWrite(ledPin, LOW);
  pinMode(interruptPin, INPUT_PULLUP);                                        // Set the interrupt pin as input with internal pull-up resistor
  attachInterrupt(digitalPinToInterrupt(interruptPin), lightUpLED, FALLING);  // Attach interrupt to pin 2
}

void loop() {
  // Main loop does nothing, everything is handled by the interrupt

  delay(10000);
}
