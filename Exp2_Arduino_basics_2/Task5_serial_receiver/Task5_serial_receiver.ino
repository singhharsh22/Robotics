// ME381 ROBOTICS: Experiment 2
// Task5: UART serial receiver
// Open the serial monitor and enter a value between 0 to 10. Observe the LED.

const int ledPin = 13;  // LED pin

void setup() {
  pinMode(ledPin, OUTPUT);  // Set the LED pin as an output
  Serial.begin(9600);       // Initialize serial communication at 9600 bps
}

void loop() {
  if (Serial.available() > 0) {   // Check if data is available to read
    int num = Serial.parseInt();  // Read the number
    blinkLED(num);                // Blink the LED the number of times received
  }
}

void blinkLED(int num) {
  int interval = 5000 / num;  // Calculate the interval for blinking within 5 seconds
  for (int i = 0; i < num; i++) {
    digitalWrite(ledPin, HIGH);  // Turn the LED on
    delay(interval / 2);         // Wait for half of the interval
    digitalWrite(ledPin, LOW);   // Turn the LED off
    delay(interval / 2);         // Wait for the other half of the interval
  }
}
