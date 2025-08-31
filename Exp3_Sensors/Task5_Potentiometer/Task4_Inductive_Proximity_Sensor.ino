// ME381 ROBOTICS: Experiment 3
// Task 4: Inductive Proximity Sensor
// Rotate the wheel on the setup in single direction only and observe the Serial Monitor

// Define the pin where the sensor output is connected
const int sensorPin = 2;  // Digital pin 2 (example pin)

// Variables for counting pulses
volatile int pulseCount = 0;

void setup() {
  Serial.begin(9600);  // Initialize serial communication at 9600 baud 
  pinMode(sensorPin, INPUT_PULLUP);  // Set sensor pin as input with internal pull-up resistor 
  attachInterrupt(digitalPinToInterrupt(sensorPin), countPulse, FALLING);  // Attach interrupt to sensor pin
}

void loop() {
  // Display the pulse count in the Serial Monitor
  Serial.print("Pulse count: ");
  Serial.println(pulseCount); 
  delay(1000);  // Update every 1 second (adjust as needed)
}

// Interrupt service routine to count pulses
void countPulse() {
  pulseCount++;
}
