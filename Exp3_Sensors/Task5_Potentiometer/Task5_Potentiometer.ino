// ME381 ROBOTICS: Experiment 3
// Task 5: Potentiometer
// Rotate the knob of the potentiometer and observe the Serial Monitor

const int potentiometerPin = A0;  // Analog input pin for potentiometer

void setup() {
  Serial.begin(9600);  // Initialize serial communication for debugging
}

void loop() {
  // Read analog value from the potentiometer
  int sensorValue = analogRead(potentiometerPin);
  
  // Print the sensor value (for debugging)
  Serial.print("Sensor Value: ");
  Serial.println(sensorValue);
  
  // Example: Map the sensor value to a range (0-255)
  int mappedValue = map(sensorValue, 0, 1023, 0, 255);  // Map range from 0-1023 to 0-255
  
  // Print the mapped value
  Serial.print("Mapped Value: ");
  Serial.println(mappedValue);
  
  delay(2000);  // Adjust delay as needed
}
