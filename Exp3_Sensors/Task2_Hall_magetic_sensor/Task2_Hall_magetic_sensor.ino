// ME381 ROBOTICS: Experiment 3
// Task 2: Hall magnetic sensor
// Move a magnet across the hall sensor in swiping fashion. Observe the LED on the sensor module and the output in Serial Monitor

// Define the input pin for the Hall effect sensor
const int hallPin = 2;  // Replace with the actual pin number you're using

void setup() {
  // Initialize serial communication at 9600 baud
  Serial.begin(9600);
  // Set the hallPin as an input
  pinMode(hallPin, INPUT);
}

void loop() {
  // Read the digital value from the Hall effect sensor
  int sensorValue = digitalRead(hallPin);
  
  // Print the sensor value to the serial monitor
  Serial.print("Sensor value read at digital input pin: ");
  Serial.print(sensorValue);

  if(sensorValue==LOW)
    Serial.println("\t---->Magnet detected");
  else
    Serial.println("\t---->Magnet not detected");

  // Delay for a short period
  delay(1000);  // Adjust delay as needed
}
