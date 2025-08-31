// ME381 ROBOTICS: Experiment 1
// Task7: Analog Read 
// Connect pin 5V to multimeter +ve and GND to multimeter -ve. Replace the value V1 below with voltage observed in millivolts for the 5V pin in the multimeter.

const int V1=5000;          // replace this with voltage value converted to millivolts as read from multimeter
const int analogPin = A1;  // Analog input pin

void setup() {
  Serial.begin(9600);       // Initialize serial communication
  pinMode(analogPin, INPUT);
}

void loop() {
  int analogValue = analogRead(analogPin);                      // Read the analog value
  float voltage = map(analogValue, 0, 1023, 0, V1) / 1000.0;  // Map the analogValue to voltage (V1 to be recorded from multimeter)

  Serial.print("\tAnalog Value: ");
  Serial.print(analogValue);
  Serial.print("\tVoltage: ");
  Serial.print(voltage);
  Serial.println(" V");

}
