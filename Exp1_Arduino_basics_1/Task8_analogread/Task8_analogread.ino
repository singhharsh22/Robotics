// ME381 ROBOTICS: Experiment 1
// Task8: Analog Read 
// Connect pin 9 to pin A1. Change the dutyCycle variable to 30, 60 and 85 and observe the serial plotter

const int pwmPin = 9;      // PWM output pin
const int analogPin = A1;  // Analog input pin

int dutyCycle = 30;                // duty cycle in percentage (change this according to the duty cycle required)
int pwmValue = ((float(dutyCycle)/100)*255); // mapping the duty cycle to nearest integer value to the used a argument for analogWrite

float voltageBuffer[250];
int bufferIndex=0,bufferIndexmax=250;

void setup() {
  Serial.begin(9600);         // Initialize serial communication
  pinMode(pwmPin, OUTPUT);    // Set the PWM pin as an output
  pinMode(analogPin, INPUT);  // Set anolgPin A1 as input
  analogWrite(pwmPin, pwmValue);  // Apply PWM once (that is why this code is in setup())
}

void loop() {

  delay(5000);                     // Allow some time for observing the serial plotter before the next buffer readings are plotted
  
  // buffer the values read at analogPin
  for(bufferIndex=0;bufferIndex<bufferIndexmax;bufferIndex++)
  {
  int analogValue = analogRead(analogPin);                      // Read the analog value
  float voltage = map(analogValue, 0, 1023, 0, 5000) / 1000.0;  // Map the value to voltage (0-5V)
  voltageBuffer[bufferIndex] = voltage;
  }

  // print the previously buffered values
  for(bufferIndex=0;bufferIndex<bufferIndexmax;bufferIndex++)
  {
    Serial.print("\tVoltage:");
    Serial.print(voltageBuffer[bufferIndex]);
    Serial.println(" V");
  }

}
