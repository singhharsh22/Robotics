// ME381 ROBOTICS: Experiment 3
// Task 7: Joystick with RGB LED 
// Move the joystick from one extreme to another along the x and the y axis and observe the LED. 

#include <Arduino.h>

// Define the pins for RGB LED
const int redPin = 9;    // Red LED pin (PWM)
const int bluePin = 11;  // Blue LED pin (PWM)
int Led=13;//define LED pin


// Define the pin for the X axis potentiometer
const int potPinX = A0;  // Analog input pin for X axis
const int potPinY = A1;  // Analog input pin for Y axis

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Set RGB LED pins as outputs
  pinMode(redPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(Led,OUTPUT);
  digitalWrite(Led,LOW);
}

void loop() {

  int potValueX = analogRead(potPinX);

  // Map the potentiometer value (0-1023) to the range of PWM (0-255)
  int brightness = map(potValueX, 0, 1023, 0, 255);

  // Update the Red LED brightness
  analogWrite(redPin, brightness);

  // Output the mapped brightness value to serial monitor
  Serial.print("Potentiometer X value: ");
  Serial.print(potValueX);
  Serial.print("   Mapped brightness: ");
  Serial.println(brightness);

  // Delay to stabilize readings
  delay(100);
  // Read the potentiometer value for Y axis
  int potValueY = analogRead(potPinY);

  // Map the potentiometer value (0-1023) to the range of PWM (0-255)
  int brightness2 = map(potValueY, 0, 1023, 0, 255);

  // Update the Blue LED brightness
  analogWrite(bluePin, brightness2);

  // Output the mapped brightness value to serial monitor
  Serial.print("Potentiometer Y value: ");
  Serial.print(potValueY);
  Serial.print("   Mapped brightness: ");
  Serial.println(brightness2);

  // Delay to stabilize readings
  delay(100);
 
}