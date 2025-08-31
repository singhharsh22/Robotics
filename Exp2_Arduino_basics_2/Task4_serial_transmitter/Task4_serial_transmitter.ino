// ME381 ROBOTICS: Experiment 2
// Task4: UART serial transmitter
// Observe the serial monitor after uploading this code

void setup() {
  Serial.begin(9600);  // Initialize serial communication at 9600 bps
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW); // keep the LED off otherwise it will be floating
}

void loop() {

  for (int num = 0; num <= 10; num++) {
    Serial.println(num);  // Send the number
    delay(6000);          // Wait for 6 seconds
  }
}
