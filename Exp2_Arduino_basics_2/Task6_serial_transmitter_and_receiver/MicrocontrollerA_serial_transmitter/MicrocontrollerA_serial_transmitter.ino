// ME381 ROBOTICS: Experiment 2
// Task6: UART serial transmitter Board A
// Upload this code to Board A. Open the serial monitor and observe what Board A is transmitting

void setup() {
  Serial.begin(9600);  // Initialize serial communication at 9600 bps
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
}

void loop() {

  for (int num = 0; num <= 10; num++) {
    Serial.println(num);  // Send the number
    delay(6000);          // Wait for 6 seconds
  }
}
