// ME381 ROBOTICS: Experiment 1
// Task2: Digital Read 
// Connect the following pins to pin 8 one by one: none (open), GND, RESET, IOREF, AREF, 3.3V, 5V.

void setup() {
  // put your setup code here, to run once:
  pinMode(8,INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
int val = digitalRead(8);
Serial.print("digital read at pin 8: ");
Serial.println(val);
delay(200);         // delay placed to slow down the loop; try changing this value to 500, 1000, 5000 etc.
}