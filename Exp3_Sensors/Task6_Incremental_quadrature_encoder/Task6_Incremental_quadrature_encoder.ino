// ME381 ROBOTICS: Experiment 3
// Task 6: Incremental Quadrature Encoder
// Rotate the knob of the incremental encoder very slowly and observe the Serial Monitor

int encoderPin1 = 2;
int encoderPin2 = 3;

volatile int lastEncoded = 0;
volatile long encoderValue = 0;

long lastencoderValue = 0;

int lastMSB = 0;
int lastLSB = 0;

void updateEncoder();                 // function declaration

void setup() {
  Serial.begin(115200);               // set the same baud rate on the Serial Monitor

  pinMode(encoderPin1, INPUT_PULLUP);  // turn pullup resistor on
  pinMode(encoderPin2, INPUT_PULLUP);  // turn pullup resistor on

  // call the ISR: updateEncoder() when any high/low changed seen
  // on interrupt 0 (pin 2), or interrupt 1 (pin 3)
  attachInterrupt(digitalPinToInterrupt(encoderPin1), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPin2), updateEncoder, CHANGE);
}

void loop() {
  Serial.println(encoderValue);
  delay(50);  // just here to slow down the output, and show it will work  even during a delay
}


void updateEncoder() {
  int MSB = digitalRead(encoderPin1);  // MSB = most significant bit
  int LSB = digitalRead(encoderPin2);  // LSB = least significant bit

  int encoded = (MSB << 1) | LSB;          // converting the 2 pin value to single number
  int sum = (lastEncoded << 2) | encoded;  // adding it to the previous encoded value

  // quadrature logic of transition (See notes)
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue--;

  lastEncoded = encoded;  // store this value for next time
}