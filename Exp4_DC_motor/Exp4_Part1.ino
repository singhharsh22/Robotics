#define ENCA 2 // Yellow
#define ENCB 3 // White

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
}

void loop() {
  int a = digitalRead(ENCA);
  int b = digitalRead(ENCB);
  Serial.print(a*10); // To scale up the signal for visibility
  Serial.print(" ");
  Serial.print(b*5);
  Serial.println();
}
