

#define ENA 5
#define IN1 6
#define IN2 7
#define IN3 8
#define IN4 9
#define ENB 10 

#define carSpeed 200

void Forward() {
  analogWrite(ENA, carSpeed); //enable L298n A channel
  analogWrite(ENB, carSpeed); //enable L298n B channel
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("go forward!");
}

void Back() {
  analogWrite(ENA, carSpeed); //enable L298n A channel
  analogWrite(ENB, carSpeed); //enable L298n B channel
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("go back!");
}

void Left() {
  analogWrite(ENA, carSpeed); //enable L298n A channel
  analogWrite(ENB, carSpeed); //enable L298n B channel
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("go left!");
}


void Right() {
  analogWrite(ENA, carSpeed); //enable L298n A channel
  analogWrite(ENB, carSpeed); //enable L298n B channel
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  

}


void Stop() {
  analogWrite(ENA, 0); //enable L298n A channel
  analogWrite(ENB, 0); //enable L298n B channel
  Serial.println("Stop!");
}

void setup() {
    Serial.begin(19200);
    pinMode(IN1, OUTPUT); //before useing io pin, pin mode must be set first
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
}

void loop() {

Forward();
delay(2000);
Left();
delay(1000);
Forward();
delay(2000);
Right();
delay(1000);
Back();
delay(2000);
}
