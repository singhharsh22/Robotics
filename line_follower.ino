//Line Tracking IO define
#define LEFT_LINE_TRACJING          2
#define CENTER_LINE_TRACJING        3
#define right_LINE_TRACJING         4
int Left_Tra_Value;
int Center_Tra_Value;
int Right_Tra_Value;

#define ENA 5
#define IN1 6
#define IN2 7
#define IN3 8
#define IN4 9
#define ENB 10 

#define carSpeed 200
#define turnSpeed 100

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
    pinMode(LEFT_LINE_TRACJING, INPUT);
    pinMode(CENTER_LINE_TRACJING, INPUT);
    pinMode(right_LINE_TRACJING, INPUT);
    pinMode(IN1, OUTPUT); //before useing io pin, pin mode must be set first
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Left_Tra_Value = digitalRead(LEFT_LINE_TRACJING);
  Center_Tra_Value = digitalRead(CENTER_LINE_TRACJING);
  Right_Tra_Value = digitalRead(right_LINE_TRACJING);
  
  Serial.println("l,c,r:");
  Serial.println(Left_Tra_Value);
  Serial.println(Center_Tra_Value);
  Serial.println(Right_Tra_Value);
  
  if (Left_Tra_Value == 1 && Center_Tra_Value ==1 && Right_Tra_Value == 1)
  {
    Stop();
  }
  if (Left_Tra_Value == 1 && Right_Tra_Value == 0)
  {
    Left();
  }
  if (Left_Tra_Value == 0 && Right_Tra_Value == 1)
  {
    Right();
  }
  if (Left_Tra_Value == 0 && Center_Tra_Value ==1 && Right_Tra_Value == 0)
  {
    Forward();
  }
  if (Left_Tra_Value == 0 && Center_Tra_Value ==0 && Right_Tra_Value == 0)
  {
    Left();
  }

}
