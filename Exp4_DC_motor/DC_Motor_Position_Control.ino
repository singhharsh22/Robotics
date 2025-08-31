/**
 * @file DC_Motor_Position_Control.ino
 * 
 * @brief This code implements PID control for the position of a DC motor using an encoder for feedback.
 * 
 * The system uses a quadrature encoder to measure the motor's position. The PID controller
 * calculates the appropriate control signal (PWM value) to drive the motor towards a target position.
 * The position is measured and the difference from the target (error) is used to adjust the motor's
 * speed and direction. The code leverages interrupts to handle the encoder input, ensuring that position
 * readings are accurate and up-to-date. The ATOMIC_BLOCK macro is used to prevent inconsistent reads
 * of the position variable, which is modified within the interrupt service routine (ISR).
 */

//#include <util/atomic.h> // For the ATOMIC_BLOCK macro, ensuring atomic operations

#define ENCA 2 // Encoder A pin
#define ENCB 3 // Encoder B pin
#define PWM 5  // PWM output pin for motor speed control
#define IN1 6  // Motor control pin 1
#define IN2 7  // Motor control pin 2

// Global variables
volatile float posi = 0; // Position, declared as volatile because it is modified within an interrupt
long prevT = 0;          // Previous timestamp in microseconds
int posPrev = 0;         // Previous encoder position
float e_pos_prev = 0;    // Previous positional error
float e_integral = 0;    // Integral of the positional error for PID control

/**
 * @brief Initializes the pins and attaches the encoder interrupt.
 * 
 * The setup function configures the necessary pins for input (encoder) and output (motor control).
 * It also attaches an interrupt to the encoder pin to track the motor's position.
 */
void setup() {
  Serial.begin(19200);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, CHANGE);
}

/**
 * @brief Main loop for controlling motor position using PID.
 * 
 * The loop function continuously computes the control signal based on the PID controller.
 * It reads the current motor position, calculates the positional error, and then uses the
 * PID formula to determine the required motor power and direction to reach the target position.
 */
void loop() {
  // TARGET POSITION
  int target = 2400; // Desired target position

  // PID CONSTANTS
  float kp = 3;     // Proportional gain
  float kd = 0.5;   // Derivative gain
  float ki = 0.05;  // Integral gain

  // // Read the position in an atomic block to avoid a potential
  // // misread if the interrupt coincides with this code running
  // int pos = 0; 
  // ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
  //   pos = posi;
  // }

  //Code without the use of atomic block library
  noInterrupts();
  int pos = 0;
  pos = posi;
  interrupts();


  // TIME DIFFERENCE CALCULATION
  long currT = micros(); // Get the current time in microseconds
  float deltaT = ((float)(currT - prevT)) / 1.0e6; // Calculate elapsed time in seconds
  float velocity = (pos - posPrev) / deltaT; // Calculate velocity as delta_position / delta_time
  prevT = currT; // Update the previous time
  posPrev = pos; // Update the previous position

  // PID TERMS - POSITIONAL ERROR
  int e_pos = target - pos; // Calculate the positional error (difference from target)
  
  // PID TERMS - DERIVATIVE ERROR
  float dedt = (e_pos - e_pos_prev) / deltaT; // Calculate the rate of change of positional error
  e_pos_prev = e_pos; // Update the previous positional error

  // PID TERMS - INTEGRAL ERROR
  e_integral += e_pos * deltaT; // Update the integral of the positional error

  // CONTROL SIGNAL
  float u = kp * e_pos + kd * dedt + ki * e_integral; // Calculate the control signal using the PID formula

  // CONVERTING TO MOTOR ACCEPTABLE VALUES
  float pwr = fabs(u); // Take the absolute value of the control signal (motor power)
  if (pwr > 255) {
    pwr = 255; // Limit the maximum motor power to 255
  }

  // MOTOR DIRECTION
  int dir = 1;
  if (u < 0) {
    dir = -1;
  }
  
  setMotor(dir, pwr, PWM, IN1, IN2); // Apply the control signal to the motor
  
  // Print the target position and current position to the serial monitor
  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
}

/**
 * @brief Sets the motor's speed and direction based on the control signal.
 * 
 * @param dir The direction of the motor (1 for forward, -1 for reverse).
 * @param pwmVal The PWM value (motor speed).
 * @param pwm The PWM pin for motor control.
 * @param in1 The motor control pin 1.
 * @param in2 The motor control pin 2.
 */
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal); // Set the motor speed (PWM value)
  if (dir == 1) { 
    // Rotate motor in one direction
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    // Rotate motor in the opposite direction
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    // Stop the motor
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);    
  }
}

/**
 * @brief Interrupt Service Routine (ISR) for reading the encoder position.
 * 
 * The ISR updates the motor's position based on the state of the encoder pins.
 * It increments or decrements the position counter based on the direction of rotation.
 */
void readEncoder() {
  if (digitalRead(ENCB) != digitalRead(ENCA)) {
    posi += 1; // Increment position if encoder indicates forward rotation
  } else {
    posi -= 1; // Decrement position if encoder indicates reverse rotation
  }
}
