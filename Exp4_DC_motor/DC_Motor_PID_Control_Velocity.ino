/**
 * @file DC_Motor_PID_Control_Velocity.ino
 * 
 * @brief This code implements PID control for a DC motor using an encoder for feedback.
 * 
 * The system uses a quadrature encoder to measure the motor's position and calculates 
 * the velocity. The velocity is filtered using a low-pass filter to reduce noise. The 
 * filtered velocity is then passed to a PID controller, which calculates the appropriate 
 * control signal (PWM value) to drive the motor towards a target velocity. The setMotor 
 * function adjusts the motor's speed and direction based on this control signal.
 * 
 * The code leverages interrupts to handle the encoder input, ensuring that position 
 * readings are accurate and up-to-date. The ATOMIC_BLOCK macro is used to prevent 
 * inconsistent reads of the position variable, which is modified within the interrupt service routine (ISR).
 */

// #include <util/atomic.h> // For the ATOMIC_BLOCK macro, which ensures atomic operations

#define ENCA 2  // Encoder A pin
#define ENCB 3  // Encoder B pin
#define PWM 5   // PWM output pin for motor speed control
#define IN1 6   // Motor control pin 1
#define IN2 7   // Motor control pin 2

// Global variables
volatile float posi = 0;  // Position, declared as volatile because it is modified within an interrupt
long prevT = 0;           // Previous timestamp in microseconds
int posPrev = 0;          // Previous encoder position
float e_ref_prev = 0;     // Previous error in velocity
float e_integral = 0;     // Integral of the error for PID control
float v1Prev = 0;         // Previous filtered velocity
float v1Filt = 0;         // Filtered velocity

/**
 * Setup function initializes the serial communication, pin modes, 
 * and attaches the interrupt for reading the encoder.
 */
void setup() {
  Serial.begin(19200);   // Start serial communication at 19200 baud rate
  pinMode(ENCA, INPUT);  // Set encoder A pin as input
  pinMode(ENCB, INPUT);  // Set encoder B pin as input
  pinMode(PWM, OUTPUT);  // Set PWM pin as output
  pinMode(IN1, OUTPUT);  // Set motor control pin 1 as output
  pinMode(IN2, OUTPUT);  // Set motor control pin 2 as output

  // Attach an interrupt to the ENCA pin, triggered on any change
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, CHANGE);
}

/**
 * Main loop that continuously executes PID control, calculates velocity,
 * and sends motor commands based on the target velocity.
 */
void loop() {
  // PID CONSTANTS
  float kp = 4;     // Proportional gain
  float kd = 0.2;  // Derivative gain
  float ki = 5;    // Integral gain

  // // Read the position in an atomic block to avoid potential
  // // misreads if the interrupt coincides with this code running
  // int pos = 0;
  // ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
  //   pos = posi;
  // }

  // Code without the use of atomic block library. We are blocking the interrupt operation while the variable is being read in the loop.
  noInterrupts();
  int pos = 0;
  pos = posi;
  interrupts();


  // TIME DIFFERENCE CALCULATION
  long currT = micros();                            // Get the current time in microseconds
  float deltaT = ((float)(currT - prevT)) / 1.0e6;  // Calculate elapsed time in seconds
  float velocity = (pos - posPrev) / deltaT;        // Calculate velocity as delta_position/delta_time
  prevT = currT;                                    // Update previous time
  posPrev = pos;                                    // Update previous position

  // TARGET VELOCITY
  float v_target = 250 * (sin(currT / 1e6) > 0);  // Generate a target velocity based on a sine wave

  // VELOCITY CALCULATION (RPM)
  float velocity1 = velocity / 1560 * 60.0;  // Convert encoder ticks to RPM

  // Apply a low-pass filter to the velocity measurement for smoothening the reading
  v1Filt = filterVelocity(velocity1, v1Filt, v1Prev);
  v1Prev = velocity1;

  // PID CONTROL TERMS
  // PID TERMS - PROPORTIONAL TERM
  float e_ref = v_target - v1Filt;  // Calculate error between target and actual velocity (using filtered velocity)
  //float e_ref = v_target - velocity1;  // Calculate error between target and actual velocity (without using filtered velocity)

  //PID TERMS - INTEGRAL TERM
  e_integral = e_integral + e_ref * deltaT;  // Integrate error over time

  // PID TERMS - DERIVATIVE TERM
  float dvdt = (e_ref - e_ref_prev) / deltaT;  // Calculate the rate of change of error
  e_ref_prev = e_ref;                          // Update the previous error

  // CONTROL SIGNAL (PID OUTPUT)
  float u = kp * e_ref + kd * dvdt + ki * e_integral;  // Calculate control signal based on PID terms

  // CONVERT CONTROL SIGNAL TO MOTOR ACCEPTABLE VALUES
  float pwr = fabs(u);  // Take the absolute value of the control signal
  if (pwr > 255) {
    pwr = 255;  // Limit the maximum value to 255 (for 8-bit PWM)
  }

  // MOTOR DIRECTION
  int dir = 1;
  if (u < 0) {
    dir = -1;
  }

  // Set motor direction and speed
  setMotor(dir, pwr, PWM, IN1, IN2);

  // Print target and actual filtered velocity for monitoring
  Serial.print(v_target);
  Serial.print(" ");
  Serial.print(v1Filt);
  Serial.println();

  delay(1);  // Delay for consistent sampling
}

/**
 * @brief Filters the velocity input using a low-pass filter.
 *
 * This function applies a low-pass filter to smooth the input velocity signal. 
 * The filter uses the current velocity (`velocity1`), the previously filtered 
 * velocity (`v1Filt`), and the previous velocity input (`v1Prev`) to calculate 
 * the new filtered velocity. The coefficients used in the filter equation are 
 * designed to balance responsiveness and noise reduction.
 *
 * @param velocity1 The current velocity input.
 * @param v1Filt The previously filtered velocity.
 * @param v1Prev The previous velocity input.
 * @return The new filtered velocity.
 */
float filterVelocity(float velocity1, float v1Filt, float v1Prev) {
  v1Filt = 0.854 * v1Filt + 0.0728 * velocity1 + 0.0728 * v1Prev;
  return v1Filt;
}

/**
 * Sets the motor speed and direction based on the control signal.
 * 
 * @param dir  Direction of the motor (-1 for reverse, 1 for forward)
 * @param pwmVal  PWM value (speed of the motor)
 * @param pwm  PWM pin
 * @param in1  Motor control pin 1
 * @param in2  Motor control pin 2
 */
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);  // Set motor speed
  if (dir == 1) {
    // Turn motor forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    // Turn motor reverse
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    // Stop motor
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

/**
 * Interrupt Service Routine (ISR) for reading the encoder.
 * Increments or decrements the position based on encoder state.
 */
void readEncoder() {
  if (digitalRead(ENCB) != digitalRead(ENCA)) {
    posi = posi + 1;  // Increment position if ENCB is not equal to ENCA
  } else {
    posi = posi - 1;  // Decrement position if ENCB equals ENCA
  }
}
