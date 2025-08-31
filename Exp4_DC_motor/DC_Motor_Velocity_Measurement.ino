/**
 * @file DC_Motor_Velocity_Measurement.ino
 * 
 * @brief This code measures the velocity of a DC motor using encoder feedback and applies a low-pass filter to smooth the velocity reading.
 * 
 * The motor is controlled via PWM, and the velocity is measured using a quadrature encoder. The measured velocity is then filtered to remove high-frequency noise,
 * providing a more stable and accurate velocity reading. The code also allows the motor to run at a target velocity, which is calculated using the rotor's gear ratio (GR)
 * and encoder counts per revolution (CPR).
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
float v1Prev = 0;        // Previous velocity measurement
float v1Filt = 0;        // Filtered velocity measurement

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
 * @brief Main loop for controlling motor velocity and measuring it.
 * 
 * The loop function calculates the motor's velocity based on encoder feedback, applies a low-pass filter
 * to the velocity measurement, and then outputs the filtered and unfiltered velocity values.
 */
void loop() {
  
  int pwr = 50; // Target velocity for the motor. You can try setting some integer values as well.
  setMotor(-1, pwr, PWM, IN1, IN2); // Set motor direction and power
  
  // // Read the position in an atomic block to avoid potential misreads
  // int pos = 0; 
  // ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
  //   pos = posi;
  // }
  
  // Code without the use of atomic block library. We are blocking the interrupt operation while the variable is being read in the loop.
  noInterrupts();
  int pos = 0;
  pos = posi;
  interrupts();


  // Calculate time difference
  long currT = micros(); // Get the current time in microseconds
  float deltaT = ((float)(currT - prevT)) / 1.0e6; // Calculate elapsed time in seconds
  float velocity = (pos - posPrev) / deltaT; // Calculate velocity as delta_position / delta_time
  posPrev = pos; // Update the previous position
  prevT = currT; // Update the previous time

  // Convert the velocity to rotor velocity in RPM
  float velocity1 = velocity / 1560 * 60.0; // Conversion based on gear ratio and encoder counts per revolution

  // Apply a low-pass filter to the velocity measurement for smoothening the reading
  v1Filt = filterVelocity(velocity1, v1Filt, v1Prev);
  v1Prev = velocity1; // Update the previous velocity measurement
  
  // Output the unfiltered and filtered velocity values to the serial monitor
  Serial.print(velocity1);
  Serial.print(" ");
  Serial.print(v1Filt);
  Serial.println();
  
  delay(1); // Short delay for consistent sampling
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
