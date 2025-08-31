/**
 * @file stepper_motor_control.ino
 * @brief This code controls a stepper motor by setting up the necessary pins
 * and sending a series of steps to the motor. The motor is configured to take 
 * a user-specified number of steps in a particular direction.
 * 
 * Pin Configuration:
 * - dir_pin: Direction control pin for the motor.
 * - step_pin: Step control pin for the motor.
 * - MS_1, MS_2: Microstepping control pins to adjust the motor's step resolution.
 * 
 * Microstep Settings:
 * - MS1 = LOW, MS2 = LOW: Full Step (2 phase)
 * - MS1 = HIGH, MS2 = LOW: Half Step
 * - MS1 = LOW, MS2 = HIGH: Quarter Step
 * - MS1 = HIGH, MS2 = HIGH: Eight Step
 * 
 * Variables:
 * - numSteps: Determines the number of steps the motor should move, set via Serial Monitor input.
 */

#define dir_pin 3  // Define the pin used to control the direction of the motor
#define step_pin 2 // Define the pin used to control the stepping of the motor
#define MS_1 4     // Define the pin used for microstepping control (MS1)
#define MS_2 5     // Define the pin used for microstepping control (MS2)

int x;              // Variable used for counting the number of steps
int numSteps;       // Number of steps the motor should take, will be set via Serial Monitor

void setup() {
    // Initialize serial communication at 9600 baud rate
    Serial.begin(9600);

    // Wait for the serial monitor to connect
    while (!Serial) {
        ; // Wait for serial port to connect. Needed for native USB port only
    }
    
    // Prompt the user to enter the number of steps
    Serial.println("Enter the number of steps for the motor:");

    // Wait until the user enters a number and presses enter
    while (Serial.available() == 0) {
        // Do nothing, just wait for input
    }

    // Read the input value and convert it to an integer
    numSteps = Serial.parseInt();

    // Confirm the number of steps entered
    Serial.print("Number of steps entered: ");
    Serial.println(numSteps);

    // Set the direction and step pins as output pins
    pinMode(dir_pin, OUTPUT);
    pinMode(step_pin, OUTPUT);

    // Set the microstepping control pins as output pins
    pinMode(MS_1, OUTPUT);
    pinMode(MS_2, OUTPUT);

    // Set the motor to rotate in one direction
    digitalWrite(dir_pin, HIGH);

    // Set initial state for the step pin
    digitalWrite(step_pin, LOW);

    // Configure microstepping: setting MS1 to LOW and MS2 to HIGH for Quarter Step
    digitalWrite(MS_1, LOW);
    digitalWrite(MS_2, LOW);

    // Loop through the number of steps to move the motor
    for (x = 1; x <= 2*numSteps; x++) {
        digitalWrite(step_pin, HIGH);  // Trigger a step
        delay(1);                      // Small delay to allow the motor to step
        digitalWrite(step_pin, LOW);   // Reset step pin
        delay(1);                      // Small delay before the next step
    }

    // Indicate that the motor has completed the movement
    Serial.print("Motor has moved ");
    Serial.print(numSteps);
    Serial.println(" steps.");
}

void loop() {
    // The loop is intentionally left empty
    // The motor only moves during the setup phase
}
