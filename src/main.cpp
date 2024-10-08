#include <Arduino.h>
#include <PID_v1.h>
// Pin definitions for Encoders
#define encoderPinA1 2  // Encoder A Channel 1
#define encoderPinA2 4  // Encoder A Channel 2
#define encoderPinB1 3  // Encoder B Channel 1
#define encoderPinB2 5  // Encoder B Channel 2

// Pin definitions for Motor A
const int motorA_IN1 = 6;  // Motor A IN1 pin
const int motorA_PWM = 8;  // Motor A PWM pin (Enable pin)

// Pin definitions for Motor B
const int motorB_IN1 = 9;  // Motor B IN1 pin
const int motorB_PWM = 11;  // Motor B PWM pin (Enable pin)

// Variables to hold encoder counts
volatile long countA = 0;  // Encoder count for Motor A
volatile long countB = 0;  // Encoder count for Motor B

// Constants for encoder and motor calculations
const float countsPerRotation = 375.0;  // 500 encoder counts per motor rotation

// Previous values for RPM calculation
unsigned long prevTimeA = 0;  // Previous timestamp for Motor A
long prevCountA = 0;          // Previous encoder count for Motor A
unsigned long prevTimeB = 0;  // Previous timestamp for Motor B
long prevCountB = 0;          // Previous encoder count for Motor B


// PID constants
float kp1, ki1, kd1;

double angle1 = 0;
double target = 0;
double output = 0;    // Output to control motor/pwm
double Kp = 1.0, Ki = 0.5, Kd = 0.1;  // Initial PID values

// PID Controller Object
PID motorAnglePIDA(&angle1, &target, &output, Kp, Ki, Kd, DIRECT);

// Timing variables for running PID at a set interval (30 times per second)
unsigned long lastUpdateTime = 0;
const int interval = 33; // Time interval in milliseconds (1000ms/30 = ~33ms)

// Function declarations
void manageCountA(); // ISR for Encoder A
void manageCountB(); // ISR for Encoder B
float calculateRPMA(); // Function to calculate RPM for Motor A
float calculateRPMB(); // Function to calculate RPM for Motor B
float calculateAngleA(); // Function to calculate the angle of rotation for Motor A
float calculateAngleB(); // Function to calculate the angle of rotation for Motor B
void controlMotorA(int speed); // Function to control Motor A speed and direction
void controlMotorB(int speed); // Function to control Motor B speed and direction
void controlMotor(int speed, int IN1, int PWM); // General function to control any motor based on speed (negative for reverse, positive for forward)


// Setup function to initialize motors, encoders, and serial communication
void setup() {
  // Attaching interrupts for encoders (trigger on RISING edge)
  attachInterrupt(digitalPinToInterrupt(encoderPinA1), manageCountA, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinB1), manageCountB, RISING);

  // Initialize serial communication
  Serial.begin(115200);

  // Set motor control pins as outputs for Motor A
  pinMode(motorA_IN1, OUTPUT);
  pinMode(motorA_PWM, OUTPUT);

  // Set motor control pins as outputs for Motor B
  pinMode(motorB_IN1, OUTPUT);
  pinMode(motorB_PWM, OUTPUT);


    // Initial settings for AutoPID
  motorAnglePIDA.SetSampleTime(interval);  // Set PID update interval
  motorAnglePIDA.SetOutputLimits(-255, 255);  // Set output range

  Serial.println("PID Controller Initialized.");
  Serial.println("Use the following format to tune: Kp, Ki, Kd, target");
  Serial.println("Example: 2.5, 1.0, 0.5, 50");

  motorAnglePIDA.SetMode(AUTOMATIC);  // Set PID mode to automatic
}

// Main loop function
void loop() {
  // Check if it's time to update the PID (30 times per second)
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= interval) {
    lastUpdateTime = currentTime;

    // Update input from your sensor or system (replace this with actual sensor reading)
    angle1 = calculateAngleA();

    // Run PID calculation
    motorAnglePIDA.Compute();

    // Apply the output (e.g., control motor speed or any other actuator)
    controlMotorA(output);
    
    // Print current values for monitoring
    Serial.print("Input: ");
    Serial.print(angle1);
    Serial.print(" | Output: ");
    Serial.print(output);
    Serial.print(" | Target: ");
    Serial.print(target);
    Serial.print("  countA:");
    Serial.println(countA);
  }

  // Check for incoming serial data to update PID gains and target
  if (Serial.available()) {
    String received = Serial.readStringUntil('\n');
    received.trim(); // Remove any leading or trailing whitespaces

    // Debugging: Print the received string to check input
    Serial.print("Received input: ");
    Serial.println(received);

    // Split the string by spaces
    int firstSpace = received.indexOf(' ');
    int secondSpace = received.indexOf(' ', firstSpace + 1);
    int thirdSpace = received.indexOf(' ', secondSpace + 1);

    // Check if we found three spaces (which would give us four values)
    if (firstSpace > 0 && secondSpace > firstSpace && thirdSpace > secondSpace) {
      // Extract each value as a substring and convert to double
      String kpString = received.substring(0, firstSpace);
      String kiString = received.substring(firstSpace + 1, secondSpace);
      String kdString = received.substring(secondSpace + 1, thirdSpace);
      String targetString = received.substring(thirdSpace + 1);
      //Storing the new values
      double newKp = kpString.toDouble();
      double newKi = kiString.toDouble();
      double newKd = kdString.toDouble();
      double newTarget = targetString.toDouble();

      // Update PID gains and target
      Kp = newKp;
      Ki = newKi;
      Kd = newKd;
      target = newTarget;

      // Set the new gains to the PID controller
      motorAnglePIDA.SetTunings(Kp, Ki, Kd);

      // Print the updated values
      Serial.println("Updated PID Parameters:");
      Serial.print("Kp: ");
      Serial.print(Kp);
      Serial.print(" | Ki: ");
      Serial.print(Ki);
      Serial.print(" | Kd: ");
      Serial.print(Kd);
      Serial.print(" | Target: ");
      Serial.println(target);
    } else {
      Serial.println("Invalid input. Use the format: Kp Ki Kd target");
    }
  }
}

// Interrupt service routine (ISR) for Encoder A
void manageCountA() {
  // Check the state of encoderPinA2 to determine direction
  if (digitalRead(encoderPinA2) == 0) {
    countA++;  // Forward rotation
  } else {
    countA--;  // Reverse rotation
  }
}

// Interrupt service routine (ISR) for Encoder B
void manageCountB() {
  // Check the state of encoderPinB2 to determine direction
  if (digitalRead(encoderPinB2) == 0) {
    countB++;  // Forward rotation
  } else {
    countB--;  // Reverse rotation
  }
}

// Function to calculate RPM for Motor A
float calculateRPMA() {
  unsigned long currentTime = millis();  // Get current time in milliseconds
  long currentCountA = countA;           // Get current encoder count for Motor A

  // Calculate time difference in minutes
  float timeDiffMinutesA = (currentTime - prevTimeA) / 60000.0;

  // Calculate the change in encoder count
  long countDiffA = currentCountA - prevCountA;

  // Calculate RPM
  float rpmA = (countDiffA / countsPerRotation) / timeDiffMinutesA;

  // Update previous values for next calculation
  prevTimeA = currentTime;
  prevCountA = currentCountA;

  return rpmA;
}

// Function to calculate RPM for Motor B
float calculateRPMB() {
  unsigned long currentTime = millis();  // Get current time in milliseconds
  long currentCountB = countB;           // Get current encoder count for Motor B

  // Calculate time difference in minutes
  float timeDiffMinutesB = (currentTime - prevTimeB) / 60000.0;

  // Calculate the change in encoder count
  long countDiffB = currentCountB - prevCountB;

  // Calculate RPM
  float rpmB = (countDiffB / countsPerRotation) / timeDiffMinutesB;

  // Update previous values for next calculation
  prevTimeB = currentTime;
  prevCountB = currentCountB;

  return rpmB;
}

// Function to calculate the angle of rotation for Motor A
float calculateAngleA() {
  // Calculate angle in degrees (360 degrees per full rotation)
  return (countA)*360 / 375;
}

// Function to calculate the angle of rotation for Motor B
float calculateAngleB() {
  // Calculate angle in degrees (360 degrees per full rotation)
  return ((countB)*360 / 375);
}

// Function to control Motor A speed and direction
void controlMotorA(int speed) {
  controlMotor(speed, motorA_IN1, motorA_PWM);
}

// Function to control Motor B speed and direction
void controlMotorB(int speed) {
  controlMotor(speed, motorB_IN1, motorB_PWM);
}

// General function to control any motor based on speed (negative for reverse, positive for forward)
void controlMotor(int speed, int IN1, int PWM) {
  if (speed > 0) {
    // Forward direction
    digitalWrite(IN1, HIGH);
    analogWrite(PWM, speed);  // Set speed (0-255)
  } else if (speed < 0) {
    // Reverse direction
    digitalWrite(IN1, LOW);
    analogWrite(PWM, -speed);  // Set speed (0-255), negate the value for reverse
  } else {
    // Stop the motor
    digitalWrite(IN1, LOW);
    analogWrite(PWM, 0);
  }
}
