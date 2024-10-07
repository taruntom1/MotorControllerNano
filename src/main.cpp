#include <Arduino.h>

// Pin definitions for Encoders
#define encoderPinA1 2  // Encoder A Channel 1
#define encoderPinA2 4  // Encoder A Channel 2
#define encoderPinB1 3  // Encoder B Channel 1
#define encoderPinB2 5  // Encoder B Channel 2

// Pin definitions for Motor A
const int motorA_IN1 = 6;  // Motor A IN1 pin
const int motorA_IN2 = 7;  // Motor A IN2 pin
const int motorA_PWM = 8;  // Motor A PWM pin (Enable pin)

// Pin definitions for Motor B
const int motorB_IN1 = 9;  // Motor B IN1 pin
const int motorB_IN2 = 10;  // Motor B IN2 pin
const int motorB_PWM = 11;  // Motor B PWM pin (Enable pin)

// Variables to hold encoder counts
volatile long countA = 0;  // Encoder count for Motor A
volatile long countB = 0;  // Encoder count for Motor B

// Constants for encoder and motor calculations
const float countsPerRotation = 500.0;  // 500 encoder counts per motor rotation

// Previous values for RPM calculation
unsigned long prevTimeA = 0;  // Previous timestamp for Motor A
long prevCountA = 0;          // Previous encoder count for Motor A
unsigned long prevTimeB = 0;  // Previous timestamp for Motor B
long prevCountB = 0;          // Previous encoder count for Motor B

// Setup function to initialize motors, encoders, and serial communication
void setup() {
  // Attaching interrupts for encoders (trigger on RISING edge)
  attachInterrupt(digitalPinToInterrupt(encoderPinA1), manageCountA, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinB1), manageCountB, RISING);

  // Initialize serial communication
  Serial.begin(115200);

  // Set motor control pins as outputs for Motor A
  pinMode(motorA_IN1, OUTPUT);
  pinMode(motorA_IN2, OUTPUT);
  pinMode(motorA_PWM, OUTPUT);

  // Set motor control pins as outputs for Motor B
  pinMode(motorB_IN1, OUTPUT);
  pinMode(motorB_IN2, OUTPUT);
  pinMode(motorB_PWM, OUTPUT);
}

// Main loop function
void loop() {
  // Print encoder counts, RPM, and rotation angle of Motor A
  Serial.print(countA);
  Serial.print("\t");
  Serial.print(countB);
  Serial.print("\t");
  Serial.print(calculateRPMA());
  Serial.print("\t");
  Serial.println(calculateAngleA());
  delay(100);  // 100ms delay between readings
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
  return (countA % (long)countsPerRotation) * (360.0 / countsPerRotation);
}

// Function to calculate the angle of rotation for Motor B
float calculateAngleB() {
  // Calculate angle in degrees (360 degrees per full rotation)
  return (countB % (long)countsPerRotation) * (360.0 / countsPerRotation);
}

// Function to control Motor A speed and direction
void controlMotorA(int speed) {
  controlMotor(speed, motorA_IN1, motorA_IN2, motorA_PWM);
}

// Function to control Motor B speed and direction
void controlMotorB(int speed) {
  controlMotor(speed, motorB_IN1, motorB_IN2, motorB_PWM);
}

// General function to control any motor based on speed (negative for reverse, positive for forward)
void controlMotor(int speed, int IN1, int IN2, int PWM) {
  if (speed > 0) {
    // Forward direction
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(PWM, speed);  // Set speed (0-255)
  } else if (speed < 0) {
    // Reverse direction
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(PWM, -speed);  // Set speed (0-255), negate the value for reverse
  } else {
    // Stop the motor
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(PWM, 0);
  }
}
