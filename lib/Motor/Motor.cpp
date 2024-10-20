#include "Motor.h"

// Constructor
Motor::Motor(int directionPin, int pwmPin, volatile long &encCount, int countsPerRot, int pwmOff)
    : dirPin(directionPin), pwmPin(pwmPin), encoderCount(encCount), countsPerRotation(countsPerRot), pwmOffset(pwmOff)
{
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  prevTime = millis();      // Initialize previous time
  prevCount = encoderCount; // Initialize previous encoder count
  alpha = 0.01;
  rpmConstant = 60000000.0 / countsPerRotation; // RPM constant
}
// Function to control Motor speed and direction
void Motor::controlMotor(double speed)
{
  if (speed > 0)
  {
    // Forward direction
    digitalWrite(dirPin, HIGH);
    analogWrite(pwmPin, speed + pwmOffset); // Set speed (0-255)
  }
  else if (speed < 0)
  {
    // Reverse direction
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, -speed + pwmOffset); // Set speed (0-255), negate for reverse
  }
  else
  {
    // Stop the motor
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, 0);
  }
}

void Motor::updateAlpha(float newAlpha)
{
  alpha = newAlpha;
}
// Function to calculate the angle of rotation for the motor
double Motor::calculateAngle()
{
  // Calculate angle in degrees (360 degrees per full rotation)
  return (encoderCount * 360.0) / countsPerRotation;
}

// Function to calculate RPM with running average of the last 5 values
int Motor::calculateRPM()
{
  long currentTime = micros(); // Get current time in milliseconds

  // Calculate time difference in milliseconds
  long timeDiffMillis = currentTime - prevTime;

  // Calculate the change in encoder count
  long countDiff = encoderCount - prevCount;

  // Avoid division by zero
  if (timeDiffMillis == 0)
  {
    return filteredRPM; // Return 0 RPM if no time has passed
  }

  // Calculate RPM
  float rpm = ((countDiff * rpmConstant) / timeDiffMillis);

  filteredRPM = (alpha * rpm) + ((1 - alpha) * filteredRPM);

  // Update previous time and encoder count for the next calculation
  prevTime = currentTime;
  prevCount = encoderCount;
  return filteredRPM;
}
