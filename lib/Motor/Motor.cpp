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
  unsigned long currentTime = millis(); // Get current time in milliseconds

  // Calculate time difference in milliseconds
  unsigned long timeDiffMillis = currentTime - prevTime;

  // Calculate the change in encoder count
  long countDiff = encoderCount - prevCount;

  // Avoid division by zero by returning the previous filtered RPM value
  if (timeDiffMillis == 0)
  {
    return filteredRPM; // Return the previously filtered RPM value
  }

  long rpm = (countDiff * 60000L) / (countsPerRotation * timeDiffMillis);

  // Apply low-pass filter using exponential moving average
  // alpha is the smoothing factor (0 < alpha < 1), here we use 0.1 as an example

  filteredRPM = (alpha * rpm) + ((1 - alpha) * filteredRPM);

  // Update previous time and encoder count for next calculation
  prevTime = currentTime;
  prevCount = encoderCount;

  return filteredRPM;
}
