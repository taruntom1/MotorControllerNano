#include "Motor.h"

// Constructor
Motor::Motor(int directionPin, int pwmPin, volatile long &encCount, int countsPerRot, int pwmOff)
  : dirPin(directionPin), pwmPin(pwmPin), encoderCount(encCount), countsPerRotation(countsPerRot), pwmOffset(pwmOff)
{
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  prevTime = millis();  // Initialize previous time
  prevCount = encoderCount;  // Initialize previous encoder count
  memset(rpmBuffer, 0, sizeof(rpmBuffer));  // Clear the RPM buffer
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

  // Calculate time difference in milliseconds (integer math)
  unsigned long timeDiffMillis = currentTime - prevTime;

  // Calculate the change in encoder count
  long countDiff = encoderCount - prevCount;

  // Avoid division by zero by checking for very small time differences
  if (timeDiffMillis == 0)
  {
    return 0;
  }

  // Calculate RPM using integer math where possible, then convert to float only for final division
  float timeDiffMinutes = timeDiffMillis / static_cast<float>(millisToMinutes);
  float rpm = (countDiff / static_cast<float>(countsPerRotation)) / timeDiffMinutes;

  // Update previous values for next calculation
  prevTime = currentTime;
  prevCount = encoderCount;

  // Calculate running average of the last 5 RPM values
  sumRPM -= rpmBuffer[bufferIndex];          // Subtract the old value from the sum
  rpmBuffer[bufferIndex] = static_cast<int>(rpm); // Store the new RPM value
  sumRPM += rpmBuffer[bufferIndex];          // Add the new value to the sum

  bufferIndex = (bufferIndex + 1) % bufferSize; // Move to the next buffer position

  // Return the average of the last 5 RPM values
  return sumRPM / bufferSize;
}
