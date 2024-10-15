#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor
{
  private:
    int dirPin;                  // Motor direction pin
    int pwmPin;                  // Motor PWM pin
    volatile long &encoderCount; // Reference to the volatile global encoder count
    int pwmOffset;               // PWM offset for calibration
    unsigned long prevTime;      // Previous time for RPM calculation
    long prevCount;              // Previous encoder count for RPM calculation
    const int countsPerRotation; // Encoder counts per full rotation
    const unsigned long millisToMinutes = 60000; // Milliseconds to minutes for RPM calculation
    const int bufferSize = 5;    // Number of values for the running average
    int rpmBuffer[5];            // Array to store the last 5 RPM values
    int bufferIndex = 0;         // Index for the next value in the buffer
    int sumRPM = 0;              // Sum of the values in the buffer

  public:
    // Constructor
    Motor(int directionPin, int pwmPin, volatile long &encCount, int countsPerRot, int pwmOff = 0);

    // Function to control Motor speed and direction
    void controlMotor(double speed);

    // Function to calculate the angle of rotation for the motor
    double calculateAngle();

    // Function to calculate RPM with running average of the last 5 values
    int calculateRPM();
};

#endif