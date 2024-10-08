#include <Arduino.h>
#include <PID_v1.h>
//////////////////////////////////////////////////////////////////////////////////////
///////////////////////////Control via Serial/////////////////////////////////////
// Define command characters for various actions
#define MOTOR_ANGLES   'a' // Set motor angles (degrees) eg: a 100 200
#define READ_ENCODERS  'e' // Read encoder counts (counts) eg: e
#define MOTOR_SPEEDS   'm' // Set motor speeds (RPM) eg: m 100 200
#define PING           'p' // Ping the Arduino
#define RESET_ENCODERS 'r' // Reset encoder counts (counts) eg: r
#define UPDATE_PIDAA    'u' // Update angle PID values for Motor A (Kp, Ki, Kd) eg: u 10:20:30
#define UPDATE_PIDAB    'v' // Update angle PID values for Motor B (Kp, Ki, Kd) eg: v 10:20:30
#define UPDATE_PIDSA    'w' // Update speed PID values for Motor A (Kp, Ki, Kd) eg: w 10:20:30
#define UPDATE_PIDSB    'x' // Update speed PID values for Motor B (Kp, Ki, Kd) eg: x 10:20:30


// Variables to handle serial command parsing
int arg = 0;
int index = 0;
char chr;            // Holds the input character
char cmd;            // Holds the current command
char argv1[16];      // Holds the first argument as a string
char argv2[16];      // Holds the second argument as a string
long arg1;           // First argument converted to integer
long arg2;           // Second argument converted to integer

// Function to clear command parameters
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}


// Function to run the appropriate command based on serial input
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];   // Holds parsed PID values
  arg1 = atoi(argv1);   // Convert argv1 to integer
  arg2 = atoi(argv2);   // Convert argv2 to integer
  
  switch (cmd) {
    case PING:
      Serial.println(arg1);  // Respond with argument 1
      break;
    
    case READ_ENCODERS:
      Serial.println("Encoder count OK");
      break;
    
    case RESET_ENCODERS:
      Serial.println("Encoders reset");
      break;
    
    case MOTOR_SPEEDS:
      if (arg1 == 0 && arg2 == 0) {
        Serial.println("Motors stopped");
      } else {
        Serial.print("Speed 1: ");
        Serial.println(arg1);
        Serial.print("Speed 2: ");
        Serial.println(arg2);
      }
      break;
    
    case UPDATE_PIDAA:
      // Parse PID values for Motor A
      while ((str = strtok_r(p, ":", &p)) != NULL) {
        pid_args[i] = atoi(str);
        i++;
      }
      Kp1 = pid_args[0];
      Kd1 = pid_args[1];
      Ki1 = pid_args[2];

      Serial.println("PID A updated");
      Serial.print("Kp1: "); Serial.println(Kp1);
      Serial.print("Kd1: "); Serial.println(Kd1);
      Serial.print("Ki1: "); Serial.println(Ki1);
      break;
    
    case UPDATE_PIDAB:
      // Parse PID values for Motor B
      while ((str = strtok_r(p, ":", &p)) != NULL) {
        pid_args[i] = atoi(str);
        i++;
      }
      Kp2 = pid_args[0];
      Kd2 = pid_args[1];
      Ki2 = pid_args[2];

      Serial.println("PID B updated");
      Serial.print("Kp2: "); Serial.println(Kp2);
      Serial.print("Kd2: "); Serial.println(Kd2);
      Serial.print("Ki2: "); Serial.println(Ki2);
      break;

    case UPDATE_PIDSA:
      // Parse PID values for Motor A
      while ((str = strtok_r(p, ":", &p)) != NULL) {
        pid_args[i] = atoi(str);
        i++;
      }
      Kp1 = pid_args[0];
      Kd1 = pid_args[1];
      Ki1 = pid_args[2];

      Serial.println("PID A updated");
      Serial.print("Kp1: "); Serial.println(Kp1);
      Serial.print("Kd1: "); Serial.println(Kd1);
      Serial.print("Ki1: "); Serial.println(Ki1);
      break;
    case UPDATE_PIDSB:
      // Parse PID values for Motor A
      while ((str = strtok_r(p, ":", &p)) != NULL) {
        pid_args[i] = atoi(str);
        i++;
      }
      Kp1 = pid_args[0];
      Kd1 = pid_args[1];
      Ki1 = pid_args[2];

      Serial.println("PID A updated");
      Serial.print("Kp1: "); Serial.println(Kp1);
      Serial.print("Kd1: "); Serial.println(Kd1);
      Serial.print("Ki1: "); Serial.println(Ki1);
      break;
    case MOTOR_ANGLES:
      Serial.println("Motor angles set");
      break;

    default:
      Serial.println("Invalid command");
      break;
  }
}

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

// Pin definitions for Encoders
#define encoderPinA1 2  // Encoder A Channel 1
#define encoderPinA2 4  // Encoder A Channel 2
#define encoderPinB1 3  // Encoder B Channel 1
#define encoderPinB2 5  // Encoder B Channel 2

// Pin definitions for Motor A
const int motorA_IN1 = 8;  // Motor A IN1 pin
const int motorA_PWM = 6;  // Motor A PWM pin (Enable pin)

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


// PID Controller Mode
bool mode; // Mode for PID controller mode 0 for angle PID and 1 for speed PID

double output1 = 0; // Output for Motor A
double output2 = 0; // Output for Motor B
/////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////Angle PID//////////////////////////////////////////////////////

double angle1 = 0; // Angle of rotation for Motor A
double targetAngle1 = 0; // Target angle for Motor A

double angle2 = 0; // Angle of rotation for Motor B
double targetAngle2 = 0; // Target angle for Motor B

// PID constants for motor 1
double KpA1 = 1.0, KiA1 = 0.5, KdA1 = 0.1;
// PID constants for motor 2
double KpA2 = 1.0, KiA2 = 0.5, KdA2 = 0.1;

// PID Controller Object
PID motorAnglePIDA(&angle1, &output1, &targetAngle1, KpA1, KiA1, KdA1, DIRECT);
PID motorAnglePIDB(&angle2, &output2, &targetAngle1, KpA2, KiA2, KdA1, DIRECT);
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////Speed PID//////////////////////////////////////////////////////

double speed1 = 0; // Speed of Motor A
double targetSpeed1 = 0; // Target speed for Motor A

double speed2 = 0; // Speed of Motor B
double targetSpeed2 = 0; // Target speed for Motor B

// PID constants for motor 1
double KpB1 = 1.0, KiB1 = 0.5, KdB1 = 0.1;
// PID constants for motor 2
double KpB2 = 1.0, KiB2 = 0.5, KdB2 = 0.1;

PID motorSpeedPIDA(&speed1, &output1, &targetSpeed1, KpB1, KiB1, KdB1, DIRECT);
PID motorSpeedPIDB(&speed2, &output2, &targetSpeed1, KpB2, KiB2, KdB1, DIRECT);
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////




// Timing variables for running PID at a set interval (30 times per second)
unsigned long lastUpdateTime = 0;
const int interval = 33; // Time interval in milliseconds (1000ms/30 = ~33ms)

// Function declarations
void manageCountA(); // ISR for Encoder A
void manageCountB(); // ISR for Encoder B
int calculateRPMA(); // Function to calculate RPM for Motor A
int calculateRPMB(); // Function to calculate RPM for Motor B
double calculateAngleA(); // Function to calculate the angle of rotation for Motor A
double calculateAngleB(); // Function to calculate the angle of rotation for Motor B
void controlMotorA(double speed); // Function to control Motor A speed and direction
void controlMotorB(double speed); // Function to control Motor B speed and direction
void controlMotor(double speed, int IN1, int PWM); // General function to control any motor based on speed (negative for reverse, positive for forward)


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
  motorAnglePIDA.SetOutputLimits(-240, 240);  // Set output range
  motorAnglePIDB.SetSampleTime(interval);  // Set PID update interval
  motorAnglePIDB.SetOutputLimits(-240, 240);  // Set output range

  motorSpeedPIDA.SetSampleTime(interval);  // Set PID update interval
  motorSpeedPIDA.SetOutputLimits(-240, 240);  // Set output range
  motorSpeedPIDB.SetSampleTime(interval);  // Set PID update interval
  motorSpeedPIDB.SetOutputLimits(-240, 240);  // Set output range

  motorAnglePIDA.SetMode(AUTOMATIC);  // Set PID mode to automatic
  motorAnglePIDB.SetMode(AUTOMATIC);  // Set PID mode to automatic

}

// Main loop function
void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= interval) {
    lastUpdateTime = currentTime;

    if (mode == 0) {
      // Calculate RPM for Motor A
      angle1 = calculateAngleA();
      motorAnglePIDA.Compute();
      // Calculate RPM for Motor B
      angle2 = calculateAngleB();
      motorAnglePIDB.Compute();
      // Control Motor A and B
      controlMotorA(output1);
      controlMotorB(output2);
    }
    else if (mode == 1) {
      // Calculate RPM for Motor A
      speed1 = calculateRPMA();
      motorSpeedPIDA.Compute();
      // Calculate RPM for Motor B
      speed2 = calculateRPMB();
      motorSpeedPIDB.Compute();
      // Control Motor A and B
      controlMotorA(output1);
      controlMotorB(output2);
    }
  }

  while (Serial.available() > 0) {
    
    chr = Serial.read();  // Read the next character

    // Execute command on carriage return (CR)
    if (chr == 13) {  // ASCII value for CR
      if (arg == 1) argv1[index] = '\0';
      else if (arg == 2) argv2[index] = '\0';
      runCommand();       // Run the parsed command
      resetCommand();     // Reset command variables for next input
    } 
    // Use spaces to delimit arguments
    else if (chr == ' ') {
      if (arg == 0) arg = 1;     // Move to first argument
      else if (arg == 1) {       // Move to second argument
        argv1[index] = '\0';
        arg = 2;
        index = 0;
      }
      continue;
    }
    // Store command and arguments
    else {
      if (arg == 0) {
        cmd = chr;  // Store the command
      } else if (arg == 1) {
        argv1[index++] = chr;  // Store the first argument
      } else if (arg == 2) {
        argv2[index++] = chr;  // Store the second argument
      }
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
int calculateRPMA() {
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

  return static_cast<int>(rpmA);
;
}

// Function to calculate RPM for Motor B
int calculateRPMB() {
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

  return static_cast<int>(rpmA);
}

// Function to calculate the angle of rotation for Motor A
double calculateAngleA() {
  // Calculate angle in degrees (360 degrees per full rotation)
  return (countA)*360 / 375;
}

// Function to calculate the angle of rotation for Motor B
double calculateAngleB() {
  // Calculate angle in degrees (360 degrees per full rotation)
  return ((countB)*360 / 375);
}

// Function to control Motor A speed and direction
void controlMotorA(double speed) {
  controlMotor(speed, motorA_IN1, motorA_PWM);
}

// Function to control Motor B speed and direction
void controlMotorB(double speed) {
  controlMotor(speed, motorB_IN1, motorB_PWM);
}

// General function to control any motor based on speed (negative for reverse, positive for forward)
void controlMotor(double speed, int IN1, int PWM) {
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
