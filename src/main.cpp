#include <Arduino.h>
#include <PID_v1.h>

//#define DEBUG
#define ARDUINO_NANO
//#define nodeMCU
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////Serial Control Commands////////////////////////////////////////////
// Define command characters for various actions
#define SET_MOTOR_PWM_OFSET 'o' // Set motor PWM offset (0-255) eg: o 100 100
#define MOTOR_ANGLES   'a' // Set motor angles (degrees) eg: a 100 200
#define READ_ENCODERS  'e' // Read encoder counts (counts) eg: e
#define READ_SPEEDS   's' // Read motor speeds (RPM) eg: s
#define MOTOR_SPEEDS   'm' // Set motor speeds (RPM) eg: m 100 200
#define MOTOR_PWM      'n' // Set motor PWM (0-255) eg: f 100 200
#define PING           'p' // Ping the Arduino
#define RESET_ENCODERS 'r' // Reset encoder counts (counts) eg: r
#define UPDATE_PIDAA    'u' // Update angle PID values for Motor A (Kp, Ki, Kd) eg: u 10:20:30
#define UPDATE_PIDAB    'v' // Update angle PID values for Motor B (Kp, Ki, Kd) eg: v 10:20:30
#define UPDATE_PIDSA    'w' // Update speed PID values for Motor A (Kp, Ki, Kd) eg: w 10:20:30
#define UPDATE_PIDSB    'x' // Update speed PID values for Motor B (Kp, Ki, Kd) eg: x 10:20:30

//////////////////////////////////////////////////////////////////////////////////////
#ifdef ARDUINO_NANO
// Pin definitions for Encoders
#define encoderPinA1 2  // Encoder A Channel 1
#define encoderPinA2 4  // Encoder A Channel 2
#define encoderPinB1 3  // Encoder B Channel 1
#define encoderPinB2 5  // Encoder B Channel 2

// Pin definitions for Motor A
const int motorA_IN1 = 7;  // Motor A IN1 pin
const int motorA_PWM = 6;  // Motor A PWM pin (Enable pin)

// Pin definitions for Motor B
const int motorB_IN1 = 8;  // Motor B IN1 pin
const int motorB_PWM = 9;  // Motor B PWM pin (Enable pin)
#endif

#ifdef nodeMCU
// Pin definitions for Encoders
#define encoderPinA1 5  // Encoder A Channel 1
#define encoderPinA2 4  // Encoder A Channel 2
#define encoderPinB1 14  // Encoder B Channel 1
#define encoderPinB2 12  // Encoder B Channel 2

// Pin definitions for Motor A
const int motorA_IN1 = 0;  // Motor A IN1 pin
const int motorA_PWM = 2;  // Motor A PWM pin (Enable pin)

// Pin definitions for Motor B
const int motorB_IN1 = 15;  // Motor B IN1 pin
const int motorB_PWM = 13;  // Motor B PWM pin (Enable pin)
#endif

// PWM starting points for Motors
uint8_t pwmOffsetA = 0; // PWM value for Motor A
uint8_t pwmOffsetB = 0; // PWM value for Motor B

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
double KpA1 = 0.7, KiA1 = 0.1, KdA1 = 0.1;
// PID constants for motor 2
double KpA2 = 0.7, KiA2 = 0.1, KdA2 = 0.1;

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

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////Serial Controls//////////////////////////////////////////////////////

// Variables to handle serial command parsing
int arg = 0;
int index1 = 0;
char chr;            // Holds the input character
char cmd;            // Holds the current command
char argv1[16];      // Holds the first argument as a string
char argv2[16];      // Holds the second argument as a string
long arg1;           // First argument converted to integer
long arg2;           // Second argument converted to integer

// Function declarations
#ifdef ARDUINO_NANO
void manageCountA(); // ISR for Encoder A
void manageCountB(); // ISR for Encoder B
#endif
#ifdef nodeMCU
ICACHE_RAM_ATTR void manageCountA(); // ISR for Encoder A
ICACHE_RAM_ATTR void manageCountB(); // ISR for Encoder B
#endif

int calculateRPMA(); // Function to calculate RPM for Motor A
int calculateRPMB(); // Function to calculate RPM for Motor B
double calculateAngleA(); // Function to calculate the angle of rotation for Motor A
double calculateAngleB(); // Function to calculate the angle of rotation for Motor B
void controlMotorA(double speed); // Function to control Motor A speed and direction
void controlMotorB(double speed); // Function to control Motor B speed and direction
void controlMotor(double speed, int IN1, int PWM); // General function to control any motor based on speed (negative for reverse, positive for forward)

// Function to clear command parameters
void resetCommand() {
  cmd = '\0';
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index1 = 0;
}

// Timing variables for running PID at a set interval (30 times per second)
unsigned long lastUpdateTime = 0;
const int interval = 33; // Time interval in milliseconds (1000ms/30 = ~33ms)




// Function to run the appropriate command based on serial input
void runCommand() {
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
      Serial.print(countA);    // Respond with encoder count A
      Serial.print(" ");
      Serial.println(countB);  // Respond with encoder count B
      break;
    
    case SET_MOTOR_PWM_OFSET:
      pwmOffsetA = arg1;  // Set PWM offset for Motor A
      pwmOffsetB = arg2;  // Set PWM offset for Motor B
      motorAnglePIDA.SetOutputLimits(-255 + pwmOffsetA, 255 - pwmOffsetA);  // Set output range for Motor A
      motorAnglePIDB.SetOutputLimits(-255 + pwmOffsetB, 255 - pwmOffsetB);  // Set output range for Motor B
      motorSpeedPIDA.SetOutputLimits(-255 + pwmOffsetA, 255 - pwmOffsetA);  // Set output range for Motor A
      motorSpeedPIDB.SetOutputLimits(-255 + pwmOffsetB, 255 - pwmOffsetB);  // Set output range for Motor B
      #ifdef DEBUG
      Serial.print("PWM Offset A: ");
      Serial.println(pwmOffsetA);
      Serial.print("PWM Offset B: ");
      Serial.println(pwmOffsetB);
      #endif
      break;
    case READ_SPEEDS:
      Serial.print(calculateRPMA());    // Respond with RPM of motor A
      Serial.print(" ");
      Serial.println(calculateRPMB());  // Respond with RPM of motor B
      break;
    case RESET_ENCODERS:
      countA = 0;     // Reset encoder count A to zero
      countB = 0;     // Reset encoder count B to zero
      Serial.println("Encoders reseted");
      break;
    
    case MOTOR_SPEEDS:
      
      motorAnglePIDA.SetMode(MANUAL);  // Set PID mode of angle PID A to manual
      motorAnglePIDB.SetMode(MANUAL);  // Set PID mode of angle PID B to manual
      motorSpeedPIDA.SetMode(AUTOMATIC);  // Set PID mode of speed PID A to automatic
      motorSpeedPIDB.SetMode(AUTOMATIC);  // Set PID mode of speed PID B to automatic

      mode = 1;   // Set mode flag to 1 (manual)
      
      targetSpeed1 = arg1;    // Set target speed of motor 1 to argument 1
      targetSpeed2 = arg2;    // Set target speed of motor 2 to argument 2

      #ifdef DEBUG
      Serial.print("Speed 1: ");
      Serial.println(arg1);
      Serial.print("Speed 2: ");
      Serial.println(arg2);
      #endif

      break;
    
    case MOTOR_ANGLES:

      motorSpeedPIDA.SetMode(MANUAL);  // Set PID mode of speed PID A to manual
      motorSpeedPIDB.SetMode(MANUAL);  // Set PID mode of speed PID B to manual
      motorAnglePIDA.SetMode(AUTOMATIC);  // Set PID mode of angle PID A to automatic
      motorAnglePIDB.SetMode(AUTOMATIC);  // Set PID mode of angle PID B to automatic

      

      mode = 0;   // Set mode flag to 0 (manual)

      targetAngle1 = calculateAngleA() + arg1;    // Set target angle of motor 1 to argument 1 + current angle
      targetAngle2 = calculateAngleB() + arg2;    // Set target angle of motor 2 to argument 2 + current angle

      #ifdef DEBUG
      Serial.print("Speed 1: ");
      Serial.println(arg1);
      Serial.print("Speed 2: ");
      Serial.println(arg2);
      #endif

      break;

    case MOTOR_PWM:
      controlMotorA(arg1);    // Set PWM of motor 1 to argument 1
      controlMotorB(arg2);    // Set PWM of motor 2 to argument 2
      break;
    case UPDATE_PIDAA:
      // Parse PID values for Motor A
      while ((str = strtok_r(p, ":", &p)) != NULL) {
        pid_args[i] = atoi(str);
        i++;
      }
      KpA1 = pid_args[0];
      KiA1 = pid_args[1];
      KdA1 = pid_args[2];
      // Set PID values for motor A
      motorAnglePIDA.SetTunings(KpA1, KiA1, KdA1);

      #ifdef DEBUG
      Serial.println("PID A updated");
      Serial.print("Kp1: "); Serial.println(KpA1);
      Serial.print("Ki1: "); Serial.println(KiA1);
      Serial.print("Kd1: "); Serial.println(KdA1);
      #endif
      break;
    
    case UPDATE_PIDAB:
      // Parse PID values for Motor B
      while ((str = strtok_r(p, ":", &p)) != NULL) {
        pid_args[i] = atoi(str);
        i++;
      }
      KpA2 = pid_args[0];
      KiA2 = pid_args[1];
      KdA2 = pid_args[2];
      // Set PID values for motor A
      motorSpeedPIDB.SetTunings(KpA2, KiA2, KdA2);
      #ifdef DEBUG
      Serial.println("PID B updated");
      Serial.print("Kp2: "); Serial.println(KpA2);
      Serial.print("Ki2: "); Serial.println(KiA2);
      Serial.print("Kd2: "); Serial.println(KdA2);
      #endif
      break;

    case UPDATE_PIDSA:
      // Parse PID values for Motor A
      while ((str = strtok_r(p, ":", &p)) != NULL) {
        pid_args[i] = atoi(str);
        i++;
      }
      KpB1 = pid_args[0];
      KiB1 = pid_args[1];
      KdB1 = pid_args[2];
      // Set PID values for motor B
      motorSpeedPIDA.SetTunings(KpB1, KiB1, KdB1);
      #ifdef DEBUG
      Serial.println("PID A updated");
      Serial.print("Kp1: "); Serial.println(KpB1);
      Serial.print("Ki1: "); Serial.println(KiB1);
      Serial.print("Kd1: "); Serial.println(KdB1);
      #endif
      break;

    case UPDATE_PIDSB:
      // Parse PID values for Motor A
      while ((str = strtok_r(p, ":", &p)) != NULL) {
        pid_args[i] = atoi(str);
        i++;
      }
      KpB2 = pid_args[0];
      KiB2 = pid_args[1];
      KdB2 = pid_args[2];
      // Set PID values for motor B
      motorSpeedPIDB.SetTunings(KpB2, KiB2, KdB2);

      #ifdef DEBUG
      Serial.println("PID A updated");
      Serial.print("Kp1: "); Serial.println(KpB2);
      Serial.print("Ki1: "); Serial.println(KiB2);
      Serial.print("Kd1: "); Serial.println(KdB2);
      #endif
      break;

    
    default:
      Serial.println("Invalid command");
      break;
  }
}

////////////////////////////////////////////////////




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


  // Initial settings for PID
  motorAnglePIDA.SetSampleTime(interval);  // Set PID update interval
  motorAnglePIDA.SetOutputLimits(-255 + pwmOffsetA, 255 - pwmOffsetA);   // Set output range
  motorAnglePIDB.SetSampleTime(interval);  // Set PID update interval
  motorAnglePIDB.SetOutputLimits(-255 + pwmOffsetB, 255 - pwmOffsetB);  // Set output range

  motorSpeedPIDA.SetSampleTime(interval);  // Set PID update interval
  motorSpeedPIDA.SetOutputLimits(-255 + pwmOffsetA, 255 - pwmOffsetA);  // Set output range
  motorSpeedPIDB.SetSampleTime(interval);  // Set PID update interval
  motorSpeedPIDB.SetOutputLimits(-255 + pwmOffsetB, 255 - pwmOffsetB);  // Set output range

  motorAnglePIDA.SetMode(AUTOMATIC);  // Set PID mode to automatic
  motorAnglePIDB.SetMode(AUTOMATIC);  // Set PID mode to automatic
  motorSpeedPIDA.SetMode(MANUAL);  // Set PID mode to manual
  motorSpeedPIDB.SetMode(MANUAL);  // Set PID mode to manual

}

// Main loop function
void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= interval) {
    lastUpdateTime = currentTime;

    if (mode == 0) {
      // Calculate Angle for Motor A
      angle1 = calculateAngleA();
      motorAnglePIDA.Compute();
      // Calculate Angle for Motor B
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
      if (arg == 1) argv1[index1] = '\0';
      else if (arg == 2) argv2[index1] = '\0';
      runCommand();       // Run the parsed command
      resetCommand();     // Reset command variables for next input
    } 
    // Use spaces to delimit arguments
    else if (chr == ' ') {
      if (arg == 0) arg = 1;     // Move to first argument
      else if (arg == 1) {       // Move to second argument
        argv1[index1] = '\0';
        arg = 2;
        index1 = 0;
      }
      continue;
    }
    // Store command and arguments
    else {
      if (arg == 0) {
        cmd = chr;  // Store the command
      } else if (arg == 1) {
        argv1[index1++] = chr;  // Store the first argument
      } else if (arg == 2) {
        argv2[index1++] = chr;  // Store the second argument
      }
    }
  }

}

#ifdef ARDUINO_NANO

// Interrupt service routine (ISR) for Encoder A
void manageCountA() {
  // Check the state of encoderPinA2 to determine direction
  if (digitalRead(encoderPinA2) == 0) {
    countA--;  // Forward rotation
  } else {
    countA++;  // Reverse rotation
  }
}

// Interrupt service routine (ISR) for Encoder B
void manageCountB() {
  // Check the state of encoderPinB2 to determine direction
  if (digitalRead(encoderPinB2) == 0) {
    countB--;  // Forward rotation
  } else {
    countB++;  // Reverse rotation
  }
}
#endif

#ifdef nodeMCU

// Interrupt service routine (ISR) for Encoder A
ICACHE_RAM_ATTR void manageCountA() {
  // Check the state of encoderPinA2 to determine direction
  if (digitalRead(encoderPinA2) == 0) {
    countA--;  // Forward rotation
  } else {
    countA++;  // Reverse rotation
  }
}

// Interrupt service routine (ISR) for Encoder B
ICACHE_RAM_ATTR void manageCountB() {
  // Check the state of encoderPinB2 to determine direction
  if (digitalRead(encoderPinB2) == 0) {
    countB--;  // Forward rotation
  } else {
    countB++;  // Reverse rotation
  }
}

#endif

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

  return static_cast<int>(rpmB);
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
  if (speed > 0) {
    // Forward direction
    digitalWrite(motorA_IN1, HIGH);
    analogWrite(motorA_PWM, speed + pwmOffsetA);  // Set speed (0-255)
  } else if (speed < 0) {
    // Reverse direction
    digitalWrite(motorA_IN1, LOW);
    analogWrite(motorA_PWM, -speed + pwmOffsetA);  // Set speed (0-255), negate for reverse
  } else {
    // Stop the motor
    digitalWrite(motorA_IN1, LOW);
    analogWrite(motorA_PWM, 0);
  }
}

// Function to control Motor B speed and direction
void controlMotorB(double speed) {
  if (speed > 0) {
    // Forward direction
    digitalWrite(motorB_IN1, HIGH);
    analogWrite(motorB_PWM, speed + pwmOffsetB);  // Set speed (0-255)
  } else if (speed < 0) {
    // Reverse direction
    digitalWrite(motorB_IN1, LOW);
    analogWrite(motorB_PWM, -speed + pwmOffsetB);  // Set speed (0-255), negate for reverse
  } else {
    // Stop the motor
    digitalWrite(motorB_IN1, LOW);
    analogWrite(motorB_PWM, 0);
  }
}

