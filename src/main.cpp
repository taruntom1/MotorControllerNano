#include <Arduino.h>
#include <QuickPID.h>

// #define DEBUG
// #define BENCHMARK

// #define ARDUINO_NANO
#define nodeMCU
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////Serial Control Commands////////////////////////////////////////////
// Define command characters for various actions
#define SET_MOTOR_PWM_OFSET 'o' // Set motor PWM offset (0-255) eg: o 100 100
#define MOTOR_ANGLES 'a'        // Set motor angles (degrees) eg: a 100 200
#define READ_ENCODERS 'e'       // Read encoder counts (counts) eg: e
#define READ_SPEEDS 's'         // Read motor speeds (RPM) eg: s
#define MOTOR_SPEEDS 'm'        // Set motor speeds (RPM) eg: m 100 200
#define MOTOR_PWM 'n'           // Set motor PWM (0-255) eg: f 100 200
#define PING 'p'                // Ping the Arduino
#define RESET_ENCODERS 'r'      // Reset encoder counts (counts) eg: r
#define UPDATE_PIDAA 'u'        // Update angle PID values for Motor A (Kp, Ki, Kd) eg: u 10:20:30
#define UPDATE_PIDAB 'v'        // Update angle PID values for Motor B (Kp, Ki, Kd) eg: v 10:20:30
#define UPDATE_PIDSA 'w'        // Update speed PID values for Motor A (Kp, Ki, Kd) eg: w 10:20:30
#define UPDATE_PIDSB 'x'        // Update speed PID values for Motor B (Kp, Ki, Kd) eg: x 10:20:30
#define GET_INP_TAR 'g'         // prints target and input values for specified pid eg: g 1 , g 0 for disabling the printing
#define PRINT_PPM 'z'           // print PPM signal
#define PPM_INTRRUPT 'i'        // enable/disable ppm interrupt
#define PPM_TUNE 't'            // set the ppm output pin
#define PRINT_PID_CONST 'k'     // ppm output pin

//////////////////////////////////////////////////////////////////////////////////////
#ifdef ARDUINO_NANO
// Pin definitions for Encodersk

#define encoderPinA1 2 // Encoder A Channel 1
#define encoderPinA2 4 // Encoder A Channel 2
#define encoderPinB1 3 // Encoder B Channel 1
#define encoderPinB2 5 // Encoder B Channel 2

// Pin definitions for Motor A
const int motorA_IN1 = 7; // Motor A IN1 pin
const int motorA_PWM = 6; // Motor A PWM pin (Enable pin)

// Pin definitions for Motor B
const int motorB_IN1 = 8; // Motor B IN1 pin
const int motorB_PWM = 9; // Motor B PWM pin (Enable pin)
#endif

#ifdef nodeMCU
// Pin definitions for Encoders
#define encoderPinA1 5  // Encoder A Channel 1
#define encoderPinA2 4  // Encoder A Channel 2
#define encoderPinB1 14 // Encoder B Channel 1
#define encoderPinB2 12 // Encoder B Channel 2

// Pin definitions for Motor A
const int motorA_IN1 = 0; // Motor A IN1 pin
const int motorA_PWM = 2; // Motor A PWM pin (Enable pin)

// Pin definitions for Motor B
const int motorB_IN1 = 16; // Motor B IN1 pin
const int motorB_PWM = 15; // Motor B PWM pin (Enable pin)
#endif

// PWM starting points for Motors
uint8_t pwmOffsetA = 0; // PWM value for Motor A
uint8_t pwmOffsetB = 0; // PWM value for Motor B

// Variables to hold encoder counts
volatile long countA = 0; // Encoder count for Motor A
volatile long countB = 0; // Encoder count for Motor B

// Constants for encoder and motor calculations
const float countsPerRotation = 375.0; // 500 encoder counts per motor rotation

// Previous values for RPM calculation
unsigned long prevTimeA = 0; // Previous timestamp for Motor A
long prevCountA = 0;         // Previous encoder count for Motor A
unsigned long prevTimeB = 0; // Previous timestamp for Motor B
long prevCountB = 0;         // Previous encoder count for Motor B

// Timing variables for running PID at a set interval (30 times per second)
unsigned long lastUpdateTime = 0;
const int interval = 33333; // Time interval in microseconds (1000ms/30 = ~33ms)

// Timing variables for running ppm tuning at a set interval (5 times per second)
unsigned long lastUpdateTimePPM = 0;
const int intervalPPM = 200000; // Time interval in microseconds (1000ms/5 = 200ms)
// PID Controller Mode
bool mode; // Mode for PID controller mode 0 for angle PID and 1 for speed PID

// PPM print mode
bool ppmPrint = 0;     // Print PPM signal to serial monitor when true
bool ppminterrupt = 0; // Turns on or off ppm interrupt
// ppm-pid tuner on/off
bool ppmTuner = 0; // Turns on or off ppm tuner

// Mode select for printing feedback and target values
u8 modePrint = 0; // 0 for none, 1 for angle PIDA, 2 for angle PIDB etc

float output1 = 0; // Output for Motor A
float output2 = 0; // Output for Motor B
/////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////Angle PID//////////////////////////////////////////////////////

float angle1 = 0;       // Angle of rotation for Motor A
float targetAngle1 = 0; // Target angle for Motor A

float angle2 = 0;       // Angle of rotation for Motor B
float targetAngle2 = 0; // Target angle for Motor B

// PID constants for motor 1
float KpA1 = 0.7, KiA1 = 0.1, KdA1 = 0.1;
// PID constants for motor 2
float KpA2 = 0.7, KiA2 = 0.1, KdA2 = 0.1;

// PID Controller Object
QuickPID motorAnglePIDA(&angle1, &output1, &targetAngle1, KpA1, KiA1, KdA1, /* OPTIONS */
                        motorAnglePIDA.pMode::pOnError,
                        motorAnglePIDA.dMode::dOnError,
                        motorAnglePIDA.iAwMode::iAwCondition,
                        motorAnglePIDA.Action::direct);
QuickPID motorAnglePIDB(&angle2, &output2, &targetAngle1, KpA2, KiA2, KdA1, /* OPTIONS */
                        motorAnglePIDB.pMode::pOnError,
                        motorAnglePIDB.dMode::dOnError,
                        motorAnglePIDB.iAwMode::iAwCondition,
                        motorAnglePIDB.Action::direct);
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////Speed PID//////////////////////////////////////////////////////

float speed1 = 0;       // Speed of Motor A
float targetSpeed1 = 0; // Target speed for Motor A

float speed2 = 0;       // Speed of Motor B
float targetSpeed2 = 0; // Target speed for Motor B

// PID constants for motor 1
float KpB1 = 1.0, KiB1 = 0.5, KdB1 = 0.1;
// PID constants for motor 2
float KpB2 = 1.0, KiB2 = 0.5, KdB2 = 0.1;

QuickPID motorSpeedPIDA(&speed1, &output1, &targetSpeed1, KpB1, KiB1, KdB1, /* OPTIONS */
                        motorSpeedPIDA.pMode::pOnError,
                        motorSpeedPIDA.dMode::dOnError,
                        motorSpeedPIDA.iAwMode::iAwCondition,
                        motorSpeedPIDA.Action::direct);
QuickPID motorSpeedPIDB(&speed2, &output2, &targetSpeed1, KpB2, KiB2, KdB1, /* OPTIONS */
                        motorSpeedPIDB.pMode::pOnError,
                        motorSpeedPIDB.dMode::dOnError,
                        motorSpeedPIDB.iAwMode::iAwCondition,
                        motorSpeedPIDB.Action::direct);
/////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////PPM Reading//////////////////////////////////////////////////////////
// Pin where the PPM signal is connected
const int PPM_PIN = 13;

// Number of PPM channels
const int NUM_CHANNELS = 8;

// Array to store pulse width (in microseconds) for each channel
volatile int ppmChannels[NUM_CHANNELS] = {0};

// Variable to track current channel
volatile int currentChannel = 0;

// Variable to store the last pulse time
volatile unsigned long lastPulseTime = 0;

// Time threshold for frame sync pulse (in microseconds)
const int SYNC_GAP = 3000; // 3ms gap considered as sync

// Interrupt Service Routine (ISR) to handle the PPM signal
IRAM_ATTR void ppmISR()
{
  // Measure the time since the last pulse

  unsigned long currenttime = micros();
  unsigned long pulseWidth = currenttime - lastPulseTime;
  lastPulseTime = currenttime;

  // Check if the pulse width indicates a sync pulse
  if (pulseWidth >= SYNC_GAP)
  {
    // Sync pulse detected, reset to the first channel
    currentChannel = 0;
  }
  else
  {
    // If valid channel, store the pulse width
    if (currentChannel < NUM_CHANNELS)
    {
      ppmChannels[currentChannel] = pulseWidth;
      currentChannel++;
    }
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
// Function declarations
#ifdef ARDUINO_NANO
void manageCountA(); // ISR for Encoder A
void manageCountB(); // ISR for Encoder B
#endif
#ifdef nodeMCU
IRAM_ATTR void manageCountA(); // ISR for Encoder A
IRAM_ATTR void manageCountB(); // ISR for Encoder B
#endif

int calculateRPMA();                               // Function to calculate RPM for Motor A
int calculateRPMB();                               // Function to calculate RPM for Motor B
double calculateAngleA();                          // Function to calculate the angle of rotation for Motor A
double calculateAngleB();                          // Function to calculate the angle of rotation for Motor B
void controlMotorA(double speed);                  // Function to control Motor A speed and direction
void controlMotorB(double speed);                  // Function to control Motor B speed and direction
void controlMotor(double speed, int IN1, int PWM); // General function to control any motor based on speed (negative for reverse, positive for forward)
void ppm_pid_tuner();                              // Function to control PID tuners using RF remote
void PrintPIDValues();                              // Function to print PID input, target, output values to Serial Monitor for tuning
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////Serial Controls//////////////////////////////////////////////////////

// Variables to handle serial command parsing
int arg = 0;
int index1 = 0;
char chr;       // Holds the input character
char cmd;       // Holds the current command
char argv1[16]; // Holds the first argument as a string
char argv2[16]; // Holds the second argument as a string
long arg1;      // First argument converted to integer
long arg2;      // Second argument converted to integer

// Function to clear command parameters
void resetCommand()
{
  cmd = '\0';
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index1 = 0;
}

// Function to run the appropriate command based on serial input
void runCommand()
{
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];    // Holds parsed PID values
  arg1 = atoi(argv1); // Convert argv1 to integer
  arg2 = atoi(argv2); // Convert argv2 to integer

  switch (cmd)
  {
  case PING:
    Serial.println(arg1); // Respond with argument 1
    break;

  case READ_ENCODERS:
    Serial.print(countA); // Respond with encoder count A
    Serial.print(" ");
    Serial.println(countB); // Respond with encoder count B
    break;

  case SET_MOTOR_PWM_OFSET:
    pwmOffsetA = arg1;                                                   // Set PWM offset for Motor A
    pwmOffsetB = arg2;                                                   // Set PWM offset for Motor B
    motorAnglePIDA.SetOutputLimits(-255 + pwmOffsetA, 255 - pwmOffsetA); // Set output range for Motor A
    motorAnglePIDB.SetOutputLimits(-255 + pwmOffsetB, 255 - pwmOffsetB); // Set output range for Motor B
    motorSpeedPIDA.SetOutputLimits(-255 + pwmOffsetA, 255 - pwmOffsetA); // Set output range for Motor A
    motorSpeedPIDB.SetOutputLimits(-255 + pwmOffsetB, 255 - pwmOffsetB); // Set output range for Motor B
#ifdef DEBUG
    Serial.print("PWM Offset A: ");
    Serial.println(pwmOffsetA);
    Serial.print("PWM Offset B: ");
    Serial.println(pwmOffsetB);
#endif
    break;
  case READ_SPEEDS:
    Serial.print(calculateRPMA()); // Respond with RPM of motor A
    Serial.print(" ");
    Serial.println(calculateRPMB()); // Respond with RPM of motor B
    break;
  case RESET_ENCODERS:
    countA = 0; // Reset encoder count A to zero
    countB = 0; // Reset encoder count B to zero
    Serial.println("Encoders reseted");
    break;

  case MOTOR_SPEEDS:

    motorAnglePIDA.SetMode(motorAnglePIDA.Control::manual); // Set PID mode of angle PID A to manual
    motorAnglePIDB.SetMode(motorAnglePIDB.Control::manual); // Set PID mode of angle PID B to manual
    motorSpeedPIDA.SetMode(motorSpeedPIDA.Control::timer);  // Set PID mode of speed PID A to automatic
    motorSpeedPIDB.SetMode(motorSpeedPIDB.Control::timer);  // Set PID mode of speed PID B to automatic

    mode = 1; // Set mode flag to 1 (manual)

    targetSpeed1 = arg1; // Set target speed of motor 1 to argument 1
    targetSpeed2 = arg2; // Set target speed of motor 2 to argument 2

#ifdef DEBUG
    Serial.print("Speed 1: ");
    Serial.println(arg1);
    Serial.print("Speed 2: ");
    Serial.println(arg2);
#endif

    break;

  case MOTOR_ANGLES:

    motorSpeedPIDA.SetMode(motorSpeedPIDA.Control::manual); // Set PID mode of speed PID A to manual
    motorSpeedPIDB.SetMode(motorSpeedPIDB.Control::manual); // Set PID mode of speed PID B to manual
    motorAnglePIDA.SetMode(motorAnglePIDA.Control::timer);  // Set PID mode of angle PID A to automatic
    motorAnglePIDB.SetMode(motorAnglePIDB.Control::timer);  // Set PID mode of angle PID B to automatic

    mode = 0; // Set mode flag to 0 (manual)

    targetAngle1 = calculateAngleA() + arg1; // Set target angle of motor 1 to argument 1 + current angle
    targetAngle2 = calculateAngleB() + arg2; // Set target angle of motor 2 to argument 2 + current angle

#ifdef DEBUG
    Serial.print("Speed 1: ");
    Serial.println(arg1);
    Serial.print("Speed 2: ");
    Serial.println(arg2);
#endif

    break;

  case MOTOR_PWM:
    controlMotorA(arg1); // Set PWM of motor 1 to argument 1
    controlMotorB(arg2); // Set PWM of motor 2 to argument 2
    break;
  case UPDATE_PIDAA:
    // Parse PID values for Motor A
    while ((str = strtok_r(p, ":", &p)) != NULL)
    {
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
    Serial.print("Kp1: ");
    Serial.println(KpA1);
    Serial.print("Ki1: ");
    Serial.println(KiA1);
    Serial.print("Kd1: ");
    Serial.println(KdA1);
#endif
    break;

  case UPDATE_PIDAB:
    // Parse PID values for Motor B
    while ((str = strtok_r(p, ":", &p)) != NULL)
    {
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
    Serial.print("Kp2: ");
    Serial.println(KpA2);
    Serial.print("Ki2: ");
    Serial.println(KiA2);
    Serial.print("Kd2: ");
    Serial.println(KdA2);
#endif
    break;

  case UPDATE_PIDSA:
    // Parse PID values for Motor A
    while ((str = strtok_r(p, ":", &p)) != NULL)
    {
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
    Serial.print("Kp1: ");
    Serial.println(KpB1);
    Serial.print("Ki1: ");
    Serial.println(KiB1);
    Serial.print("Kd1: ");
    Serial.println(KdB1);
#endif
    break;

  case UPDATE_PIDSB:
    // Parse PID values for Motor A
    while ((str = strtok_r(p, ":", &p)) != NULL)
    {
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
    Serial.print("Kp1: ");
    Serial.println(KpB2);
    Serial.print("Ki1: ");
    Serial.println(KiB2);
    Serial.print("Kd1: ");
    Serial.println(KdB2);
#endif
    break;

  case GET_INP_TAR:
#ifdef DEBUG
    Serial.print("Starting to print input and target")
#endif
        modePrint = arg1;
    break;

  case PRINT_PPM:
    ppmPrint = !ppmPrint;
    if (ppmPrint)
    {
      attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, FALLING);
      ppminterrupt = true;
    }
    else
    {
      detachInterrupt(digitalPinToInterrupt(PPM_PIN));
    }
    break;
  case PPM_INTRRUPT:
    ppminterrupt = !ppminterrupt;
    if (ppminterrupt)
    {
      attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, FALLING);
    }
    else
    {
      detachInterrupt(digitalPinToInterrupt(PPM_PIN));
    }
    break;
  case PRINT_PID_CONST:
    Serial.print("AnglePID 1 kp: ");
    Serial.print(motorAnglePIDA.GetKp());
    Serial.print("\tki: ");
    Serial.print(motorAnglePIDA.GetKi());
    Serial.print("\tkd: ");
    Serial.println(motorAnglePIDA.GetKd());
    Serial.print("AnglePID 2 kp:");
    Serial.print(motorAnglePIDB.GetKp());
    Serial.print("\tki: ");
    Serial.print(motorAnglePIDB.GetKi());
    Serial.print("\tkd: ");
    Serial.println(motorAnglePIDB.GetKd());
    Serial.print("SpeedPID 1 kp: ");
    Serial.print(motorSpeedPIDA.GetKp());
    Serial.print("\tki: ");
    Serial.print(motorSpeedPIDA.GetKi());
    Serial.print("\tkd: ");
    Serial.println(motorSpeedPIDA.GetKd());
    Serial.print("SpeedPID 2 kp:");
    Serial.print(motorSpeedPIDB.GetKp());
    Serial.print("\tki: ");
    Serial.print(motorSpeedPIDB.GetKi());
    Serial.print("\tkd: ");
    Serial.println(motorSpeedPIDB.GetKd());
    break;
  case PPM_TUNE:
    ppmTuner = !ppmTuner;
    if (ppmTuner)
    {
      ppminterrupt = true;
      Serial.println("ppmTuner on");
    }
    else
    {
      Serial.println("ppmTuner off");
    }
    break;
  default:
    Serial.println("Invalid command");
    break;
  }
}

////////////////////////////////////////////////////

#ifdef BENCHMARK
// Function to measure the loop frequency
unsigned long previousMillis = 0;
unsigned long loopCounter = 0;

void benchmarkLoopFrequency()
{
  unsigned long currentMillis = millis();

  // Count the number of times the loop runs
  loopCounter++;

  // Check if one second (1000 milliseconds) has passed
  if (currentMillis - previousMillis >= 1000)
  {
    // Print the number of loops in the past second
    Serial.print("Loops per second: ");
    Serial.println(loopCounter);

    // Reset the counter for the next second
    loopCounter = 0;

    // Update the previousMillis for the next timing interval
    previousMillis = currentMillis;
  }
}

#endif

// Setup function to initialize motors, encoders, and serial communication
void setup()
{
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

  // Set up the PPM input pin
  pinMode(PPM_PIN, INPUT);

  // Initial settings for PID
  motorAnglePIDA.SetSampleTimeUs(interval);                            // Set PID update interval
  motorAnglePIDA.SetOutputLimits(-255 + pwmOffsetA, 255 - pwmOffsetA); // Set output range
  motorAnglePIDB.SetSampleTimeUs(interval);                            // Set PID update interval
  motorAnglePIDB.SetOutputLimits(-255 + pwmOffsetB, 255 - pwmOffsetB); // Set output range

  motorSpeedPIDA.SetSampleTimeUs(interval);                            // Set PID update interval
  motorSpeedPIDA.SetOutputLimits(-255 + pwmOffsetA, 255 - pwmOffsetA); // Set output range
  motorSpeedPIDB.SetSampleTimeUs(interval);                            // Set PID update interval
  motorSpeedPIDB.SetOutputLimits(-255 + pwmOffsetB, 255 - pwmOffsetB); // Set output range

  motorAnglePIDA.SetMode(motorAnglePIDA.Control::timer);  // Set PID mode to automatic
  motorAnglePIDB.SetMode(motorAnglePIDB.Control::timer);  // Set PID mode to automatic
  motorSpeedPIDA.SetMode(motorSpeedPIDA.Control::manual); // Set PID mode to manual
  motorSpeedPIDB.SetMode(motorSpeedPIDB.Control::manual); // Set PID mode to manual
}

// Main loop function
void loop()
{

#ifdef BENCHMARK
  // benchmarks loop frequency
  benchmarkLoopFrequency();
#endif

  unsigned long currentTime = micros();
  if (currentTime - lastUpdateTime >= interval)
  {
    lastUpdateTime = currentTime;

    // For printing PID values to serial for tuning
    if (mode != 0)
    {
      PrintPIDValues();
    }

    if (ppmPrint)
    {
      Serial.print("Channels: ");
      for (int i = 0; i < NUM_CHANNELS; i++)
      {
        Serial.print(ppmChannels[i]);
        Serial.print("\t");
      }
      Serial.println();
    }

    // works as angle PID
    if (mode == 0)
    {
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
    // works as speed PID
    else if (mode == 1)
    {
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
  if (currentTime - lastUpdateTimePPM >= intervalPPM)
  {
    lastUpdateTimePPM = currentTime;
    // tuning through ppm
    if (ppmTuner)
    {
      ppm_pid_tuner();
    }
  }
  while (Serial.available() > 0)
  {

    chr = Serial.read(); // Read the next character

    // Execute command on carriage return (CR)
    if (chr == 13)
    { // ASCII value for CR
      if (arg == 1)
        argv1[index1] = '\0';
      else if (arg == 2)
        argv2[index1] = '\0';
      runCommand();   // Run the parsed command
      resetCommand(); // Reset command variables for next input
    }
    // Use spaces to delimit arguments
    else if (chr == ' ')
    {
      if (arg == 0)
        arg = 1; // Move to first argument
      else if (arg == 1)
      { // Move to second argument
        argv1[index1] = '\0';
        arg = 2;
        index1 = 0;
      }
      continue;
    }
    // Store command and arguments
    else
    {
      if (arg == 0)
      {
        cmd = chr; // Store the command
      }
      else if (arg == 1)
      {
        argv1[index1++] = chr; // Store the first argument
      }
      else if (arg == 2)
      {
        argv2[index1++] = chr; // Store the second argument
      }
    }
  }
}

#ifdef ARDUINO_NANO

// Interrupt service routine (ISR) for Encoder A
void manageCountA()
{
  // Check the state of encoderPinA2 to determine direction
  if (digitalRead(encoderPinA2) == 0)
  {
    countA--; // Forward rotation
  }
  else
  {
    countA++; // Reverse rotation
  }
}

// Interrupt service routine (ISR) for Encoder B
void manageCountB()
{
  // Check the state of encoderPinB2 to determine direction
  if (digitalRead(encoderPinB2) == 0)
  {
    countB--; // Forward rotation
  }
  else
  {
    countB++; // Reverse rotation
  }
}
#endif

#ifdef nodeMCU

// Interrupt service routine (ISR) for Encoder A
IRAM_ATTR void manageCountA()
{
  // Check the state of encoderPinA2 to determine direction
  if (digitalRead(encoderPinA2) == 0)
  {
    countA--; // Forward rotation
  }
  else
  {
    countA++; // Reverse rotation
  }
}

// Interrupt service routine (ISR) for Encoder B
IRAM_ATTR void manageCountB()
{
  // Check the state of encoderPinB2 to determine direction
  if (digitalRead(encoderPinB2) == 0)
  {
    countB--; // Forward rotation
  }
  else
  {
    countB++; // Reverse rotation
  }
}

#endif

// Function to calculate RPM for Motor A
int calculateRPMA()
{
  unsigned long currentTime = millis(); // Get current time in milliseconds
  long currentCountA = countA;          // Get current encoder count for Motor A

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
int calculateRPMB()
{
  unsigned long currentTime = millis(); // Get current time in milliseconds
  long currentCountB = countB;          // Get current encoder count for Motor B

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
double calculateAngleA()
{
  // Calculate angle in degrees (360 degrees per full rotation)
  return (countA) * 360 / 375;
}

// Function to calculate the angle of rotation for Motor B
double calculateAngleB()
{
  // Calculate angle in degrees (360 degrees per full rotation)
  return ((countB) * 360 / 375);
}

// Function to control Motor A speed and direction
void controlMotorA(double speed)
{
  if (speed > 0)
  {
    // Forward direction
    digitalWrite(motorA_IN1, HIGH);
    analogWrite(motorA_PWM, speed + pwmOffsetA); // Set speed (0-255)
  }
  else if (speed < 0)
  {
    // Reverse direction
    digitalWrite(motorA_IN1, LOW);
    analogWrite(motorA_PWM, -speed + pwmOffsetA); // Set speed (0-255), negate for reverse
  }
  else
  {
    // Stop the motor
    digitalWrite(motorA_IN1, LOW);
    analogWrite(motorA_PWM, 0);
  }
}

// Function to control Motor B speed and direction
void controlMotorB(double speed)
{
  if (speed > 0)
  {
    // Forward direction
    digitalWrite(motorB_IN1, HIGH);
    analogWrite(motorB_PWM, speed + pwmOffsetB); // Set speed (0-255)
  }
  else if (speed < 0)
  {
    // Reverse direction
    digitalWrite(motorB_IN1, LOW);
    analogWrite(motorB_PWM, -speed + pwmOffsetB); // Set speed (0-255), negate for reverse
  }
  else
  {
    // Stop the motor
    digitalWrite(motorB_IN1, LOW);
    analogWrite(motorB_PWM, 0);
  }
}

void PrintPIDValues()
{
  switch (modePrint)
  {
  case 0:
    // Do nothing
    break;

  case 1:
    Serial.print(angle1);
    Serial.print(" ");
    Serial.print(targetAngle1);
    Serial.print(" ");
    Serial.println(output1);
    break;

  case 2:
    Serial.print(angle2);
    Serial.print(" ");
    Serial.print(targetAngle2);
    Serial.print(" ");
    Serial.println(output2);
    break;

  case 3:
    Serial.print(speed1);
    Serial.print(" ");
    Serial.print(targetSpeed1);
    Serial.print(" ");
    Serial.println(output1);
    break;

  case 4:
    Serial.print(speed2);
    Serial.print(" ");
    Serial.print(targetSpeed2);
    Serial.print(" ");
    Serial.println(output2);
    break;
  default:
    Serial.println("Invalid argument");
  }
}

void ppm_pid_tuner()
{
  u8 control1 = 0;
  u8 control2 = 0;

  float val = ((float)(ppmChannels[2] - 1000)) / 1000;
  if (ppmChannels[4] < 1150)
  {
    control1 = 0;
  }
  else if (ppmChannels[4] < 1310)
  {
    control1 = 1;
  }
  else if (ppmChannels[4] < 1480)
  {
    control1 = 2;
  }
  else if (ppmChannels[4] < 1650)
  {
    control1 = 3;
  }
  else if (ppmChannels[4] < 1820)
  {
    control1 = 4;
  }
  else
  {
    control1 = 5;
  }

  if (ppmChannels[5] < 1300)
  {
    control2 = 0;
  }
  else if (ppmChannels[5] < 1600)
  {
    control2 = 1;
  }
  else
  {
    control2 = 2;
  }

  switch (control1)
  {
  case 0:
    switch (control2)
    {
    case 0:
      KpA1 = val;
      motorAnglePIDA.SetTunings(KpA1, KiA1, KdA1);
      break;
    case 1:
      KiA1 = val;
      motorAnglePIDA.SetTunings(KpA1, KiA1, KdA1);
      break;
    case 2:
      KdA1 = val;
      motorAnglePIDA.SetTunings(KpA1, KiA1, KdA1);
      break;
    default:
      break;
    }
    break;
  case 1:
    switch (control2)
    {
    case 0:
      KpA2 = val;
      motorAnglePIDB.SetTunings(KpA2, KiA2, KdA2);
      break;
    case 1:
      KiA2 = val;
      motorAnglePIDB.SetTunings(KpA2, KiA2, KdA2);
      break;
    case 2:
      KdA2 = val;
      motorAnglePIDB.SetTunings(KpA2, KiA2, KdA2);
      break;
    default:
      break;
    }
    break;
  case 2:
    switch (control2)
    {
    case 0:
      KpB1 = val;
      motorSpeedPIDA.SetTunings(KpB1, KiB1, KdB1);
      break;
    case 1:
      KiB1 = val;
      motorSpeedPIDA.SetTunings(KpB1, KiB1, KdB1);
      break;
    case 2:
      KdB1 = val;
      motorSpeedPIDA.SetTunings(KpB1, KiB1, KdB1);
      break;
    default:
      break;
    }
    break;
  case 3:
  {
    switch (control2)
    {
    case 0:
      KpB2 = val;
      motorSpeedPIDB.SetTunings(KpB2, KiB2, KdB2);
      break;
    case 1:
      KiB2 = val;
      motorSpeedPIDB.SetTunings(KpB2, KiB2, KdB2);
      break;
    case 2:
      KdB2 = val;
      motorSpeedPIDB.SetTunings(KpB2, KiB2, KdB2);
      break;
    default:
      break;
    }
    break;
  }
  default:
    break;
  }
}
