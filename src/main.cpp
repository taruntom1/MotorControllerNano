#include <Arduino.h>
#include <QuickPID.h>

// #define DEBUG
//#define BENCHMARK

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

///////////////////////////////////////////////////////////////////////////////////////////////
// Pin definitions

#ifdef ARDUINO_NANO
  // Nano pin definitions for Encoders and Motors
  #define encoderPinA1 2
  #define encoderPinA2 4
  #define encoderPinB1 3
  #define encoderPinB2 5
  const int motorA_DIR = 7;
  const int motorA_PWM = 6;
  const int motorB_DIR = 8;
  const int motorB_PWM = 9;
#endif

#ifdef nodeMCU
  // NodeMCU pin definitions for Encoders and Motors
  #define encoderPinA1 5
  #define encoderPinA2 4
  #define encoderPinB1 14
  #define encoderPinB2 12
  const int motorA_DIR = 0;
  const int motorA_PWM = 2;
  const int motorB_DIR = 16;
  const int motorB_PWM = 15;
// Pin where the PPM signal is connected
#define PPM_PIN 13
#endif

///////////////////////////////////////////////////////////////////////////////////////////////

// Constants for PWM limit
const int PWM_MAX = 255;

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
const int interval = 10000; // Time interval in microseconds (1000ms/30 = ~33ms)

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
#define SYNC_GAP 3000 // Adjust as per sync gap width in microseconds
#define NUM_CHANNELS 8
#define READ_INTERVAL 50000 // 100 ms in microseconds

volatile unsigned long lastPulseTime = 0;
volatile unsigned long lastReadTime = 0; // Time when last data was read
volatile int currentChannel = 0;
volatile bool readyToRead = false;

unsigned long ppmChannels[NUM_CHANNELS];

// ISR to handle PPM signal
IRAM_ATTR void ppmISR()
{
  unsigned long currentTime = micros();
  unsigned long pulseWidth = currentTime - lastPulseTime;
  lastPulseTime = currentTime;

  // Check if 100ms has passed since the last data read
  if ((currentTime - lastReadTime) < READ_INTERVAL)
  {
    return; // Exit if 100ms hasn't passed
  }

  // Check if the pulse width indicates a sync pulse
  if (pulseWidth >= SYNC_GAP)
  {
    // Sync pulse detected, reset to the first channel
    currentChannel = 0;
    readyToRead = true;
  }
  else if (readyToRead && currentChannel < NUM_CHANNELS)
  {
    // Store the pulse width if within the allowed channel range
    ppmChannels[currentChannel] = pulseWidth;
    currentChannel++;

    // If all channels have been read, mark the time and stop reading
    if (currentChannel >= NUM_CHANNELS)
    {
      lastReadTime = currentTime;
      readyToRead = false; // Wait for the next sync pulse
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
void PrintPIDValues();                             // Function to print PID input, target, output values to Serial Monitor for tuning
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


// Helper function to parse and set PID values
void updatePID(float &Kp, float &Ki, float &Kd, char *input) {
    int pid_args[3] = {0}; // Holds parsed PID values
    char *str;
    int i = 0;
    
    while ((str = strtok_r(input, ":", &input)) != NULL && i < 3) {
        pid_args[i++] = atoi(str);
    }

    Kp = pid_args[0];
    Ki = pid_args[1];
    Kd = pid_args[2];
}

void runCommand() {
    int arg1 = atoi(argv1); // Convert argv1 to integer
    int arg2 = atoi(argv2); // Convert argv2 to integer
    char *p = argv1;        // Points to argv1

    switch (cmd) {
    case PING:
        Serial.println(arg1); // Respond with argument 1
        break;

    case READ_ENCODERS:
        Serial.print(countA); Serial.print(" ");
        Serial.println(countB); // Respond with encoder counts
        break;

    case SET_MOTOR_PWM_OFSET:
        pwmOffsetA = arg1;
        pwmOffsetB = arg2;
        motorAnglePIDA.SetOutputLimits(-PWM_MAX + pwmOffsetA, PWM_MAX - pwmOffsetA);
        motorAnglePIDB.SetOutputLimits(-PWM_MAX + pwmOffsetB, PWM_MAX - pwmOffsetB);
        motorSpeedPIDA.SetOutputLimits(-PWM_MAX + pwmOffsetA, PWM_MAX - pwmOffsetA);
        motorSpeedPIDB.SetOutputLimits(-PWM_MAX + pwmOffsetB, PWM_MAX - pwmOffsetB);
        #ifdef DEBUG
            Serial.print("PWM Offset A: "); Serial.println(pwmOffsetA);
            Serial.print("PWM Offset B: "); Serial.println(pwmOffsetB);
        #endif
        break;

    case READ_SPEEDS:
        Serial.print(calculateRPMA()); Serial.print(" ");
        Serial.println(calculateRPMB()); // Respond with RPM of both motors
        break;

    case RESET_ENCODERS:
        countA = 0; countB = 0; // Reset encoder counts
        Serial.println("Encoders reset");
        break;

    case MOTOR_SPEEDS:
        motorAnglePIDA.SetMode(motorAnglePIDA.Control::manual); // Angle PID to manual
        motorAnglePIDB.SetMode(motorAnglePIDB.Control::manual);
        motorSpeedPIDA.SetMode(motorSpeedPIDA.Control::timer);  // Speed PID to auto
        motorSpeedPIDB.SetMode(motorSpeedPIDB.Control::timer);

        mode = 1; // Set mode flag to manual
        targetSpeed1 = arg1;
        targetSpeed2 = arg2;
        #ifdef DEBUG
            Serial.print("Speed 1: "); Serial.println(arg1);
            Serial.print("Speed 2: "); Serial.println(arg2);
        #endif
        break;

    case MOTOR_ANGLES:
        motorSpeedPIDA.SetMode(motorSpeedPIDA.Control::manual); // Speed PID to manual
        motorSpeedPIDB.SetMode(motorSpeedPIDB.Control::manual);
        motorAnglePIDA.SetMode(motorAnglePIDA.Control::timer);  // Angle PID to auto
        motorAnglePIDB.SetMode(motorAnglePIDB.Control::timer);

        mode = 0; // Set mode flag to angle
        targetAngle1 = calculateAngleA() + arg1;
        targetAngle2 = calculateAngleB() + arg2;
        #ifdef DEBUG
            Serial.print("Target Angle 1: "); Serial.println(targetAngle1);
            Serial.print("Target Angle 2: "); Serial.println(targetAngle2);
        #endif
        break;

    case MOTOR_PWM:
        controlMotorA(arg1); controlMotorB(arg2); // Set PWM values
        break;

    case UPDATE_PIDAA:
        updatePID(KpA1, KiA1, KdA1, p); // Update PID for Motor A angle
        motorAnglePIDA.SetTunings(KpA1, KiA1, KdA1);
        #ifdef DEBUG
            Serial.print("PID A updated: Kp="); Serial.print(KpA1);
            Serial.print(", Ki="); Serial.print(KiA1);
            Serial.print(", Kd="); Serial.println(KdA1);
        #endif
        break;

    case UPDATE_PIDAB:
        updatePID(KpA2, KiA2, KdA2, p); // Update PID for Motor B angle
        motorSpeedPIDB.SetTunings(KpA2, KiA2, KdA2);
        #ifdef DEBUG
            Serial.print("PID B updated: Kp="); Serial.print(KpA2);
            Serial.print(", Ki="); Serial.print(KiA2);
            Serial.print(", Kd="); Serial.println(KdA2);
        #endif
        break;

    case UPDATE_PIDSA:
        updatePID(KpB1, KiB1, KdB1, p); // Update PID for Motor A speed
        motorSpeedPIDA.SetTunings(KpB1, KiB1, KdB1);
        #ifdef DEBUG
            Serial.print("PID A updated: Kp="); Serial.print(KpB1);
            Serial.print(", Ki="); Serial.print(KiB1);
            Serial.print(", Kd="); Serial.println(KdB1);
        #endif
        break;

    case UPDATE_PIDSB:
        updatePID(KpB2, KiB2, KdB2, p); // Update PID for Motor B speed
        motorSpeedPIDB.SetTunings(KpB2, KiB2, KdB2);
        #ifdef DEBUG
            Serial.print("PID B updated: Kp="); Serial.print(KpB2);
            Serial.print(", Ki="); Serial.print(KiB2);
            Serial.print(", Kd="); Serial.println(KdB2);
        #endif
        break;

    case GET_INP_TAR:
        modePrint = arg1; // Set mode for printing
        #ifdef DEBUG
            Serial.print("Input Target Print Mode: "); Serial.println(modePrint);
        #endif
        break;

    case PRINT_PPM:
        ppmPrint = !ppmPrint;
        if (ppmPrint) {
            attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, FALLING);
        } else {
            detachInterrupt(digitalPinToInterrupt(PPM_PIN));
        }
        break;

    case PPM_INTRRUPT:
        ppminterrupt = !ppminterrupt;
        if (ppminterrupt) {
            attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, FALLING);
            Serial.println("PPM interrupt started");
        } else {
            detachInterrupt(digitalPinToInterrupt(PPM_PIN));
            Serial.println("PPM interrupt stopped");
        }
        break;

    case PRINT_PID_CONST:
        Serial.print("AnglePID 1 kp: "); Serial.print(motorAnglePIDA.GetKp());
        Serial.print("\tki: "); Serial.print(motorAnglePIDA.GetKi());
        Serial.print("\tkd: "); Serial.println(motorAnglePIDA.GetKd());
        Serial.print("AnglePID 2 kp:"); Serial.print(motorAnglePIDB.GetKp());
        Serial.print("\tki: "); Serial.print(motorAnglePIDB.GetKi());
        Serial.print("\tkd: "); Serial.println(motorAnglePIDB.GetKd());
        Serial.print("SpeedPID 1 kp: "); Serial.print(motorSpeedPIDA.GetKp());
        Serial.print("\tki: "); Serial.print(motorSpeedPIDA.GetKi());
        Serial.print("\tkd: "); Serial.println(motorSpeedPIDA.GetKd());
        Serial.print("SpeedPID 2 kp:"); Serial.print(motorSpeedPIDB.GetKp());
        Serial.print("\tki: "); Serial.print(motorSpeedPIDB.GetKi());
        Serial.print("\tkd: "); Serial.println(motorSpeedPIDB.GetKd());
        break;

    case PPM_TUNE:
        ppmTuner = !ppmTuner;
        if (ppmTuner) {
            attachInterrupt(digitalPinToInterrupt(PPM_PIN), ppmISR, FALLING);
            Serial.println("PPM Tuner on");
        } else {
            detachInterrupt(digitalPinToInterrupt(PPM_PIN));
            Serial.println("PPM Tuner off");
        }
        break;

    default:
        Serial.println("Invalid command");
        break;
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
  pinMode(motorA_DIR, OUTPUT);
  pinMode(motorA_PWM, OUTPUT);

  // Set motor control pins as outputs for Motor B
  pinMode(motorB_DIR, OUTPUT);
  pinMode(motorB_PWM, OUTPUT);

  // Set up the PPM input pin
  pinMode(PPM_PIN, INPUT);

  // Initial settings for PID
  motorAnglePIDA.SetSampleTimeUs(interval);                            // Set PID update interval
  motorAnglePIDA.SetOutputLimits(-PWM_MAX + pwmOffsetA, PWM_MAX - pwmOffsetA); // Set output range
  motorAnglePIDB.SetSampleTimeUs(interval);                            // Set PID update interval
  motorAnglePIDB.SetOutputLimits(-PWM_MAX + pwmOffsetB, PWM_MAX - pwmOffsetB); // Set output range

  motorSpeedPIDA.SetSampleTimeUs(interval);                            // Set PID update interval
  motorSpeedPIDA.SetOutputLimits(-PWM_MAX + pwmOffsetA, PWM_MAX - pwmOffsetA); // Set output range
  motorSpeedPIDB.SetSampleTimeUs(interval);                            // Set PID update interval
  motorSpeedPIDB.SetOutputLimits(-PWM_MAX + pwmOffsetB, PWM_MAX - pwmOffsetB); // Set output range

  motorAnglePIDA.SetMode(motorAnglePIDA.Control::timer);  // Set PID mode to automatic
  motorAnglePIDB.SetMode(motorAnglePIDB.Control::timer);  // Set PID mode to automatic
  motorSpeedPIDA.SetMode(motorSpeedPIDA.Control::manual); // Set PID mode to manual
  motorSpeedPIDB.SetMode(motorSpeedPIDB.Control::manual); // Set PID mode to manual
}
void runPIDControl();
void printPPMChannels();
void processSerialInput();

void loop()
{
#ifdef BENCHMARK
  benchmarkLoopFrequency(); // Benchmarks loop frequency if enabled
#endif

  unsigned long currentTime = micros(); // Store current time once

  // Handle PID updates at regular intervals
  if (currentTime - lastUpdateTime >= interval)
  {
    lastUpdateTime = currentTime;

    // Handle PID control (either angle or speed mode)
    runPIDControl();

    // Optionally print PID values for tuning
    if (modePrint != 0)
    {
      PrintPIDValues();
    }

    // Handle PPM channel printing if enabled
    if (ppmPrint)
    {
      printPPMChannels();
    }
  }

  // Handle PPM tuner at regular intervals
  if (currentTime - lastUpdateTimePPM >= intervalPPM)
  {
    lastUpdateTimePPM = currentTime;
    if (ppmTuner)
    {
      ppm_pid_tuner();
    }
  }

  // Process incoming serial commands
  if (Serial.available() > 0)
    processSerialInput();
}

// Function to run the appropriate PID control based on mode
void runPIDControl()
{
  if (mode == 0) // Angle PID mode
  {
    angle1 = calculateAngleA(); // Calculate angle for Motor A
    motorAnglePIDA.Compute();

    angle2 = calculateAngleB(); // Calculate angle for Motor B
    motorAnglePIDB.Compute();

    controlMotorA(output1);
    controlMotorB(output2);
  }
  else if (mode == 1) // Speed PID mode
  {
    speed1 = calculateRPMA(); // Calculate RPM for Motor A
    motorSpeedPIDA.Compute();

    speed2 = calculateRPMB(); // Calculate RPM for Motor B
    motorSpeedPIDB.Compute();

    controlMotorA(output1);
    controlMotorB(output2);
  }
}

// Function to print PPM channels
void printPPMChannels()
{
  Serial.print("Channels: ");
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    Serial.print(ppmChannels[i]);
    Serial.print("\t");
  }
  Serial.println();
}

// Function to process serial input and commands
void processSerialInput()
{
  while (Serial.available() > 0)
  {
    char chr = Serial.read(); // Read the next character

    if (chr == '\r') // Check for carriage return (CR)
    {
      if (arg == 1)
        argv1[index1] = '\0'; // Null-terminate first argument
      else if (arg == 2)
        argv2[index1] = '\0'; // Null-terminate second argument

      runCommand();   // Execute the parsed command
      resetCommand(); // Reset command variables for next input
    }
    else if (chr == ' ') // Handle argument delimiter
    {
      if (arg == 0)
        arg = 1; // First argument detected
      else if (arg == 1)
      {
        argv1[index1] = '\0'; // Null-terminate first argument
        arg = 2;              // Move to second argument
        index1 = 0;           // Reset the index for the next argument
      }
    }
    else
    {
      if (arg == 0)
        cmd = chr; // Store the command
      else if (arg == 1)
        argv1[index1++] = chr; // Store the first argument
      else if (arg == 2)
        argv2[index1++] = chr; // Store the second argument
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

// Define constant for time conversion to avoid using floating-point division early
const unsigned long millisToMinutes = 60000; // Keep as integer

// General function to calculate RPM for any motor
int calculateRPM(unsigned long &prevTime, long &prevCount, volatile long currentCount)
{
  unsigned long currentTime = millis(); // Get current time in milliseconds

  // Calculate time difference in milliseconds (integer math)
  unsigned long timeDiffMillis = currentTime - prevTime;

  // Calculate the change in encoder count
  long countDiff = currentCount - prevCount;

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
  prevCount = currentCount;

  return static_cast<int>(rpm); // Return the integer value of the RPM
}

// Function to calculate RPM for Motor A
int calculateRPMA()
{
  return calculateRPM(prevTimeA, prevCountA, countA);
}

// Function to calculate RPM for Motor B
int calculateRPMB()
{
  return calculateRPM(prevTimeB, prevCountB, countB);
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
    digitalWrite(motorA_DIR, HIGH);
    analogWrite(motorA_PWM, speed + pwmOffsetA); // Set speed (0-255)
  }
  else if (speed < 0)
  {
    // Reverse direction
    digitalWrite(motorA_DIR, LOW);
    analogWrite(motorA_PWM, -speed + pwmOffsetA); // Set speed (0-255), negate for reverse
  }
  else
  {
    // Stop the motor
    digitalWrite(motorA_DIR, LOW);
    analogWrite(motorA_PWM, 0);
  }
}

// Function to control Motor B speed and direction
void controlMotorB(double speed)
{
  if (speed > 0)
  {
    // Forward direction
    digitalWrite(motorB_DIR, HIGH);
    analogWrite(motorB_PWM, speed + pwmOffsetB); // Set speed (0-255)
  }
  else if (speed < 0)
  {
    // Reverse direction
    digitalWrite(motorB_DIR, LOW);
    analogWrite(motorB_PWM, -speed + pwmOffsetB); // Set speed (0-255), negate for reverse
  }
  else
  {
    // Stop the motor
    digitalWrite(motorB_DIR, LOW);
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

void setPIDTunings(float val, float &Kp, float &Ki, float &Kd, QuickPID &pid, u8 control2)
{
  bool updated = false;

  if (control2 == 0 && Kp != val)
  {
    Kp = val;
    updated = true;
  }
  else if (control2 == 1 && Ki != val)
  {
    Ki = val;
    updated = true;
  }
  else if (control2 == 2 && Kd != val)
  {
    Kd = val;
    updated = true;
  }

  if (updated)
  {
    pid.SetTunings(Kp, Ki, Kd); // Only update PID tunings if any value changes
  }
}

void ppm_pid_tuner()
{
  // Determine control1 based on ppmChannels[4]
  u8 control1 = 0;
  if (ppmChannels[4] >= 1150)
  {
    if (ppmChannels[4] < 1310)
      control1 = 1;
    else if (ppmChannels[4] < 1480)
      control1 = 2;
    else if (ppmChannels[4] < 1650)
      control1 = 3;
    else if (ppmChannels[4] < 1820)
      control1 = 4;
    else
      control1 = 5;
  }

  // Determine control2 based on ppmChannels[5]
  u8 control2 = (ppmChannels[5] < 1300) ? 0 : ((ppmChannels[5] < 1600) ? 1 : 2);

  float val = (ppmChannels[2] - 1000) * 0.001; // Scale 'val' between 0 and 1

  switch (control1)
  {
  case 0:
    setPIDTunings(val, KpA1, KiA1, KdA1, motorAnglePIDA, control2);
    break;
  case 1:
    setPIDTunings(val, KpA2, KiA2, KdA2, motorAnglePIDB, control2);
    break;
  case 2:
    setPIDTunings(val, KpB1, KiB1, KdB1, motorSpeedPIDA, control2);
    break;
  case 3:
    setPIDTunings(val, KpB2, KiB2, KdB2, motorSpeedPIDB, control2);
    break;
  case 4:
    // Disable all PID controllers
    motorAnglePIDA.SetMode(0);
    motorAnglePIDB.SetMode(0);
    motorSpeedPIDA.SetMode(0);
    motorSpeedPIDB.SetMode(0);

    // Set direct outputs based on control2
    if (control2 == 0)
    {
      output1 = (int)(val * 255);
      Serial.println(output1);
    }
    else if (control2 == 1)
    {
      output2 = (int)(val * 255);
      Serial.println(output2);
    }
    break;
  default:
    break;
  }
}
