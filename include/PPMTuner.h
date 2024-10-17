#ifndef PPM_TUNER_H
#define PPM_TUNER_H

#include <Arduino.h>
#include <QuickPID.h>
#include "Motor.h"

// Constants
#define SYNC_GAP 3000
#define NUM_CHANNELS 8
#define READ_INTERVAL 50000 // 100 ms in microseconds

// Global Variables
extern unsigned long lastUpdateTimePPM;
extern const unsigned long intervalPPM;
extern bool ppmPrint;
extern bool ppminterrupt;
extern bool ppmTuner;

extern volatile unsigned long lastPulseTime;
extern volatile unsigned long lastReadTime;
extern volatile int currentChannel;
extern volatile bool readyToRead;

extern unsigned long ppmChannels[NUM_CHANNELS];

// External PID variables (you need to declare these elsewhere in your project)
extern float KpA1, KiA1, KdA1;
extern float KpA2, KiA2, KdA2;
extern float KpB1, KiB1, KdB1;
extern float KpB2, KiB2, KdB2;

extern QuickPID motorAnglePIDA, motorAnglePIDB;
extern QuickPID motorSpeedPIDA, motorSpeedPIDB;

extern Motor motorA, motorB;

// Function Declarations
void ppmISR();
void printPPMChannels();
void setPIDTunings(float val, float &Kp, float &Ki, float &Kd, QuickPID &pid, uint8_t control2);
void ppm_pid_tuner();

#endif // PPM_TUNER_H
    