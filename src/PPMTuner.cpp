#include "PPMTuner.h"

// Global Variables
unsigned long lastUpdateTimePPM = 0;
const unsigned long intervalPPM = 200000;  // or unsigned long if the value is non-negative
bool ppmPrint = 0;
bool ppminterrupt = 0;
bool ppmTuner = 0;

volatile unsigned long lastPulseTime = 0;
volatile unsigned long lastReadTime = 0;
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
        currentChannel = 0;
        readyToRead = true;
    }
    else if (readyToRead && currentChannel < NUM_CHANNELS)
    {
        // Store the pulse width if within the allowed channel range
        ppmChannels[currentChannel] = pulseWidth;
        currentChannel++;

        if (currentChannel >= NUM_CHANNELS)
        {
            lastReadTime = currentTime;
            readyToRead = false; // Wait for the next sync pulse
        }
    }
}

// Print PPM channel values
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

// Set PID Tunings
void setPIDTunings(float val, float &Kp, float &Ki, float &Kd, QuickPID &pid, uint8_t control2)
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

// PPM PID Tuner function
void ppm_pid_tuner()
{
    // Determine control1 based on ppmChannels[4]
    uint8_t control1 = 0;
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
    uint8_t control2 = (ppmChannels[5] < 1300) ? 0 : ((ppmChannels[5] < 1600) ? 1 : 2);

    uint8_t multiplier = (ppmChannels[7] < 1500) ? 1 : 2;

    float val = (ppmChannels[2] - 1000) * 0.001 * multiplier; // Scale 'val' between 0 and 1

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
        motorAnglePIDA.SetMode(0);
        motorAnglePIDB.SetMode(0);
        motorSpeedPIDA.SetMode(0);
        motorSpeedPIDB.SetMode(0);

        if (control2 == 0)
        {
            motorA.controlMotor((int)(val * 255));
        }
        else if (control2 == 1)
        {
            motorB.controlMotor((int)(val * 255));
        }
        break;
    default:
        break;
    }
}
