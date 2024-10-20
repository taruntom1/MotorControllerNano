# Motor Controller with Arduino

## Overview

This project is a motor controller built using an Arduino, designed to control motors via a motor driver. The system takes feedback from a rotary encoder attached to the motors, allowing precise control over motor speed and position. Communication with the Arduino is done through serial commands, enabling the user to control motor speeds, set angles, update PID parameters, tuning PID values through as RC controller and more.

## Serial Command List
The controller can perform various actions based on the commands sent via serial communication. Below is a list of the available commands and their functionalities.

| Command Character | Description | Example Usage |
|-------------------|-------------|---------------|
| **o**             | Set motor PWM offset (0-255). This is used to set a base PWM value for the motors to overcome mechanical friction. | `o 100 100` (Set Motor A and B PWM offset to 100) |
| **a**             | Set motor angles in degrees. | `a 90 45` (Set Motor A to 90° and Motor B to 45°) |
| **e**             | Read the encoder counts (feedback from the rotary encoder). | `e` (Returns current encoder counts for both motors) |
| **s**             | Read motor speeds in RPM. | `s` (Returns the current RPM of both motors) |
| **m**             | Set motor speeds in RPM. | `m 150 100` (Set Motor A to 150 RPM and Motor B to 100 RPM) |
| **n**             | Set motor PWM value (0-255). Directly controls motor PWM signal. | `n 200 150` (Set Motor A to 200 PWM and Motor B to 150 PWM) |
| **p**             | Ping the Arduino. Useful for checking if the Arduino is active. | `p` (Returns a confirmation response) |
| **r**             | Reset encoder counts. | `r` (Resets encoder counts to 0) |
| **u**             | Update angle PID values for Motor A (Kp, Ki, Kd). | `u 10:20:30` (Set PID for angle control of Motor A to Kp=10, Ki=20, Kd=30) |
| **v**             | Update angle PID values for Motor B (Kp, Ki, Kd). | `v 15:25:35` (Set PID for angle control of Motor B to Kp=15, Ki=25, Kd=35) |
| **w**             | Update speed PID values for Motor A (Kp, Ki, Kd). | `w 12:18:24` (Set PID for speed control of Motor A to Kp=12, Ki=18, Kd=24) |
| **x**             | Update speed PID values for Motor B (Kp, Ki, Kd). | `x 14:20:28` (Set PID for speed control of Motor B to Kp=14, Ki=20, Kd=28) |
| **g**             | Print target and input values for the specified PID. Use `g 1` for Motor A angle, `g 2` for Motor B angle, `g 3` for Motor A speed, and `g 4` for Motor B speed. | `g 1` (Start printing angle PID data for Motor A) |
| **z**             | Continuously print PPM signal received from the transmitter. | `z` (Prints raw PPM signal values) |
| **i**             | Enable or disable PPM interrupt. Useful when controlling motors using an RC transmitter. | `i` (Toggles PPM interrupt) |
| **t**             | Enable tuning using PPM from RC transmitter and receiver. | `t` (Enables tuning mode) |
| **k**             | Print PID constants. | `k` (Returns current PID constants for all motors) |
| **j**             | Update the alpha value of the low-pass filter used when calculating motor speed from encoder feedback. | `j 15` (Sets alpha to 15) |

## Usage Instructions

### Controlling the Motors
After setting up, use a serial monitor (such as the one in the Arduino IDE) or any serial communication tool (like PuTTY) to send commands. The available commands allow you to:
- Set motor speeds using the `m` command.
- Set motor angles using the `a` command.
- Monitor the motor speeds or encoder values with `s` and `e` commands.
- Adjust PID parameters dynamically using `u`, `v`, `w`, and `x`.

### Example Commands
- To set Motor A to 100 RPM and Motor B to 150 RPM:
  ```
  m 100 150
  ```
- To reset both motor encoders:
  ```
  r
  ```
- To update PID constants for Motor A's speed control (Kp=10, Ki=5, Kd=3):
  ```
  w 10:5:3
  ```
  
### Debugging and Monitoring
You can monitor the system performance using these commands:
- Use the `g` command to print target and input values for PID control.
- Use the `k` command to display the current PID values for both motors.
- The `z` command helps in debugging by continuously printing the PPM signal from an RC transmitter.
### Tuning through Serial
You can monitor and tune the PID constants for each PID through serial and monitor and plot the feedback vs target in serial plotter
- Use the `g` command to print target and input values for PID control you want to tune.
- Use u,v,w,x according to which PID you want to tune together with the new PID constants to update and test and tune the PID
### Tuning with PPM
When using an RC transmitter, the `t` command enables tuning. This allows you to adjust the PID parameters using your transmitter instead of manually typing commands.

### Filtering
The system includes a low-pass filter for the encoder values, used to smooth out noise. You can adjust the filter's responsiveness with the `j` command by setting the alpha value. Higher values make the filter less responsive to quick changes, while lower values make it more sensitive to variations.

## Conclusion
This motor controller provides a flexible, precise control over motor speed and position, with real-time tuning capabilities through serial commands or RC input. You can dynamically update PID parameters, monitor performance, and control the motors with ease.
