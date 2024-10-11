import tkinter as tk
from tkinter import ttk
import serial
import time

# Initialize serial communication
ser = serial.Serial('COM7', 115200, timeout=1)  # Update COM port as needed

# Function to send commands to the Arduino
def send_command(cmd):
    ser.write((cmd + '\r').encode())  # Send command with carriage return
    time.sleep(0.1)  # Small delay to ensure command processing
    response = ser.readline().decode().strip()  # Read the response
    return response

# Function to send PID constants
def update_pid(controller):
    kp = kp_slider.get()
    ki = ki_slider.get()
    kd = kd_slider.get()
    command = f"{controller} {kp:.2f}:{ki:.2f}:{kd:.2f}"
    response = send_command(command)
    print(f"Sent: {command}, Response: {response}")

# Function to update motor speed
def update_motor_speed():
    speed1 = speed_slider1.get()
    speed2 = speed_slider2.get()
    command = f"m {speed1} {speed2}"
    response = send_command(command)
    print(f"Sent: {command}, Response: {response}")

# Function to update motor angle
def update_motor_angle():
    angle1 = angle_entry1.get()
    angle2 = angle_entry2.get()
    command = f"a {angle1} {angle2}"
    response = send_command(command)
    print(f"Sent: {command}, Response: {response}")

# Function to fetch and display motor speed
def fetch_motor_speed():
    response = send_command("s")
    motor_speed_display.config(state='normal')
    motor_speed_display.delete(1.0, tk.END)
    motor_speed_display.insert(tk.END, response)
    motor_speed_display.config(state='disabled')

# Function to fetch and display motor angle
def fetch_motor_angle():
    response = send_command("e")
    motor_angle_display.config(state='normal')
    motor_angle_display.delete(1.0, tk.END)
    motor_angle_display.insert(tk.END, response)
    motor_angle_display.config(state='disabled')

# Function to send and display custom commands
def send_custom_command():
    custom_cmd = custom_command_entry.get()
    response = send_command(custom_cmd)
    custom_command_response.config(state='normal')
    custom_command_response.delete(1.0, tk.END)
    custom_command_response.insert(tk.END, response)
    custom_command_response.config(state='disabled')

# GUI setup
root = tk.Tk()
root.title("PID Motor Controller Tuner")

# PID Sliders
kp_slider = tk.Scale(root, from_=0.01, to=1, resolution=0.01, orient='horizontal', label="Kp")
kp_slider.pack()
ki_slider = tk.Scale(root, from_=0.01, to=1, resolution=0.01, orient='horizontal', label="Ki")
ki_slider.pack()
kd_slider = tk.Scale(root, from_=0.01, to=1, resolution=0.01, orient='horizontal', label="Kd")
kd_slider.pack()

# PID update buttons
for i, controller in enumerate(['u', 'v', 'w', 'x'], start=1):
    button = tk.Button(root, text=f"Update PID {i}", command=lambda c=controller: update_pid(c))
    button.pack()

# Motor speed sliders
speed_slider1 = tk.Scale(root, from_=200, to=500, orient='horizontal', label="Motor 1 Speed")
speed_slider1.pack()
speed_slider2 = tk.Scale(root, from_=200, to=500, orient='horizontal', label="Motor 2 Speed")
speed_slider2.pack()

# Button to update motor speed
update_speed_button = tk.Button(root, text="Update Motor Speed", command=update_motor_speed)
update_speed_button.pack()

# Angle input fields and button
angle_entry1 = ttk.Entry(root)
angle_entry1.pack()
angle_entry1.insert(0, "Enter Angle 1")

angle_entry2 = ttk.Entry(root)
angle_entry2.pack()
angle_entry2.insert(0, "Enter Angle 2")

update_angle_button = tk.Button(root, text="Update Angle", command=update_motor_angle)
update_angle_button.pack()

# Motor speed display and fetch button
motor_speed_display = tk.Text(root, height=2, width=30, state='disabled')
motor_speed_display.pack()
fetch_speed_button = tk.Button(root, text="Fetch Motor Speed", command=fetch_motor_speed)
fetch_speed_button.pack()

# Motor angle display and fetch button
motor_angle_display = tk.Text(root, height=2, width=30, state='disabled')
motor_angle_display.pack()
fetch_angle_button = tk.Button(root, text="Fetch Motor Angle", command=fetch_motor_angle)
fetch_angle_button.pack()

# Custom command input
custom_command_entry = ttk.Entry(root)
custom_command_entry.pack()
custom_command_entry.insert(0, "Enter Custom Command")

# Button to send custom command
send_custom_button = tk.Button(root, text="Send Custom Command", command=send_custom_command)
send_custom_button.pack()

# Display custom command response
custom_command_response = tk.Text(root, height=2, width=30, state='disabled')
custom_command_response.pack()

# Start the GUI loop
root.mainloop()
