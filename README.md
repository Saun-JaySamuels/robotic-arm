# Robotic Arm — Automated Ball Pickup

An Arduino-based robotic arm that autonomously detects a ball using a proximity sensor, picks it up, and drops it into a container approximately 10 inches high.

## Hardware

- Elegoo Uno R3 (Arduino Uno clone)
- 4x Servo motors
- Servo shield
- Proximity sensor

## Software & Libraries

- Arduino IDE
- Servo library (built-in)

## How It Works

1. The proximity sensor sweeps back and forth in a 150° arc to scan for a ball
2. Once a ball is detected, the arm moves into position using 4 servos
3. The arm picks up the ball
4. The arm rotates and drops the ball into the container (10 inches high)
5. The arm returns to home position and resumes scanning

## Setup

1. Install [Arduino IDE](https://www.arduino.cc/en/software)
2. Connect the Elegoo Uno R3 via USB
3. Attach the servo shield and connect all 4 servos
4. Wire the proximity sensor to the appropriate pin
5. Open `RoboticarmAutomatedpickupcodev4.ino` in Arduino IDE
6. Upload the sketch to the board

## Author

Saun-Jay Samuels
