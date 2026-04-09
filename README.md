# Robotic Arm — Mechatronic Design Project

An ongoing Arduino-based robotics project for Mechatronic Design at Florida Gulf Coast University. The system autonomously detects, identifies, and sorts balls by color using a robotic arm.

## Project Status

> Work in progress — new modules being added throughout the semester.

## Sketches

### 1. `RoboticarmAutomatedpickupcodev4/`
The main robotic arm control sketch. The arm sweeps a proximity sensor in a 150° arc to scan for a ball, picks it up, and drops it into a container approximately 10 inches high.

### 2. `PROJECT_BALLSWITHDISPLAY/`
Ball color detection module. Uses an RGB LED and photoresistor to identify the color of a detected ball (red, green, blue, or yellow) and displays the result on a 7-segment display via a 74HC595 shift register. Auto-calibrates ambient light on startup.

## Hardware

- Elegoo Uno R3 (Arduino Uno clone)
- 4x Servo motors
- Servo shield
- Proximity sensor
- RGB LEDs (red, green, blue)
- Photoresistor (10kΩ pull-down)
- 74HC595 shift register
- 7-segment display (common cathode)

## Software & Libraries

- Arduino IDE
- Servo library (built-in)

## Setup

1. Install [Arduino IDE](https://www.arduino.cc/en/software)
2. Connect the Elegoo Uno R3 via USB
3. Open the desired sketch folder in Arduino IDE
4. Upload the sketch to the board

## Course

Intro to Mechatronic Design — Florida Gulf Coast University

## Author

Saun-Jay Samuels
