# Quadcopter PID Control System

Flight control system for a quadcopter using L3GD20H gyroscope.

## Core Components

- `YMFC-3D_PID.ino`: PID control implementation
- `YMFC-3D_gyro.ino`: Gyroscope reading and calibration
- `YMFC-3D_esc_output.ino`: Motor control and ESC management
- `YMFC-3D_receiver.ino`: RC receiver input processing

## Hardware Setup
- Arduino board
- L3GD20H gyroscope
- 4x ESCs and brushless motors
- RC receiver

## Pin Configuration
- ESC1: Pin 4 (Front Right)
- ESC2: Pin 5 (Front Left)
- ESC3: Pin 6 (Back Right)
- ESC4: Pin 7 (Back Left)
- Throttle: Pin 10
- L3GD20H: I2C (SDA/SCL)

## Current PID Values