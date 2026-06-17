# Autonomous Lane-Following Vehicle

A real-time autonomous driving project that uses onboard computer vision and embedded control to follow lane markings on a physical track. The vehicle detects lane position from camera input, estimates cross-track and heading error, and adjusts steering and speed through embedded motor control.

## Overview

This project was developed as a UC Davis senior design autonomous vehicle project. The goal was to build a small vehicle capable of following a lane track using camera-based perception and real-time control. The system had to operate on a real physical track with ramps, long straights, sharp turns, changing perspective, noisy visual input, and limited onboard compute resources.

The project combines:

* Computer vision
* Embedded systems
* Real-time control
* Sensor-based debugging
* Hardware/software integration

## Key Features

* Real-time lane detection using an OpenMV camera
* Grayscale image processing and threshold-based lane extraction
* Region-of-interest based perception for near-field and far-field lane information
* Line regression and lane-center estimation
* Cross-track error and heading error calculation
* PD-based steering control
* Dynamic speed adjustment based on lane confidence and turn severity
* PWM-based motor and servo control
* Physical track testing with ramps, straight sections, and sharp turns
* Iterative tuning to reduce oscillation and improve turn handling

## Hardware Used

* OpenMV camera module
* CC3200 embedded control board
* Servo motor for steering
* DC motor and motor driver for propulsion
* PWM control interface
* Small autonomous vehicle chassis
* Physical lane-following test track

## Software and Tools

* Python / MicroPython for OpenMV vision processing
* Embedded C / board-level control logic
* PWM motor and steering control
* Real-time image processing
* Control tuning and debugging
* Serial/debug output for testing

## System Architecture

```text id="3ekm0c"
Camera Frame
   -> Image Preprocessing
   -> Lane Detection
   -> Near/Far Lane Center Estimation
   -> Cross-Track Error + Heading Error
   -> PD Steering Controller
   -> Dynamic Speed Control
   -> PWM Output to Servo and Motor
```

The camera captures the lane view in real time. The vision pipeline extracts lane information from selected regions of interest, estimates where the vehicle is relative to the lane center, and sends control information to the steering and motor logic. The controller then adjusts steering and speed based on current lane position, heading direction, and track difficulty.

## Vision Pipeline

The perception system uses lightweight computer vision methods suitable for limited onboard compute.

Main steps:

1. Capture camera frame
2. Convert or process image in grayscale
3. Apply thresholding to isolate lane markings
4. Use near and far regions of interest to estimate lane geometry
5. Run line or regression-based detection on lane regions
6. Estimate near-field center and far-field target direction
7. Compute cross-track error and heading error
8. Send error values to the steering controller

This approach was designed to be fast enough for real-time embedded control while still providing useful lane geometry information.

## Control Strategy

The vehicle uses a PD-style control approach. The controller combines:

* Cross-track error: how far the vehicle is from the lane center
* Heading error: whether the vehicle is pointing toward or away from the desired lane direction
* Derivative smoothing: reducing sudden steering changes
* Dynamic speed adjustment: slowing down for sharper turns or uncertain detection

The goal was to balance stability on straight sections with enough steering authority to handle difficult turns.

## Real-World Testing Challenges

This project required repeated testing and tuning because the real track introduced challenges that were not obvious from code alone.

Key challenges included:

* Vehicle oscillation on straight sections
* Late turn detection before sharp curves
* Lane loss caused by ramps or changing camera perspective
* Balancing speed and steering stability
* Handling noisy or incomplete lane detection
* Tuning thresholds, PD gains, and speed limits for the physical vehicle

These challenges made the project a strong exercise in real-world robotics debugging, not just simulation or offline image processing.

## My Contributions

* Developed and tuned the real-time lane detection pipeline
* Worked on ROI-based near/far lane-center estimation
* Implemented steering correction using cross-track and heading error
* Tuned PD control behavior to reduce oscillation
* Adjusted speed control logic for straights, ramps, and turns
* Tested the vehicle repeatedly on a physical track and debugged failure cases
* Helped integrate camera perception, steering, motor control, and embedded hardware behavior

## What I Learned

This project strengthened my understanding of how computer vision algorithms behave under real-world constraints. It also taught me that autonomous systems require careful coordination between perception, control, hardware timing, and physical testing.

The most valuable lesson was that a system can work in theory but still fail on a real track because of sensor noise, latency, camera angle, speed, or control instability. Improving the vehicle required iterative testing, measurement, and tuning across both software and hardware.

## Future Improvements

* Add better lane confidence estimation
* Add curve prediction using more lookahead information
* Improve speed planning before sharp turns
* Add camera calibration and perspective correction
* Add data logging for replay-based debugging
* Explore learning-based lane detection or imitation learning
* Test reinforcement learning or simulation-based control strategies
* Improve robustness under different lighting and track conditions

## Author

Xiang Mao
B.S. Computer Engineering, University of California, Davis
GitHub: https://github.com/XiangM7
LinkedIn: https://www.linkedin.com/in/xiang-mao-78ab73301/
