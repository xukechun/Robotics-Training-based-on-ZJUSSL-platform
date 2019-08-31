# Robotics-Training-based-on-ZJUSSL-platform

## Brief Introduction

This is the project of robotics intensive training in ZJU. All codes are tested on ZJUSSL platform.

- There are three .sln in this repository aimed at three scenes.**You need to link the protobuf-3.9.1/src and robot-for-static/protobuf.**
- Robot-for-static is for static obstacle avoidance. Robot-for-dynamics is for dynamic obstacle avoidance. Robot-using potential field id for following obstacle avoidance.

## Installation 

- C++
- protobuf-3.9.1
- cmake
- Windows
- ZJUSSL platform

## Details

- UDP communication
- RRT and RRT* planner
- Discrete Speed Control and T-type Speed Control /Trajectory planning
- Increase the path nodes and constantly correct the direction of the car
- Velocity correction is introduced when migration occurs between velocity and desired path direction.
- Combined RRT and potential field to conduct dynamic obstacle avoidance.

## Test

Robot-for-static is for static obstacle avoidance. Robot-for-dynamics is for dynamic obstacle avoidance. Robot-using potential field for following obstacle avoidance. Run **corresponding robot.sln** to test. If you want to run Robot-for-dynamics and Robot-using potential field, you just need to replace the main.cpp.

For dynamics and following obstacles, you need to run the **My Program** to start the demo.

## Teammates

My teammates are [czju-hui](https://github.com/czju-hui
) and Yurui Zhang, thanks to my teammates!
