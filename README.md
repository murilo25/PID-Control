# PID-Control

This project implements a PID controller in C++. It uses Udacity Term 2 simulator cross-tracking error as an input.

The project is composed by the following:

main.cpp: Interfaces the project with Udacity Term 2 simulator and defines PID gains.


PID.cpp: Defines a PID class that is composed by 3 main functions.

* `PID::init()` that initializes PID gains given input from main.cpp.
* `PID::UpdateError()` that computes differential and integral errors given cross-tracking error (CTE) from simulation.
* `PID::TotalError()` that calculates vehicle's steer_angle input based on PID gains, CTE, differential CTE and integral CTE.