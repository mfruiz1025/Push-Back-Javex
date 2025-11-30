#pragma once
#include "vex.h"
#include <vector>
#include <functional>

using namespace vex;
using Callbacks = std::vector<std::function<void()>>;

const double WHEEL_DIAMETER = 4.0;
const double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;
const double TRACK_WIDTH = 15.5;
const double RELATIVE_DISTANCE_ERROR = 0.4445;

// Declaración de dispositivos
extern brain Brain;
extern controller Controller;

extern inertial inertialSensor;

extern motor LeftMotor1;
extern motor LeftMotor2;
extern motor LeftMotor3;
extern motor LeftMotor4;
extern motor RightMotor1;
extern motor RightMotor2;
extern motor RightMotor3;
extern motor RightMotor4;
extern motor recoleccion1;
extern motor recoleccion2;
extern motor recoleccion3;

extern motor_group LeftDrive;
extern motor_group RightDrive;
extern motor_group Recoleccion;

struct PID
{
    double kp = 0;
    double ki = 0;
    double kd = 0;

    double error = 0;
    double prevError = 0;
    double Integral = 0;
};