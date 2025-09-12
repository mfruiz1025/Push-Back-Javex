#pragma once
#include "vex.h"
using namespace vex;

// Motores individuales
extern motor leftFront;
extern motor leftMiddle;
extern motor leftMiddle1;
extern motor leftBack;
extern motor rightFront;
extern motor rightMiddle;
extern motor rightMiddle1;
extern motor rightBack;

// Sensor
extern inertial inertialSensor;

// Grupos de motores (solo extern aquí)
extern motor_group leftDrive;
extern motor_group rightDrive;
