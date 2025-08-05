#pragma once
#include "vex.h"

using namespace vex;

extern motor leftFront;
extern motor leftMiddle;
extern motor leftBack;

extern motor rightFront;
extern motor rightMiddle;
extern motor rightBack;

extern inertial InertialSensor;

void initializeDevices();
