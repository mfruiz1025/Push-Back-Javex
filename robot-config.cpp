#pragma once
#include "vex.h"

using namespace vex;

// Motores lado izquierdo
extern motor leftFront;
extern motor leftMiddle;
extern motor leftBack;

// Motores lado derecho
extern motor rightFront;
extern motor rightMiddle;
extern motor rightBack;

// Sensor de inercia
extern inertial InertialSensor;

// Declaración de la función de inicialización
void vexcodeInit(void);
