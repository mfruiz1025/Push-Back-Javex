#include "robot-config.h"

// Motores lado izquierdo
motor leftFront(PORT5, ratio18_1, false);
motor leftMiddle(PORT3, ratio18_1, false);
motor leftBack(PORT20, ratio18_1, false);

// Motores lado derecho
motor rightFront(PORT1, ratio18_1, true);
motor rightMiddle(PORT4, ratio18_1, true);
motor rightBack(PORT11, ratio18_1, true);

inertial inertialSensor(PORT10);


