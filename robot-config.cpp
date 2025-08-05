#include "robot-config.h"

// Motores lado izquierdo
motor leftFront(PORT6, ratio18_1, false);
motor leftMiddle(PORT3, ratio18_1, false);
motor leftBack(PORT20, ratio18_1, false);

// Motores lado derecho
motor rightFront(PORT4, ratio18_1, true);
motor rightMiddle(PORT18, ratio18_1, true);
motor rightBack(PORT5, ratio18_1, true);

// Sensor de inercia
inertial InertialSensor(PORT9);
