#include "robot-config.h"

// Motores lado izquierdo
motor leftFront(PORT6, ratio18_1, false);
motor leftMiddle(PORT8, ratio18_1, false);
motor leftMiddle1(PORT9,ratio18_1,false);
motor leftBack(PORT10, ratio18_1, false);

// Motores lado derecho
motor rightFront(PORT4, ratio18_1, true);
motor rightMiddle(PORT3, ratio18_1, true);
motor rightMiddle1(PORT2, ratio18_1,true);
motor rightBack(PORT1, ratio18_1, true);

inertial inertialSensor(PORT20);

motor_group leftDrive(leftFront, leftMiddle, leftMiddle1, leftBack);
motor_group rightDrive(rightFront, rightMiddle, rightMiddle1, rightBack);
