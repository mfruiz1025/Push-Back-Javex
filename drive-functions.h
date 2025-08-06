#pragma once
#include "robot-config.h"
using namespace vex;

// Avanza usando encoders (spinFor)
void moverMotores6() {
 rightFront.spin(forward,200,percent);
 rightBack.spin(forward,200, percent);
 rightMiddle.spin(forward,200,percent);
 leftFront.spin(forward,50,percent);
 leftBack.spin(forward,50,percent);
 leftMiddle.spin(forward,50,percent);
wait(2, seconds);
rightFront.stop();
rightBack.stop();
rightMiddle.stop();
leftFront.stop();
leftMiddle.stop();
leftBack.stop();
}

