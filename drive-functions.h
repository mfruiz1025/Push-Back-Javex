#pragma once
#include "robot-config.h"
using namespace vex;

// Mover recto con corrección de rumbo usando IMU
void driveStraightIMU(int speed, int tiempo_ms) {
  inertialSensor.setHeading(0, degrees); // Reiniciar rumbo
  int start = vex::timer::system();

  while (vex::timer::system() - start < tiempo_ms) {
    double error = 0 - inertialSensor.heading(); // desviación
    double correction = error * 0.5; // constante proporcional

    leftFront.spin(fwd, speed + correction, pct);
    leftMiddle.spin(fwd, speed + correction, pct);
    leftBack.spin(fwd, speed + correction, pct);

    rightFront.spin(fwd, speed - correction, pct);
    rightMiddle.spin(fwd, speed - correction, pct);
    rightBack.spin(fwd, speed - correction, pct);

    wait(20, msec);
  }

  leftFront.stop();
  leftMiddle.stop();
  leftBack.stop();
  rightFront.stop();
  rightMiddle.stop();
  rightBack.stop();
}

// Girar a un ángulo específico usando IMU
void turnToAngle(double target) {
  double kP = 0.8;
  double error = target - inertialSensor.heading();

  while (fabs(error) > 1.0) {
    error = target - inertialSensor.heading();
    double motorPower = error * kP;

    leftFront.spin(fwd, motorPower, pct);
    leftMiddle.spin(fwd, motorPower, pct);
    leftBack.spin(fwd, motorPower, pct);

    rightFront.spin(fwd, -motorPower, pct);
    rightMiddle.spin(fwd, -motorPower, pct);
    rightBack.spin(fwd, -motorPower, pct);

    wait(20, msec);
  }

  leftFront.stop();
  leftMiddle.stop();
  leftBack.stop();
  rightFront.stop();
  rightMiddle.stop();
  rightBack.stop();
}
