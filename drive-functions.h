#pragma once
#include "robot-config.h"
using namespace vex;

// Avanzar recto usando inercia y 6 motores
inline void avanzarRectoInercia(float revsObjetivo, float velBase = 40, float kP = 0.5, float extraDerecha = 5) {
  InertialSensor.resetHeading();
  float rumboObjetivo = InertialSensor.heading();

  // Reset encoders usando setPosition
  leftFront.setPosition(0, rev);
  leftMiddle.setPosition(0, rev);
  leftBack.setPosition(0, rev);
  rightFront.setPosition(0, rev);
  rightMiddle.setPosition(0, rev);
  rightBack.setPosition(0, rev);

  while (fabs(leftFront.position(rev)) < revsObjetivo) {
    float error = InertialSensor.heading() - rumboObjetivo;
    float correccion = error * kP;

    // Lado izquierdo
    float velIzq = velBase - correccion;
    // Lado derecho con corrección + compensación extra fija
    float velDer = velBase + correccion + extraDerecha;

    leftFront.spin(forward, velIzq, percent);
    leftMiddle.spin(forward, velIzq, percent);
    leftBack.spin(forward, velIzq, percent);

    rightFront.spin(forward, velDer, percent);
    rightMiddle.spin(forward, velDer, percent);
    rightBack.spin(forward, velDer, percent);

    wait(20, msec);
  }

  // Detener motores
  leftFront.stop(brake);
  leftMiddle.stop(brake);
  leftBack.stop(brake);
  rightFront.stop(brake);
  rightMiddle.stop(brake);
  rightBack.stop(brake);
}
