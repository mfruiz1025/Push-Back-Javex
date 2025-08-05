#include "vex.h"
#include "robot-config.h"
using namespace vex;

float promedioRotacionIzquierda() {
  return (leftFront.position(rev) + leftMiddle.position(rev) + leftBack.position(rev)) / 3.0;
}

void avanzarRecto(float revoluciones, float velocidad = 50) {
  float rumboObjetivo = InertialSensor.heading();

  // Reset de rotaciones de motores
  leftFront.resetPosition();
  leftMiddle.resetPosition();
  leftBack.resetPosition();
  rightFront.resetPosition();
  rightMiddle.resetPosition();
  rightBack.resetPosition();

  while (promedioRotacionIzquierda() < revoluciones) {
    float error = InertialSensor.heading() - rumboObjetivo;
    float correccion = error * 0.1;

    float velocidadIzq = velocidad - correccion;
    float velocidadDer = velocidad + correccion;

    // Asignar velocidades
    leftFront.setVelocity(velocidadIzq, percent);
    leftMiddle.setVelocity(velocidadIzq, percent);
    leftBack.setVelocity(velocidadIzq, percent);

    rightFront.setVelocity(velocidadDer, percent);
    rightMiddle.setVelocity(velocidadDer, percent);
    rightBack.setVelocity(velocidadDer, percent);

    // Mover
    leftFront.spin(forward);
    leftMiddle.spin(forward);
    leftBack.spin(forward);

    rightFront.spin(forward);
    rightMiddle.spin(forward);
    rightBack.spin(forward);

    wait(20, msec);
  }

  // Detener motores
  leftFront.stop();
  leftMiddle.stop();
  leftBack.stop();

  rightFront.stop();
  rightMiddle.stop();
  rightBack.stop();
}
