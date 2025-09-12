#pragma once
#include "robot-config.h"
using namespace vex;

// ================================
// Avanzar recto con PID (distancia en cm)
// ================================
void driveStraightPID(double targetDistanceCm, double speedCap = 80) {
  // Constantes PID (ajusta según pruebas)
  double kP = 0.5;
  double kI = 0.0007;
  double kD = 0.35;

  // Datos de las ruedas (ajusta si usas otras)
  double wheelDiameter = 10.16; // cm (rueda 4")
  double wheelCircumference = wheelDiameter * M_PI;
  double degPerCm = 360.0 / wheelCircumference;
  double targetDeg = targetDistanceCm * degPerCm;

  // Resetear encoders de motores
  leftFront.setPosition(0, degrees);
  leftMiddle.setPosition(0, degrees);
  leftMiddle1.setPosition(0, degrees);
  leftBack.setPosition(0, degrees);

  rightFront.setPosition(0, degrees);
  rightMiddle.setPosition(0, degrees);
  rightMiddle1.setPosition(0, degrees);
  rightBack.setPosition(0, degrees);

  // Resetear rumbo inicial
  inertialSensor.setHeading(0, degrees);

  // Variables PID
  double error = 0, integral = 0, derivative = 0, lastError = 0;
  int startTime = vex::timer::system();
  int timeoutMs = 15000; // seguridad: 15s

  while (true) {
    // Promedio de posiciones de motores
    double leftPos = ( leftFront.position(degrees)
                     + leftMiddle.position(degrees)
                     + leftMiddle1.position(degrees)
                     + leftBack.position(degrees) ) / 4.0;

    double rightPos = ( rightFront.position(degrees)
                      + rightMiddle.position(degrees)
                      + rightMiddle1.position(degrees)
                      + rightBack.position(degrees) ) / 4.0;

    double avgPos = (leftPos + rightPos) / 2.0;

    // Error de distancia
    error = targetDeg - avgPos;
    integral += error;
    derivative = error - lastError;

    double output = (kP * error) + (kI * integral) + (kD * derivative);

    // Limitar velocidad
    if (output > speedCap) output = speedCap;
    if (output < -speedCap) output = -speedCap;

    // Corrección de rumbo con IMU
    double headingError = 0 - inertialSensor.heading();
    if (headingError > 180) headingError -= 360;
    if (headingError < -180) headingError += 360;

    double correction = headingError * 0.6;

    double leftPower  = output + correction;
    double rightPower = output - correction;

    // Aplicar a motores
    leftFront.spin(fwd, leftPower, pct);
    leftMiddle.spin(fwd, leftPower, pct);
    leftMiddle1.spin(fwd, leftPower, pct);
    leftBack.spin(fwd, leftPower, pct);

    rightFront.spin(fwd, rightPower, pct);
    rightMiddle.spin(fwd, rightPower, pct);
    rightMiddle1.spin(fwd, rightPower, pct);
    rightBack.spin(fwd, rightPower, pct);

    lastError = error;

    // Condiciones de salida
    if (fabs(error) < (degPerCm * 1.0)) break; // tolerancia: 1 cm
    if ((vex::timer::system() - startTime) > timeoutMs) break;

    wait(20, msec);
  }

  // Detener motores
  leftFront.stop(brake);
  leftMiddle.stop(brake);
  leftMiddle1.stop(brake);
  leftBack.stop(brake);

  rightFront.stop(brake);
  rightMiddle.stop(brake);
  rightMiddle1.stop(brake);
  rightBack.stop(brake);
}

// ================================
// Girar a un ángulo con PID usando IMU
// ================================
void turnToAnglePID(double targetDeg, double speedCap = 60) {
  double kP = 0.6;
  double kI = 0.001;
  double kD = 0.35;

  double error = 0, integral = 0, derivative = 0, lastError = 0;
  int startTime = vex::timer::system();
  int timeoutMs = 8000;

  while (true) {
    error = targetDeg - inertialSensor.heading();

    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    integral += error;
    derivative = error - lastError;

    double output = (kP * error) + (kI * integral) + (kD * derivative);

    if (output > speedCap) output = speedCap;
    if (output < -speedCap) output = -speedCap;

    leftFront.spin(fwd, output, pct);
    leftMiddle.spin(fwd, output, pct);
    leftMiddle1.spin(fwd, output, pct);
    leftBack.spin(fwd, output, pct);

    rightFront.spin(fwd, -output, pct);
    rightMiddle.spin(fwd, -output, pct);
    rightMiddle1.spin(fwd, -output, pct);
    rightBack.spin(fwd, -output, pct);

    lastError = error;

    if (fabs(error) < 1.0) break;
    if ((vex::timer::system() - startTime) > timeoutMs) break;

    wait(20, msec);
  }

  leftFront.stop(brake);
  leftMiddle.stop(brake);
  leftMiddle1.stop(brake);
  leftBack.stop(brake);
  rightFront.stop(brake);
  rightMiddle.stop(brake);
  rightMiddle1.stop(brake);
  rightBack.stop(brake);
}
