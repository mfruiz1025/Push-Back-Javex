#include "vex.h"
#include <vector>
#include <functional>
#include <iostream>

#pragma once // Para evitar errores de definición múltiple

#include "configuration.h" // Incluimos la configuración de variables

using namespace vex;
using Callbacks = std::vector<std::function<void()>>;

using std::cout;
using std::endl;

//---------------- FUNCIONES DE MOVIMIENTO ----------------

void resetEncoders() {
    LeftMotor1.setPosition(0, rotationUnits::deg);
    LeftMotor2.setPosition(0, rotationUnits::deg);
    LeftMotor3.setPosition(0, rotationUnits::deg);
    LeftMotor4.setPosition(0, rotationUnits::deg);
    RightMotor1.setPosition(0, rotationUnits::deg);
    RightMotor2.setPosition(0, rotationUnits::deg);
    RightMotor3.setPosition(0, rotationUnits::deg);
    RightMotor4.setPosition(0, rotationUnits::deg);
}

void setLeftMotors(double speed) {
    LeftMotor1.spin(directionType::fwd, speed, velocityUnits::pct);
    LeftMotor2.spin(directionType::fwd, speed, velocityUnits::pct);
    LeftMotor3.spin(directionType::fwd, speed, velocityUnits::pct);
    LeftMotor4.spin(directionType::fwd, speed, velocityUnits::pct);
}

void setRightMotors(double speed) {
    RightMotor1.spin(directionType::fwd, speed, velocityUnits::pct);
    RightMotor2.spin(directionType::fwd, speed, velocityUnits::pct);
    RightMotor3.spin(directionType::fwd, speed, velocityUnits::pct);
    RightMotor4.spin(directionType::fwd, speed, velocityUnits::pct);
}

void stopAllMotors() {
    LeftMotor1.stop();
    LeftMotor2.stop();
    LeftMotor3.stop();
    LeftMotor4.stop();
    RightMotor1.stop();
    RightMotor2.stop();
    RightMotor3.stop();
    RightMotor4.stop();

}

void moveDistance(double distanceInInches, double speed) {
    resetEncoders();

    distanceInInches = (1.0 - RELATIVE_DISTANCE_ERROR) * distanceInInches;

    double targetRotations = (distanceInInches / WHEEL_CIRCUMFERENCE) * 360;

    setLeftMotors(speed);
    setRightMotors(speed);

    while (fabs(LeftMotor1.position(rotationUnits::deg)) < targetRotations &&
           fabs(RightMotor1.position(rotationUnits::deg)) < targetRotations) {
        task::sleep(10);
    }
    stopAllMotors();
}

void moveParabolic(double distanceInInches, double speedleft, double speedRight) {
    resetEncoders();

    distanceInInches = (1.0 - RELATIVE_DISTANCE_ERROR) * distanceInInches;

    double targetRotations = (distanceInInches / WHEEL_CIRCUMFERENCE) * 360;

    setLeftMotors(speedleft);
    setRightMotors(speedRight);

    while (fabs(LeftMotor1.position(rotationUnits::deg)) < targetRotations &&
           fabs(RightMotor1.position(rotationUnits::deg)) < targetRotations) {
        task::sleep(10);
    }
    stopAllMotors();
}

void moveDistanceR(double distanceInInches, double speed, double velocidad) {
    resetEncoders();

    distanceInInches = (1.0 - RELATIVE_DISTANCE_ERROR) * distanceInInches;

    double targetRotations = (distanceInInches / WHEEL_CIRCUMFERENCE) * 360;

    setLeftMotors(speed);
    setRightMotors(speed);

    while (fabs(LeftMotor1.position(rotationUnits::deg)) < targetRotations &&
           fabs(RightMotor1.position(rotationUnits::deg)) < targetRotations) {
        task::sleep(10);
    }
    stopAllMotors();
}

void moveParabolicR(double distanceInInches, double speedleft, double speedRight, double speed) {
    resetEncoders();

    distanceInInches = (1.0 - RELATIVE_DISTANCE_ERROR) * distanceInInches;

    double targetRotations = (distanceInInches / WHEEL_CIRCUMFERENCE) * 360;

    setLeftMotors(speedleft);
    setRightMotors(speedRight);

    while (fabs(LeftMotor1.position(rotationUnits::deg)) < targetRotations &&
           fabs(RightMotor1.position(rotationUnits::deg)) < targetRotations) {
        task::sleep(10);
    }
    stopAllMotors();
}

void rotateOnAxis(double angleInDegrees, double speed) {
    resetEncoders();
    inertialSensor.resetRotation();  // Reinicia la rotación del sensor de inercia para empezar desde 0

    double targetAngle = angleInDegrees;
    double currentAngle = inertialSensor.rotation();  // Obtén la orientación actual (en grados)

    setLeftMotors((angleInDegrees > 0) ? speed : -speed);
    setRightMotors((angleInDegrees > 0) ? -speed : speed);

    while (fabs(currentAngle - targetAngle) > 1.0) {  // Mantén la rotación hasta que sea precisa
        currentAngle = inertialSensor.rotation();  // Actualiza el ángulo actual

        task::sleep(10);  // Espera un pequeño tiempo antes de comprobar de nuevo
    }

    stopAllMotors();  // Detenemos todos los motores cuando se alcanza la rotación deseada
}

void recoleccion(int speed,double duration) {
  Recoleccion.spin(reverse, speed, percent);
  wait(duration, seconds);
  stopAllMotors();
}