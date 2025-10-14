#include "vex.h"
#include <cmath>
using namespace vex;

brain Brain;

// Sensor de inercia en el puerto 1
inertial InertialSensor = inertial(PORT5);

double RELATIVE_DISTANCE_ERROR = 0.4445;

// Motores izquierda
motor LeftMotor1 = motor(PORT11, ratio18_1, false);
motor LeftMotor2 = motor(PORT12, ratio18_1, true);
motor LeftMotor3 = motor(PORT13, ratio18_1, false);
motor LeftMotor4 = motor(PORT14, ratio18_1, true);

// Motores derecha
motor RightMotor1 = motor(PORT16, ratio18_1, true);
motor RightMotor2 = motor(PORT18, ratio18_1, false);
motor RightMotor3 = motor(PORT19, ratio18_1, true);
motor RightMotor4 = motor(PORT20, ratio18_1, false);

//motores recoleccion 
motor Recoleccion1 = motor(PORT6, ratio18_1, true);
motor Recoleccion2 = motor(PORT7, ratio18_1, false);
motor_group Recoleccion (Recoleccion1, Recoleccion2); 

// Constantes de rueda
const double WHEEL_DIAMETER_IN = 2.75;
const double WHEEL_CIRCUMFERENCE = M_PI * WHEEL_DIAMETER_IN;

// ---------------- FUNCIONES ----------------

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

void setMotors(double leftSpeed, double rightSpeed) {
  LeftMotor1.spin(directionType::fwd, leftSpeed, velocityUnits::pct);
  LeftMotor2.spin(directionType::fwd, leftSpeed, velocityUnits::pct);
  LeftMotor3.spin(directionType::fwd, leftSpeed, velocityUnits::pct);
  LeftMotor4.spin(directionType::fwd, leftSpeed, velocityUnits::pct);

  RightMotor1.spin(directionType::fwd, rightSpeed, velocityUnits::pct);
  RightMotor2.spin(directionType::fwd, rightSpeed, velocityUnits::pct);
  RightMotor3.spin(directionType::fwd, rightSpeed, velocityUnits::pct);
  RightMotor4.spin(directionType::fwd, rightSpeed, velocityUnits::pct);
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

void move(double distanceInInches, double speed, double leftSpeedFactor = 1.0, double rightSpeedFactor = 1.0) {
  resetEncoders();

  // Ajuste por error estimado
  distanceInInches = (1.0 - RELATIVE_DISTANCE_ERROR) * distanceInInches;
  // Convertir a grados de rueda
  double targetDegrees = (distanceInInches / WHEEL_CIRCUMFERENCE) * 360.0;

  setMotors(speed * leftSpeedFactor, speed * rightSpeedFactor);

  // Esperar hasta que al menos un motor alcance el objetivo
  while (fabs(LeftMotor1.position(rotationUnits::deg)) < fabs(targetDegrees) &&
         fabs(RightMotor1.position(rotationUnits::deg)) < fabs(targetDegrees)) {
    task::sleep(10);
  }
  stopAllMotors();
}

// Función para girar con el sensor de inercia
void rotateOnAxis(double targetAngleRelative, double speed) {
  // Reiniciar el sensor de rotación
  InertialSensor.resetRotation();
  wait(200, msec);

  double error = targetAngleRelative;
  const double kTolerance = 1.5;  // tolerancia en grados

  while (fabs(error) > kTolerance) {
    double currentAngle = InertialSensor.rotation();
    error = targetAngleRelative - currentAngle;

    double direction = (error > 0) ? 1 : -1;

    // Velocidad ajustada: al menos 20, no más que “speed”
    double speedAdj = fabs(error); 
    if (speedAdj < 20.0) speedAdj = 20.0;
    if (speedAdj > speed) speedAdj = speed;

    setMotors(direction * speedAdj, -direction * speedAdj);
    wait(10, msec);
  }

  stopAllMotors();
}



// ---------------- FUNCIÓN PRINCIPAL ----------------

int main() {
  // Calibrar el sensor de inercia
  InertialSensor.calibrate();
  while (InertialSensor.isCalibrating()) {
    wait(100, msec);
  }

  Recoleccion.spin (directionType::rev, 100, velocityUnits::pct);

  // 1 - Avance hacia adelante y primer giro, con último avance al pt. 2
  move(-33, 90); 
  rotateOnAxis (90, 50);

  // 2 - Giro y primera diagonal
  move(-50, 90);
  rotateOnAxis (-26 , 50);

  // 3 - Avance a segunda posición y giro para diagonal
  move(-40, 90);
  rotateOnAxis (-70 , 50);

  // 4 - Avance de diagonal y giro para diagonal
  move(-30, 90);
  rotateOnAxis (-28 , 50);

  // 5 - Avanza derecho y gira diagonal
  move(-52, 90);
  rotateOnAxis(-40, 50);

  // 6 - Avanza en diagonal y gira
  move(-35, 90);
  rotateOnAxis(-118, 50);

  // 7 - Avanza derecho
  move(-35, 100);
                                                                                                                                                       
  while (true) {
    wait(20, msec);
  }
}
