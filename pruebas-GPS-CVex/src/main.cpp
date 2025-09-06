/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       USER                                                      */
/*    Created:      8/15/2025, 5:42:19 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>
#include <algorithm>

// A global instance of competition
vex::competition Competition;

// DECLARACIONES GLOBALES
// Declaración del controlador
vex::controller Controller = vex::controller(vex::controllerType::primary);

// Declaración del GPS
vex::gps GPS = vex::gps(vex::PORT20, 750.0, 400.0, vex::distanceUnits::mm, 0.0, vex::turnType::right);

// Declaraciones de motores
vex::motor LeftMotor1 = vex::motor(vex::PORT1, vex::gearSetting::ratio18_1, false);
vex::motor LeftMotor2 = vex::motor(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor LeftMotor3 = vex::motor(vex::PORT3, vex::gearSetting::ratio18_1, false);
vex::motor LeftMotor4 = vex::motor(vex::PORT4, vex::gearSetting::ratio18_1, false);

vex::motor RightMotor1 = vex::motor(vex::PORT7, vex::gearSetting::ratio18_1, true);
vex::motor RightMotor2 = vex::motor(vex::PORT8, vex::gearSetting::ratio18_1, true);
vex::motor RightMotor3 = vex::motor(vex::PORT9, vex::gearSetting::ratio18_1, true);
vex::motor RightMotor4 = vex::motor(vex::PORT10, vex::gearSetting::ratio18_1, true);

// Grupos de motores para la base
vex::motor_group LeftDrive = vex::motor_group(LeftMotor1, LeftMotor2, LeftMotor3, LeftMotor4);
vex::motor_group RightDrive = vex::motor_group(RightMotor1, RightMotor2, RightMotor3, RightMotor4);

// Constantes para PID/proporcional (ajustar según pruebas reales en el robot)
const double KP_TURN = 0.8;       // Ganancia proporcional para giros
const double KP_DRIVE = 0.5;      // Ganancia proporcional para avance
const double KP_HEADING = 1.0;    // Ganancia para corrección de heading
const double TURN_THRESHOLD = 1.0; // Umbral de error en grados para giros
const double DRIVE_THRESHOLD = 20.0; // Umbral de error en mm para posición
const double MAX_TURN_SPEED = 50.0;  // Velocidad máxima para giros (%)
const double MAX_DRIVE_SPEED = 80.0; // Velocidad máxima para avance (%)
const double LOOP_DELAY = 20.0;      // Delay en ms para loops de control
const double TIMEOUT_SEC = 15.0;     // Timeout en segundos (aumentado para mayor margen)

// Función auxiliar para calcular la diferencia de ángulo normalizada (-180 a 180 grados)
double angleDifference(double current, double target) {
  double diff = std::fmod(target - current + 180.0, 360.0) - 180.0;
  return diff;
}

// Función para girar el robot a un heading específico usando GPS
void turnToHeading(double targetHeading) {
  vex::timer timeoutTimer;
  timeoutTimer.clear();

  double error = angleDifference(GPS.heading(vex::degrees), targetHeading);

  while (std::fabs(error) > TURN_THRESHOLD && timeoutTimer.time(vex::seconds) < TIMEOUT_SEC) {
    double speed = KP_TURN * error;

    // Clamp de velocidad
    speed = std::max(-MAX_TURN_SPEED, std::min(MAX_TURN_SPEED, speed));

    // Giro en el lugar
    LeftDrive.spin(vex::forward, speed, vex::percent);
    RightDrive.spin(vex::forward, -speed, vex::percent);

    vex::task::sleep(static_cast<int>(LOOP_DELAY)); // Cambiado a vex::task::sleep

    error = angleDifference(GPS.heading(vex::degrees), targetHeading);
  }

  // Detener motores
  LeftDrive.stop(vex::brakeType::brake);
  RightDrive.stop(vex::brakeType::brake);

  // Log para depuración (mantener comentado)
  /*if (timeoutTimer.time(vex::seconds) >= TIMEOUT_SEC) {
    Brain.Screen.print("Timeout en turnToHeading");
    Brain.Screen.newLine();
  }*/
}

// Función para mover el robot a una posición específica (x, y) en mm usando GPS
void driveToPosition(double targetX, double targetY) {
  vex::timer timeoutTimer;
  timeoutTimer.clear();

  double currentX = GPS.xPosition(vex::distanceUnits::mm);
  double currentY = GPS.yPosition(vex::distanceUnits::mm);
  double distance = std::sqrt(std::pow(targetX - currentX, 2) + std::pow(targetY - currentY, 2));

  while (distance > DRIVE_THRESHOLD && timeoutTimer.time(vex::seconds) < TIMEOUT_SEC) {
    // Calcular heading deseado
    double desiredHeading = std::atan2(targetY - currentY, targetX - currentX) * 180.0 / M_PI;
    if (desiredHeading < 0) desiredHeading += 360.0;

    double headingError = angleDifference(GPS.heading(vex::degrees), desiredHeading);

    // Velocidad base basada en distancia
    double driveSpeed = KP_DRIVE * distance;
    driveSpeed = std::max(0.0, std::min(MAX_DRIVE_SPEED, driveSpeed));

    // Corrección diferencial para mantener heading
    double leftSpeed = driveSpeed - (KP_HEADING * headingError);
    double rightSpeed = driveSpeed + (KP_HEADING * headingError);

    // Clamp de velocidades
    leftSpeed = std::max(-MAX_DRIVE_SPEED, std::min(MAX_DRIVE_SPEED, leftSpeed));
    rightSpeed = std::max(-MAX_DRIVE_SPEED, std::min(MAX_DRIVE_SPEED, rightSpeed));

    LeftDrive.spin(vex::forward, leftSpeed, vex::percent);
    RightDrive.spin(vex::forward, rightSpeed, vex::percent);

    vex::task::sleep(static_cast<int>(LOOP_DELAY)); // Cambiado a vex::task::sleep

    // Actualizar posición y distancia
    currentX = GPS.xPosition(vex::distanceUnits::mm);
    currentY = GPS.yPosition(vex::distanceUnits::mm);
    distance = std::sqrt(std::pow(targetX - currentX, 2) + std::pow(targetY - currentY, 2));
  }

  // Detener motores
  LeftDrive.stop(vex::brakeType::brake);
  RightDrive.stop(vex::brakeType::brake);

  // Log para depuración (mantener comentado)
  /*if (timeoutTimer.time(vex::seconds) >= TIMEOUT_SEC) {
    Brain.Screen.print("Timeout en driveToPosition");
    Brain.Screen.newLine();
  }*/
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Inicializar GPS
  GPS.calibrate();
  while (GPS.isCalibrating()) {
    vex::task::sleep(50); // Cambiado a vex::task::sleep
  }

  // Resetear encoders de motores
  LeftDrive.resetPosition();
  RightDrive.resetPosition();
}

/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // Pruebita: Mover en un cuadrado de 500 mm x 500 mm, ajustado por offsets (750, 400)
  // Mover a (1250, 400) - Primer lado
  driveToPosition(1250.0, 400.0);
  // Girar a 90 grados
  turnToHeading(90.0);
  // Mover a (1250, 900) - Segundo lado
  driveToPosition(1250.0, 900.0);
  // Girar a 180 grados
  turnToHeading(180.0);
  // Mover a (750, 900) - Tercer lado
  driveToPosition(750.0, 900.0);
  // Girar a 270 grados
  turnToHeading(270.0);
  // Volver a (750, 400) - Cuarto lado
  driveToPosition(750.0, 400.0);
}

/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  while (1) {
    // Modo de manejo Tank
    LeftDrive.spin(vex::forward, Controller.Axis3.position(), vex::percent);
    RightDrive.spin(vex::forward, Controller.Axis2.position(), vex::percent);

    vex::task::sleep(20); // Cambiado a vex::task::sleep
  }
}

/*---------------------------------------------------------------------------*/
/*                              Main Function                                */
/*---------------------------------------------------------------------------*/

int main() {
  // Configurar callbacks para autónomo y control de usuario
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Ejecutar pre-autonomous
  pre_auton();

  // Bucle
  while (true) {
    vex::task::sleep(100); // Cambiado a vex::task::sleep
  }
}