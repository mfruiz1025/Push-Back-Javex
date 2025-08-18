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

using namespace vex;

// A global instance of competition
competition Competition;

// DECLARACIONES GLOBALES
// Declaración del controlador
controller Controller = controller(primary);

// Declaración del GPS
gps GPS = gps(PORT10, 0.0, 0.0, distanceUnits::mm, 0.0, turnType::right);

// Declaraciones de motores
motor LeftMotor1 = motor(PORT1, ratio18_1, false);
motor LeftMotor2 = motor(PORT2, ratio18_1, false);
motor LeftMotor3 = motor(PORT3, ratio18_1, false);
motor LeftMotor4 = motor(PORT4, ratio18_1, false);

motor RightMotor1 = motor(PORT5, ratio18_1, true);
motor RightMotor2 = motor(PORT6, ratio18_1, true);
motor RightMotor3 = motor(PORT7, ratio18_1, true);
motor RightMotor4 = motor(PORT8, ratio18_1, true);

// Grupos de motores para la base
motor_group LeftDrive = motor_group(LeftMotor1, LeftMotor2, LeftMotor3, LeftMotor4);
motor_group RightDrive = motor_group(RightMotor1, RightMotor2, RightMotor3, RightMotor4);

// Constantes para PID/proporcional (ajustar según pruebas reales en el robot)
const double KP_TURN = 0.8;       // Ganancia proporcional para giros
const double KP_DRIVE = 0.5;      // Ganancia proporcional para avance
const double KP_HEADING = 1.0;    // Ganancia para corrección de heading
const double TURN_THRESHOLD = 1.0; // Umbral de error en grados para giros
const double DRIVE_THRESHOLD = 20.0; // Umbral de error en mm para posición
const double MAX_TURN_SPEED = 50.0;  // Velocidad máxima para giros (%)
const double MAX_DRIVE_SPEED = 80.0; // Velocidad máxima para avance (%)
const double LOOP_DELAY = 20.0;      // Delay en ms para loops de control
const double TIMEOUT_SEC = 10.0;     // Timeout en segundos

// Función auxiliar para calcular la diferencia de ángulo normalizada (-180 a 180 grados)
double angleDifference(double current, double target) {
  double diff = fmod(target - current + 180.0, 360.0) - 180.0;
  return diff;
}

// Función para girar el robot a un heading específico usando GPS
void turnToHeading(double targetHeading) {
  timer timeoutTimer;
  timeoutTimer.clear();

  double error = angleDifference(GPS.heading(degrees), targetHeading);

  while (fabs(error) > TURN_THRESHOLD && timeoutTimer.time(seconds) < TIMEOUT_SEC) {
    double speed = KP_TURN * error;

    // Clamp de velocidad
    speed = std::max(-MAX_TURN_SPEED, std::min(MAX_TURN_SPEED, speed));

    // Giro en el lugar
    LeftDrive.spin(forward, speed, percent);
    RightDrive.spin(forward, -speed, percent);

    wait(LOOP_DELAY, msec);

    error = angleDifference(GPS.heading(degrees), targetHeading);
  }

  // Detener motores
  LeftDrive.stop(brakeType::brake);
  RightDrive.stop(brakeType::brake);

  // Log para depuración
  /*if (timeoutTimer.time(seconds) >= TIMEOUT_SEC) {
    Brain.Screen.print("Timeout en turnToHeading");
    Brain.Screen.newLine();
  }*/
}

// Función para mover el robot a una posición específica (x, y) en mm usando GPS
void driveToPosition(double targetX, double targetY) {
  timer timeoutTimer;
  timeoutTimer.clear();

  double currentX = GPS.xPosition(mm);
  double currentY = GPS.yPosition(mm);
  double distance = sqrt(pow(targetX - currentX, 2) + pow(targetY - currentY, 2));

  while (distance > DRIVE_THRESHOLD && timeoutTimer.time(seconds) < TIMEOUT_SEC) {
    // Calcular heading deseado
    double desiredHeading = atan2(targetY - currentY, targetX - currentX) * 180.0 / M_PI;
    if (desiredHeading < 0) desiredHeading += 360.0;

    double headingError = angleDifference(GPS.heading(degrees), desiredHeading);

    // Velocidad base basada en distancia
    double driveSpeed = KP_DRIVE * distance;
    driveSpeed = std::max(0.0, std::min(MAX_DRIVE_SPEED, driveSpeed));

    // Corrección diferencial para mantener heading
    double leftSpeed = driveSpeed - (KP_HEADING * headingError);
    double rightSpeed = driveSpeed + (KP_HEADING * headingError);

    // Clamp de velocidades
    leftSpeed = std::max(-MAX_DRIVE_SPEED, std::min(MAX_DRIVE_SPEED, leftSpeed));
    rightSpeed = std::max(-MAX_DRIVE_SPEED, std::min(MAX_DRIVE_SPEED, rightSpeed));

    LeftDrive.spin(forward, leftSpeed, percent);
    RightDrive.spin(forward, rightSpeed, percent);

    wait(LOOP_DELAY, msec);

    // Actualizar posición y distancia
    currentX = GPS.xPosition(mm);
    currentY = GPS.yPosition(mm);
    distance = sqrt(pow(targetX - currentX, 2) + pow(targetY - currentY, 2));
  }

  // Detener motores
  LeftDrive.stop(brakeType::brake);
  RightDrive.stop(brakeType::brake);

  // Log para depuración
  /*if (timeoutTimer.time(seconds) >= TIMEOUT_SEC) {
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
    wait(50, msec);
  }

  // Confirmación en pantalla
  /*Brain.Screen.print("GPS calibrado exitosamente.");
  Brain.Screen.newLine();
  */
 
  // Resetear encoders de motores
  LeftDrive.resetPosition();
  RightDrive.resetPosition();
}

/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // Pruebita: Mover en un cuadrado de 500 mm x 500 mm
  // Mover a (500, 0)
  driveToPosition(500.0, 0.0);
  // Girar a 90 grados
  turnToHeading(90.0);
  // Mover a (500, 500)
  driveToPosition(500.0, 500.0);
  // Girar a 180 grados
  turnToHeading(180.0);
  // Mover a (0, 500)
  driveToPosition(0.0, 500.0);
  // Girar a 270 grados
  turnToHeading(270.0);
  // Volver a (0, 0)
  driveToPosition(0.0, 0.0);
}

/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  while (1) {
    // Modo de manejo Tank
    LeftDrive.spin(forward, Controller.Axis3.position(), percent);
    RightDrive.spin(forward, Controller.Axis2.position(), percent);

    // Imprimir en el cerebro
    /*
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("X: %.2f mm, Y: %.2f mm, H: %.2f deg",GPS.xPosition(mm), GPS.yPosition(mm), GPS.heading(degrees));
    */

    wait(20, msec);
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
    wait(100, msec);
  }
}