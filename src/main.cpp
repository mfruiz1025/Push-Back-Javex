/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       kenneth_busta                                             */
/*    Created:      8/11/2025, 5:12:57 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "basic-functions.h"
#include "parameters.h"
#include "autonomous.h"
#include "my-timer.h"
#include "controller.h"
#include "cstdlib"
#include "iostream"
#include "GPS.h"
#include "sensors.h"
#include "adjusment.h"

using namespace vex;
competition Competition;

int auton_strategy = 0;

// ===================== Control mappings (previous project style) =====================
int controlMode = 0;           // 0: twoJoysticksControl (arcade mix), 1: joystickNewControl (tank)
bool pistonAbierto = false;    // Toggle for Pinza
bool piston2Abierto = false;   // Toggle for brazo

void switchControlMode() {
  controlMode = (controlMode + 1) % 2;
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Modo de control: %d", controlMode);
}

void joystickNewControl(){
  // Tank: left = Axis3, right = Axis2
  int leftSpeed = Controller1.Axis3.position(vex::percentUnits::pct);
  int rightSpeed = Controller1.Axis2.position(vex::percentUnits::pct);
  Left.spin(vex::directionType::fwd, leftSpeed, vex::velocityUnits::pct);
  Right.spin(vex::directionType::fwd, rightSpeed, vex::velocityUnits::pct);
}

void twoJoysticksControl() {
  // Arcade: forward on Axis3, turn on Axis1
  int leftSpeed = Controller1.Axis3.position(vex::percentUnits::pct) + Controller1.Axis1.position(vex::percentUnits::pct);
  int rightSpeed = Controller1.Axis3.position(vex::percentUnits::pct) - Controller1.Axis1.position(vex::percentUnits::pct);
  Left.spin(vex::directionType::fwd, leftSpeed, vex::velocityUnits::pct);
  Right.spin(vex::directionType::fwd, rightSpeed, vex::velocityUnits::pct);
}

void pre_auton(void) {
  vexcodeInit();
}

void autonomous(void) {
  // SELECCIÓN DE ESTRATEGIA AUTÓNOMA MEDIANTE EL BOTÓN "RIGHT" DEL CONTROLADOR
  switch(auton_strategy) {
    case 0:
      auton_gps_precision(); // GPS-based precise autonomous sequence
      break;
    case 1:
      //auto two
      break;
    case 2:
      //auto three
      break;
    case 3:
      //auto four
      break;
    case 4:
      auton_sb();
      break;
    }
}

// (Se comenta la versión previa del controlador para referencia, ver bloque al final)

// ==================== PREVIOUS PROJECT STYLE CONTROLS ====================
// Implementación solicitada de usercontrol (con selección de autónomo + telemetría)
void usercontrol(void) {
  while (true) {
    // Cambiar el modo de control con el botón Y (y esperar a soltarlo)
    if (Controller1.ButtonY.pressing()) {
      switchControlMode();
      while (Controller1.ButtonY.pressing()) { wait(10, msec); }
    }

    // Control del robot basado en el modo seleccionado
    if (controlMode == 0) {
      twoJoysticksControl();
    } else if (controlMode == 1) {
      joystickNewControl();
    }

    // Control del motor recolector y rampa usando L1 y L2
    if (Controller1.ButtonL1.pressing()) {
      Recolector.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
      Rampa.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    } else if (Controller1.ButtonL2.pressing()) {
      Recolector.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
      Rampa.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    } else {
      Recolector.stop(vex::brakeType::hold);
      Rampa.stop(vex::brakeType::hold);
    }

    // Lugar reservado para garra (X/A) si aplica
    if (Controller1.ButtonX.pressing()) {
      // Garra.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    } else if (Controller1.ButtonA.pressing()) {
      // Garra.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    } else {
      // Garra.stop(vex::brakeType::hold);
    }

    // Toggle Pinza con R2 (espera a soltar)
    if (Controller1.ButtonR2.pressing()) {
      while (Controller1.ButtonR2.pressing()) { wait(10, msec); }
      pistonAbierto = !pistonAbierto;
      if (pistonAbierto) { Pinza.open(); } else { Pinza.close(); }
    }

    // Recolector neumático (mantener R1 para abrir)
    if (Controller1.ButtonR1.pressing()) { RecolectorNeumatica.open(); } else { RecolectorNeumatica.close(); }

    // Toggle brazo con B (espera a soltar)
    if (Controller1.ButtonB.pressing()) {
      while (Controller1.ButtonB.pressing()) { wait(10, msec); }
      piston2Abierto = !piston2Abierto;
      if (piston2Abierto) { brazo.open(); } else { brazo.close(); }
    }

    // Pruebas de hardware: forzar movimiento de cada lado
    if (Controller1.ButtonUp.pressing()) { Left.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct); }
    if (Controller1.ButtonLeft.pressing()) { Right.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct); }

    // Selección de estrategia autónoma (RIGHT) con eco en pantalla
    if (Controller1.ButtonRight.pressing()) {
      auton_strategy++;
      auton_strategy = auton_strategy % 4; // mantener 0..3
      switch (auton_strategy) {
        case 0: Controller1.Screen.setCursor(5, 1); Controller1.Screen.print("%10s", "GPS Precision"); break;
        case 1: Controller1.Screen.setCursor(5, 1); Controller1.Screen.print("%10s", "auto two"); break;
        case 2: Controller1.Screen.setCursor(5, 1); Controller1.Screen.print("%10s", "auto three"); break;
        case 3: Controller1.Screen.setCursor(5, 1); Controller1.Screen.print("%10s", "auto four"); break;
        case 4: Controller1.Screen.setCursor(5, 1); Controller1.Screen.print("%10s", "yousb"); break;
      }
      while (Controller1.ButtonRight.pressing()) { wait(10, msec); }
    }

    // Disparo manual de autónomo (DOWN)
    if (Controller1.ButtonDown.pressing()) { runAuton(); while (Controller1.ButtonDown.pressing()) { wait(10, msec); } }

    // Telemetría al Brain
    Brain.Screen.setCursor(1, 1); Brain.Screen.print("Heading: %3.2f   ", my_sensors.getBaseHeading());
    Brain.Screen.setCursor(2, 1); Brain.Screen.print("Forward Position: %4.1f   ", my_sensors.getBaseForwardPos());
    Brain.Screen.setCursor(3, 1);
    switch (auton_strategy) {
      case 0: Brain.Screen.print("%10s", "GPS Precision"); break;
      case 1: Brain.Screen.print("%10s", "auto two"); break;
      case 2: Brain.Screen.print("%10s", "auto three"); break;
      case 3: Brain.Screen.print("%10s", "auto four"); break;
      case 4: Brain.Screen.print("%10s", "yousb"); break;
    }

    // Espera para evitar saturar el CPU
    wait(20, msec);
  }
}

int main() {
  thread AutonSensors(autonSensors);
  thread AutonGps(autonGPS);
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  // Run the pre-autonomous function.
  pre_auton();
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}