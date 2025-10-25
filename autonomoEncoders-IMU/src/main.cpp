/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       David                                                     */
/*    Created:      10/9/2025, 4:48:29 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "configuration.h"
#include "driver.h"
#include "funciones.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  inertialSensor.calibrate();
  while (inertialSensor.isCalibrating()) {
    vex::task::sleep(100); // Espera hasta que la calibración esté completa
  }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  
  // PASO 1: Movimiento recto, recolectar el primer ring y giro derecha 90 grados
  moveDistance(14, 90); // Mover hacia adelante 14 pulgadas a velocidad 90%
  rotateOnAxis(90, 60); // Girar a la derecha 90 grados a velocidad 60%

  // PASO 2: Movimiento recto, recolectar el segundo ring y giro izquierda 30 grados
  moveDistance(24, 90); // Mover hacia adelante 24 pulgadas a velocidad 90%
  rotateOnAxis(-30, 60); // Girar a la izquierda 30 grados a velocidad 60%

  // PASO 3: Movimiento recto, recolectar el block naranja y giro derecha 90 grados
  moveDistance(14, 90); // Mover hacia adelante 14 pulgadas a velocidad 90%
  rotateOnAxis(90, 60); // Girar a la derecha 60 grados a velocidad 60%

  // PASO 4: Movimiento recto, recolectar el ring rojo y giro de 30 grados
  moveDistance(24, 90); // Mover hacia adelante 24 pulgadas a velocidad 90%
  rotateOnAxis(-30, 60); // Girar a la izquierda 30 grados a velocidad 60%

  

  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
    // User control code here, inside the loop
    while (true) {
      // Cambiar el modo de control con el botón A
      if (Controller.ButtonY.pressing()) {
          switchControlMode();
          while (Controller.ButtonA.pressing()) {
              // Esperar a que se suelte el botón A
              task::sleep(10);
          }
      }
  
      // Control del robot basado en el modo seleccionado
      if (controlMode == 0) {
          twoJoysticksControl();
      }else if(controlMode == 1){
          joystickNewControl(); 
      }
  
      // Control del motor recolector y rampa usando L1 y L2
      if (Controller.ButtonL1.pressing()) {
          Recoleccion.spin(directionType::fwd, 100, velocityUnits::pct);
      } else if (Controller.ButtonL2.pressing()) {
          Recoleccion.spin(directionType::rev, 100, velocityUnits::pct);
      } else {
          Recoleccion.stop(brakeType::hold);
      }
    }
    
      // Espera para evitar saturar el CPU
      wait(20, msec);
  }

//
// Main will set up the competition functions and callbacks.
//
int main() {
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