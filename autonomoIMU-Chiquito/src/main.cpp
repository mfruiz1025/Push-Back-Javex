/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       USER                                                      */
/*    Created:      10/9/2025, 4:48:29 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "configuration.h"
#include "driver.h"
#include "funciones.h"

using namespace vex;

brain Brain;
controller Controller;

// Instanciar motores y sensores inerciales
competition Competition;
inertial inertialSensor = inertial(PORT21);

motor LeftMotor4 = motor(PORT5, gearSetting::ratio18_1, true);
motor LeftMotor1 = motor(PORT8, gearSetting::ratio18_1, false);
motor LeftMotor2 = motor(PORT9, gearSetting::ratio18_1, true);
motor LeftMotor3 = motor(PORT10, gearSetting::ratio18_1, false);

motor RightMotor1 = motor(PORT1, gearSetting::ratio18_1, true);
motor RightMotor2 = motor(PORT2, gearSetting::ratio18_1, false);
motor RightMotor3 = motor(PORT3, gearSetting::ratio18_1, true);
motor RightMotor4 = motor(PORT4, gearSetting::ratio18_1, false);

motor Recoleccion1(PORT11, false);
motor Recoleccion2(PORT15, true);
motor Recoleccion3(PORT18, true);
motor Recoleccion4(PORT20, false);
motor Recoleccion5(PORT19, false);

motor_group LeftDrive = motor_group(LeftMotor1, LeftMotor2, LeftMotor3, LeftMotor4);
motor_group RightDrive = motor_group(RightMotor1, RightMotor2, RightMotor3, RightMotor4);
motor_group Recoleccion = motor_group(Recoleccion1, Recoleccion2, Recoleccion3, Recoleccion4, Recoleccion5);

// Var PID
PID pid;

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
  pid.kp = 0.3;
  pid.ki = 0.009;
  pid.kd = 0.035;
  // ..........................................................................
  calibrateInertial();

  // PASO 1: Movimiento recto, girar y avanzar hacia el Goal del medio
  moveDistance(65, 70); // Mover hacia adelante 14 pulgadas a velocidad 80%
  rotateOnAxis(-45, 60, pid); // Girar a la izquierda 90 grados a velocidad 80%
  moveDistance(14, 70);
  activarRecoleccion(90, 3);

  //PASO 2: Dirigirse al Loader y recolectar bloques
  moveDistance(25, -65);
  rotateOnAxis(170, 50, pid);
  moveDistance(20, 65);
  rotateOnAxis(45, 40, pid);
  moveDistance(5, 65);
  activarRecoleccion(-80, 2);

  //PASO 3: Rotar y dirigirse al Goal
  moveDistance(8, -60);
  rotateOnAxis(180, 40, pid);
  moveDistance(8.5, 60);
  activarRecoleccion(-80, 5);

  //PASO 3: Rotar y dirigirse al Loader
  moveDistance(7, -60);
  rotateOnAxis(-180, 40, pid);
  moveDistance(8, 60);
  activarRecoleccion(-80, 3);

  //PASO 4: Rotar y dirigirse al Goal
  moveDistance(8, -60);
  rotateOnAxis(180, 40, pid);
  moveDistance(8.5, 60);
  activarRecoleccion(-80, 5);

  //PASO 4: Rotar e ir a la Parking Zone
  moveDistance(5, -60);
  rotateOnAxis(-100, 40, pid);
  moveDistance(40, 90);

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
  
      // Control de la recoleccion L1(Sube) y L2(Baja)
      if (Controller.ButtonL1.pressing()) {
          Recoleccion.spin(directionType::rev, 100, velocityUnits::pct);
      } else if (Controller.ButtonL2.pressing()) {
          Recoleccion.spin(directionType::fwd, 100, velocityUnits::pct);
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