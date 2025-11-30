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
inertial inertialSensor = inertial(PORT4);

motor LeftMotor1 = motor(PORT14, gearSetting::ratio18_1, true);
motor LeftMotor2 = motor(PORT15, gearSetting::ratio18_1, false);
motor LeftMotor3 = motor(PORT11, gearSetting::ratio18_1, true);
motor LeftMotor4 = motor(PORT16, gearSetting::ratio18_1, false);

motor RightMotor1 = motor(PORT5, gearSetting::ratio18_1, false);
motor RightMotor2 = motor(PORT6, gearSetting::ratio18_1, true);
motor RightMotor3 = motor(PORT7, gearSetting::ratio18_1, false);
motor RightMotor4 = motor(PORT8, gearSetting::ratio18_1, true);

motor RecolectionBack1 = motor(PORT12, gearSetting::ratio18_1, false);
motor RecolectionBack2 = motor(PORT13, gearSetting::ratio18_1, true);
motor RecolectionFront1 = motor(PORT3, gearSetting::ratio18_1, true);
motor RecolectionFront2 = motor(PORT18, gearSetting::ratio18_1, true);
motor RecolectionFront3 = motor(PORT19, gearSetting::ratio18_1, false);

motor_group LeftDrive = motor_group(LeftMotor1, LeftMotor2, LeftMotor3, LeftMotor4);
motor_group RightDrive = motor_group(RightMotor1, RightMotor2, RightMotor3, RightMotor4);
motor_group Recoleccion = motor_group(RecolectionBack1, RecolectionBack2, RecolectionFront1, RecolectionFront2, RecolectionFront3 );

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
  moveDistance(13, 40); // Mover hacia adelante 14 pulgadas a velocidad 80%
  rotateOnAxis(45, 60, pid); // Girar a la derecha 90 grados a velocidad 80%
  moveDistance(1.5, 20);
  activarRecoleccion(80, 1);
  activarRecoleccion(-80, 2);

  //PASO 2: Dirigirse al Loader y recolectar bloques
  moveDistance(12.5, -50);
  rotateOnAxis(180, 40, pid);
  moveDistance(2, 30);
  moveDistance(2, 30);
  moveDistanceRecolection(4, 7, 70);
  activarRecoleccion(60, 3);

  //PASO 3: Rotar y dirigirse al otro Loader
  moveDistance(2, -60);
  rotateOnAxis(0, 40, pid);
  moveDistance(2, 60);
  activarRecoleccion(80, 5);

  //PASO 4: Rotar y dirigirse al Loader
  moveDistance(2, -60);
  rotateOnAxis(180, 40, pid);
  moveDistance(3, 60);
  activarRecoleccion(80, 2);

  //PASO 5: Rotar y dirigirse al Goal
  moveDistance(2, -60);
  rotateOnAxis(0, 40, pid);
  moveDistance(2, 60);
  activarRecoleccion(80, 2);
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