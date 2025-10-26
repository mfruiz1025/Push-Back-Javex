#include "vex.h"
using namespace vex;

// Librerías propias
#include "configuration.h"
#include "funciones.h"

// Modo de control
int controlMode = 0;

// Función para cambiar el modo de control
void switchControlMode() {
    controlMode = (controlMode + 1) % 2;
    Brain.Screen.clearScreen();
    Brain.Screen.print("Modo de control: %d", controlMode);
}

void joystickNewControl(){
  int leftSpeed=Controller.Axis3.position(); 
  int rightSpeed = Controller.Axis2.position(); 
  LeftDrive.spin(forward, leftSpeed, percent);
  RightDrive.spin(forward, rightSpeed, percent); 
}


// Función para controlar con dos joysticks
void twoJoysticksControl() {
    int leftSpeed = Controller.Axis3.position()+Controller.Axis1.position();
    int rightSpeed = Controller.Axis3.position()- Controller.Axis1.position();
    LeftDrive.spin(forward, leftSpeed, percent);
    RightDrive.spin(forward, rightSpeed, percent);
}