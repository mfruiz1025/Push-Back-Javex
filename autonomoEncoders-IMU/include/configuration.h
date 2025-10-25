/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Kenneth Bustamante                                        */
/*    Description:  V5 project                                                */
/*                                                                            */

#include "vex.h"
#include <vector>
#include <functional>

#pragma once // Para evitar errores de definición múltiple

using namespace vex;
using Callbacks = std::vector<std::function<void()>>; // Callbacks

const double WHEEL_DIAMETER = 4.0;
const double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;
const double TRACK_WIDTH = 15.5;
const double RELATIVE_DISTANCE_ERROR = 0.4445;

//  ____
// |  [LA1]    [RA1]    |
// |[LB2]         [RB2] |
// |                    |
// |  [LA3]    [RA3]    |
// |[L4B]         [RB4] |
//  --------------------
brain Brain;
controller Controller;

// Sensor de Inercia
vex::inertial inertialSensor = vex::inertial(vex::PORT14);

// Declaraciones de motores
vex::motor LeftMotor4 = vex::motor(vex::PORT6, vex::gearSetting::ratio18_1, false);
vex::motor LeftMotor1 = vex::motor(vex::PORT8, vex::gearSetting::ratio18_1, false);
vex::motor LeftMotor2 = vex::motor(vex::PORT9, vex::gearSetting::ratio18_1, true);
vex::motor LeftMotor3 = vex::motor(vex::PORT10, vex::gearSetting::ratio18_1, false);

vex::motor RightMotor1 = vex::motor(vex::PORT1, vex::gearSetting::ratio18_1, true);
vex::motor RightMotor2 = vex::motor(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor RightMotor3 = vex::motor(vex::PORT3, vex::gearSetting::ratio18_1, true);
vex::motor RightMotor4 = vex::motor(vex::PORT4, vex::gearSetting::ratio18_1, true);

vex::motor recoleccion1 = vex::motor(vex::PORT11, vex::gearSetting::ratio18_1, true);
vex::motor recoleccion2 = vex::motor(vex::PORT12, vex::gearSetting::ratio18_1, false);

// Grupos de motores para la base
vex::motor_group LeftDrive = vex::motor_group(LeftMotor1, LeftMotor2, LeftMotor3, LeftMotor4);
vex::motor_group RightDrive = vex::motor_group(RightMotor1, RightMotor2, RightMotor3, RightMotor4);
vex::motor_group Recoleccion = vex::motor_group(recoleccion1, recoleccion2);