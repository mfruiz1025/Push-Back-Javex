#pragma once // Evita errores de doble inclusión
#include "vex.h"

using namespace vex;

extern brain Brain;
extern controller Controller1;

// --- Chasis ---
extern motor MotorL1;
extern motor MotorL2;
extern motor MotorL3;
extern motor MotorL4;
extern motor_group Left;

extern motor MotorR1;
extern motor MotorR2;
extern motor MotorR3;
extern motor MotorR4;
extern motor_group Right;

// --- Sensores ---
extern inertial IMU;
extern gps GPS_Sensor; // ¡Sensor GPS añadido!

// --- Mecanismos ---
extern motor Rec1;
extern motor Rec2;
extern motor_group Recolector;
extern motor Rampa;

extern pneumatics Pinza;
extern pneumatics RecolectorNeumatica;
extern triport expansor;
extern pneumatics brazo;

void vexcodeInit(void);