#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

#include "api.h"

#ifdef __cplusplus
extern "C" {
#endif

// Prototipos estándar de competencia
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
// Aquí agregamos tus declaraciones personalizadas

// Controlador
extern pros::Controller master;

// Motores del lado izquierdo
extern pros::Motor leftMotor1;
extern pros::Motor leftMotor2;
extern pros::Motor leftMotor3;

// Motores del lado derecho
extern pros::Motor rightMotor1;
extern pros::Motor rightMotor2;
extern pros::Motor rightMotor3;

// Funciones auxiliares (autónomo)
void driveForward(int speed, int time_ms);
void turnLeft(int speed, int time_ms);
void turnRight(int speed, int time_ms);

#endif

#endif  // _PROS_MAIN_H_
