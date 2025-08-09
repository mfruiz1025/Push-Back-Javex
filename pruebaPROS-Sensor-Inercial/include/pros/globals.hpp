#pragma once
#include "pros/motors.hpp"
#include "pros/imu.hpp"

// Motores del lado izquierdo
extern pros::Motor leftMotor1;
extern pros::Motor leftMotor2;
extern pros::Motor leftMotor3;

// Motores del lado derecho
extern pros::Motor rightMotor1;
extern pros::Motor rightMotor2;
extern pros::Motor rightMotor3;

// Sensor de inercia
extern pros::Imu imu_sensor;