#include "main.h"
#include "pros/globals.hpp"

// Controlador
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Motores del lado izquierdo
// Motores del lado izquierdo
pros::Motor leftMotor1(11, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor leftMotor2(15, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor leftMotor3(16, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

// Motores del lado derecho (los reversas se ajustan luego con set_reversed si se requiere)
pros::Motor rightMotor1(2, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor rightMotor2(3, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
pros::Motor rightMotor3(4, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

// Sensor de inercia (puerto 13)
pros::Imu imu_sensor(13);