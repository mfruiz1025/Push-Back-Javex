#include "main.h"
#include "pros/globals.hpp"
#include "pros/drive.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2); 
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// Invertimos motores izquierdos
	leftMotor1.set_reversed(true);
	leftMotor3.set_reversed(true);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
    driveForward(100, 1000); // Avanza rápido por 1 segundo
    turnLeft(80, 500);       // Gira a la izquierda por 0.5 segundos
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    while (imu_sensor.is_calibrating()) {
        pros::delay(10);
    }

    avanzarGrados(100);    // Avanza ~100 grados en motor
    girarGrados(20);        // Gira 90° a la derecha
    avanzarGrados(500);
    girarGrados(-45);       // Gira 45° a la izquierda
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	while (true) {
        int leftPower = master.get_analog(ANALOG_LEFT_Y);
        int rightPower = master.get_analog(ANALOG_RIGHT_Y);

        leftMotor1.move(leftPower);
        leftMotor2.move(leftPower);
        leftMotor3.move(leftPower);

        rightMotor1.move(rightPower);
        rightMotor2.move(rightPower);
        rightMotor3.move(rightPower);

        pros::delay(20);
    }
}