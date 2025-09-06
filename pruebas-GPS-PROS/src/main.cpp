#include "main.h"
#include "pros/gps.hpp"
#include "pros/motors.hpp"

// Controlador principal
pros::Controller master(pros::E_CONTROLLER_MASTER);

// GPS
pros::Gps gps(10);

// Motores izquierdos
pros::Motor m1(2, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor m2(3, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor m3(4, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor m4(5, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::MotorGroup left_motors({m1, m2, m3, m4});

// Motores derechos
pros::Motor m5(6, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor m6(7, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor m7(8, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor m8(9, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::MotorGroup right_motors({m5, m6, m7, m8});

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
	pros::lcd::initialize();

    pros::lcd::register_btn1_cb(on_center_button);

    // Configurar el GPS en puerto que es
    pros::Gps gps(10);
    gps.initialize_full(0.0, 0.0, 0.0, 0.0, 0.0);  // Ajustar la posicion inicial dependiendo de donde se coloque
    gps.set_data_rate(10);  // Tasa de datos de 10 ms
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
void competition_initialize() {}

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
	// Configura el GPS (mueve aquí para asegurar reinicio)
    pros::Gps gps(10);
    gps.initialize_full(0.0, 0.0, 0.0, 0.0, 0.0);  // Ajusta offset si es necesario

    // Constantes PID (ajusta con pruebas)
    const double kP = 100.0;
    const double kI = 0.1;
    const double kD = 10.0;

    // Coordenada objetivo en metros
    double target_x = 1.0;
    double target_y = 1.0;

    // Variables PID
    double error_x = 0, error_y = 0;
    double integral_x = 0, integral_y = 0;
    double last_error_x = 0, last_error_y = 0;

    // Función para movimientos
    auto drive = [&](double left_speed, double right_speed) {
        left_motors.move_velocity(left_speed);
        right_motors.move_velocity(right_speed);
    };

    while (true) {
        // Obtén posición actual del GPS
        double current_x = gps.get_position_x();
        double current_y = gps.get_position_y();

        // Calcula errores
        error_x = target_x - current_x;
        error_y = target_y - current_y;

        // Integral
        integral_x += error_x;
        integral_y += error_y;

        // Derivativo
        double derivative_x = error_x - last_error_x;
        double derivative_y = error_y - last_error_y;

        // Correcciones PID
        double correction_x = kP * error_x + kI * integral_x + kD * derivative_x;
        double correction_y = kP * error_y + kI * integral_y + kD * derivative_y;

        // Control diferencial: y para avance, x para giro
        double left_speed = correction_y + correction_x;
        double right_speed = correction_y - correction_x;

        // Limita velocidades (máx ~200 rpm para green gears)
        if (left_speed > 200) left_speed = 200;
        if (left_speed < -200) left_speed = -200;
        if (right_speed > 200) right_speed = 200;
        if (right_speed < -200) right_speed = -200;

        // Mueve el chasis
        drive(left_speed, right_speed);

        // Actualiza errores
        last_error_x = error_x;
        last_error_y = error_y;

        // Detiene si cerca del objetivo (tolerancia 0.05m = 5cm)
        if (std::fabs(error_x) < 0.05 && std::fabs(error_y) < 0.05) {
            drive(0, 0);
            break;
        }

        pros::delay(10);
    }
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
	pros::Controller master(pros::E_CONTROLLER_MASTER);

    // Configura el GPS para depuración
    pros::Gps gps(10);

    while (true) {
        pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & pros::E_LCD_BTN_LEFT) >> 2,
                         (pros::lcd::read_buttons() & pros::E_LCD_BTN_CENTER) >> 1,
                         (pros::lcd::read_buttons() & pros::E_LCD_BTN_RIGHT) >> 0);

        // Arcade control scheme
        int dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);    // Avance
        int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);  // Giro
        left_motors.move(dir - turn);                                    // Izquierda
        right_motors.move(dir + turn);                                   // Derecha

        // Imprime posición GPS para depuración
        pros::lcd::print(1, "X: %.2f Y: %.2f H: %.2f", gps.get_position_x(), gps.get_position_y(), gps.get_heading());

        pros::delay(20);
    }
}