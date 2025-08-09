#include "main.h"
#include "pros/globals.hpp"
#include "pros/drive.hpp"
#include <cmath>

// Función para establecer la potencia de los motores
void setMotorPower(int left, int right) {
    leftMotor1.move(left);
    leftMotor2.move(left);
    leftMotor3.move(left);

    rightMotor1.move(right);
    rightMotor2.move(right);
    rightMotor3.move(right);
}

// Funciones para obtener la posición promedio de cada lado
double getLeftPosition() {
    return (fabs(leftMotor1.get_position()) + fabs(leftMotor2.get_position()) + fabs(leftMotor3.get_position())) / 3.0;
}

double getRightPosition() {
    return (fabs(rightMotor1.get_position()) + fabs(rightMotor2.get_position()) + fabs(rightMotor3.get_position())) / 3.0;
}

void driveForward(int speed, int time_ms) {
    setMotorPower(speed, speed);
    pros::delay(time_ms);
    setMotorPower(0, 0);
}

void turnLeft(int speed, int time_ms) {
    setMotorPower(-speed, speed);
    pros::delay(time_ms);
    setMotorPower(0, 0);
}

void turnRight(int speed, int time_ms) {
    setMotorPower(speed, -speed);
    pros::delay(time_ms);
    setMotorPower(0, 0);
}

void avanzarGrados(int grados) {
    // Reseteamos posición de todos los motores
    leftMotor1.tare_position();
    leftMotor2.tare_position();
    leftMotor3.tare_position();

    rightMotor1.tare_position();
    rightMotor2.tare_position();
    rightMotor3.tare_position();

    setMotorPower(50, 50);

    // Esperamos hasta que ambos lados hayan avanzado al menos "grados"
    while (getLeftPosition() < grados || getRightPosition() < grados) {
        pros::delay(10);
    }

    setMotorPower(0, 0);
}

void girarGrados(double grados) {
    while (imu_sensor.is_calibrating()) {
        pros::delay(10);
    }

    imu_sensor.reset();
    pros::delay(200);  // espera extra para reset

    double rotInicial = imu_sensor.get_rotation();
    printf("Rotación inicial IMU: %f\n", rotInicial);

    int direccion = grados > 0 ? 1 : -1;
    double objetivo = fabs(grados);

    setMotorPower(90 * direccion, -90 * direccion);

    int timeout_ms = 5000; // 5 segundos timeout
    int elapsed_ms = 0;
    const int delay_ms = 20;

    while (elapsed_ms < timeout_ms) {
        double rotActual = imu_sensor.get_rotation();
        double deltaRot = direccion * (rotActual - rotInicial);  // ajusta para dirección
        printf("Rotación actual: %f, Delta rotación: %f\n", rotActual, deltaRot);

        if (deltaRot >= objetivo) {
            break;  // objetivo alcanzado
        }

        pros::delay(delay_ms);
        elapsed_ms += delay_ms;
    }

    setMotorPower(0, 0);
}

void testGiro() {
    setMotorPower(80, -80);  // Gira con potencia 80 (izquierda y derecha opuestos)
    pros::delay(1000);       // Mantiene el giro por 1 segundo
    setMotorPower(0, 0);     // Para motores
}