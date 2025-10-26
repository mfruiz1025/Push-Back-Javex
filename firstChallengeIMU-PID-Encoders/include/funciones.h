#include "vex.h"
#include <vector>
#include <functional>
#include <iostream>

#pragma once // Para evitar errores de definición múltiple

#include "configuration.h" // Incluimos la configuración de variables
#include <algorithm>

using namespace vex;
using Callbacks = std::vector<std::function<void()>>;

using std::cout;
using std::endl;

//---------------- FUNCIONES DE MOVIMIENTO ----------------

void resetEncoders()
{
    LeftMotor1.setPosition(0, rotationUnits::deg);
    LeftMotor2.setPosition(0, rotationUnits::deg);
    LeftMotor3.setPosition(0, rotationUnits::deg);
    LeftMotor4.setPosition(0, rotationUnits::deg);
    RightMotor1.setPosition(0, rotationUnits::deg);
    RightMotor2.setPosition(0, rotationUnits::deg);
    RightMotor3.setPosition(0, rotationUnits::deg);
    RightMotor4.setPosition(0, rotationUnits::deg);
}

void setLeftMotors(double speed)
{
    LeftMotor1.spin(directionType::fwd, speed, velocityUnits::pct);
    LeftMotor2.spin(directionType::fwd, speed, velocityUnits::pct);
    LeftMotor3.spin(directionType::fwd, speed, velocityUnits::pct);
    LeftMotor4.spin(directionType::fwd, speed, velocityUnits::pct);
}

void setRightMotors(double speed)
{
    RightMotor1.spin(directionType::fwd, speed, velocityUnits::pct);
    RightMotor2.spin(directionType::fwd, speed, velocityUnits::pct);
    RightMotor3.spin(directionType::fwd, speed, velocityUnits::pct);
    RightMotor4.spin(directionType::fwd, speed, velocityUnits::pct);
}

void stopAllMotors()
{
    LeftMotor1.stop();
    LeftMotor2.stop();
    LeftMotor3.stop();
    LeftMotor4.stop();
    RightMotor1.stop();
    RightMotor2.stop();
    RightMotor3.stop();
    RightMotor4.stop();
}

void moveDistance(double distanceInInches, double speed)
{
    wait(100, msec); // Espera breve para asegurar que se reinicie
    resetEncoders();
    // inertialSensor.resetRotation(); // Asegúrate de que el sensor esté calibrado antes de llamar esto

    // Compensar error de distancia si es necesario
    distanceInInches = (1.0 - RELATIVE_DISTANCE_ERROR) * distanceInInches;

    double targetRotations = (distanceInInches / WHEEL_CIRCUMFERENCE) * 360;

    double initialAngle = inertialSensor.rotation(); // Ángulo inicial
    double kP = 0.5;                                 // Constante proporcional, puedes ajustar esto

    while (fabs(LeftMotor1.position(rotationUnits::deg)) < targetRotations &&
           fabs(RightMotor1.position(rotationUnits::deg)) < targetRotations)
    {

        double currentAngle = inertialSensor.rotation();
        double error = currentAngle - initialAngle; // Positivo = desviado a la derecha

        double correction = error * kP;

        // Ajuste de velocidad con corrección
        double leftSpeed = speed - correction;
        double rightSpeed = speed + correction;

        // Limitar velocidades si es necesario
        leftSpeed = std::max(std::min(leftSpeed, 100.0), -100.0);
        rightSpeed = std::max(std::min(rightSpeed, 100.0), -100.0);

        setLeftMotors(leftSpeed);
        setRightMotors(rightSpeed);

        task::sleep(10); // Pequeña pausa para evitar sobrecarga del CPU
    }

    stopAllMotors();
    wait(100, msec); // Espera breve para asegurar que se reinicie
}

void moveDistanceRecolection(double distanceInInches, double speed)
{
    wait(100, msec); // Espera breve para asegurar que se reinicie
    resetEncoders();
    // inertialSensor.resetRotation(); // Asegúrate de que el sensor esté calibrado antes de llamar esto

    // Compensar error de distancia si es necesario
    distanceInInches = (1.0 - RELATIVE_DISTANCE_ERROR) * distanceInInches;

    double targetRotations = (distanceInInches / WHEEL_CIRCUMFERENCE) * 360;

    double initialAngle = inertialSensor.rotation(); // Ángulo inicial
    double kP = 0.5;                                 // Constante proporcional, puedes ajustar esto

    //Activar la reccoleccion
    Recoleccion.spin(reverse, 70, percent);

    while (fabs(LeftMotor1.position(rotationUnits::deg)) < targetRotations &&
           fabs(RightMotor1.position(rotationUnits::deg)) < targetRotations)
    {

        double currentAngle = inertialSensor.rotation();
        double error = currentAngle - initialAngle; // Positivo = desviado a la derecha

        double correction = error * kP;

        // Ajuste de velocidad con corrección
        double leftSpeed = speed - correction;
        double rightSpeed = speed + correction;

        // Limitar velocidades si es necesario
        leftSpeed = std::max(std::min(leftSpeed, 100.0), -100.0);
        rightSpeed = std::max(std::min(rightSpeed, 100.0), -100.0);

        setLeftMotors(leftSpeed);
        setRightMotors(rightSpeed);

        task::sleep(10); // Pequeña pausa para evitar sobrecarga del CPU
    }

    stopAllMotors();
    wait(100, msec); // Espera breve para asegurar que se reinicie
}

double computerPID(PID &pid, double setpoint, double current, double dt)
{
    pid.error = setpoint - current;
    pid.Integral += pid.error * dt;

    double derivative = (pid.error - pid.prevError) / dt;
    double output = pid.kp * pid.error + pid.ki * pid.Integral + pid.kd * derivative;
    pid.prevError = pid.error;
    return output;
}

void resetPID(PID &pid)
{
    pid.error = 0;
    pid.prevError = 0;
    pid.Integral = 0;
}

void rotateOnAxis(double angleInDegrees, double speed, PID pid_t)
{
    // inertialSensor.resetRotation(); // Comenzamos desde 0 grados
    wait(100, msec); // Espera breve para asegurar que se reinicie
    resetPID(pid_t);
    double maxSpeed = speed;
    double angle = inertialSensor.rotation(degrees);

    while (fabs(angleInDegrees - angle) > 2)
    {
        angle = inertialSensor.rotation(degrees);
        // Precisión de 0.5 grados
        double power = computerPID(pid_t, angleInDegrees, angle, 0.01);

        // Mando
        Brain.Screen.setFont(mono40); // Letra grande para verla bien
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Angulo: %.1f", angle);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Potencia: %.1f", power);
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("Error %.1f", angleInDegrees - angle);

        // Limita la velocidad
        if (power > maxSpeed)
            power = maxSpeed;
        else if (power < -maxSpeed)
            power = -maxSpeed;

        setLeftMotors(power);
        setRightMotors(-power);

        task::sleep(10);
    }

    stopAllMotors();
    wait(100, msec); // Espera breve para evitar que siga rotando
}

void rotateOnAxisRecoleccion(double angleInDegrees, double speed, PID pid_t)
{
    // inertialSensor.resetRotation(); // Comenzamos desde 0 grados
    wait(100, msec); // Espera breve para asegurar que se reinicie
    resetPID(pid_t);
    double maxSpeed = speed;
    double angle = inertialSensor.rotation(degrees);

    //Activar la reccoleccion
    Recoleccion.spin(reverse, 60, percent);

    while (fabs(angleInDegrees - angle) > 2)
    {
        angle = inertialSensor.rotation(degrees);
        // Precisión de 0.5 grados
        double power = computerPID(pid_t, angleInDegrees, angle, 0.01);

        // Mando
        Brain.Screen.setFont(mono40); // Letra grande para verla bien
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Angulo: %.1f", angle);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Potencia: %.1f", power);
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("Error %.1f", angleInDegrees - angle);

        // Limita la velocidad
        if (power > maxSpeed)
            power = maxSpeed;
        else if (power < -maxSpeed)
            power = -maxSpeed;

        setLeftMotors(power);
        setRightMotors(-power);

        task::sleep(10);
    }

    stopAllMotors();
    wait(100, msec); // Espera breve para evitar que siga rotando
}

void activarRecoleccion(int speed, double duration)
{
    Recoleccion.spin(reverse, speed, percent);
    wait(duration, seconds);
    stopAllMotors();
}

void calibrateInertial()
{
    inertialSensor.calibrate();
    while (inertialSensor.isCalibrating())
    {
        wait(100, msec);
    }
}