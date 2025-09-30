/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       MSI                                                       */
/*    Created:      30/9/2025, 7:18:50                                        */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

brain Brain;
controller Controller1;

// Motores del lado izquierdo (puertos 1-4)
motor LeftMotor1(PORT6, true);
motor LeftMotor2(PORT8, false);
motor LeftMotor3(PORT9, true);
motor LeftMotor4(PORT10, false);
motor_group Left(LeftMotor1, LeftMotor2, LeftMotor3, LeftMotor4);

// Motores del lado derecho (puertos 7-10)
motor RightMotor1(PORT1, true);
motor RightMotor2(PORT2, false);
motor RightMotor3(PORT3, true);
motor RightMotor4(PORT4, false);
motor_group Right(RightMotor1, RightMotor2, RightMotor3, RightMotor4);

// Sensor de Inercia
inertial imuSensor(PORT20);

// Sensor de Distancia
distance SensorDeDistancia = distance(PORT19);

struct PID
{
    double kp = 0;
    double ki = 0;
    double kd = 0;

    double error = 0;
    double prevError = 0;
    double Integral = 0;
};

double computerPID(PID &pid, double setpoint, double current, double dt)
{
    pid.error = setpoint - current;
    pid.Integral += pid.error * dt;

    double derivative = (pid.error - pid.prevError) / dt;
    double output = pid.kp * pid.error + pid.ki * pid.Integral + pid.kd * derivative;
    pid.prevError = pid.error;
    return output;
}

void resetPID(PID &pid) {
    pid.error = 0;
    pid.prevError = 0;
    pid.Integral = 0;
}


int main()
{
    imuSensor.calibrate();
    while (imuSensor.isCalibrating()) {
        wait(20, msec);
    }

    // --- Variables de control ---
    bool realizarGiro = false;
    double anguloObjetivoGiro=0;

    // --- Constantes del Robot ---
    double diametroRueda_cm = 10.16; // Ruedas de 4 pulgadas
    double circunferenciaRueda_cm = diametroRueda_cm * 3.14159;

    // --- Objetivos del Movimiento ---
    double distanciaAvanzar_cm = 50.0; // Avanzar 50 cm
    double gradosMotorObjetivo = (distanciaAvanzar_cm / circunferenciaRueda_cm) * 360;
    double rumboRectoObjetivo = 0.0; // Mantenerse a 0 grados al avanzar


    // Reiniciar sensores antes de empezar
    Left.resetPosition();
    imuSensor.setHeading(0, degrees);

    PID pidGiro, pidAvance, pidRumboRecto;

    // Bucle principal del robot
    while (true)
    {
        if (realizarGiro)
        {
            // =================================================
            // FASE 2: REALIZAR EL GIRO
            // =================================================
            resetPID(pidGiro);
            pidGiro.kp = 0.4; // Constantes específicas para un giro rápido y preciso
            pidGiro.ki = 0.005;
            pidGiro.kd = 0.05;

            double anguloActual;
            do {
                anguloActual = imuSensor.heading(degrees);
                double powerGiro = computerPID(pidGiro, anguloObjetivoGiro, anguloActual, 0.02); // dt fijo para simplicidad aquí
                
                Left.spin(forward, powerGiro, percent);
                Right.spin(forward, -powerGiro, percent);
                
                wait(20, msec);
            } while (fabs(anguloActual - anguloObjetivoGiro) > 2.0);

            // Giro completado, reiniciar para el próximo avance
            realizarGiro = false;
            Left.resetPosition(); // Reiniciar encoders para el próximo avance
            rumboRectoObjetivo = imuSensor.heading(degrees); // El nuevo objetivo es la dirección actual
            wait(500, msec); // Pequeña pausa
        }
        else
        {
            // =================================================
            // FASE 1: AVANZAR EN LÍNEA RECTA
            // =================================================
                        
            // Constantes para el avance 
            pidAvance.kp = 0.05;
            pidAvance.ki = 0.0004;
            pidAvance.kd = 0.005;

            // Constantes para la corrección de rumbo
            pidRumboRecto.kp = 0.5;
            pidRumboRecto.ki = 0;
            pidRumboRecto.kd = 0.1;

            // Bucle de control para el avance
            while (Left.position(degrees) < gradosMotorObjetivo)
            {
                // Leer sensores
                double posicionActualMotor = Left.position(degrees);
                double rumboActual = imuSensor.heading(degrees);

                // Calcular las dos potencias por separado
                double potenciaAvance = computerPID(pidAvance, gradosMotorObjetivo, posicionActualMotor, 0.02);
                double potenciaGiroCorreccion = computerPID(pidRumboRecto, rumboRectoObjetivo, rumboActual, 0.02);

                // Combinar las potencias
                Left.spin(forward, potenciaAvance + potenciaGiroCorreccion, percent);
                Right.spin(forward, potenciaAvance - potenciaGiroCorreccion, percent);

                wait(20, msec);
            }

            // --- Transición al giro ---
            Left.stop(brake);
            Right.stop(brake);
            
            // Calcular el próximo objetivo de giro
            anguloObjetivoGiro = imuSensor.heading(degrees) + 90;
            realizarGiro = true; // Activar la fase de giro para el siguiente ciclo del while(true)
            wait(500, msec); // Pequeña pausa
        }
    }
}