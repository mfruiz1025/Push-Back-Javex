/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       MSI                                                       */
/*    Created:      8/9/2025, 12:24:07                                        */
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

// Estructura pra definir el PID
struct PID
{
    double kp = 0;
    double ki = 0;
    double kd = 0;

    double error = 0;
    double prevError = 0;
    double Integral = 0;
};

// Función para calcular el PID
double computerPID(PID &pid, double setpoint, double current, double dt)
{
    pid.error = setpoint - current;
    pid.Integral += pid.error * dt;

    double derivative = (pid.error - pid.prevError) / dt;
    double output = pid.kp * pid.error + pid.ki * pid.Integral + pid.kd * derivative;
    pid.prevError = pid.error;
    return output;
}

int main()
{

    imuSensor.calibrate();

    while (imuSensor.isCalibrating())
    {
        wait(20, msec);
    }

    // Crear las estructuras PID para los dos sensores
    PID pidDistancia;
    PID pidInercia;

    // Definir los parametros de cada sensor
    pidDistancia.kp = 0.05;
    pidDistancia.ki = 0.0005;
    pidDistancia.kd = 0.005;
    double targetDistancia = 300;

    pidInercia.kp = 0.3;
    pidInercia.ki = 0.009;
    pidInercia.kd = 0.035;
    double targetInercia1 = 90;
    double targetInercia2 = -90;

    timer t;
    double lastTime = t.time(msec);

    double angle;

    while (true)
    {
        double currentTime = t.time(msec);

        double dt = (currentTime - lastTime) / 1000.0;

        lastTime = currentTime;

        double distancia = SensorDeDistancia.objectDistance(mm);

        double powerDistancia = computerPID(pidDistancia, targetDistancia, distancia, dt);
        if (powerDistancia > 100)
            powerDistancia = 100;

        if (powerDistancia < -100)
            powerDistancia = -100;

        Left.spin(forward, -powerDistancia, percent);

        Right.spin(forward, -powerDistancia, percent);

        Brain.Screen.clearLine(1);                        // Limpia la primera fila de la pantalla
        Brain.Screen.setCursor(1, 1);                     // Coloca el cursor en la fila 1, columna 1
        Brain.Screen.print("Distancia: %.2f", distancia); // Imprime el texto y el valor de la variable

        if (distancia >= 300 && distancia <= 310)
        {
            do
            {
                currentTime = t.time(msec);
                dt = (currentTime - lastTime) / 1000.0;
                lastTime = currentTime;
                angle = imuSensor.angle(degrees);

                double powerGiro = computerPID(pidInercia, targetInercia2, angle, dt);
                if (powerGiro > 100)
                    powerDistancia = 100;
                if (powerDistancia < -100)
                    powerDistancia = -100;

                Left.spin(forward, powerGiro, percent);

                Right.spin(forward, -powerGiro, percent);

                Brain.Screen.clearLine(1);                 // Limpia la primera fila de la pantalla
                Brain.Screen.setCursor(2, 1);              // Coloca el cursor en la fila 1, columna 1
                Brain.Screen.print("Angulo: %.2f", angle); // Imprime el texto y el valor de la variable
                wait(20, msec);
            } while (fabs(angle - targetInercia2) > 2.0);
        }
        wait(20, msec);
    }
}
