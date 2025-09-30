/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       MSI                                                       */
/*    Created:      30/9/2025, 10:22:45                                       */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

// --- Dispositivos (sin cambios) ---
brain Brain;
controller Controller1;

// Motores del lado izquierdo
motor LeftMotor1(PORT6, true);
motor LeftMotor2(PORT8, false);
motor LeftMotor3(PORT9, true);
motor LeftMotor4(PORT10, false);
motor_group Left(LeftMotor1, LeftMotor2, LeftMotor3, LeftMotor4);

// Motores del lado derecho
motor RightMotor1(PORT1, true);
motor RightMotor2(PORT2, false);
motor RightMotor3(PORT3, true);
motor RightMotor4(PORT4, false);
motor_group Right(RightMotor1, RightMotor2, RightMotor3, RightMotor4);

// Motores Recolección
motor Recolección1(PORT12, false);
motor Recolección2(PORT11, false);
motor_group recoleccion(Recolección1, Recolección2);

// Visión
// Firma para la dona roja (ID 1)
vision::signature DONAROJA(1, 11425, 13517, 12471, -1, 763, 381, 2.5, 0);
// Firma para la dona azul (ID 2)
vision::signature DONAAZUL(2, -4509, -3861, -4185, 4557, 7439, 5998, 2.5, 0);
vision::signature ESTACA(3, -2287, -1627, -1957, -7241, -6429, -6835, 2.5, 0);
vex::vision Vision1(vex::PORT15, 90, DONAROJA, DONAAZUL, ESTACA);

// --- Funciones de Movimiento CORREGIDAS ---

// Función para avanzar
void moveForward(int speed)
{
    Left.spin(forward, speed, percent);
    Right.spin(forward, speed, percent);
}

// Función para girar a la derecha
void turnRight(int speed)
{
    Left.spin(forward, speed, percent);
    Right.spin(reverse, speed, percent);
}

// Función para girar a la izquierda (nombre corregido para mayor claridad)
void turnLeft(int speed)
{
    Left.spin(reverse, speed, percent);
    Right.spin(forward, speed, percent);
}

// Función para detener el chasis
void stopChassis()
{
    Left.stop(brake);
    Right.stop(brake);
}

int main()
{
    Vision1.setBrightness(90);

    // CORREGIDO: Toda la lógica va dentro de un bucle infinito
    while (true)
    {
        recoleccion.stop();

        // Toma una captura para buscar el objeto
        Vision1.takeSnapshot(DONAROJA);

        // CORREGIDO: Se usa una estructura if / else
        if (Vision1.objectCount == 0)
        {
            // Si no se ve ningún objeto, gira para buscarlo
            turnRight(10);
        }
        else
        {
            // Si se ve un objeto, intenta centrarlo y acercarte
            int x_pos = Vision1.largestObject.centerX;
            int width = Vision1.largestObject.width;

            // Lógica Proporcional para centrar el objeto
            // El centro de la cámara es aprox. 158.
            // Creamos un "margen de error" para no temblar.
            if (x_pos < 100)
            {
                // El objeto está a la izquierda, gira a la izquierda
                turnLeft(5);
            }
            else if (x_pos > 200)
            {
                // El objeto está a la derecha, gira a la derecha
                turnRight(5);
            }
            else
            {
                moveForward(20);
                recoleccion.spin(forward, 100, percent);
            }
        }

        // Pausa breve para la próxima captura
        wait(20, msec);
    }
}