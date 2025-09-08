/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       MSI                                                       */
/*    Created:      1/8/2025, 15:06:40                                        */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "vex.h"
#include <iostream>

using namespace vex;

/* Funciones
void moveForward(int speed, int duration);
void moveBackward(int speed, int duration);
void turnRight90(int speed);
void turnLeft90(int speed);
void stop(); */

// Configuración del cerebro y los motores
brain Brain;

// Crear instancias de los motores y asignarlos a los puertos correspondientes del cerebro
motor MotorL1(PORT5, false);
motor MotorL2(PORT3, false);
motor MotorL3(PORT20, false);

motor MotorR1(PORT1, true); 
motor MotorR2(PORT4, true);  
motor MotorR3(PORT11, true);  

// Visión
vision::signature DONAAZUL(1, -4615, -4347, -4481, 5805, 6993, 6399, 2.5, 0);
vision::signature DONAROJA(2, 10467, 12105, 11286, -1123, -831, -977, 2.5, 0);
vision::signature ESTACA(3, -2287, -1627, -1957, -7241, -6429, -6835, 2.5, 0);
vex::vision Vision1(vex::PORT10, 80, DONAAZUL, DONAROJA, ESTACA);

// Inercia
inertial Inertial21 = inertial(PORT21);

// Normaliza error de ángulo a [-180, 180] para evitar saltos de 360/0
double angleError(double target, double current) {
  double e = target - current;
  while (e > 180) e -= 360;
  while (e < -180) e += 360;
  return e;
}

// Función para mover el robot hacia adelante sin tiempo
void moveForward(int speed)
{
    MotorL1.spin(forward, speed, percent);
    MotorL2.spin(forward, speed, percent);
    MotorL3.spin(forward, speed, percent);
    MotorR1.spin(forward, speed, percent);
    MotorR2.spin(forward, speed, percent);
    MotorR3.spin(forward, speed, percent);
}

// Función para girar a la derecha
void turnRight(int speed)
{
    MotorL1.spin(forward, speed, percent);
    MotorL2.spin(forward, speed, percent);
    MotorL3.spin(forward, speed, percent);
    MotorR1.spin(reverse, speed, percent);
    MotorR2.spin(reverse, speed, percent);
    MotorR3.spin(reverse, speed, percent);
}

// Función para girar a la izquierda
void moveLeft(int speed)
{
    MotorL1.spin(reverse, speed, percent);
    MotorL2.spin(reverse, speed, percent);
    MotorL3.spin(reverse, speed, percent);
    MotorR1.spin(forward, speed, percent);
    MotorR2.spin(forward, speed, percent);
    MotorR3.spin(forward, speed, percent);
}

// Función para mover el robot hacia la izquierda (giro en arco)
void moveLeft(int speed, int duration)
{
    MotorL1.spin(reverse, speed, percent);
    MotorL2.spin(reverse, speed, percent);
    MotorL3.spin(reverse, speed, percent);
    MotorR1.spin(forward, speed, percent);
    MotorR2.spin(forward, speed, percent);
    MotorR3.spin(forward, speed, percent);
    wait(duration, seconds);
    MotorL1.stop();
    MotorL2.stop();
    MotorL3.stop();
    MotorR1.stop();
    MotorR2.stop();
    MotorR3.stop();
}

// Funcion detener
void stop()
{
    MotorL1.stop();
    MotorL2.stop();
    MotorL3.stop();
    MotorR1.stop();
    MotorR2.stop();
    MotorR3.stop();
}


void moveForward(int speed, double duration)
{
    MotorL1.spin(forward, speed, percent);
    MotorL2.spin(forward, speed, percent);
    MotorL3.spin(forward, speed, percent);
    MotorR1.spin(forward, speed, percent);
    MotorR2.spin(forward, speed, percent);
    MotorR3.spin(forward, speed, percent);
    wait(duration, seconds);
    stop();
}



int main() {
    // Configura el brillo solo una vez
    Vision1.setBrightness(50);

    while (true) {
        // Captura imagen con la firma DONAROJA
        Vision1.takeSnapshot(DONAROJA);

        // Limpia pantalla y muestra información
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("Object Count: %d", Vision1.objectCount);

        if (Vision1.objectCount > 0) {
            // Hay al menos una dona detectada
            int x = Vision1.objects[0].centerX;
            int y = Vision1.objects[0].centerY;
            int tam = Vision1.objects[0].width;

            Brain.Screen.setCursor(2, 1);
            Brain.Screen.print("Width: %d", tam);
            Brain.Screen.setCursor(3, 1);
            Brain.Screen.print("Center x: %d", x);

            // Ajuste de dirección
            if (x < 80) {
                // Muy a la izquierda
                moveLeft(5);
            } else if (x > 220) {
                // Muy a la derecha
                turnRight(5);
            } else {
                // Centrado: avanza hacia la dona
                moveForward(20);
            }
        } else {
            // Si no hay dona: gira para buscarla
            Brain.Screen.setCursor(2, 1);
            Brain.Screen.print("No object detected");
            turnRight(10);
        }

        wait(100, msec); // Pausa breve para evitar sobrecarga de CPU
    }
}