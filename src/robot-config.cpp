//Contenido para VEX-V5-Program-Template/src/robot-config.cpp

#include "vex.h"

using namespace vex;

// Instancias globales
brain Brain;
controller Controller1 = controller(primary);

// --- Configuración de Puertos (Exacta de AutonomoGrandeRojoBaseNormal) ---

// Motores del lado izquierdo
motor MotorL1(PORT14, ratio18_1, true);
motor MotorL2(PORT18, ratio18_1, false);
motor MotorL3(PORT19, ratio18_1, false);
motor MotorL4(PORT20, ratio18_1, true);
motor_group Left(MotorL1, MotorL2, MotorL3, MotorL4);

// Motores del lado derecho
motor MotorR1(PORT7, ratio18_1, false);
motor MotorR2(PORT8, ratio18_1, true);
motor MotorR3(PORT9, ratio18_1, true);
motor MotorR4(PORT10, ratio18_1, false);
motor_group Right(MotorR1, MotorR2, MotorR3, MotorR4);

// Sensores (¡DEBES ASIGNAR PUERTOS REALES!)
inertial IMU = inertial(PORT11); // Ejemplo: Asignado al puerto 11
gps GPS_Sensor = gps(PORT15, 0, 0, distanceUnits::mm, 180); // Ejemplo: Asignado al puerto 15

// Mecanismos
motor Rec1 = motor(PORT13, ratio18_1, true);
motor Rec2 = motor(PORT5, ratio18_1, false);
motor_group Recolector = motor_group(Rec1, Rec2);
motor Rampa = motor(PORT12, ratio18_1, true);

pneumatics Pinza = pneumatics(Brain.ThreeWirePort.A);
pneumatics RecolectorNeumatica = pneumatics(Brain.ThreeWirePort.B);
triport expansor = triport(PORT16);
pneumatics brazo = pneumatics(expansor.C);


// Función de inicialización
bool RemoteControlCodeEnabled = true;
void vexcodeInit(void) {
  IMU.startCalibration();
  while (IMU.isCalibrating()) {
    this_thread::sleep_for(5);
  }
  Controller1.Screen.setCursor(5, 1);
  Controller1.Screen.print("%19s", "LISTO");
}