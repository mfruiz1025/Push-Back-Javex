#include "vex.h"

using namespace vex;

brain Brain;
controller Controller1;

// Motores lado izquierdo
motor LeftMotor1(PORT7, true);
motor LeftMotor2(PORT8, false);
motor LeftMotor3(PORT9, true);
motor LeftMotor4(PORT10, false);
motor_group Left(LeftMotor1, LeftMotor2, LeftMotor3, LeftMotor4);

// Motores lado derecho
motor RightMotor1(PORT1, true);
motor RightMotor2(PORT2, false);
motor RightMotor3(PORT3, true);
motor RightMotor4(PORT4, false);
motor_group Right(RightMotor1, RightMotor2, RightMotor3, RightMotor4);

// IMU
inertial imuSensor(PORT20);

// Estructura PID
struct PID {
  double kp = 0;
  double ki = 0;
  double kd = 0;

  double error = 0;
  double prevError = 0;
  double integral = 0;
};

// Función PID genérica
double computePID(PID &pid, double setpoint, double current, double dt) {
  pid.error = setpoint - current;
  pid.integral += pid.error * dt;
  double derivative = (pid.error - pid.prevError) / dt;
  double output = pid.kp * pid.error + pid.ki * pid.integral + pid.kd * derivative;
  pid.prevError = pid.error;
  return output;
}

int main() {
  // Calibrar IMU
  imuSensor.calibrate();
  while (imuSensor.isCalibrating()) {
    wait(20, msec);
  }

  // Reset encoders
  Left.resetPosition();
  Right.resetPosition();

  // Crear e inicializar PID para distancia
  PID distPID;
  distPID.kp = 0.6;
  distPID.ki = 0.001;
  distPID.kd = 0.2;

  // Crear PID para rumbo (IMU)
  PID headingPID;
  headingPID.kp = 1.0;
  headingPID.ki = 0.0;
  headingPID.kd = 0.2;

  // Parámetros
  double targetDistanceCm = 20; // queremos avanzar 100 cm
  double wheelDiameter = 10.16;  // 4 pulgadas en cm
  double wheelCirc = wheelDiameter * M_PI;
  double degPerCm = 360.0 / wheelCirc;
  double targetDeg = targetDistanceCm * degPerCm;

  timer t;
  double lastTime = t.time(msec);

  while (true) {
    double currentTime = t.time(msec);
    double dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    // Posición promedio de los lados
    double leftPos = Left.position(degrees);
    double rightPos = Right.position(degrees);
    double avgPos = (leftPos + rightPos) / 2.0;

    // PID de distancia
    double distPower = computePID(distPID, targetDeg, avgPos, dt);

    // PID de rumbo (queremos heading = 0°)
    double angle = imuSensor.rotation(degrees);
    double headingPower = computePID(headingPID, 0, angle, dt);

    // Combinar
    double leftPower = distPower - headingPower;
    double rightPower = distPower + headingPower;

    // Limitar potencias
    if (leftPower > 100) leftPower = 100;
    if (leftPower < -100) leftPower = -100;
    if (rightPower > 100) rightPower = 100;
    if (rightPower < -100) rightPower = -100;

    // Mandar a motores
    Left.spin(forward, leftPower, percent);
    Right.spin(forward, rightPower, percent);

    // Telemetría
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("Distancia: %.2f cm", avgPos / degPerCm);
    Brain.Screen.setCursor(2,1);
    Brain.Screen.print("Heading: %.2f", angle);

    // Condición de salida
    if (fabs(targetDeg - avgPos) < degPerCm * 1.0) {
      break; // dentro de tolerancia de 1 cm
    }

    wait(20, msec);
  }

  Left.stop(brake);
  Right.stop(brake);
}